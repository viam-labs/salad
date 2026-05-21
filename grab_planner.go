package salad

import (
	"context"
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

// GrabStepAction is an optional gripper operation performed after the arm executes a step's trajectory.
type GrabStepAction int

const (
	GrabStepActionNone  GrabStepAction = iota
	GrabStepActionOpen                 // open gripper
	GrabStepActionClose                // close gripper (Grab)
)

// GrabStep holds the pre-computed trajectory for one motion phase.
type GrabStep struct {
	Name         string
	Trajectory   motionplan.Trajectory
	PlanningTime time.Duration
	PostAction   GrabStepAction
}

// GrabPlan is the fully pre-computed set of trajectories for a single ingredient grab.
// TODO: add ID field for per-grab tracing/debugging
type GrabPlan struct {
	BinName   string
	ZoneID    int
	Steps     []GrabStep
	PlannedAt time.Time
}

type grabStepSpec struct {
	name        string
	goal        spatialmath.Pose
	constraints *motionplan.Constraints
	postAction  GrabStepAction
}

func (s *grabberControls) planGrab(ctx context.Context, bin *grabberBinSwitches, zoneID int, zone *segmentation.Zone, depthOffsetMM float64) (*GrabPlan, error) {
	grabPose, err := s.computeGrabPose(ctx, zone, depthOffsetMM)
	if err != nil {
		return nil, err
	}

	// hover := s.applyXYOffset(bin.hoverPose)
	// grab := s.applyXYOffset(grabPose)

	hoverWithOffset := s.applyXYOffset(bin.hoverPose)
	// use the grab pose orientation so the orientations are consistent between the two
	hover := spatialmath.NewPose(hoverWithOffset.Point(), grabPose.Orientation())

	specs := []grabStepSpec{
		{name: "above_bin", goal: hover, postAction: GrabStepActionOpen},
		{name: "descend", goal: grabPose, constraints: s.grabLinearConstraints(), postAction: GrabStepActionClose},
		{name: "ascend", goal: hover, constraints: s.grabLinearConstraints()},
	}

	if s.cfg.EnableBinClearance {
		hoverPt := hover.Point()
		clearancePose := spatialmath.NewPose(
			r3.Vector{
				X: hoverPt.X + s.cfg.BinClearanceXOffsetMM,
				Y: hoverPt.Y,
				Z: hoverPt.Z + s.cfg.BinClearanceZOffsetMM,
			},
			hover.Orientation(),
		)
		specs = append(specs, grabStepSpec{name: "clearance", goal: clearancePose, constraints: s.clearanceLinearConstraints()})
	}

	specs = append(specs,
		grabStepSpec{name: "bowl_hover", goal: s.bowlHoverPose},
		grabStepSpec{name: "drop", goal: s.droppingPose, postAction: GrabStepActionOpen},
		grabStepSpec{name: "return_bowl_hover", goal: s.bowlHoverPose},
	)

	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return nil, fmt.Errorf("building frame system: %w", err)
	}

	armCurrentInputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		return nil, fmt.Errorf("getting arm current inputs: %w", err)
	}
	s.logger.Debugf("arm frame key %q, inputs len=%d, frame system frames: %v", s.cfg.Arm, len(armCurrentInputs), fs.FrameNames())
	startInputs := referenceframe.NewZeroInputs(fs)
	if len(armCurrentInputs) > 0 {
		startInputs[s.cfg.Arm] = armCurrentInputs
	}
	startState := armplanning.NewPlanState(nil, startInputs)

	steps := make([]GrabStep, 0, len(specs))
	for _, spec := range specs {
		goalState := armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{
				s.cfg.Arm: referenceframe.NewPoseInFrame(referenceframe.World, spec.goal),
			},
			nil,
		)
		req := &armplanning.PlanRequest{
			FrameSystem: fs,
			WorldState:  s.worldState,
			StartState:  startState,
			Goals:       []*armplanning.PlanState{goalState},
			Constraints: spec.constraints,
		}

		t := time.Now()
		plan, _, err := armplanning.PlanMotion(ctx, s.logger, req)
		planDur := time.Since(t)
		s.logger.Infof("planned step %q in %.2fs", spec.name, planDur.Seconds())
		if err != nil {
			return nil, fmt.Errorf("planning step %q: %w", spec.name, err)
		}

		traj := plan.Trajectory()
		steps = append(steps, GrabStep{
			Name:         spec.name,
			Trajectory:   traj,
			PlanningTime: planDur,
			PostAction:   spec.postAction,
		})

		if len(traj) > 0 {
			startState = armplanning.NewPlanState(nil, traj[len(traj)-1])
		}
	}

	return &GrabPlan{BinName: bin.name, ZoneID: zoneID, Steps: steps, PlannedAt: time.Now()}, nil
}

func (s *grabberControls) grabLinearConstraints() *motionplan.Constraints {
	lineTol := s.cfg.GrabLineToleranceMM
	if lineTol == 0 {
		lineTol = 1.0
	}
	orientTol := s.cfg.GrabOrientationToleranceDegs
	if orientTol == 0 {
		orientTol = 1.0
	}
	return &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{{
			LineToleranceMm:          lineTol,
			OrientationToleranceDegs: orientTol,
		}},
	}
}

func (s *grabberControls) clearanceLinearConstraints() *motionplan.Constraints {
	lineTol := s.cfg.ClearanceLineToleranceMM
	if lineTol == 0 {
		lineTol = 1.0
	}
	orientTol := s.cfg.ClearanceOrientationToleranceDegs
	if orientTol == 0 {
		orientTol = 45.0
	}
	return &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{{
			LineToleranceMm:          lineTol,
			OrientationToleranceDegs: orientTol,
		}},
	}
}
