package salad

import (
	"github.com/golang/geo/r3"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

// GrabStepAction is an optional gripper operation performed after the arm reaches a step's pose.
type GrabStepAction int

const (
	GrabStepActionNone  GrabStepAction = iota
	GrabStepActionOpen                 // open gripper
	GrabStepActionClose                // close gripper (Grab)
)

// GrabStep is a single motion step: move the arm to Pose under Constraints, then perform PostAction.
type GrabStep struct {
	Name        string
	Pose        spatialmath.Pose
	Constraints *motionplan.Constraints // nil = unconstrained
	PostAction  GrabStepAction
}

// GrabPlan is the pre-computed sequence of steps for a single ingredient grab.
// TODO: add ID field for per-grab tracing/debugging
type GrabPlan struct {
	BinName string
	ZoneID  int
	Steps   []GrabStep
}

func (s *grabberControls) planGrab(bin *grabberBinSwitches, zoneID int, zone *segmentation.Zone, depthOffsetMM float64) (*GrabPlan, error) {
	grabPose, err := s.computeGrabPose(zone, depthOffsetMM)
	if err != nil {
		return nil, err
	}

	hover := s.applyXYOffset(bin.hoverPose)
	grab := s.applyXYOffset(grabPose)
	tilted := spatialmath.NewPose(grab.Point(), s.cfg.GrabOrientation)
	grabConstraints := s.grabLinearConstraints()

	steps := []GrabStep{
		{Name: "above_bin", Pose: hover, PostAction: GrabStepActionOpen},
		{Name: "descend", Pose: grab, Constraints: grabConstraints},
		{Name: "tilt", Pose: tilted, PostAction: GrabStepActionClose},
		{Name: "untilt", Pose: grab},
		{Name: "ascend", Pose: hover, Constraints: grabConstraints},
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
		steps = append(steps, GrabStep{Name: "clearance", Pose: clearancePose, Constraints: s.clearanceLinearConstraints()})
	}

	steps = append(steps,
		GrabStep{Name: "bowl_hover", Pose: s.bowlHoverPose},
		GrabStep{Name: "drop", Pose: s.droppingPose, PostAction: GrabStepActionOpen},
		GrabStep{Name: "return_bowl_hover", Pose: s.bowlHoverPose},
	)

	return &GrabPlan{BinName: bin.name, ZoneID: zoneID, Steps: steps}, nil
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
