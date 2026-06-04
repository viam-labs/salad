package salad

import (
	"context"
	"fmt"
	"math"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"

	"salad/lib/fileio"
)

type dressingStepSpec struct {
	name        string
	goal        spatialmath.Pose
	constraints *motionplan.Constraints
	postAction  GrabStepAction
	postShake   bool
	postSqueeze bool
	moveOptions *arm.MoveOptions
}

type dressingStep struct {
	name         string
	trajectory   motionplan.Trajectory
	planningTime time.Duration
	postAction   GrabStepAction
	postShake    bool
	postSqueeze  bool
	moveOptions  *arm.MoveOptions
	revolutions  int
}

type dressingPlan struct {
	dressingName string
	steps        []dressingStep
	plannedAt    time.Time
}

func (s *dressingControls) planDressing(ctx context.Context, name, buildID string) (*dressingPlan, error) {
	opt, ok := s.cfg.Dressings[name]
	if !ok {
		return nil, fmt.Errorf("unknown dressing %q", name)
	}

	if err := s.loadWorldState(); err != nil {
		return nil, err
	}

	grabSpeedDegsPerSec := s.cfg.GrabSpeedDegsPerSec
	if grabSpeedDegsPerSec == 0 {
		grabSpeedDegsPerSec = 30
	}
	grabMoveOptions := &arm.MoveOptions{MaxVelRads: grabSpeedDegsPerSec * math.Pi / 180}

	specs := []dressingStepSpec{
		{name: "approach_grab",        goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "grab",                 goal: opt.Grab.toPose(),                constraints: opt.Grab.Constraints,               postAction: GrabStepActionClose, moveOptions: grabMoveOptions},
		{name: "approach_grab_up",     goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "prepare_dressing",     goal: s.cfg.PrepareDressing.toPose(),  constraints: s.cfg.PrepareDressing.Constraints},
		{name: "pour",                 goal: s.cfg.PourDressing.toPose(),     constraints: s.cfg.PourDressing.Constraints,      postSqueeze: true},
		{name: "post_pour",            goal: s.cfg.PostPourDressing.toPose(), constraints: s.cfg.PostPourDressing.Constraints},
		{name: "prepare_return",       goal: s.cfg.PrepareDressing.toPose(),  constraints: s.cfg.PrepareDressing.Constraints},
		{name: "approach_grab_return", goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "grab_return",          goal: opt.Grab.toPose(),               constraints: opt.Grab.Constraints,                postAction: GrabStepActionOpen,  moveOptions: grabMoveOptions},
		{name: "approach_grab_final",  goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
	}

	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return nil, fmt.Errorf("building frame system: %w", err)
	}

	armCurrentInputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		return nil, fmt.Errorf("getting arm current inputs: %w", err)
	}
	startInputs := referenceframe.NewZeroInputs(fs)
	if len(armCurrentInputs) > 0 {
		startInputs[s.cfg.Arm] = armCurrentInputs
	}
	startState := armplanning.NewPlanState(nil, startInputs)

	steps := make([]dressingStep, 0, len(specs))
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
		s.fileSaver.SaveAsync(ctx, fileio.NewPlanRequestSaveFile(
			req, buildID,
			fmt.Sprintf("dressing_%s_%s_plan_request.json", name, spec.name),
			t, planDur,
		))
		if err != nil {
			return nil, fmt.Errorf("planning step %q: %w", spec.name, err)
		}

		traj := plan.Trajectory()
		steps = append(steps, dressingStep{
			name:         spec.name,
			trajectory:   traj,
			planningTime: planDur,
			postAction:   spec.postAction,
			postShake:    spec.postShake,
			postSqueeze:  spec.postSqueeze,
			moveOptions:  spec.moveOptions,
		})

		if len(traj) > 0 {
			startState = armplanning.NewPlanState(nil, traj[len(traj)-1])
		}

		if spec.name == "pour" && s.cfg.CircularPour != nil {
			circStep, newStartState, err := s.planCircularPour(ctx, fs, startState, name, buildID)
			if err != nil {
				return nil, err
			}
			steps = append(steps, *circStep)
			startState = newStartState
		}
	}

	return &dressingPlan{dressingName: name, steps: steps, plannedAt: time.Now()}, nil
}

func (s *dressingControls) planCircularPour(ctx context.Context, fs *referenceframe.FrameSystem, startState *armplanning.PlanState, name, buildID string) (*dressingStep, *armplanning.PlanState, error) {
	cfg := s.cfg.CircularPour
	pointsPerRev := cfg.PointsPerRev
	if pointsPerRev == 0 {
		pointsPerRev = 8
	}
	revolutions := cfg.Revolutions
	if revolutions < 1 {
		revolutions = 1
	}

	poses := computeCircularPoses(s.cfg.PourDressing.toPose(), cfg.RadiusMm, pointsPerRev)
	goals := make([]*armplanning.PlanState, len(poses))
	for i, pose := range poses {
		goals[i] = armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{
				s.cfg.Arm: referenceframe.NewPoseInFrame(referenceframe.World, pose),
			},
			nil,
		)
	}

	req := &armplanning.PlanRequest{
		FrameSystem: fs,
		WorldState:  s.worldState,
		StartState:  startState,
		Goals:       goals,
		Constraints: cfg.Constraints,
	}

	t := time.Now()
	plan, _, err := armplanning.PlanMotion(ctx, s.logger, req)
	planDur := time.Since(t)
	s.logger.Infof("planned step %q in %.2fs", "circular_pour", planDur.Seconds())
	s.fileSaver.SaveAsync(ctx, fileio.NewPlanRequestSaveFile(
		req, buildID,
		fmt.Sprintf("dressing_%s_circular_pour_plan_request.json", name),
		t, planDur,
	))
	if err != nil {
		return nil, nil, fmt.Errorf("planning step %q: %w", "circular_pour", err)
	}

	traj := plan.Trajectory()
	var newStartState *armplanning.PlanState
	if len(traj) > 0 {
		newStartState = armplanning.NewPlanState(nil, traj[len(traj)-1])
	} else {
		newStartState = startState
	}

	return &dressingStep{
		name:         "circular_pour",
		trajectory:   traj,
		planningTime: planDur,
		revolutions:  revolutions,
		postShake:    true,
	}, newStartState, nil
}

func computeCircularPoses(centerPose spatialmath.Pose, radiusMm float64, pointsPerRev int) []spatialmath.Pose {
	center := centerPose.Point()
	poses := make([]spatialmath.Pose, pointsPerRev)
	for i := range pointsPerRev {
		angle := 2 * math.Pi * float64(i) / float64(pointsPerRev)
		offset := r3.Vector{X: radiusMm * math.Cos(angle), Y: radiusMm * math.Sin(angle)}
		poses[i] = spatialmath.NewPose(center.Add(offset), centerPose.Orientation())
	}
	return poses
}
