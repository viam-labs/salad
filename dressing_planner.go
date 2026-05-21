package salad

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/framesystem"
	"go.viam.com/rdk/spatialmath"
)

type dressingStepSpec struct {
	name        string
	goal        spatialmath.Pose
	constraints *motionplan.Constraints
	postAction  GrabStepAction
	postShake   bool
}

type dressingStep struct {
	name         string
	trajectory   motionplan.Trajectory
	planningTime time.Duration
	postAction   GrabStepAction
	postShake    bool
}

type dressingPlan struct {
	dressingName string
	steps        []dressingStep
	plannedAt    time.Time
}

func (s *dressingControls) planDressing(ctx context.Context, name string) (*dressingPlan, error) {
	opt, ok := s.cfg.Dressings[name]
	if !ok {
		return nil, fmt.Errorf("unknown dressing %q", name)
	}

	if err := s.loadWorldState(); err != nil {
		return nil, err
	}

	specs := []dressingStepSpec{
		{name: "approach_grab",        goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "grab",                 goal: opt.Grab.toPose(),                constraints: opt.Grab.Constraints,               postAction: GrabStepActionClose},
		{name: "approach_grab_up",     goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "prepare_dressing",     goal: s.cfg.PrepareDressing.toPose(),  constraints: s.cfg.PrepareDressing.Constraints},
		{name: "pour",      goal: s.cfg.PourDressing.toPose(),     constraints: s.cfg.PourDressing.Constraints,      postShake: true},
		{name: "post_pour", goal: s.cfg.PostPourDressing.toPose(), constraints: s.cfg.PostPourDressing.Constraints},
		{name: "prepare_return",       goal: s.cfg.PrepareDressing.toPose(),  constraints: s.cfg.PrepareDressing.Constraints},
		{name: "approach_grab_return", goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "grab_return",          goal: opt.Grab.toPose(),               constraints: opt.Grab.Constraints,                postAction: GrabStepActionOpen},
		{name: "approach_grab_final",  goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "home",                 goal: s.cfg.Home.toPose(),             constraints: s.cfg.Home.Constraints},
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
		})

		if len(traj) > 0 {
			startState = armplanning.NewPlanState(nil, traj[len(traj)-1])
		}
	}

	return &dressingPlan{dressingName: name, steps: steps, plannedAt: time.Now()}, nil
}
