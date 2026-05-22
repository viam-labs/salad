package salad

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"golang.org/x/sync/errgroup"
)

func (s *grabberControls) executePrePostAction(ctx context.Context, action GrabStepAction) error {
	switch action {
	case GrabStepActionGoHome:
		if err := s.leftHome.SetPosition(ctx, 2, nil); err != nil {
			return fmt.Errorf("set left home: %w", err)
		}
		s.logger.Debugf("set left home")
		return nil
	case GrabStepActionOpen:
		if err := s.gripper.Open(ctx, nil); err != nil {
			return fmt.Errorf("open gripper: %w", err)
		}
		s.logger.Debugf("opened gripper")
	case GrabStepActionClose:
		g, ctx := errgroup.WithContext(ctx)
		g.Go(func() error {
			if _, err := s.gripper.Grab(ctx, nil); err != nil {
				return fmt.Errorf("close gripper: %w", err)
			}
			return nil
		})
		// shake arm while closing the gripper to loosen food
		g.Go(func() error {
			if _, err := s.scoopShakeService.DoCommand(ctx, map[string]interface{}{"shake_arm": true}); err != nil {
				return fmt.Errorf("shake arm: %w", err)
			}
			return nil
		})
		err := g.Wait()
		if err != nil {
			return fmt.Errorf("%w", err)
		}
		s.logger.Debugf("closed gripper")
	}
	return nil
}

func (s *grabberControls) executeGrab(ctx context.Context, plan *GrabPlan) (retErr error) {
	var record *grabPlanRecord
	if s.cfg.SavePlans {
		record = &grabPlanRecord{
			StartedAt: plan.PlannedAt.UTC().Format(time.RFC3339Nano),
			BinName:   plan.BinName,
			ZoneID:    plan.ZoneID,
		}
		defer func() {
			record.Success = retErr == nil
			if retErr != nil {
				record.Error = retErr.Error()
			}
			if saveErr := s.savePlan(record); saveErr != nil {
				s.logger.Warnf("failed to save grab plan: %v", saveErr)
			}
		}()
	}

	for _, step := range plan.Steps {
		if err := s.executePrePostAction(ctx, step.PreAction); err != nil {
			return fmt.Errorf("execute pre action: %w", err)
		}

		armInputs := make([][]referenceframe.Input, len(step.Trajectory))
		for i, fsInputs := range step.Trajectory {
			armInputs[i] = fsInputs[s.cfg.Arm]
		}

		execErr := s.arm.MoveThroughJointPositions(ctx, armInputs, nil, nil)
		if record != nil {
			ps := grabPlanStep{
				Step:          step.Name,
				TrajectoryLen: len(step.Trajectory),
				PlanningDurMS: step.PlanningTime.Milliseconds(),
			}
			if execErr != nil {
				ps.ExecError = execErr.Error()
			}
			record.Steps = append(record.Steps, ps)
		}
		if execErr != nil {
			return fmt.Errorf("step %q: %w", step.Name, execErr)
		}
		s.logger.Debugf("completed step %q", step.Name)

		if err := s.executePrePostAction(ctx, step.PostAction); err != nil {
			return fmt.Errorf("execute post action: %w", err)
		}
	}
	return nil
}

// moveArm is used by doHover, which still goes through the motion service.
func (s *grabberControls) moveArm(ctx context.Context, dest spatialmath.Pose, constraints *motionplan.Constraints) error {
	pt := dest.Point()
	s.logger.Infof("moving arm to x=%.2f y=%.2f z=%.2f (linear=%v)", pt.X, pt.Y, pt.Z, constraints != nil)
	start := time.Now()
	_, err := s.motionService.Move(ctx, motion.MoveReq{
		ComponentName: s.arm.Name().ShortName(),
		Destination:   referenceframe.NewPoseInFrame(referenceframe.World, dest),
		WorldState:    s.worldState,
		Constraints:   constraints,
	})
	s.logger.Infof("motion planning took %.2fs", time.Since(start).Seconds())
	return err
}
