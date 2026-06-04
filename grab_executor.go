package salad

import (
	"context"
	"fmt"

	"go.viam.com/rdk/referenceframe"
	"golang.org/x/sync/errgroup"
)

func (s *grabberControls) executePrePostAction(ctx context.Context, action GrabStepAction) error {
	switch action {
	case GrabStepActionNone:
		return nil
	case GrabStepActionShake:
		if s.shakeArmService != nil {
			if _, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{"shake_arm": true}); err != nil {
				return fmt.Errorf("shake arm: %w", err)
			}
			s.logger.Debugf("shook arm")
			return nil
		}
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

func (s *grabberControls) executeGrab(ctx context.Context, plan *GrabPlan) error {
	for _, step := range plan.Steps {
		if err := s.executePrePostAction(ctx, step.PreAction); err != nil {
			return fmt.Errorf("execute pre action: %w", err)
		}

		armInputs := make([][]referenceframe.Input, len(step.Trajectory))
		for i, fsInputs := range step.Trajectory {
			armInputs[i] = fsInputs[s.cfg.Arm]
		}

		if err := s.arm.MoveThroughJointPositions(ctx, armInputs, nil, nil); err != nil {
			return fmt.Errorf("step %q: %w", step.Name, err)
		}
		s.logger.Debugf("completed step %q", step.Name)

		if err := s.executePrePostAction(ctx, step.PostAction); err != nil {
			return fmt.Errorf("execute post action: %w", err)
		}
	}
	return nil
}
