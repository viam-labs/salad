package salad

import (
	"context"
	"fmt"

	"go.viam.com/rdk/referenceframe"
)

func (s *dressingControls) executeDressing(ctx context.Context, plan *dressingPlan) error {
	if err := s.gripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open gripper: %w", err)
	}

	for _, step := range plan.steps {
		armInputs := make([][]referenceframe.Input, len(step.trajectory))
		for i, fsInputs := range step.trajectory {
			armInputs[i] = fsInputs[s.cfg.Arm]
		}

		if err := s.arm.MoveThroughJointPositions(ctx, armInputs, step.moveOptions, nil); err != nil {
			return fmt.Errorf("step %q: %w", step.name, err)
		}
		s.logger.Debugf("completed step %q", step.name)

		switch step.postAction {
		case GrabStepActionOpen:
			if err := s.gripper.Open(ctx, nil); err != nil {
				return fmt.Errorf("step %q: open gripper: %w", step.name, err)
			}
			s.logger.Debugf("opened gripper after %q", step.name)
		case GrabStepActionClose:
			if _, err := s.gripper.Grab(ctx, nil); err != nil {
				return fmt.Errorf("step %q: close gripper: %w", step.name, err)
			}
			s.logger.Debugf("closed gripper after %q", step.name)
		}

		if step.postShake && s.shakeArmService != nil {
			if _, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{"shake_arm": true}); err != nil {
				return fmt.Errorf("step %q: shake arm: %w", step.name, err)
			}
			s.logger.Debugf("shook arm after %q", step.name)
		}
	}
	return nil
}
