package salad

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/referenceframe"

	"salad/lib/fileio"
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

		revolutions := step.revolutions
		if revolutions < 1 {
			revolutions = 1
		}
		for rev := range revolutions {
			if err := s.arm.MoveThroughJointPositions(ctx, armInputs, step.moveOptions, nil); err != nil {
				s.saveFailedExecutionJointPosition(plan.buildID)
				return fmt.Errorf("step %q rev %d: %w", step.name, rev+1, err)
			}
		}
		s.logger.Debugf("completed step %q", step.name)

		switch step.postAction {
		case GrabStepActionNone, GrabStepActionGoHome, GrabStepActionShake:
			// dressing only handles open/close post-actions
		// Open with grab_with_torque (not gripper.Open) because once we've grabbed with
		// torque, we need to apply some torque to actively push the jaws open.
		case GrabStepActionOpen:
			if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
				"grab_with_torque": map[string]interface{}{
					"position": 850.0,
					"speed":    3000.0,
					"torque":   100,
				},
			}); err != nil {
				return fmt.Errorf("step %q: open gripper: %w", step.name, err)
			}
			s.logger.Debugf("opened gripper after %q", step.name)
		case GrabStepActionClose:
			if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
				"grab_with_torque": map[string]interface{}{
					"position": 20.0,
					"speed":    3000.0,
					"torque":   0,
				},
			}); err != nil {
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

func (s *dressingControls) saveFailedExecutionJointPosition(buildID string) {
	if buildID == "" {
		return
	}
	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()
	inputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		s.logger.Warnw("could not read arm inputs at execution failure", "err", err)
		return
	}
	if err := fileio.SaveJsonToSync(inputs, "failed_execution_joint_position.json", buildID, time.Now()); err != nil {
		s.logger.Warnw("could not save failed execution joint position", "err", err)
	}
}
