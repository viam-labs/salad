package salad

import (
	"context"
	"fmt"
	"time"

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

		revolutions := step.revolutions
		if revolutions < 1 {
			revolutions = 1
		}
		for rev := range revolutions {
			if err := s.arm.MoveThroughJointPositions(ctx, armInputs, step.moveOptions, nil); err != nil {
				return fmt.Errorf("step %q rev %d: %w", step.name, rev+1, err)
			}
		}
		s.logger.Debugf("completed step %q", step.name)

		switch step.postAction {
		case GrabStepActionOpen:
			if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
				"grab_with_torque": map[string]interface{}{
					"position": 850.0,
					"speed":    3000.0,
					"torque":   0,
				},
			}); err != nil {
				return fmt.Errorf("step %q: open gripper: %w", step.name, err)
			}
			s.logger.Debugf("opened gripper after %q", step.name)
		case GrabStepActionClose:
			if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
				"grab_with_torque": map[string]interface{}{
					"position": 20.0,
					"speed":    2000.0,
					"torque":   0,
				},
			}); err != nil {
				return fmt.Errorf("step %q: close gripper: %w", step.name, err)
			}
			s.logger.Debugf("closed gripper after %q", step.name)
		}

		if step.postSqueeze {
			for _, pos := range []float64{10.0, 5.0, 2.0} {
				if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
					"grab_with_torque": map[string]interface{}{
						"position": pos,
						"speed":    2000.0,
						"torque":   100.0,
					},
				}); err != nil {
					return fmt.Errorf("step %q: squeeze to %v: %w", step.name, pos, err)
				}
				s.logger.Debugf("squeezed to position %v after %q", pos, step.name)
				time.Sleep(700 * time.Millisecond)
			}
			resp, err := s.gripper.DoCommand(ctx, map[string]interface{}{"get": true})
			if err != nil {
				return fmt.Errorf("step %q: get gripper position: %w", step.name, err)
			}
			rawPos, ok := resp["pos"]
			if !ok {
				return fmt.Errorf("step %q: gripper get response missing \"pos\": %v", step.name, resp)
			}
			curPos, ok := rawPos.(float64)
			if !ok {
				return fmt.Errorf("step %q: gripper get \"pos\" not a number: %T", step.name, rawPos)
			}
			releasePos := curPos + 20
			if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
				"grab_with_torque": map[string]interface{}{
					"position": releasePos,
					"speed":    2000.0,
					"torque":   0,
				},
			}); err != nil {
				return fmt.Errorf("step %q: release squeeze: %w", step.name, err)
			}
			s.logger.Debugf("released gripper from %v to %v after %q", curPos, releasePos, step.name)
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
