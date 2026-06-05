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
		squeezeDuringCircularMove := step.name == "circular_pour" && step.postSqueeze
		squeezeStarted := false
		for rev := range revolutions {
			if squeezeDuringCircularMove && !squeezeStarted {
				squeezeStarted = true
				squeezeErrCh := make(chan error, 1)
				go func() {
					squeezeErrCh <- s.runPostSqueeze(ctx, step.name)
				}()

				moveErr := s.arm.MoveThroughJointPositions(ctx, armInputs, step.moveOptions, nil)
				squeezeErr := <-squeezeErrCh
				if moveErr != nil {
					s.saveFailedExecutionJointPosition(plan.buildID)
					return fmt.Errorf("step %q rev %d: %w", step.name, rev+1, moveErr)
				}
				if squeezeErr != nil {
					return squeezeErr
				}
				continue
			}

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

		if step.postSqueeze && !squeezeDuringCircularMove {
			if err := s.runPostSqueeze(ctx, step.name); err != nil {
				return err
			}
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

func (s *dressingControls) runPostSqueeze(ctx context.Context, stepName string) error {
	for _, pos := range []float64{10.0, 5.0, 2.0} {
		if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
			"grab_with_torque": map[string]interface{}{
				"position": pos,
				"speed":    3000.0,
				"torque":   100.0,
			},
		}); err != nil {
			return fmt.Errorf("step %q: squeeze to %v: %w", stepName, pos, err)
		}
		s.logger.Debugf("squeezed to position %v after %q", pos, stepName)
		time.Sleep(700 * time.Millisecond)
	}

	resp, err := s.gripper.DoCommand(ctx, map[string]interface{}{"get": true})
	if err != nil {
		return fmt.Errorf("step %q: get gripper position: %w", stepName, err)
	}
	rawPos, ok := resp["pos"]
	if !ok {
		return fmt.Errorf("step %q: gripper get response missing \"pos\": %v", stepName, resp)
	}
	curPos, ok := rawPos.(float64)
	if !ok {
		return fmt.Errorf("step %q: gripper get \"pos\" not a number: %T", stepName, rawPos)
	}
	releasePos := curPos + 20
	if _, err := s.gripper.DoCommand(ctx, map[string]interface{}{
		"grab_with_torque": map[string]interface{}{
			"position": releasePos,
			"speed":    3000.0,
			"torque":   0,
		},
	}); err != nil {
		return fmt.Errorf("step %q: release squeeze: %w", stepName, err)
	}
	s.logger.Debugf("released gripper from %v to %v after %q", curPos, releasePos, stepName)
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
