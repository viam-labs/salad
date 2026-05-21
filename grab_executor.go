package salad

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

func (s *grabberControls) executeGrab(ctx context.Context, plan *GrabPlan) (retErr error) {
	var record *grabPlanRecord
	if s.cfg.SavePlans {
		record = &grabPlanRecord{
			StartedAt: time.Now().UTC().Format(time.RFC3339Nano),
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
		moveErr := s.moveArm(ctx, step.Pose, step.Constraints)
		if record != nil {
			record.Steps = append(record.Steps, poseToStep(step.Name, step.Pose, step.Constraints != nil, moveErr))
		}
		if moveErr != nil {
			return fmt.Errorf("step %q: %w", step.Name, moveErr)
		}
		s.logger.Debugf("completed step %q", step.Name)

		switch step.PostAction {
		case GrabStepActionOpen:
			if err := s.gripper.Open(ctx, nil); err != nil {
				return fmt.Errorf("step %q: open gripper: %w", step.Name, err)
			}
			s.logger.Debugf("opened gripper after %q", step.Name)
		case GrabStepActionClose:
			if _, err := s.gripper.Grab(ctx, nil); err != nil {
				return fmt.Errorf("step %q: close gripper: %w", step.Name, err)
			}
			s.logger.Debugf("closed gripper after %q", step.Name)
		}
	}
	return nil
}

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
