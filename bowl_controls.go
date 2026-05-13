package salad

import (
	"context"
	"fmt"
	"time"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	sw "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
)

var BowlControls = resource.NewModel("ncs", "salad", "bowl-controls")

func init() {
	resource.RegisterService(genericservice.API, BowlControls,
		resource.Registration[resource.Resource, *BowlControlsConfig]{
			Constructor: newBowlControls,
		},
	)
}

type LilArmPoseConfig struct {
	Name        string `json:"name"`
	Above       string `json:"above"`
	At          string `json:"at"`
	CenterAbove string `json:"center-above"`
	CenterAt    string `json:"center-at"`
}

type BowlControlsConfig struct {
	RightGripper       string             `json:"right-gripper"`
	RightAboveBowl     string             `json:"right-above-bowl"`
	RightGrabBowl      string             `json:"right-grab-bowl"`
	RightAboveDelivery string             `json:"right-above-delivery"`
	RightBowlDelivery  string             `json:"right-bowl-delivery"`
	RightHome          string             `json:"right-home"`
	LittleArm          string             `json:"little-arm"`
	LilArmGripper      string             `json:"lil-arm-gripper"`
	LilArmHome         string             `json:"lil-arm-home"`
	LilArmPoses        []LilArmPoseConfig `json:"lil-arm-poses"`
	XArmForceMover     string             `json:"xarm-force-mover,omitempty"`
}

func (cfg *BowlControlsConfig) Validate(path string) ([]string, []string, error) {
	if cfg.RightGripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-gripper")
	}

	if cfg.RightAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-above-bowl")
	}

	if cfg.RightGrabBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-grab-bowl")
	}

	if cfg.RightAboveDelivery == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-above-delivery")
	}

	if cfg.RightBowlDelivery == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-bowl-delivery")
	}

	if cfg.RightHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-home")
	}

	if cfg.LittleArm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "little-arm")
	}

	requiredDeps := []string{
		cfg.RightGripper,
		cfg.RightAboveBowl, cfg.RightGrabBowl, cfg.RightAboveDelivery, cfg.RightBowlDelivery, cfg.RightHome,
		cfg.LittleArm,
	}

	var optionalDeps []string

	if cfg.XArmForceMover != "" {
		optionalDeps = append(optionalDeps, cfg.XArmForceMover)
	}

	if cfg.LilArmGripper != "" {
		if cfg.LilArmHome == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "lil-arm-home")
		}
		optionalDeps = append(optionalDeps, cfg.LilArmGripper, cfg.LilArmHome)

		for i, pose := range cfg.LilArmPoses {
			if pose.Name == "" {
				return nil, nil, fmt.Errorf("%s.lil-arm-poses[%d]: 'name' field is required", path, i)
			}
			if pose.Above == "" {
				return nil, nil, fmt.Errorf("%s.lil-arm-poses[%d]: 'above' field is required", path, i)
			}
			if pose.At == "" {
				return nil, nil, fmt.Errorf("%s.lil-arm-poses[%d]: 'at' field is required", path, i)
			}
			if pose.CenterAbove == "" {
				return nil, nil, fmt.Errorf("%s.lil-arm-poses[%d]: 'center-above' field is required", path, i)
			}
			if pose.CenterAt == "" {
				return nil, nil, fmt.Errorf("%s.lil-arm-poses[%d]: 'center-at' field is required", path, i)
			}
			optionalDeps = append(optionalDeps, pose.Above, pose.At, pose.CenterAbove, pose.CenterAt)
		}
	}

	return requiredDeps, optionalDeps, nil
}

type lilArmPoseSwitches struct {
	abovePose       sw.Switch
	atPose          sw.Switch
	centerAbovePose sw.Switch
	centerAtPose    sw.Switch
}

type bowlControls struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *BowlControlsConfig

	cancelCtx  context.Context
	cancelFunc func()

	rightGripper       gripper.Gripper
	rightAboveBowl     sw.Switch
	rightGrabBowl      sw.Switch
	rightAboveDelivery sw.Switch
	rightBowlDelivery  sw.Switch
	rightHome          sw.Switch
	littleArm          arm.Arm

	lilArmGripper gripper.Gripper
	lilArmHome    sw.Switch
	lilArmPoses   map[string]*lilArmPoseSwitches

	xarmForceMover resource.Resource
}

func newBowlControls(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*BowlControlsConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewBowlControls(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewBowlControls(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *BowlControlsConfig, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &bowlControls{
		name:        name,
		logger:      logger,
		cfg:         conf,
		cancelCtx:   cancelCtx,
		cancelFunc:  cancelFunc,
		lilArmPoses: make(map[string]*lilArmPoseSwitches),
	}

	rightGripperComponent, err := gripper.FromProvider(deps, conf.RightGripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get right gripper '%s': %w", conf.RightGripper, err)
	}
	s.rightGripper = rightGripperComponent

	rightAboveBowlSwitch, err := sw.FromProvider(deps, conf.RightAboveBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-above-bowl switch '%s': %w", conf.RightAboveBowl, err)
	}
	s.rightAboveBowl = rightAboveBowlSwitch

	rightGrabBowlSwitch, err := sw.FromProvider(deps, conf.RightGrabBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-grab-bowl switch '%s': %w", conf.RightGrabBowl, err)
	}
	s.rightGrabBowl = rightGrabBowlSwitch

	rightAboveDeliverySwitch, err := sw.FromProvider(deps, conf.RightAboveDelivery)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-above-delivery switch '%s': %w", conf.RightAboveDelivery, err)
	}
	s.rightAboveDelivery = rightAboveDeliverySwitch

	rightBowlDeliverySwitch, err := sw.FromProvider(deps, conf.RightBowlDelivery)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-bowl-delivery switch '%s': %w", conf.RightBowlDelivery, err)
	}
	s.rightBowlDelivery = rightBowlDeliverySwitch

	rightHomeSwitch, err := sw.FromProvider(deps, conf.RightHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-home switch '%s': %w", conf.RightHome, err)
	}
	s.rightHome = rightHomeSwitch

	littleArmComponent, err := arm.FromProvider(deps, conf.LittleArm)
	if err != nil {
		return nil, fmt.Errorf("failed to get little-arm '%s': %w", conf.LittleArm, err)
	}
	s.littleArm = littleArmComponent

	if conf.XArmForceMover != "" {
		mover, err := genericservice.FromProvider(deps, conf.XArmForceMover)
		if err != nil {
			return nil, fmt.Errorf("failed to get xarm-force-mover '%s': %w", conf.XArmForceMover, err)
		}
		s.xarmForceMover = mover
	}

	if conf.LilArmGripper != "" {
		lilArmGripperComponent, err := gripper.FromProvider(deps, conf.LilArmGripper)
		if err != nil {
			return nil, fmt.Errorf("failed to get lil-arm gripper '%s': %w", conf.LilArmGripper, err)
		}
		s.lilArmGripper = lilArmGripperComponent

		lilArmHomeSwitch, err := sw.FromProvider(deps, conf.LilArmHome)
		if err != nil {
			return nil, fmt.Errorf("failed to get lil-arm home switch '%s': %w", conf.LilArmHome, err)
		}
		s.lilArmHome = lilArmHomeSwitch

		for _, poseCfg := range conf.LilArmPoses {
			aboveSwitch, err := sw.FromProvider(deps, poseCfg.Above)
			if err != nil {
				return nil, fmt.Errorf("failed to get above switch '%s' for lil-arm pose '%s': %w", poseCfg.Above, poseCfg.Name, err)
			}
			atSwitch, err := sw.FromProvider(deps, poseCfg.At)
			if err != nil {
				return nil, fmt.Errorf("failed to get at switch '%s' for lil-arm pose '%s': %w", poseCfg.At, poseCfg.Name, err)
			}
			centerAboveSwitch, err := sw.FromProvider(deps, poseCfg.CenterAbove)
			if err != nil {
				return nil, fmt.Errorf("failed to get center-above switch '%s' for lil-arm pose '%s': %w", poseCfg.CenterAbove, poseCfg.Name, err)
			}
			centerAtSwitch, err := sw.FromProvider(deps, poseCfg.CenterAt)
			if err != nil {
				return nil, fmt.Errorf("failed to get center-at switch '%s' for lil-arm pose '%s': %w", poseCfg.CenterAt, poseCfg.Name, err)
			}
			s.lilArmPoses[poseCfg.Name] = &lilArmPoseSwitches{
				abovePose:       aboveSwitch,
				atPose:          atSwitch,
				centerAbovePose: centerAboveSwitch,
				centerAtPose:    centerAtSwitch,
			}
		}
	}

	s.logger.Infof("Bowl controls initialized")
	return s, nil
}

func (s *bowlControls) Name() resource.Name {
	return s.name
}

func (s *bowlControls) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["deliver_bowl"]; ok {
		return s.doDeliverBowl(ctx)
	}
	if _, ok := cmd["reset"]; ok {
		skipLilArm, _ := cmd["skip_lil_arm"].(bool)
		return s.reset(ctx, skipLilArm)
	}
	if _, ok := cmd["prepare_bowl"]; ok {
		return s.doPrepareBowl(ctx)
	}
	if _, ok := cmd["grab_lid"]; ok {
		return s.doGrabLid(ctx, cmd)
	}
	if _, ok := cmd["grab_bowl"]; ok {
		return s.doGrabBowl(ctx, cmd)
	}
	if _, ok := cmd["lil_arm_home"]; ok {
		return s.doLilArmHome(ctx)
	}
	if _, ok := cmd["move_down_to_bowl"]; ok {
		if err := s.moveDownTo(ctx, "bowl"); err != nil {
			return nil, err
		}
		return map[string]interface{}{"success": true}, nil
	}
	if _, ok := cmd["move_down_to_lid"]; ok {
		if err := s.moveDownTo(ctx, "lid"); err != nil {
			return nil, err
		}
		return map[string]interface{}{"success": true}, nil
	}
	if _, ok := cmd["force_move"]; ok {
		return s.doForceMove(ctx, cmd)
	}
	if _, ok := cmd["use_tool"]; ok {
		return s.doUseTool(ctx, cmd)
	}
	if _, ok := cmd["grab_and_use_tool"]; ok {
		return s.doGrabAndUseTool(ctx, cmd)
	}
	return nil, fmt.Errorf("unknown command, expected 'deliver_bowl', 'prepare_bowl', 'grab_lid', 'grab_bowl', 'move_down_to_bowl', 'move_down_to_lid', 'force_move', 'use_tool', 'grab_and_use_tool', or 'reset' field")
}

// doForceMove forwards a call to the configured xarm-force-mover service.
// Expected cmd fields: "joint" (number), "axis" (string), "target" (number).
func (s *bowlControls) doForceMove(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if s.xarmForceMover == nil {
		return nil, fmt.Errorf("xarm-force-mover is not configured")
	}
	for _, k := range []string{"joint", "axis", "target"} {
		if _, ok := cmd[k]; !ok {
			return nil, fmt.Errorf("force_move requires field %q", k)
		}
	}
	args := map[string]interface{}{
		"joint":  cmd["joint"],
		"axis":   cmd["axis"],
		"target": cmd["target"],
	}
	resp, err := s.xarmForceMover.DoCommand(ctx, args)
	if err != nil {
		return nil, fmt.Errorf("xarm-force-mover failed: %w", err)
	}
	return resp, nil
}

func (s *bowlControls) moveDownTo(ctx context.Context, name string) error {
	pose, ok := s.lilArmPoses[name]
	if !ok {
		return fmt.Errorf("lil-arm pose '%s' not found in configuration", name)
	}
	if err := pose.atPose.SetPosition(ctx, 2, nil); err != nil {
		return fmt.Errorf("failed to set %s at-pose switch: %w", name, err)
	}

	var lastVal *float64
	for {
		if err := ctx.Err(); err != nil {
			return err
		}

		// Move down 2mm
		currentPose, err := s.littleArm.EndPosition(ctx, nil)
		if err != nil {
			return fmt.Errorf("failed to get arm position: %w", err)
		}
		point := currentPose.Point()
		point.Z -= 2
		newPose := spatialmath.NewPose(point, currentPose.Orientation())
		if err := s.littleArm.MoveToPosition(ctx, newPose, nil); err != nil {
			return fmt.Errorf("failed to move arm down: %w", err)
		}

		// Check load
		resp, err := s.littleArm.DoCommand(ctx, map[string]interface{}{"load": true})
		if err != nil {
			return fmt.Errorf("failed to send load command: %w", err)
		}
		loadRaw, ok := resp["load"]
		if !ok {
			return fmt.Errorf("load command response missing 'load' key")
		}
		load, ok := loadRaw.([]interface{})
		if !ok {
			return fmt.Errorf("load response is not an array")
		}
		if len(load) < 2 {
			return fmt.Errorf("expected at least 2 values in load response, got %d", len(load))
		}
		val, ok := load[1].(float64)
		if !ok {
			return fmt.Errorf("load[1] is not a float64")
		}
		if lastVal != nil && (val < 0) != (*lastVal < 0) {
			s.logger.Infof("Contact detected: sign flip from %f to %f at Z = %f", *lastVal, val, currentPose.Point().Z-2)
			// Contact detected, stop the arm
			if err := s.littleArm.Stop(ctx, nil); err != nil {
				return fmt.Errorf("failed to stop arm: %w", err)
			}
			return nil
		}
		lastVal = &val
	}
}

func (s *bowlControls) doPrepareBowl(ctx context.Context) (map[string]interface{}, error) {
	s.logger.Infof("Executing prepare_bowl")

	// open gripper
	if err := s.rightGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open right gripper: %w", err)
	}
	s.logger.Debugf("Opened right gripper")

	if err := s.rightAboveDelivery.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-above-delivery switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-above-delivery switch to position 2")

	if err := s.rightBowlDelivery.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-bowl-delivery switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-bowl-delivery switch to position 2")

	if _, err := s.rightGripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to close right gripper: %w", err)
	}
	s.logger.Debugf("Closed right gripper")

	if err := s.rightAboveDelivery.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-above-delivery switch to position 2 (second time): %w", err)
	}
	s.logger.Debugf("Set right-above-delivery switch to position 2 (second time)")

	if err := s.rightAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-above-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-above-bowl switch to position 2")

	if err := s.moveDownTo(ctx, "bowl"); err != nil {
		return nil, fmt.Errorf("failed to move down to bowl: %w", err)
	}
	s.logger.Debugf("Moved down to bowl")

	if err := s.rightGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open right gripper: %w", err)
	}
	s.logger.Debugf("Opened right gripper")

	if err := s.rightAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-above-bowl switch to position 2 (second time): %w", err)
	}
	s.logger.Debugf("Set right-above-bowl switch to position 2 (second time)")

	s.logger.Infof("Successfully completed prepare_bowl")

	return map[string]interface{}{
		"success": true,
		"message": "Successfully prepared bowl",
	}, nil
}

func (s *bowlControls) doDeliverBowl(ctx context.Context) (map[string]interface{}, error) {
	s.logger.Infof("Executing deliver_bowl")

	// open gripper
	if err := s.rightGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open right gripper: %w", err)
	}
	s.logger.Debugf("Opened right gripper")

	if err := s.rightAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-above-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-above-bowl switch to position 2")

	if err := s.rightGrabBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-grab-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-grab-bowl switch to position 2")

	if _, err := s.rightGripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to close right gripper: %w", err)
	}
	s.logger.Debugf("Closed right gripper")

	time.Sleep(1000 * time.Millisecond)

	if err := s.rightAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-above-bowl switch to position 2 (second time): %w", err)
	}
	s.logger.Debugf("Set right-above-bowl switch to position 2 (second time)")

	if err := s.rightAboveDelivery.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-above-delivery switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-above-delivery switch to position 2")

	if err := s.rightBowlDelivery.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-bowl-delivery switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-bowl-delivery switch to position 2")

	if _, err := s.rightGripper.DoCommand(ctx, map[string]interface{}{"set_position": 400}); err != nil {
		return nil, fmt.Errorf("failed to open right gripper: %w", err)
	}

	s.logger.Debugf("Opened right gripper")

	s.logger.Debugf("Set right-home switch to position 2")

	s.logger.Infof("Successfully completed deliver_bowl")

	return map[string]interface{}{
		"success": true,
		"message": "Successfully delivered bowl",
	}, nil
}

func (s *bowlControls) doGrabLid(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.logger.Infof("Executing grab_lid")
	if s.lilArmGripper == nil {
		return nil, fmt.Errorf("lil-arm is not configured")
	}
	pose, ok := s.lilArmPoses["lid"]
	if !ok {
		return nil, fmt.Errorf("lil-arm pose 'lid' not found in configuration")
	}
	target, err := extractTarget(cmd, "grab_lid")
	if err != nil {
		return nil, err
	}
	return s.doLilArmGrab(ctx, pose, "lid", float64(1), "z", target)
}

func (s *bowlControls) doGrabBowl(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.logger.Infof("Executing grab_bowl")
	if s.lilArmGripper == nil {
		return nil, fmt.Errorf("lil-arm is not configured")
	}
	pose, ok := s.lilArmPoses["bowl"]
	if !ok {
		return nil, fmt.Errorf("lil-arm pose 'bowl' not found in configuration")
	}
	target, err := extractTarget(cmd, "grab_bowl")
	if err != nil {
		return nil, err
	}
	return s.doLilArmGrab(ctx, pose, "bowl", float64(1), "z", target)
}

// extractTarget reads the "target" field from cmd, required for grab_lid/grab_bowl
// when using the xarm-force-mover for the descent.
func extractTarget(cmd map[string]interface{}, op string) (float64, error) {
	raw, ok := cmd["target"]
	if !ok {
		return 0, fmt.Errorf("%s requires 'target' field (number, e.g. {%q: true, \"target\": 50})", op, op)
	}
	switch v := raw.(type) {
	case float64:
		return v, nil
	case int:
		return float64(v), nil
	default:
		return 0, fmt.Errorf("%s 'target' must be a number, got %T", op, raw)
	}
}

func (s *bowlControls) doLilArmGrab(ctx context.Context, pose *lilArmPoseSwitches, name string, joint interface{}, axis interface{}, target float64) (map[string]interface{}, error) {
	if err := s.lilArmGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open lil-arm gripper: %w", err)
	}
	s.logger.Debugf("Opened lil-arm gripper")

	if err := pose.abovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-%s switch to position 2: %w", name, err)
	}
	s.logger.Debugf("Set above-%s switch to position 2", name)

	// if err := s.moveDownTo(ctx, name); err != nil {
	// 	return nil, fmt.Errorf("failed to move down to %s: %w", name, err)
	// }
	if s.xarmForceMover == nil {
		return nil, fmt.Errorf("xarm-force-mover is not configured")
	}
	if _, err := s.xarmForceMover.DoCommand(ctx, map[string]interface{}{
		"joint":  joint,
		"axis":   axis,
		"target": target,
	}); err != nil {
		return nil, fmt.Errorf("force descent to %s failed: %w", name, err)
	}
	s.logger.Debugf("Force-descended to %s (target=%f)", name, target)

	if _, err := s.lilArmGripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to grab %s: %w", name, err)
	}
	s.logger.Debugf("Grabbed %s", name)

	if err := pose.abovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-%s switch to position 2 (second time): %w", name, err)
	}
	s.logger.Debugf("Set above-%s switch to position 2 (second time)", name)

	if err := pose.centerAbovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set center-above-%s switch to position 2: %w", name, err)
	}
	s.logger.Debugf("Set center-above-%s switch to position 2", name)

	if err := pose.centerAtPose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set center-at-%s switch to position 2: %w", name, err)
	}
	s.logger.Debugf("Set center-at-%s switch to position 2", name)

	if err := s.lilArmGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open lil-arm gripper: %w", err)
	}
	s.logger.Debugf("Opened lil-arm gripper")

	if err := pose.centerAbovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set center-above-%s switch to position 2: %w", name, err)
	}
	s.logger.Debugf("Set center-above-%s switch to position 2", name)

	s.logger.Infof("Successfully completed grab_%s", name)

	return map[string]interface{}{
		"success": true,
		"message": fmt.Sprintf("Successfully grabbed %s", name),
	}, nil
}

func (s *bowlControls) doUseTool(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.logger.Infof("Executing use_tool")
	if s.lilArmGripper == nil {
		return nil, fmt.Errorf("lil-arm is not configured")
	}
	toolPose, ok := s.lilArmPoses["tool"]
	if !ok {
		return nil, fmt.Errorf("lil-arm pose 'tool' not found in configuration")
	}

	if err := s.lilArmGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open lil-arm gripper: %w", err)
	}

	// Move above the tool, then to the tool.
	if err := toolPose.abovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-tool switch: %w", err)
	}
	if err := toolPose.atPose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set at-tool switch: %w", err)
	}

	// Pick up the tool.
	if _, err := s.lilArmGripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to grab tool: %w", err)
	}

	// Move above the tool, then above the scale.
	if err := toolPose.abovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-tool switch (return): %w", err)
	}
	if err := toolPose.centerAbovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set center-above (above-scale) switch: %w", err)
	}

	// Apply force to secure the lid.
	if _, err := s.doForceMove(ctx, cmd); err != nil {
		return nil, fmt.Errorf("force_move failed: %w", err)
	}

	// Move back to the scale.
	if err := toolPose.centerAtPose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set center-at (on-scale) switch: %w", err)
	}

	// Move back to where we picked up the tool.
	if err := toolPose.centerAbovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set center-above (above-scale) switch (return): %w", err)
	}
	if err := toolPose.abovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-tool switch (return to tool): %w", err)
	}
	if err := toolPose.atPose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set at-tool switch (return to tool): %w", err)
	}

	// Drop the tool.
	if err := s.lilArmGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open lil-arm gripper: %w", err)
	}

	// Return to home.
	if err := toolPose.abovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-tool switch (after drop): %w", err)
	}
	if s.lilArmHome != nil {
		if err := s.lilArmHome.SetPosition(ctx, 2, nil); err != nil {
			return nil, fmt.Errorf("failed to set lil-arm home switch: %w", err)
		}
	}

	s.logger.Infof("Successfully completed use_tool")
	return map[string]interface{}{
		"success": true,
		"message": "Successfully used tool",
	}, nil
}

// doGrabAndUseTool runs grab_bowl, grab_lid, and use_tool back-to-back, sharing
// a single joint and axis across all three force-driven descents. Expected cmd
// fields: "joint" (number), "axis" (string), "targets" (array of 3 numbers in
// the order [bowl, lid, tool]).
func (s *bowlControls) doGrabAndUseTool(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	s.logger.Infof("Executing grab_and_use_tool")

	joint, ok := cmd["joint"]
	if !ok {
		return nil, fmt.Errorf("grab_and_use_tool requires 'joint' field")
	}
	axis, ok := cmd["axis"]
	if !ok {
		return nil, fmt.Errorf("grab_and_use_tool requires 'axis' field")
	}
	targetsRaw, ok := cmd["targets"]
	if !ok {
		return nil, fmt.Errorf("grab_and_use_tool requires 'targets' field (array of 3 numbers: [bowl, lid, tool])")
	}
	targets, ok := targetsRaw.([]interface{})
	if !ok {
		return nil, fmt.Errorf("grab_and_use_tool 'targets' must be an array, got %T", targetsRaw)
	}
	if len(targets) != 3 {
		return nil, fmt.Errorf("grab_and_use_tool 'targets' must have exactly 3 entries, got %d", len(targets))
	}
	bowlTarget, ok := targets[0].(float64)
	if !ok {
		return nil, fmt.Errorf("grab_and_use_tool targets[0] (bowl) must be a number, got %T", targets[0])
	}
	lidTarget, ok := targets[1].(float64)
	if !ok {
		return nil, fmt.Errorf("grab_and_use_tool targets[1] (lid) must be a number, got %T", targets[1])
	}
	toolTarget, ok := targets[2].(float64)
	if !ok {
		return nil, fmt.Errorf("grab_and_use_tool targets[2] (tool) must be a number, got %T", targets[2])
	}

	if s.lilArmGripper == nil {
		return nil, fmt.Errorf("lil-arm is not configured")
	}

	bowlPose, ok := s.lilArmPoses["bowl"]
	if !ok {
		return nil, fmt.Errorf("lil-arm pose 'bowl' not found in configuration")
	}
	if _, err := s.doLilArmGrab(ctx, bowlPose, "bowl", joint, axis, bowlTarget); err != nil {
		return nil, fmt.Errorf("grab_bowl failed: %w", err)
	}

	lidPose, ok := s.lilArmPoses["lid"]
	if !ok {
		return nil, fmt.Errorf("lil-arm pose 'lid' not found in configuration")
	}
	if _, err := s.doLilArmGrab(ctx, lidPose, "lid", joint, axis, lidTarget); err != nil {
		return nil, fmt.Errorf("grab_lid failed: %w", err)
	}

	if _, err := s.doUseTool(ctx, map[string]interface{}{
		"joint":  joint,
		"axis":   axis,
		"target": toolTarget,
	}); err != nil {
		return nil, fmt.Errorf("use_tool failed: %w", err)
	}

	s.logger.Infof("Successfully completed grab_and_use_tool")
	return map[string]interface{}{
		"success": true,
		"message": "Successfully grabbed bowl, grabbed lid, and used tool",
	}, nil
}

func (s *bowlControls) reset(ctx context.Context, skipLilArm bool) (map[string]interface{}, error) {
	if err := s.rightHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-home switch to position 2")

	if s.lilArmHome != nil {
		if skipLilArm {
			s.logger.Debugf("skip_lil_arm set; not moving lil-arm home")
		} else {
			if err := s.lilArmHome.SetPosition(ctx, 2, nil); err != nil {
				return nil, fmt.Errorf("failed to set lil-arm home switch to position 2: %w", err)
			}
			s.logger.Debugf("Set lil-arm home switch to position 2")
		}
	}

	return nil, nil
}

func (s *bowlControls) doLilArmHome(ctx context.Context) (map[string]interface{}, error) {
	if s.lilArmHome == nil {
		return nil, fmt.Errorf("lil-arm-home is not configured")
	}
	s.logger.Infof("Executing lil_arm_home")
	if err := s.lilArmHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set lil-arm home switch to position 2: %w", err)
	}
	return map[string]interface{}{
		"success": true,
		"message": "Sent lil-arm home",
	}, nil
}

func (s *bowlControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
