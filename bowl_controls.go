package salad

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/gripper"
	sw "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
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
	LilArmGripper      string             `json:"lil-arm-gripper"`
	LilArmHome         string             `json:"lil-arm-home"`
	LilArmPoses        []LilArmPoseConfig `json:"lil-arm-poses"`
}

func (cfg *BowlControlsConfig) Validate(path string) ([]string, []string, error) {
	if cfg.RightGripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-gripper")
	}

	if cfg.RightAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-above-bowl")
	}

	if cfg.RightAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-above-delivery")
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

	if cfg.LilArmGripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "lil-arm-gripper")
	}

	if cfg.LilArmHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "lil-arm-home")
	}

	if len(cfg.LilArmPoses) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "lil-arm-poses")
	}

	requiredDeps := []string{
		cfg.RightGripper,
		cfg.RightAboveBowl, cfg.RightGrabBowl, cfg.RightAboveDelivery, cfg.RightBowlDelivery, cfg.RightHome,
		cfg.LilArmGripper, cfg.LilArmHome,
	}

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
		requiredDeps = append(requiredDeps, pose.Above, pose.At, pose.CenterAbove, pose.CenterAt)
	}

	return requiredDeps, []string{}, nil
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

	lilArmGripper gripper.Gripper
	lilArmHome    sw.Switch
	lilArmPoses   map[string]*lilArmPoseSwitches
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
		return s.reset(ctx)
	}
	if _, ok := cmd["prepare_bowl"]; ok {
		return s.doPrepareBowl(ctx)
	}
	if _, ok := cmd["grab_lid"]; ok {
		return s.doGrabLid(ctx)
	}
	if _, ok := cmd["grab_bowl"]; ok {
		return s.doGrabBowl(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'deliver_bowl', 'prepare_bowl', 'grab_lid', 'grab_bowl', or 'reset' field")
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

	if err := s.rightGrabBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-grab-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-grab-bowl switch to position 2")

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

	if err := s.rightGripper.Open(ctx, nil); err != nil {
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

func (s *bowlControls) doGrabLid(ctx context.Context) (map[string]interface{}, error) {
	s.logger.Infof("Executing grab_lid")
	pose, ok := s.lilArmPoses["lid"]
	if !ok {
		return nil, fmt.Errorf("lil-arm pose 'lid' not found in configuration")
	}
	return s.doLilArmGrab(ctx, pose, "lid")
}

func (s *bowlControls) doGrabBowl(ctx context.Context) (map[string]interface{}, error) {
	s.logger.Infof("Executing grab_bowl")
	pose, ok := s.lilArmPoses["bowl"]
	if !ok {
		return nil, fmt.Errorf("lil-arm pose 'bowl' not found in configuration")
	}
	return s.doLilArmGrab(ctx, pose, "bowl")
}

func (s *bowlControls) doLilArmGrab(ctx context.Context, pose *lilArmPoseSwitches, name string) (map[string]interface{}, error) {
	if err := pose.abovePose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-%s switch to position 2: %w", name, err)
	}
	s.logger.Debugf("Set above-%s switch to position 2", name)

	if err := pose.atPose.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set at-%s switch to position 2: %w", name, err)
	}
	s.logger.Debugf("Set at-%s switch to position 2", name)

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

func (s *bowlControls) reset(ctx context.Context) (map[string]interface{}, error) {
	if err := s.rightHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-home switch to position 2")

	if err := s.lilArmHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set lil-arm home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set lil-arm home switch to position 2")

	return nil, nil
}

func (s *bowlControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
