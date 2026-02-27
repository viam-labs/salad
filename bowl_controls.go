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

type BowlControlsConfig struct {
	RightGripper       string `json:"right-gripper"`
	RightAboveBowl     string `json:"right-above-bowl"`
	RightGrabBowl      string `json:"right-grab-bowl"`
	RightAboveDelivery string `json:"right-above-delivery"`
	RightBowlDelivery  string `json:"right-bowl-delivery"`
	RightHome          string `json:"right-home"`
}

func (cfg *BowlControlsConfig) Validate(path string) ([]string, []string, error) {
	if cfg.RightGripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.RightGripper)
	}

	if cfg.RightAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.RightAboveBowl)
	}

	if cfg.RightAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.RightAboveBowl)
	}

	if cfg.RightGrabBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.RightGrabBowl)
	}

	if cfg.RightAboveDelivery == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.RightAboveDelivery)
	}

	if cfg.RightBowlDelivery == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.RightBowlDelivery)
	}

	if cfg.RightHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.RightHome)
	}

	requiredDeps := []string{}

	requiredDeps = append(requiredDeps, cfg.RightGripper)
	requiredDeps = append(requiredDeps, cfg.RightAboveBowl, cfg.RightGrabBowl, cfg.RightAboveDelivery, cfg.RightBowlDelivery, cfg.RightHome)

	return requiredDeps, []string{}, nil
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
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
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
	return nil, fmt.Errorf("unknown command, expected 'deliver_bowl', 'prepare_bowl', or 'reset' field")
}

func (s *bowlControls) doPrepareBowl(ctx context.Context) (map[string]interface{}, error) {
	s.logger.Infof("Executing prepare_bowl")

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

func (s *bowlControls) reset(ctx context.Context) (map[string]interface{}, error) {
	if err := s.rightHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-home switch to position 2")

	return nil, nil
}

func (s *bowlControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
