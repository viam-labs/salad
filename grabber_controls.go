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

var GrabberControls = resource.NewModel("ncs", "salad", "grabber-controls")

func init() {
	resource.RegisterService(genericservice.API, GrabberControls,
		resource.Registration[resource.Resource, *GrabberControlsConfig]{
			Constructor: newGrabberControls,
		},
	)
}

// GrabberControlsBinConfig represents a single bin configuration with switches.
type GrabberControlsBinConfig struct {
	Name     string `json:"name"`
	AboveBin string `json:"above-bin"`
	InBin    string `json:"in-bin"`
}

type GrabberControlsConfig struct {
	Bins               []GrabberControlsBinConfig `json:"bins"`
	HighAboveBowl      string                     `json:"high-above-bowl"`
	LeftGripper        string                     `json:"left-gripper"`
	LeftHome           string                     `json:"left-home"`
	RightGripper       string                     `json:"right-gripper"`
	RightAboveBowl     string                     `json:"right-above-bowl"`
	RightGrabBowl      string                     `json:"right-grab-bowl"`
	RightAboveDelivery string                     `json:"right-above-delivery"`
	RightBowlDelivery  string                     `json:"right-bowl-delivery"`
	RightHome          string                     `json:"right-home"`
}

func (cfg *GrabberControlsConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.Bins) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bins")
	}

	if cfg.HighAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "high-above-bowl")
	}

	if cfg.LeftGripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-gripper")
	}

	if cfg.LeftHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-home")
	}

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

	requiredDeps := []string{}

	requiredDeps = append(requiredDeps, cfg.HighAboveBowl)
	requiredDeps = append(requiredDeps, cfg.LeftGripper)
	requiredDeps = append(requiredDeps, cfg.LeftHome)
	requiredDeps = append(requiredDeps, cfg.RightGripper)
	requiredDeps = append(requiredDeps, cfg.RightAboveBowl, cfg.RightGrabBowl, cfg.RightAboveDelivery, cfg.RightBowlDelivery, cfg.RightHome)

	for i, bin := range cfg.Bins {
		if bin.Name == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'name' field is required", path, i)
		}
		if bin.AboveBin == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'above-bin' field is required", path, i)
		}
		if bin.InBin == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'in-bin' field is required", path, i)
		}

		requiredDeps = append(requiredDeps, bin.AboveBin, bin.InBin)
	}

	return requiredDeps, []string{}, nil
}

type grabberControls struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *GrabberControlsConfig

	cancelCtx  context.Context
	cancelFunc func()

	bins               map[string]*grabberBinSwitches
	highAboveBowl      sw.Switch
	leftGripper        gripper.Gripper
	leftHome           sw.Switch
	rightGripper       gripper.Gripper
	rightAboveBowl     sw.Switch
	rightGrabBowl      sw.Switch
	rightAboveDelivery sw.Switch
	rightBowlDelivery  sw.Switch
	rightHome          sw.Switch
}

type grabberBinSwitches struct {
	aboveBin sw.Switch
	inBin    sw.Switch
}

func newGrabberControls(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*GrabberControlsConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewGrabberControls(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewGrabberControls(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *GrabberControlsConfig, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &grabberControls{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
		bins:       make(map[string]*grabberBinSwitches),
	}

	highAboveBowlSwitch, err := sw.FromDependencies(deps, conf.HighAboveBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get high-above-bowl switch '%s': %w", conf.HighAboveBowl, err)
	}
	s.highAboveBowl = highAboveBowlSwitch

	leftGripperComponent, err := gripper.FromDependencies(deps, conf.LeftGripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get left gripper '%s': %w", conf.LeftGripper, err)
	}
	s.leftGripper = leftGripperComponent

	leftHomeSwitch, err := sw.FromDependencies(deps, conf.LeftHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get left-home switch '%s': %w", conf.LeftHome, err)
	}
	s.leftHome = leftHomeSwitch

	rightGripperComponent, err := gripper.FromDependencies(deps, conf.RightGripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get right gripper '%s': %w", conf.RightGripper, err)
	}
	s.rightGripper = rightGripperComponent

	rightAboveBowlSwitch, err := sw.FromDependencies(deps, conf.RightAboveBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-above-bowl switch '%s': %w", conf.RightAboveBowl, err)
	}
	s.rightAboveBowl = rightAboveBowlSwitch

	rightGrabBowlSwitch, err := sw.FromDependencies(deps, conf.RightGrabBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-grab-bowl switch '%s': %w", conf.RightGrabBowl, err)
	}
	s.rightGrabBowl = rightGrabBowlSwitch

	rightAboveDeliverySwitch, err := sw.FromDependencies(deps, conf.RightAboveDelivery)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-above-delivery switch '%s': %w", conf.RightAboveDelivery, err)
	}
	s.rightAboveDelivery = rightAboveDeliverySwitch

	rightBowlDeliverySwitch, err := sw.FromDependencies(deps, conf.RightBowlDelivery)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-bowl-delivery switch '%s': %w", conf.RightBowlDelivery, err)
	}
	s.rightBowlDelivery = rightBowlDeliverySwitch

	rightHomeSwitch, err := sw.FromDependencies(deps, conf.RightHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-home switch '%s': %w", conf.RightHome, err)
	}
	s.rightHome = rightHomeSwitch

	for _, binCfg := range conf.Bins {
		aboveBinSwitch, err := sw.FromDependencies(deps, binCfg.AboveBin)
		if err != nil {
			return nil, fmt.Errorf("failed to get above-bin switch '%s' for bin '%s': %w", binCfg.AboveBin, binCfg.Name, err)
		}

		inBinSwitch, err := sw.FromDependencies(deps, binCfg.InBin)
		if err != nil {
			return nil, fmt.Errorf("failed to get in-bin switch '%s' for bin '%s': %w", binCfg.InBin, binCfg.Name, err)
		}

		s.bins[binCfg.Name] = &grabberBinSwitches{
			aboveBin: aboveBinSwitch,
			inBin:    inBinSwitch,
		}
	}

	s.logger.Infof("Grabber controls initialized with %d bins", len(s.bins))
	return s, nil
}

func (s *grabberControls) Name() resource.Name {
	return s.name
}

func (s *grabberControls) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["get_from_bin"]; ok {
		return s.doGetFromBin(ctx, cmd)
	}
	if _, ok := cmd["deliver_bowl"]; ok {
		return s.doDeliverBowl(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'get_from_bin' or 'deliver_bowl' field")
}

func (s *grabberControls) doGetFromBin(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	getFromBin := cmd["get_from_bin"]

	binName, ok := getFromBin.(string)
	if !ok {
		return nil, fmt.Errorf("'get_from_bin' must be a string, got %T", getFromBin)
	}

	bin, ok := s.bins[binName]
	if !ok {
		return nil, fmt.Errorf("bin '%s' not found in configuration", binName)
	}

	s.logger.Infof("Executing get_from_bin for bin '%s'", binName)

	if err := bin.aboveBin.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-bin switch to position 2: %w", err)
	}
	s.logger.Debugf("Set above-bin switch to position 2")

	if err := bin.inBin.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set in-bin switch to position 2: %w", err)
	}
	s.logger.Debugf("Set in-bin switch to position 2")

	if _, err := s.leftGripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to close left gripper: %w", err)
	}
	s.logger.Debugf("Closed left gripper")

	if err := bin.aboveBin.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-bin switch to position 2 (second time): %w", err)
	}
	s.logger.Debugf("Set above-bin switch to position 2 (second time)")

	if err := s.highAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set high-above-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set high-above-bowl switch to position 2")

	if err := s.leftGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open left gripper: %w", err)
	}
	s.logger.Debugf("Opened left gripper")

	if err := s.leftHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set left-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set left-home switch to position 2")

	s.logger.Infof("Successfully completed get_from_bin for bin '%s'", binName)

	return map[string]interface{}{
		"success": true,
		"bin":     binName,
		"message": fmt.Sprintf("Successfully grabbed from bin '%s' and moved to bowl", binName),
	}, nil
}

func (s *grabberControls) doDeliverBowl(ctx context.Context) (map[string]interface{}, error) {
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

	if err := s.rightHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-home switch to position 2")

	s.logger.Infof("Successfully completed deliver_bowl")

	return map[string]interface{}{
		"success": true,
		"message": "Successfully delivered bowl",
	}, nil
}

func (s *grabberControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
