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
	Bins          []GrabberControlsBinConfig `json:"bins"`
	HighAboveBowl string                     `json:"high-above-bowl"`
	InBowl        string                     `json:"in-bowl"`
	LeftGripper   string                     `json:"left-gripper"`
	LeftHome      string                     `json:"left-home"`
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

	if cfg.InBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "in-bowl")
	}

	requiredDeps := []string{}

	requiredDeps = append(requiredDeps, cfg.HighAboveBowl)
	requiredDeps = append(requiredDeps, cfg.LeftGripper)
	requiredDeps = append(requiredDeps, cfg.LeftHome)
	requiredDeps = append(requiredDeps, cfg.InBowl)

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

	bins          map[string]*grabberBinSwitches
	highAboveBowl sw.Switch
	leftGripper   gripper.Gripper
	leftInBowl    sw.Switch
	leftHome      sw.Switch
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

	highAboveBowlSwitch, err := sw.FromProvider(deps, conf.HighAboveBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get high-above-bowl switch '%s': %w", conf.HighAboveBowl, err)
	}
	s.highAboveBowl = highAboveBowlSwitch

	leftGripperComponent, err := gripper.FromProvider(deps, conf.LeftGripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get left gripper '%s': %w", conf.LeftGripper, err)
	}
	s.leftGripper = leftGripperComponent

	leftHomeSwitch, err := sw.FromProvider(deps, conf.LeftHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get left-home switch '%s': %w", conf.LeftHome, err)
	}
	s.leftHome = leftHomeSwitch

	leftInBowlSwitch, err := sw.FromProvider(deps, conf.InBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get in-bowl switch '%s': %w", conf.InBowl, err)
	}
	s.leftInBowl = leftInBowlSwitch

	for _, binCfg := range conf.Bins {
		aboveBinSwitch, err := sw.FromProvider(deps, binCfg.AboveBin)
		if err != nil {
			return nil, fmt.Errorf("failed to get above-bin switch '%s' for bin '%s': %w", binCfg.AboveBin, binCfg.Name, err)
		}

		inBinSwitch, err := sw.FromProvider(deps, binCfg.InBin)
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
	if _, ok := cmd["reset"]; ok {
		return s.reset(ctx)
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

	if err := s.leftInBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set in-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set in-bowl switch to position 2")

	if err := s.leftGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open left gripper: %w", err)
	}
	s.logger.Debugf("Opened left gripper")

	s.logger.Infof("Successfully completed get_from_bin for bin '%s'", binName)

	return map[string]interface{}{
		"success": true,
		"bin":     binName,
		"message": fmt.Sprintf("Successfully grabbed from bin '%s' and moved to bowl", binName),
	}, nil
}

func (s *grabberControls) reset(ctx context.Context) (map[string]interface{}, error) {
	if err := s.leftHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set left-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set left-home switch to position 2")

	return nil, nil
}

func (s *grabberControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
