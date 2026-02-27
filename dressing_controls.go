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

var DressingControls = resource.NewModel("ncs", "salad", "dressing-controls")

func init() {
	resource.RegisterService(genericservice.API, DressingControls,
		resource.Registration[resource.Resource, *DressingControlsConfig]{
			Constructor: newDressingControls,
		},
	)
}

type DressingControlsConfig struct {
	Gripper          string  `json:"gripper"`
	PrepareDressing  string  `json:"prepare-dressing"`
	GrabDressing     string  `json:"grab-dressing"`
	PourDressing     string  `json:"pour-dressing"`
	PourDressing2    string  `json:"pour-dressing2"`
	PostPourDressing string  `json:"post-pour-dressing"`
	Home             string  `json:"home"`
	ShakeArmService  *string `json:"shake-arm-service,omitempty"`
}

func (cfg *DressingControlsConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Gripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.Gripper)
	}

	if cfg.PrepareDressing == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.PrepareDressing)
	}

	if cfg.GrabDressing == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.GrabDressing)
	}

	if cfg.PourDressing == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.PourDressing)
	}

	if cfg.PourDressing2 == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.PourDressing2)
	}
	if cfg.PostPourDressing == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.PostPourDressing)
	}

	if cfg.Home == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, cfg.Home)
	}

	requiredDeps := []string{}

	requiredDeps = append(requiredDeps, cfg.Gripper)
	requiredDeps = append(requiredDeps, cfg.PrepareDressing, cfg.GrabDressing, cfg.PourDressing, cfg.PourDressing2, cfg.Home, cfg.PostPourDressing)
	if cfg.ShakeArmService != nil && *cfg.ShakeArmService != "" {
		requiredDeps = append(requiredDeps, *cfg.ShakeArmService)
	}

	return requiredDeps, []string{}, nil
}

type dressingControls struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *DressingControlsConfig

	cancelCtx  context.Context
	cancelFunc func()

	gripper          gripper.Gripper
	grabDressing     sw.Switch
	prepareDressing  sw.Switch
	dressingPour     sw.Switch
	dressingPour2    sw.Switch
	postPourDressing sw.Switch
	home             sw.Switch
	shakeArmService  genericservice.Service
}

func newDressingControls(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*DressingControlsConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewDressingControls(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewDressingControls(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *DressingControlsConfig, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &dressingControls{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}

	gripperComponent, err := gripper.FromProvider(deps, conf.Gripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get gripper '%s': %w", conf.Gripper, err)
	}
	s.gripper = gripperComponent

	prepareDressingSwitch, err := sw.FromProvider(deps, conf.PrepareDressing)
	if err != nil {
		return nil, fmt.Errorf("failed to get prepare-dressing switch '%s': %w", conf.PrepareDressing, err)
	}
	s.prepareDressing = prepareDressingSwitch

	grabDressingSwitch, err := sw.FromProvider(deps, conf.GrabDressing)
	if err != nil {
		return nil, fmt.Errorf("failed to get grab-dressing switch '%s': %w", conf.GrabDressing, err)
	}
	s.grabDressing = grabDressingSwitch

	pourDressingSwitch, err := sw.FromProvider(deps, conf.PourDressing)
	if err != nil {
		return nil, fmt.Errorf("failed to get pour-dressing switch '%s': %w", conf.PourDressing, err)
	}
	s.dressingPour = pourDressingSwitch

	pourDressing2Switch, err := sw.FromProvider(deps, conf.PourDressing2)
	if err != nil {
		return nil, fmt.Errorf("failed to get pour-dressing2 switch '%s': %w", conf.PourDressing2, err)
	}
	s.dressingPour2 = pourDressing2Switch

	postPourDressingSwitch, err := sw.FromProvider(deps, conf.PostPourDressing)
	if err != nil {
		return nil, fmt.Errorf("failed to get post-pour-dressing switch '%s': %w", conf.PostPourDressing, err)
	}
	s.postPourDressing = postPourDressingSwitch

	homeSwitch, err := sw.FromProvider(deps, conf.Home)
	if err != nil {
		return nil, fmt.Errorf("failed to get home switch '%s': %w", conf.Home, err)
	}
	s.home = homeSwitch

	if conf.ShakeArmService != nil && *conf.ShakeArmService != "" {
		shakeArmService, err := genericservice.FromProvider(deps, *conf.ShakeArmService)
		if err != nil {
			return nil, fmt.Errorf("failed to get shake-arm-service '%s': %w", conf.ShakeArmService, err)
		}
		s.shakeArmService = shakeArmService
	}

	s.logger.Infof("Bowl controls initialized")
	return s, nil
}

func (s *dressingControls) Name() resource.Name {
	return s.name
}

func (s *dressingControls) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["pour_dressing"]; ok {
		return s.doPourDressing(ctx)
	}
	if _, ok := cmd["reset"]; ok {
		return s.reset(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'deliver_bowl', 'prepare_bowl', or 'reset' field")
}

func (s *dressingControls) doPourDressing(ctx context.Context) (map[string]interface{}, error) {
	s.logger.Infof("Executing prepare_bowl")

	if err := s.grabDressing.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set grab-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set grab-dressing switch to position 2")

	if _, err := s.gripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open right gripper: %w", err)
	}
	s.logger.Debugf("Opened right gripper")

	if err := s.prepareDressing.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set prepare-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set prepare-dressing switch to position 2")

	if err := s.dressingPour.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set pour-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set pour-dressing switch to position 2")

	if s.shakeArmService != nil {
		_, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{
			"shake_arm": true,
		})
		if err != nil {
			return nil, fmt.Errorf("failed to shake arm: %w", err)
		}
	}

	if err := s.dressingPour2.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set pour-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set pour-dressing switch to position 2")

	if s.shakeArmService != nil {
		_, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{
			"shake_arm": true,
		})
		if err != nil {
			return nil, fmt.Errorf("failed to shake arm: %w", err)
		}
	}

	if err := s.dressingPour.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set pour-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set pour-dressing switch to position 2")

	if s.shakeArmService != nil {
		_, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{
			"shake_arm": true,
		})
		if err != nil {
			return nil, fmt.Errorf("failed to shake arm: %w", err)
		}
	}

	if err := s.postPourDressing.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set post-pour-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set post-pour-dressing switch to position 2")

	if err := s.prepareDressing.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set prepare-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set prepare-dressing switch to position 2")

	if err := s.grabDressing.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set grab-dressing switch to position 2: %w", err)
	}
	s.logger.Debugf("Set grab-dressing switch to position 2")

	if err := s.gripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open gripper: %w", err)
	}
	s.logger.Debugf("Opened gripper")

	if err := s.home.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set home switch to position 2")

	s.logger.Infof("Successfully completed pour_dressing")

	return map[string]interface{}{
		"success": true,
		"message": "Successfully poured dressing",
	}, nil
}

func (s *dressingControls) reset(ctx context.Context) (map[string]interface{}, error) {
	if err := s.home.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-home switch to position 2")

	return nil, nil
}

func (s *dressingControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
