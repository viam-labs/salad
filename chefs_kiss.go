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

var ChefsKissControls = resource.NewModel("ncs", "salad", "chefs-kiss-controls")

func init() {
	resource.RegisterService(genericservice.API, ChefsKissControls,
		resource.Registration[resource.Resource, *ChefsKissControlsConfig]{
			Constructor: newChefsKissControls,
		},
	)
}

type ChefsKissControlsConfig struct {
	Position string `json:"position"`
	Gripper  string `json:"gripper"`
	Home     string `json:"home"`
}

func (cfg *ChefsKissControlsConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Gripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "gripper")
	}
	if cfg.Position == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "position")
	}
	if cfg.Home == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "home")
	}

	return []string{cfg.Gripper, cfg.Position, cfg.Home}, []string{}, nil
}

type chefsKissControls struct {
	resource.AlwaysRebuild

	name resource.Name

	logger logging.Logger
	cfg    *ChefsKissControlsConfig

	cancelCtx  context.Context
	cancelFunc func()

	gripper  gripper.Gripper
	position sw.Switch
	home     sw.Switch
}

func newChefsKissControls(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*ChefsKissControlsConfig](rawConf)
	if err != nil {
		return nil, err
	}

	return NewChefsKissControls(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewChefsKissControls(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *ChefsKissControlsConfig, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &chefsKissControls{
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

	positionSwitch, err := sw.FromProvider(deps, conf.Position)
	if err != nil {
		return nil, fmt.Errorf("failed to get position switch '%s': %w", conf.Position, err)
	}
	s.position = positionSwitch

	homeSwitch, err := sw.FromProvider(deps, conf.Home)
	if err != nil {
		return nil, fmt.Errorf("failed to get home switch '%s': %w", conf.Home, err)
	}
	s.home = homeSwitch

	s.logger.Infof("Chefs kiss controls initialized")
	return s, nil
}

func (s *chefsKissControls) Name() resource.Name {
	return s.name
}

func (s *chefsKissControls) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["chefs_kiss"]; ok {
		return s.doChefsKiss(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'chefs_kiss' field")
}

func (s *chefsKissControls) doChefsKiss(ctx context.Context) (map[string]interface{}, error) {
	s.logger.Infof("Executing chefs kiss")

	if _, err := s.gripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to close gripper: %w", err)
	}
	s.logger.Debugf("Closed gripper")

	if err := s.position.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set position switch to position 2: %w", err)
	}
	s.logger.Debugf("Set position switch to position 2")

	if err := s.gripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open gripper: %w", err)
	}
	s.logger.Debugf("Opened gripper")

	if err := s.home.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set home switch to position 2")

	s.logger.Infof("Successfully completed chefs kiss")

	return map[string]interface{}{
		"success": true,
		"message": "Successfully completed chefs kiss",
	}, nil
}

func (s *chefsKissControls) reset(ctx context.Context) (map[string]interface{}, error) {
	if err := s.home.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set right-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set right-home switch to position 2")

	return nil, nil
}

func (s *chefsKissControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
