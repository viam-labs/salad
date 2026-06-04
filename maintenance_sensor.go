package salad

import (
	"context"
	"fmt"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

var MaintenanceSensor = resource.NewModel("ncs", "salad", "maintenance-sensor")

func init() {
	resource.RegisterComponent(sensor.API, MaintenanceSensor,
		resource.Registration[sensor.Sensor, *MaintenanceSensorConfig]{
			Constructor: newMaintenanceSensor,
		},
	)
}

type MaintenanceSensorConfig struct {
	BuildCoordinatorName string `json:"build-coordinator-name"`
}

func (cfg *MaintenanceSensorConfig) Validate(path string) ([]string, []string, error) {
	if cfg.BuildCoordinatorName == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "build-coordinator-name")
	}
	return []string{
		genericservice.Named(cfg.BuildCoordinatorName).String(),
	}, nil, nil
}

type maintenanceSensor struct {
	resource.AlwaysRebuild

	name             resource.Name
	logger           logging.Logger
	buildCoordinator resource.Resource
}

func newMaintenanceSensor(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	conf, err := resource.NativeConfig[*MaintenanceSensorConfig](rawConf)
	if err != nil {
		return nil, err
	}

	buildCoordinatorRes, ok := deps[genericservice.Named(conf.BuildCoordinatorName)]
	if !ok {
		return nil, fmt.Errorf("build coordinator service %q not found in dependencies", conf.BuildCoordinatorName)
	}

	return &maintenanceSensor{
		name:             rawConf.ResourceName(),
		logger:           logger,
		buildCoordinator: buildCoordinatorRes,
	}, nil
}

func (m *maintenanceSensor) Name() resource.Name {
	return m.name
}

func (m *maintenanceSensor) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (m *maintenanceSensor) Readings(ctx context.Context, extra map[string]interface{}) (map[string]interface{}, error) {
	resp, err := m.buildCoordinator.DoCommand(ctx, map[string]interface{}{"status": true})
	if err != nil {
		m.logger.CWarnw(
			ctx, "maintenance-sensor: failed to query build coordinator",
			"err", err,
		)
		return nil, fmt.Errorf("failed to query build coordinator: %w", err)
	}

	status, _ := resp["status"].(string)
	isBusy := status != "" && status != "idle" && status != "complete"

	isSafe := !isBusy
	m.logger.CDebugf(
		ctx, "maintenance-sensor: is_safe=%v build_status=%q is_busy=%v",
		isSafe, status,
	)

	return map[string]interface{}{
		"is_safe":      isSafe,
		"build_status": status,
	}, nil
}

func (m *maintenanceSensor) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

func (m *maintenanceSensor) Close(context.Context) error {
	return nil
}
