package salad

import (
	"context"
	"fmt"
	"sort"
	"time"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

var SafetyCheck = resource.NewModel("ncs", "salad", "safety-check")

func init() {
	resource.RegisterService(genericservice.API, SafetyCheck,
		resource.Registration[resource.Resource, *SafetyCheckConfig]{
			Constructor: newSafetyCheck,
		},
	)
}

type SafetyCheckConfig struct {
	TempSensor string  `json:"temp-sensor"`
	MinTempC   float64 `json:"min-temp-celsius"`
	MaxTempC   float64 `json:"max-temp-celsius"`
	TTS        string  `json:"text-to-speech"`
}

func (cfg *SafetyCheckConfig) Validate(path string) ([]string, []string, error) {
	if cfg.TempSensor == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "temp-sensor")
	}
	if cfg.MinTempC >= cfg.MaxTempC {
		return nil, nil, fmt.Errorf("min-temp-celsius must be less than max-temp-celsius")
	}
	deps := []string{cfg.TempSensor}
	if cfg.TTS != "" {
		deps = append(deps, cfg.TTS)
	}
	return deps, nil, nil
}

type safetyCheck struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *SafetyCheckConfig

	cancelCtx  context.Context
	cancelFunc func()

	tempSensor   sensor.Sensor
	textToSpeech resource.Resource
}

func newSafetyCheck(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*SafetyCheckConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewSafetyCheck(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewSafetyCheck(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *SafetyCheckConfig, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &safetyCheck{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}

	tempSensor, err := sensor.FromProvider(deps, conf.TempSensor)
	if err != nil {
		return nil, fmt.Errorf("failed to get temp sensor %q: %w", conf.TempSensor, err)
	}
	s.tempSensor = tempSensor

	if conf.TTS != "" {
		tts, ok := deps[genericservice.Named(conf.TTS)]
		if !ok {
			return nil, fmt.Errorf("text-to-speech service %q not found in dependencies", conf.TTS)
		}
		s.textToSpeech = tts
	}

	s.logger.Infof("Safety check initialized")
	return s, nil
}

func (s *safetyCheck) Name() resource.Name {
	return s.name
}

func (s *safetyCheck) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["check_temp"]; ok {
		return s.checkTemp(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'check_temp' field")
}

func (s *safetyCheck) checkTemp(ctx context.Context) (map[string]interface{}, error) {
	const numReadings = 5
	readings := make([]float64, 0, numReadings)
	for i := 0; i < numReadings; i++ {
		if i > 0 {
			select {
			case <-ctx.Done():
				return nil, ctx.Err()
			case <-time.After(time.Second):
			}
		}
		raw, err := s.tempSensor.Readings(ctx, nil)
		if err != nil {
			return nil, fmt.Errorf("failed to read temp sensor: %w", err)
		}
		tempRaw, ok := raw["degrees_celsius"]
		if !ok {
			return nil, fmt.Errorf("temp sensor missing degrees_celsius reading")
		}
		temp, err := toFloat64(tempRaw)
		if err != nil {
			return nil, fmt.Errorf("could not parse temp reading: %w", err)
		}
		readings = append(readings, temp)
	}

	// Trim min and max, average the remaining 3.
	sort.Float64s(readings)
	trimmed := readings[1 : numReadings-1]
	var sum float64
	for _, v := range trimmed {
		sum += v
	}
	avg := sum / float64(len(trimmed))

	inRange := avg >= s.cfg.MinTempC && avg <= s.cfg.MaxTempC
	if !inRange && s.textToSpeech != nil {
		msg := fmt.Sprintf(
			"Temperature is currently %.1f degrees celsius, outside safe range of %.1f to %.1f degrees celsius.",
			avg, s.cfg.MinTempC, s.cfg.MaxTempC,
		)
		if _, ttsErr := s.textToSpeech.DoCommand(ctx, map[string]interface{}{"say": msg}); ttsErr != nil {
			s.logger.Warnw("failed to announce temp warning", "err", ttsErr)
		}
	}

	return map[string]interface{}{
		"temperature_celsius": avg,
		"in_range":            inRange,
	}, nil
}

func (s *safetyCheck) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
