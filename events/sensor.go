package events

import (
	"context"
	"sync"
	"time"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/data"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
)

// Model is the registered resource model for the build-events sensor.
var Model = resource.NewModel("ncs", "salad", "build-events")

func init() {
	resource.RegisterComponent(sensor.API, Model,
		resource.Registration[sensor.Sensor, *Config]{
			Constructor: newSensor,
		},
	)
}

// Config has no attributes.
type Config struct{}

// Validate implements resource.ConfigValidator.
func (*Config) Validate(string) ([]string, []string, error) {
	return nil, nil, nil
}

// buildEventsSensor exposes a queue of emitted events as a Viam sensor.
type buildEventsSensor struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger

	mu      sync.Mutex
	pending []map[string]interface{}
}

func newSensor(_ context.Context, _ resource.Dependencies, rawConf resource.Config, logger logging.Logger) (sensor.Sensor, error) {
	if _, err := resource.NativeConfig[*Config](rawConf); err != nil {
		return nil, err
	}
	return &buildEventsSensor{
		name:   rawConf.ResourceName(),
		logger: logger,
	}, nil
}

// Name implements resource.Resource.
func (s *buildEventsSensor) Name() resource.Name {
	return s.name
}

// Status is required by the sensor.Sensor interface.
func (s *buildEventsSensor) Status(context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

// Readings drains one queued event. When the queue is empty it returns
// data.ErrNoCaptureToStore so Viam Data Management writes nothing for that
// poll — only real events become rows in the tabular store.
func (s *buildEventsSensor) Readings(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()
	if len(s.pending) == 0 {
		return nil, data.ErrNoCaptureToStore
	}
	payload := s.pending[0]
	s.pending[0] = nil
	s.pending = s.pending[1:]
	return payload, nil
}

// DoCommand is unused; events flow in via Emit, not DoCommand.
func (s *buildEventsSensor) DoCommand(_ context.Context, _ map[string]interface{}) (map[string]interface{}, error) {
	return nil, nil
}

// Close releases the queue; in-flight events are dropped.
func (s *buildEventsSensor) Close(context.Context) error {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pending = nil
	return nil
}

// Emit appends an event to the queue. Safe for concurrent use. The event is
// flattened to a single map: common fields plus every key in Event.Fields.
func (s *buildEventsSensor) Emit(_ context.Context, e Event) {
	payload := map[string]interface{}{
		"event_type":            e.Type,
		"build_id":              e.BuildID,
		"customer_name":         e.CustomerName,
		"customer_name_display": e.CustomerNameDisplay,
		"theme":                 e.Theme,
		"timestamp":             e.Timestamp.UTC().Format(time.RFC3339Nano),
	}
	for k, v := range e.Fields {
		payload[k] = v
	}
	s.mu.Lock()
	defer s.mu.Unlock()
	s.pending = append(s.pending, payload)
}
