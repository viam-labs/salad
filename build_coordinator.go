package salad

import (
	"context"
	"fmt"
	"sync"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

var BuildCoordinator = resource.NewModel("ncs", "salad", "build-coordinator")

type BuildCoordinatorIngredientConfig struct {
	Name            string  `json:"name"`
	GramsPerServing float64 `json:"grams-per-serving"`
}

type BuildCoordinatorConfig struct {
	GrabberControls string                             `json:"grabber-controls"`
	BowlControls    string                             `json:"bowl-controls"`
	ScaleSensor     string                             `json:"scale-sensor"`
	Ingredients     []BuildCoordinatorIngredientConfig `json:"ingredients"`
}

func init() {
	resource.RegisterService(genericservice.API, BuildCoordinator,
		resource.Registration[resource.Resource, *BuildCoordinatorConfig]{
			Constructor: newBuildCoordinator,
		},
	)
}

func (cfg *BuildCoordinatorConfig) Validate(path string) ([]string, []string, error) {
	if cfg.GrabberControls == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "grabber-controls")
	}
	if cfg.BowlControls == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bowl-controls")
	}
	if cfg.ScaleSensor == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "scale-sensor")
	}
	if len(cfg.Ingredients) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "ingredients")
	}

	deps := []string{cfg.GrabberControls, cfg.BowlControls, cfg.ScaleSensor}

	for i, ing := range cfg.Ingredients {
		if ing.Name == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(
				fmt.Sprintf("%s.ingredients.%d", path, i), "name",
			)
		}
		if ing.GramsPerServing <= 0 {
			return nil, nil, fmt.Errorf(
				"ingredient %q at %s.ingredients.%d must have a positive grams-per-serving",
				ing.Name, path, i,
			)
		}
	}

	return deps, nil, nil
}

type buildCoordinator struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *BuildCoordinatorConfig

	cancelCtx  context.Context
	cancelFunc func()

	grabberControls resource.Resource
	bowlControls    resource.Resource
	scaleSensor     sensor.Sensor
	ingredients     map[string]float64 // name -> grams per serving

	mu       sync.RWMutex
	status   string
	progress float64
}

func newBuildCoordinator(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*BuildCoordinatorConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewBuildCoordinator(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewBuildCoordinator(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *BuildCoordinatorConfig, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &buildCoordinator{
		name:        name,
		logger:      logger,
		cfg:         conf,
		cancelCtx:   cancelCtx,
		cancelFunc:  cancelFunc,
		ingredients: make(map[string]float64),
		status:      "idle",
	}

	grabber, ok := deps[genericservice.Named(conf.GrabberControls)]
	if !ok {
		return nil, fmt.Errorf("grabber controls service %q not found in dependencies", conf.GrabberControls)
	}
	s.grabberControls = grabber

	bowlControls, ok := deps[genericservice.Named(conf.BowlControls)]
	if !ok {
		return nil, fmt.Errorf("bowl controls service %q not found in dependencies", conf.BowlControls)
	}
	s.bowlControls = bowlControls

	scale, err := sensor.FromProvider(deps, conf.ScaleSensor)
	if err != nil {
		return nil, fmt.Errorf("failed to get scale sensor %q: %w", conf.ScaleSensor, err)
	}
	s.scaleSensor = scale

	for _, ing := range conf.Ingredients {
		s.ingredients[ing.Name] = ing.GramsPerServing
	}

	s.logger.Infof("Build coordinator initialized with %d ingredients", len(s.ingredients))
	return s, nil
}

func (s *buildCoordinator) Name() resource.Name {
	return s.name
}

func (s *buildCoordinator) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if val, ok := cmd["build_salad"]; ok {
		return s.doBuildSalad(ctx, val)
	}
	if _, ok := cmd["status"]; ok {
		return s.getStatus(), nil
	}
	return nil, fmt.Errorf("unknown command, expected 'build_salad' or 'status' field")
}

func (s *buildCoordinator) updateStatus(status string, progress float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.status = status
	s.progress = progress
}

func (s *buildCoordinator) getStatus() map[string]interface{} {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return map[string]interface{}{
		"status":   s.status,
		"progress": s.progress,
	}
}

func (s *buildCoordinator) doBuildSalad(ctx context.Context, value interface{}) (map[string]interface{}, error) {
	ingredientMap, ok := value.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("build_salad value must be a map of ingredient name to servings count")
	}
	if len(ingredientMap) == 0 {
		return nil, fmt.Errorf("build_salad requires at least one ingredient")
	}

	result, err := s.bowlControls.DoCommand(ctx, map[string]interface{}{
		"prepare_bowl": true,
	})
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to prepare bowl: %v", err),
		}, nil
	}

	result, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
		"reset": true,
	})
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to reset bowl controls after preparing: %v", err),
		}, nil
	}

	type ingredientTarget struct {
		name        string
		servings    float64
		targetGrams float64
	}
	var targets []ingredientTarget
	var totalServings float64

	for name, servingsRaw := range ingredientMap {
		gramsPerServing, exists := s.ingredients[name]
		if !exists {
			return nil, fmt.Errorf("unknown ingredient %q, not in configuration", name)
		}

		servings, err := toFloat64(servingsRaw)
		if err != nil {
			return nil, fmt.Errorf("invalid servings value for ingredient %q: %w", name, err)
		}
		if servings <= 0 {
			return nil, fmt.Errorf("servings for ingredient %q must be positive", name)
		}

		targets = append(targets, ingredientTarget{
			name:        name,
			servings:    servings,
			targetGrams: gramsPerServing * servings,
		})
		totalServings += servings
	}

	// Total steps = all servings + 1 for bowl delivery
	totalSteps := totalServings + 1
	var completedServings float64

	s.logger.Infof("Building salad with %d ingredients", len(targets))

	for _, target := range targets {
		s.updateStatus(fmt.Sprintf("adding %s", target.name), completedServings/totalSteps*100)
		s.logger.Infof("Adding ingredient %q: target %.1fg", target.name, target.targetGrams)
		if err := s.addIngredient(ctx, target.name, target.targetGrams); err != nil {
			return map[string]interface{}{
				"success": false,
				"message": fmt.Sprintf("Failed to add ingredient %q: %v", target.name, err),
			}, nil
		}
		completedServings += target.servings
	}
	result, err = s.grabberControls.DoCommand(ctx, map[string]interface{}{
		"reset": true,
	})
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to reset grabber controls: %v", err),
		}, nil
	}

	s.updateStatus("delivering salad", completedServings/totalSteps*100)
	s.logger.Infof("All ingredients added, delivering bowl")
	result, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
		"deliver_bowl": true,
	})
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to deliver bowl: %v", err),
		}, nil
	}
	if success, ok := result["success"].(bool); ok && !success {
		msg, _ := result["message"].(string)
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to deliver bowl: %s", msg),
		}, nil
	}

	result, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
		"reset": true,
	})
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to reset grabber controls: %v", err),
		}, nil
	}
	s.updateStatus("complete", 100)

	return map[string]interface{}{
		"success": true,
		"message": "Salad built and delivered successfully",
	}, nil
}

const zeroChangeTolerance = 0.5 // grams

func (s *buildCoordinator) addIngredient(ctx context.Context, name string, targetGrams float64) error {
	var totalAdded float64
	var zeroChangeStreak int

	for totalAdded < targetGrams {
		weightBefore, err := s.readScaleWeight(ctx)
		if err != nil {
			return fmt.Errorf("failed to read scale before grab: %w", err)
		}

		s.logger.Infof("Grabbing %q (added so far: %.1fg / %.1fg)", name, totalAdded, targetGrams)
		result, err := s.grabberControls.DoCommand(ctx, map[string]interface{}{
			"get_from_bin": name,
		})
		if err != nil {
			return fmt.Errorf("failed to grab from bin %q: %w", name, err)
		}
		if success, ok := result["success"].(bool); ok && !success {
			msg, _ := result["message"].(string)
			return fmt.Errorf("grab from bin %q failed: %s", name, msg)
		}

		weightAfter, err := s.readScaleWeight(ctx)
		if err != nil {
			return fmt.Errorf("failed to read scale after grab: %w", err)
		}

		change := weightAfter - weightBefore
		s.logger.Infof("Scale change for %q: %.1fg (before: %.1fg, after: %.1fg)",
			name, change, weightBefore, weightAfter)

		if change < zeroChangeTolerance {
			zeroChangeStreak++
			if zeroChangeStreak >= 3 {
				s.logger.Errorf("3 consecutive grabs with no weight change for ingredient %q, possible empty bin", name)
				break
			}
			s.logger.Warnf("No weight change detected for %q (streak: %d/3)", name, zeroChangeStreak)
		} else {
			zeroChangeStreak = 0
		}

		totalAdded += change
	}

	s.logger.Infof("Ingredient %q complete: added %.1fg (target: %.1fg)", name, totalAdded, targetGrams)
	return nil
}

func (s *buildCoordinator) readScaleWeight(ctx context.Context) (float64, error) {
	readings, err := s.scaleSensor.Readings(ctx, nil)
	if err != nil {
		return 0, fmt.Errorf("failed to read scale sensor: %w", err)
	}

	for _, v := range readings {
		if val, err := toFloat64(v); err == nil {
			return val, nil
		}
	}

	return 0, fmt.Errorf("no numeric reading found from scale sensor")
}

func toFloat64(v interface{}) (float64, error) {
	switch val := v.(type) {
	case float64:
		return val, nil
	case float32:
		return float64(val), nil
	case int:
		return float64(val), nil
	case int64:
		return float64(val), nil
	case int32:
		return float64(val), nil
	default:
		return 0, fmt.Errorf("cannot convert %T to float64", v)
	}
}

func (s *buildCoordinator) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
