package salad

import (
	"context"
	"fmt"
	"sort"
	"sync"

	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
)

var BuildCoordinator = resource.NewModel("ncs", "salad", "build-coordinator")

// categoryOrder defines the build sequence for ingredient categories.
// Ingredients are added to the bowl in this order.
var categoryOrder = map[string]int{
	"base":     0,
	"protein":  1,
	"topping":  2,
	"dressing": 3,
}

type BuildCoordinatorIngredientConfig struct {
	Name            string  `json:"name"`
	GramsPerServing float64 `json:"grams-per-serving"`
	Category        string  `json:"category"`
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
		if ing.Category == "" {
			return nil, nil, resource.NewConfigValidationFieldRequiredError(
				fmt.Sprintf("%s.ingredients.%d", path, i), "category",
			)
		}
		if _, ok := categoryOrder[ing.Category]; !ok {
			return nil, nil, fmt.Errorf(
				"ingredient %q at %s.ingredients.%d has unknown category %q",
				ing.Name, path, i, ing.Category,
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
	ingredients          map[string]float64 // name -> grams per serving
	ingredientCategories map[string]string  // name -> category

	mu              sync.RWMutex
	status          string
	progress        float64
	buildCancelFunc func()
	buildDone       chan struct{}
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
		name:            name,
		logger:          logger,
		cfg:             conf,
		cancelCtx:       cancelCtx,
		cancelFunc:      cancelFunc,
		ingredients:          make(map[string]float64),
		ingredientCategories: make(map[string]string),
		status:          "idle",
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
		s.ingredientCategories[ing.Name] = ing.Category
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
	if _, ok := cmd["stop"]; ok {
		return s.doStop()
	}
	if _, ok := cmd["reset"]; ok {
		err := s.resetAll(ctx)
		if err != nil {
			return map[string]interface{}{
				"success": false,
				"message": fmt.Sprintf("Failed to reset all controls: %v", err),
			}, nil
		}
		return map[string]interface{}{
			"success": true,
			"message": "Successfully reset all controls",
		}, nil
	}
	if _, ok := cmd["status"]; ok {
		return s.getStatus(), nil
	}
	if _, ok := cmd["list_ingredients"]; ok {
		return s.listIngredients(), nil
	}
	return nil, fmt.Errorf("unknown command, expected 'build_salad', 'stop', 'reset', 'status', or 'list_ingredients' field")
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

func (s *buildCoordinator) listIngredients() map[string]interface{} {
	ingredients := make([]interface{}, 0, len(s.cfg.Ingredients))
	for _, ing := range s.cfg.Ingredients {
		ingredients = append(ingredients, map[string]interface{}{
			"name":             ing.Name,
			"grams_per_serving": ing.GramsPerServing,
			"category":         ing.Category,
		})
	}
	return map[string]interface{}{
		"ingredients": ingredients,
	}
}

func (s *buildCoordinator) doStop() (map[string]interface{}, error) {
	s.mu.RLock()
	cancelFunc := s.buildCancelFunc
	done := s.buildDone
	s.mu.RUnlock()

	if cancelFunc == nil {
		return map[string]interface{}{
			"success": false,
			"message": "No build in progress",
		}, nil
	}

	s.logger.Infof("Stop requested, cancelling build")
	cancelFunc()
	<-done

	return map[string]interface{}{
		"success": true,
		"message": "Build stopped",
	}, nil
}

func (s *buildCoordinator) doBuildSalad(ctx context.Context, value interface{}) (map[string]interface{}, error) {
	// Guard against concurrent builds.
	s.mu.Lock()
	if s.buildCancelFunc != nil {
		s.mu.Unlock()
		return map[string]interface{}{
			"success": false,
			"message": "A build is already in progress, use 'stop' to cancel it first",
		}, nil
	}
	buildCtx, buildCancelFunc := context.WithCancel(s.cancelCtx)
	s.buildCancelFunc = buildCancelFunc
	s.buildDone = make(chan struct{})
	s.status = "preparing"
	s.progress = 0
	s.mu.Unlock()

	defer func() {
		s.mu.Lock()
		s.buildCancelFunc = nil
		close(s.buildDone)
		s.buildDone = nil
		s.mu.Unlock()
	}()

	result, err := s.executeBuild(buildCtx, value)
	if buildCtx.Err() != nil {
		s.logger.Infof("Build stopped, resetting hardware")
		if resetErr := s.resetAll(s.cancelCtx); resetErr != nil {
			s.logger.Errorf("Failed to reset hardware after stop: %v", resetErr)
		}
		s.updateStatus("stopped", 0)
		return map[string]interface{}{
			"success": false,
			"message": "Build stopped",
		}, nil
	}

	return result, err
}

func (s *buildCoordinator) executeBuild(ctx context.Context, value interface{}) (map[string]interface{}, error) {
	// reset to initial positions
	err := s.resetAll(ctx)
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to reset all controls: %v", err),
		}, nil
	}
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
		category    string
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
			category:    s.ingredientCategories[name],
		})
		totalServings += servings
	}

	// Sort ingredients by category build order (base -> protein -> topping -> dressing).
	sort.SliceStable(targets, func(i, j int) bool {
		return categoryOrder[targets[i].category] < categoryOrder[targets[j].category]
	})

	// Total steps = all servings + 1 for bowl delivery
	totalSteps := totalServings + 1
	var completedServings float64

	s.logger.Infof("Building salad with %d ingredients", len(targets))

	for _, target := range targets {
		if ctx.Err() != nil {
			return nil, ctx.Err()
		}
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
		if ctx.Err() != nil {
			return ctx.Err()
		}
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

func (s *buildCoordinator) resetAll(ctx context.Context) error {
	_, err := s.grabberControls.DoCommand(ctx, map[string]interface{}{
		"reset": true,
	})
	if err != nil {
		return fmt.Errorf("failed to reset grabber controls: %w", err)
	}
	_, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
		"reset": true,
	})
	if err != nil {
		return fmt.Errorf("failed to reset bowl controls: %w", err)
	}
	return nil
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
