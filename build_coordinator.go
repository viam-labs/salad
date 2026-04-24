package salad

import (
	"context"
	"fmt"
	"sort"
	"sync"
	"time"

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

// BuildState is the build-level state exposed via the status DoCommand and the Svelte app.
type BuildState string

const (
	BuildStateIdle      BuildState = "idle"
	BuildStatePreparing BuildState = "preparing"
	BuildStateAdding    BuildState = "adding"
	BuildStateDelivering BuildState = "delivering"
	BuildStateComplete  BuildState = "complete"
	BuildStateStopped   BuildState = "stopped"
	BuildStateFailed    BuildState = "failed"
)

type buildStatus struct {
	State             BuildState
	Progress          float64
	CustomerName      string
	CurrentIngredient string
	FailureReason     string
	InterruptedAt     string // ingredient name active when a stop was requested
	StartedAt         time.Time
	Warnings          []string // non-fatal post-delivery failures
}

type BuildCoordinatorIngredientConfig struct {
	Name            string  `json:"name"`
	GramsPerServing float64 `json:"grams-per-serving"`
	Category        string  `json:"category"`
}

type BuildCoordinatorConfig struct {
	GrabberControls   string                             `json:"grabber-controls"`
	BowlControls      string                             `json:"bowl-controls"`
	ScaleSensor       string                             `json:"scale-sensor"`
	Ingredients       []BuildCoordinatorIngredientConfig `json:"ingredients"`
	DressingControls  string                             `json:"dressing-controls"`
	ChefsKissControls string                             `json:"chefs-kiss-controls"`
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
	if cfg.DressingControls == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "dressing-controls")
	}
	if cfg.ChefsKissControls == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "chefs-kiss-controls")
	}
	if len(cfg.Ingredients) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "ingredients")
	}

	deps := []string{cfg.GrabberControls, cfg.BowlControls, cfg.ScaleSensor, cfg.DressingControls, cfg.ChefsKissControls}

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

	grabberControls      resource.Resource
	bowlControls         resource.Resource
	chefsKissControls    resource.Resource
	scaleSensor          sensor.Sensor
	dressingControls     resource.Resource
	ingredients          map[string]float64 // name -> grams per serving
	ingredientCategories map[string]string  // name -> category

	mu              sync.RWMutex
	buildStatus     buildStatus
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
		name:                 name,
		logger:               logger,
		cfg:                  conf,
		cancelCtx:            cancelCtx,
		cancelFunc:           cancelFunc,
		ingredients:          make(map[string]float64),
		ingredientCategories: make(map[string]string),
		buildStatus:          buildStatus{State: BuildStateIdle},
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

	dressingControls, ok := deps[genericservice.Named(conf.DressingControls)]
	if !ok {
		return nil, fmt.Errorf("dressing controls service %q not found in dependencies", conf.DressingControls)
	}
	s.dressingControls = dressingControls

	chefsKissControls, ok := deps[genericservice.Named(conf.ChefsKissControls)]
	if !ok {
		return nil, fmt.Errorf("chefs kiss controls service %q not found in dependencies", conf.ChefsKissControls)
	}
	s.chefsKissControls = chefsKissControls

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
		customerName, _ := cmd["customer_name"].(string)
		return s.doBuildSalad(ctx, val, customerName)
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

// setBuildState replaces the entire build status and logs the transition at Info level.
// Use for terminal transitions (idle, complete, failed, stopped) and build initialization.
func (s *buildCoordinator) setBuildState(bs buildStatus) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.buildStatus = bs
	switch {
	case bs.FailureReason != "":
		s.logger.Infof("build state: %s — %s", bs.State, bs.FailureReason)
	case bs.InterruptedAt != "":
		s.logger.Infof("build state: %s (interrupted at %s)", bs.State, bs.InterruptedAt)
	case bs.CurrentIngredient != "":
		s.logger.Infof("build state: %s ingredient=%s %.0f%%", bs.State, bs.CurrentIngredient, bs.Progress)
	default:
		s.logger.Infof("build state: %s %.0f%%", bs.State, bs.Progress)
	}
}

// updateActiveState updates progress fields while preserving CustomerName, StartedAt, and Warnings.
// Use for per-step transitions during an active build.
func (s *buildCoordinator) updateActiveState(state BuildState, progress float64, ingredient string) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.buildStatus.State = state
	s.buildStatus.Progress = progress
	s.buildStatus.CurrentIngredient = ingredient
	if ingredient != "" {
		s.logger.Infof("build state: %s ingredient=%s %.0f%%", state, ingredient, progress)
	} else {
		s.logger.Infof("build state: %s %.0f%%", state, progress)
	}
}

func (s *buildCoordinator) getStatus() map[string]interface{} {
	s.mu.RLock()
	bs := s.buildStatus
	s.mu.RUnlock()

	result := map[string]interface{}{
		"state":         string(bs.State),
		"progress":      bs.Progress,
		"customer_name": bs.CustomerName,
	}
	if bs.CurrentIngredient != "" {
		result["current_ingredient"] = bs.CurrentIngredient
	}
	if bs.FailureReason != "" {
		result["failure_reason"] = bs.FailureReason
	}
	if bs.InterruptedAt != "" {
		result["interrupted_at"] = bs.InterruptedAt
	}
	if len(bs.Warnings) > 0 {
		warnings := make([]interface{}, len(bs.Warnings))
		for i, w := range bs.Warnings {
			warnings[i] = w
		}
		result["warnings"] = warnings
	}
	if !bs.StartedAt.IsZero() && bs.Progress > 0 && bs.Progress < 100 {
		elapsed := time.Since(bs.StartedAt).Seconds()
		result["elapsed_seconds"] = elapsed
		result["eta_seconds"] = elapsed * (100 - bs.Progress) / bs.Progress
	}
	return result
}

func (s *buildCoordinator) listIngredients() map[string]interface{} {
	ingredients := make([]interface{}, 0, len(s.cfg.Ingredients))
	for _, ing := range s.cfg.Ingredients {
		ingredients = append(ingredients, map[string]interface{}{
			"name":              ing.Name,
			"grams_per_serving": ing.GramsPerServing,
			"category":          ing.Category,
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

func (s *buildCoordinator) doBuildSalad(ctx context.Context, value interface{}, customerName string) (map[string]interface{}, error) {
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
	s.buildStatus = buildStatus{
		State:        BuildStatePreparing,
		CustomerName: customerName,
		StartedAt:    time.Now(),
	}
	s.mu.Unlock()

	if customerName != "" {
		s.logger.Infof("New salad order received for %q: %v", customerName, value)
	} else {
		s.logger.Infof("New salad order received: %v", value)
	}

	defer func() {
		s.mu.Lock()
		s.buildCancelFunc = nil
		close(s.buildDone)
		s.buildDone = nil
		s.mu.Unlock()
	}()

	result, err := s.executeBuild(buildCtx, value)
	if buildCtx.Err() != nil {
		s.mu.RLock()
		interruptedAt := s.buildStatus.CurrentIngredient
		savedCustomerName := s.buildStatus.CustomerName
		s.mu.RUnlock()

		s.logger.Infof("Build stopped, resetting hardware")
		if resetErr := s.resetAll(s.cancelCtx); resetErr != nil {
			s.logger.Errorf("Failed to reset hardware after stop: %v", resetErr)
		}
		s.setBuildState(buildStatus{
			State:         BuildStateStopped,
			CustomerName:  savedCustomerName,
			InterruptedAt: interruptedAt,
		})
		return map[string]interface{}{
			"success": false,
			"message": "Build stopped",
		}, nil
	}

	return result, err
}

func (s *buildCoordinator) executeBuild(ctx context.Context, value interface{}) (map[string]interface{}, error) {
	s.mu.RLock()
	customerName := s.buildStatus.CustomerName
	s.mu.RUnlock()

	// failWith sets BuildStateFailed and returns a DoCommand failure response.
	failWith := func(reason string, progress float64) (map[string]interface{}, error) {
		s.setBuildState(buildStatus{
			State:         BuildStateFailed,
			Progress:      progress,
			CustomerName:  customerName,
			FailureReason: reason,
		})
		return map[string]interface{}{"success": false, "message": reason}, nil
	}

	if err := s.resetAll(ctx); err != nil {
		return failWith(fmt.Sprintf("failed to reset controls: %v", err), 0)
	}

	ingredientMap, ok := value.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("build_salad value must be a map of ingredient name to servings count")
	}
	if len(ingredientMap) == 0 {
		return nil, fmt.Errorf("build_salad requires at least one ingredient")
	}

	if _, err := s.bowlControls.DoCommand(ctx, map[string]interface{}{"prepare_bowl": true}); err != nil {
		return failWith(fmt.Sprintf("failed to prepare bowl: %v", err), 0)
	}

	if _, err := s.bowlControls.DoCommand(ctx, map[string]interface{}{"reset": true}); err != nil {
		return failWith(fmt.Sprintf("failed to reset bowl after prepare: %v", err), 0)
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
		if target.category == "dressing" {
			continue
		}
		s.updateActiveState(BuildStateAdding, completedServings/totalSteps*100, target.name)
		s.logger.Infof("Adding ingredient %q: target %.1fg", target.name, target.targetGrams)
		if err := s.addIngredient(ctx, target.name, target.targetGrams); err != nil {
			return failWith(fmt.Sprintf("ingredient %q: %v", target.name, err), completedServings/totalSteps*100)
		}
		completedServings += target.servings
	}

	if _, err := s.grabberControls.DoCommand(ctx, map[string]interface{}{"reset": true}); err != nil {
		return failWith(fmt.Sprintf("failed to reset grabber: %v", err), completedServings/totalSteps*100)
	}

	s.updateActiveState(BuildStateDelivering, completedServings/totalSteps*100, "")
	s.logger.Infof("All ingredients added, delivering bowl")

	deliverResult, err := s.bowlControls.DoCommand(ctx, map[string]interface{}{"deliver_bowl": true})
	if err != nil {
		return failWith(fmt.Sprintf("failed to deliver bowl: %v", err), completedServings/totalSteps*100)
	}
	if success, ok := deliverResult["success"].(bool); !ok || !success {
		msg, _ := deliverResult["message"].(string)
		return failWith(fmt.Sprintf("bowl delivery failed: %s", msg), completedServings/totalSteps*100)
	}

	if _, err := s.bowlControls.DoCommand(ctx, map[string]interface{}{"reset": true}); err != nil {
		return failWith(fmt.Sprintf("failed to reset bowl after delivery: %v", err), completedServings/totalSteps*100)
	}

	// Post-delivery steps: non-fatal. Collect warnings so they land atomically in the
	// complete state (the Svelte app transitions on "complete", so it sees all warnings at once).
	var postWarnings []string

	for _, target := range targets {
		if target.category == "dressing" {
			if err := s.addDressing(ctx); err != nil {
				s.logger.Warnf("dressing failed: %v", err)
				postWarnings = append(postWarnings, fmt.Sprintf("dressing failed: %v", err))
			}
		}
	}

	if _, err := s.chefsKissControls.DoCommand(ctx, map[string]interface{}{"chefs_kiss": true}); err != nil {
		s.logger.Warnf("chef's kiss failed: %v", err)
		postWarnings = append(postWarnings, fmt.Sprintf("chef's kiss failed: %v", err))
	}

	s.setBuildState(buildStatus{
		State:        BuildStateComplete,
		Progress:     100,
		CustomerName: customerName,
		Warnings:     postWarnings,
	})

	return map[string]interface{}{
		"success": true,
		"message": "Salad built and delivered successfully",
	}, nil
}

const zeroChangeTolerance = 0.5 // grams

func (s *buildCoordinator) addDressing(ctx context.Context) error {
	_, err := s.dressingControls.DoCommand(ctx, map[string]interface{}{
		"pour_dressing": true,
	})
	if err != nil {
		return fmt.Errorf("failed to pour dressing: %w", err)
	}
	return nil
}

func (s *buildCoordinator) addIngredient(ctx context.Context, name string, targetGrams float64) error {
	var totalAdded float64
	var zeroChangeStreak int

	for totalAdded < targetGrams {
		if ctx.Err() != nil {
			return fmt.Errorf("cancelled during grab of %q: %w", name, ctx.Err())
		}

		s.logger.Debugf("step: read_scale_before ingredient=%q added=%.1fg", name, totalAdded)
		weightBefore, err := s.readScaleWeight(ctx)
		if err != nil {
			return fmt.Errorf("failed to read scale before grab: %w", err)
		}

		s.logger.Debugf("step: grabbing ingredient=%q added=%.1fg target=%.1fg", name, totalAdded, targetGrams)
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

		s.logger.Debugf("step: read_scale_after ingredient=%q", name)
		weightAfter, err := s.readScaleWeight(ctx)
		if err != nil {
			return fmt.Errorf("failed to read scale after grab: %w", err)
		}

		change := weightAfter - weightBefore
		s.logger.Infof("Scale change for %q: %.1fg (before: %.1fg, after: %.1fg)",
			name, change, weightBefore, weightAfter)

		if change < zeroChangeTolerance {
			zeroChangeStreak++
			s.logger.Debugf("step: zero_change ingredient=%q streak=%d/3", name, zeroChangeStreak)
			if zeroChangeStreak >= 3 {
				return fmt.Errorf("3 consecutive grabs with no weight change for %q, possible empty bin", name)
			}
			s.logger.Warnf("No weight change detected for %q (streak: %d/3)", name, zeroChangeStreak)
		} else {
			zeroChangeStreak = 0
		}

		totalAdded += change
	}

	s.logger.Debugf("step: ingredient_complete ingredient=%q added=%.1fg target=%.1fg", name, totalAdded, targetGrams)
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
