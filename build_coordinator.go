package salad

import (
	"context"
	"encoding/base64"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
	"sync"
	"time"

	"github.com/google/uuid"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/sensor"
	sw "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"

	"salad/filter"
	"salad/segmentation"
	saladutils "salad/utils"
)

var BuildCoordinator = resource.NewModel("ncs", "salad", "build-coordinator")

// categoryOrder defines the build sequence for ingredient categories.
// Ingredients are added to the bowl in this order.
var categoryOrder = map[string]int{
	"base":           0,
	"protein":        1,
	"topping":        2,
	categoryDressing: 3,
}

const (
	categoryDressing           = "dressing"
	defaultMeshTargetTriangles = 5000
)

type BuildCoordinatorIngredientConfig struct {
	Name            string  `json:"name"`
	GramsPerServing float64 `json:"grams-per-serving"`
	Category        string  `json:"category"`
	ZoneID          *int    `json:"zone-id"`
}

type BuildCoordinatorFilterConfig struct {
	VoxelMM            *float64 `json:"voxel-mm"`
	NeighborRadius     *int     `json:"neighbor-radius"`
	MinNeighbors       *int     `json:"min-neighbors"`
	MinComponentVoxels *int     `json:"min-component-voxels"`
}

func (c *BuildCoordinatorFilterConfig) Validate(path string) error {
	if c.VoxelMM != nil && *c.VoxelMM <= 0 {
		return fmt.Errorf("%s.voxel-mm must be > 0, got %v", path, *c.VoxelMM)
	}
	if c.NeighborRadius != nil && *c.NeighborRadius < 1 {
		return fmt.Errorf("%s.neighbor-radius must be >= 1, got %d", path, *c.NeighborRadius)
	}
	if c.MinNeighbors != nil && *c.MinNeighbors < 0 {
		return fmt.Errorf("%s.min-neighbors must be >= 0, got %d", path, *c.MinNeighbors)
	}
	if c.MinComponentVoxels != nil && *c.MinComponentVoxels < 0 {
		return fmt.Errorf("%s.min-component-voxels must be >= 0, got %d", path, *c.MinComponentVoxels)
	}
	return nil
}

func (c *BuildCoordinatorFilterConfig) toOptions() filter.Options {
	opts := filter.DefaultOptions()
	if c == nil {
		return opts
	}
	if c.VoxelMM != nil {
		opts.VoxelMM = *c.VoxelMM
	}
	if c.NeighborRadius != nil {
		opts.NeighborRadius = *c.NeighborRadius
	}
	if c.MinNeighbors != nil {
		opts.MinNeighbors = *c.MinNeighbors
	}
	if c.MinComponentVoxels != nil {
		opts.MinComponentVoxels = *c.MinComponentVoxels
	}
	return opts
}

type BuildCoordinatorSegmentationConfig struct {
	CellSizeMM         *float64 `json:"cell-size-mm"`
	DividerZPercentile *float64 `json:"divider-z-percentile"`
	DividerGradientMM  *float64 `json:"divider-gradient-mm"`
	DividerDilation    *int     `json:"divider-dilation"`
	MinZoneAreaMM2     *float64 `json:"min-zone-area-mm2"`
	MaxZoneAreaMM2     *float64 `json:"max-zone-area-mm2"`
}

func (c *BuildCoordinatorSegmentationConfig) Validate(path string) error {
	if c.CellSizeMM != nil && *c.CellSizeMM <= 0 {
		return fmt.Errorf("%s.cell-size-mm must be > 0, got %v", path, *c.CellSizeMM)
	}
	if c.DividerZPercentile != nil && (*c.DividerZPercentile <= 0 || *c.DividerZPercentile > 1) {
		return fmt.Errorf("%s.divider-z-percentile must be in (0, 1], got %v", path, *c.DividerZPercentile)
	}
	if c.DividerGradientMM != nil && *c.DividerGradientMM < 0 {
		return fmt.Errorf("%s.divider-gradient-mm must be >= 0, got %v", path, *c.DividerGradientMM)
	}
	if c.DividerDilation != nil && *c.DividerDilation < 0 {
		return fmt.Errorf("%s.divider-dilation must be >= 0, got %v", path, *c.DividerDilation)
	}
	if c.MinZoneAreaMM2 != nil && *c.MinZoneAreaMM2 <= 0 {
		return fmt.Errorf("%s.min-zone-area-mm2 must be > 0, got %v", path, *c.MinZoneAreaMM2)
	}
	if c.MaxZoneAreaMM2 != nil && *c.MaxZoneAreaMM2 < 0 {
		return fmt.Errorf("%s.max-zone-area-mm2 must be >= 0, got %v", path, *c.MaxZoneAreaMM2)
	}
	return nil
}

type BuildCoordinatorConfig struct {
	GrabberControls     string                              `json:"grabber-controls"`
	BowlControls        string                              `json:"bowl-controls"`
	ScaleSensor         string                              `json:"scale-sensor"`
	DressingControls    string                              `json:"dressing-controls"`
	ChefsKissControls   string                              `json:"chefs-kiss-controls"`
	TextToSpeech        string                              `json:"text-to-speech"`
	ImagingCamera       string                              `json:"imaging-camera"`
	CaptureDir          string                              `json:"capture-dir"`
	Simulate            bool                                `json:"simulate"`
	SkipLilArm          bool                                `json:"skip-lil-arm"`
	LeftHome            string                              `json:"left-home"`
	RightHome           string                              `json:"right-home"`
	Filter              *BuildCoordinatorFilterConfig       `json:"filter"`
	Segmentation        *BuildCoordinatorSegmentationConfig `json:"segmentation"`
	MeshTargetTriangles *int                                `json:"mesh-target-triangles,omitempty"`
	DepthStepMM         *float64                            `json:"depth-step-mm,omitempty"`
	MaxDepthOffsetMM    *float64                            `json:"max-depth-offset-mm,omitempty"`
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
	if cfg.ScaleSensor == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "scale-sensor")
	}
	if cfg.DressingControls == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "dressing-controls")
	}
	if cfg.ChefsKissControls == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "chefs-kiss-controls")
	}
	if cfg.LeftHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-home")
	}
	if cfg.RightHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "right-home")
	}

	deps := []string{cfg.GrabberControls, cfg.ScaleSensor, cfg.DressingControls, cfg.ChefsKissControls, cfg.LeftHome, cfg.RightHome}
	var optDeps []string
	if cfg.BowlControls != "" {
		optDeps = append(optDeps, cfg.BowlControls)
	}
	if cfg.TextToSpeech != "" {
		optDeps = append(optDeps, cfg.TextToSpeech)
	}
	if cfg.ImagingCamera != "" {
		optDeps = append(optDeps, cfg.ImagingCamera)
	}

	if cfg.Filter != nil {
		if err := cfg.Filter.Validate(path + ".filter"); err != nil {
			return nil, nil, err
		}
	}

	if cfg.Segmentation != nil {
		if err := cfg.Segmentation.Validate(path + ".segmentation"); err != nil {
			return nil, nil, err
		}
	}

	if cfg.MeshTargetTriangles != nil && *cfg.MeshTargetTriangles < 0 {
		return nil, nil, fmt.Errorf("%s.mesh-target-triangles must be >= 0, got %d", path, *cfg.MeshTargetTriangles)
	}

	return deps, optDeps, nil
}

type buildCoordinator struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *BuildCoordinatorConfig

	cancelCtx  context.Context
	cancelFunc func()

	grabberControls   resource.Resource
	bowlControls      resource.Resource
	chefsKissControls resource.Resource
	scaleSensor       sensor.Sensor
	dressingControls  resource.Resource
	textToSpeech      resource.Resource
	imagingCamera     camera.Camera
	leftHome          sw.Switch
	rightHome         sw.Switch
	ingredients       map[string]BuildCoordinatorIngredientConfig

	// TODO: Wrap inside a state machine - restrictions on state transtions (i.e. can't transition from "add ingredients" to "go home")
	// As our system's complexity increases, inlining mutextes will without any guardrails will inevitbly lead to deadlocks.
	// status shouldn't be strings - should be a constant enum type.
	mu           sync.RWMutex
	status       string
	progress     float64
	customerName string
	errorMsg     string
	simulate     bool
	skipLilArm   bool
	opCancelFunc func()
	opDone       chan struct{}
	buildID      string

	assetsDir string
}

func newBuildCoordinator(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*BuildCoordinatorConfig](rawConf)
	if err != nil {
		return nil, err
	}
	if conf.CaptureDir == "" {
		conf.CaptureDir = "/root/.viam/capture"
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
		ingredients: map[string]BuildCoordinatorIngredientConfig{},
		status:      "idle",
		simulate:    conf.Simulate,
		skipLilArm:  conf.SkipLilArm,
		assetsDir:   "/home/viam/assets",
	}

	grabber, ok := deps[genericservice.Named(conf.GrabberControls)]
	if !ok {
		return nil, fmt.Errorf("grabber controls service %q not found in dependencies", conf.GrabberControls)
	}
	s.grabberControls = grabber

	if conf.BowlControls != "" {
		bowlControls, ok := deps[genericservice.Named(conf.BowlControls)]
		if !ok {
			return nil, fmt.Errorf("bowl controls service %q not found in dependencies", conf.BowlControls)
		}
		s.bowlControls = bowlControls
	}

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

	// TODO: Make required.
	if conf.TextToSpeech != "" {
		textToSpeech, ok := deps[genericservice.Named(conf.TextToSpeech)]
		if !ok {
			return nil, fmt.Errorf("text-to-speech service %q not found in dependencies", conf.TextToSpeech)
		}
		s.textToSpeech = textToSpeech
	}

	if conf.ImagingCamera != "" {
		cam, err := camera.FromProvider(deps, conf.ImagingCamera)
		if err != nil {
			return nil, fmt.Errorf("failed to get imaging camera %q: %w", conf.ImagingCamera, err)
		}
		s.imagingCamera = cam
	}

	scale, err := sensor.FromProvider(deps, conf.ScaleSensor)
	if err != nil {
		return nil, fmt.Errorf("failed to get scale sensor %q: %w", conf.ScaleSensor, err)
	}
	s.scaleSensor = scale

	leftHomeSwitch, err := sw.FromProvider(deps, conf.LeftHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get left-home switch %q: %w", conf.LeftHome, err)
	}
	s.leftHome = leftHomeSwitch

	rightHomeSwitch, err := sw.FromProvider(deps, conf.RightHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get right-home switch %q: %w", conf.RightHome, err)
	}
	s.rightHome = rightHomeSwitch
	// get config for grabber controls to pull out ingredients
	ingredientsResult, err := s.grabberControls.DoCommand(ctx, map[string]any{"get_ingredients": true})
	if err != nil {
		return nil, fmt.Errorf("failed to get ingredients from grabber controls: %w", err)
	}

	if ingredients, ok := ingredientsResult["ingredients"].([]map[string]any); ok {
		for _, ing := range ingredients {
			zoneID, ok := ing["zone_id"].(int)
			if !ok {
				return nil, fmt.Errorf("zone_id is not an int: %v", ing["zone_id"])
			}
			_, ok = categoryOrder[ing["category"].(string)]
			if !ok {
				return nil, fmt.Errorf("unknown category: %v", ing["category"])
			}
			s.ingredients[ing["name"].(string)] = BuildCoordinatorIngredientConfig{
				Name:            ing["name"].(string),
				GramsPerServing: ing["grams_per_serving"].(float64),
				Category:        ing["category"].(string),
				ZoneID:          &zoneID,
			}
		}
	} else {
		return nil, fmt.Errorf("failed to get ingredients from grabber controls: %w", err)
	}

	dressingsResult, err := s.dressingControls.DoCommand(ctx, map[string]any{"get_dressings": true})
	if err != nil {
		return nil, fmt.Errorf("failed to get dressings from dressing controls: %w", err)
	}
	if dressings, ok := dressingsResult["dressings"].([]map[string]any); ok {
		for _, dressing := range dressings {
			s.ingredients[dressing["name"].(string)] = BuildCoordinatorIngredientConfig{
				Name:            dressing["name"].(string),
				GramsPerServing: 5,
				Category:        categoryDressing,
				ZoneID:          nil,
			}
		}
	}

	s.logger.Infof("Build coordinator initialized with %d ingredients", len(s.ingredients))
	return s, nil
}

func (s *buildCoordinator) Name() resource.Name {
	return s.name
}

func (s *buildCoordinator) Status(ctx context.Context) (map[string]interface{}, error) {
	return s.getStatus(), nil
}

func (s *buildCoordinator) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if val, ok := cmd["build_salad"]; ok {
		customerName, _ := cmd["customer_name"].(string)
		return s.doBuildSalad(ctx, val, customerName)
	}
	if _, ok := cmd["setup_station"]; ok {
		return s.doSetupStation()
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
	if _, ok := cmd["get_setup_result"]; ok {
		return s.getSetupResult()
	}
	return nil, fmt.Errorf("unknown command, expected 'build_salad', 'setup_station', 'stop', 'reset', 'status', 'list_ingredients', or 'get_setup_result' field")
}

func (s *buildCoordinator) updateStatus(status string, progress float64) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.status = status
	s.progress = progress
}

func (s *buildCoordinator) getBuildID() string {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return s.buildID
}

func (s *buildCoordinator) getStatus() map[string]interface{} {
	s.mu.RLock()
	defer s.mu.RUnlock()
	return map[string]interface{}{
		"status":        s.status,
		"progress":      s.progress,
		"customer_name": s.customerName,
		"error_msg":     s.errorMsg,
	}
}

func (s *buildCoordinator) listIngredients() map[string]interface{} {
	ingredients := make([]interface{}, 0, len(s.ingredients))
	for _, ing := range s.ingredients {
		ingredients = append(ingredients, map[string]interface{}{
			"name":              ing.Name,
			"grams_per_serving": ing.GramsPerServing,
			"category":          ing.Category,
		})
		if ing.ZoneID != nil {
			ingredients = append(ingredients, map[string]interface{}{
				"zone_id": *ing.ZoneID,
			})
		}
	}
	return map[string]interface{}{
		"ingredients": ingredients,
	}
}

func (s *buildCoordinator) doStop() (map[string]interface{}, error) {
	s.mu.RLock()
	cancelFunc := s.opCancelFunc
	done := s.opDone
	s.mu.RUnlock()

	if cancelFunc == nil {
		return map[string]interface{}{
			"success": false,
			"message": "No operation in progress",
		}, nil
	}

	s.logger.Infof("Stop requested, cancelling operation")
	cancelFunc()
	<-done

	return map[string]interface{}{
		"success": true,
		"message": "Operation stopped",
	}, nil
}

//nolint:unparam // ctx kept for DoCommand-style API symmetry; build runs under s.cancelCtx so it survives caller cancellation
func (s *buildCoordinator) doBuildSalad(ctx context.Context, value interface{}, customerName string) (map[string]interface{}, error) {
	// Guard against concurrent builds.

	// TODO: Move inside state machine.
	// Verify we're transitioning from a valid state.
	// Also move to caller - will be easier to ensure ever DoCommand verifies sate before proceeding.
	s.mu.Lock()
	if s.opCancelFunc != nil {
		s.mu.Unlock()
		return map[string]interface{}{
			"success": false,
			"message": "An operation is already in progress, use 'stop' to cancel it first",
		}, nil
	}
	buildCtx, buildCancelFunc := context.WithCancel(s.cancelCtx)
	s.opCancelFunc = buildCancelFunc
	s.opDone = make(chan struct{})
	s.status = "preparing"
	s.progress = 0
	s.customerName = customerName
	s.errorMsg = ""
	s.buildID = fmt.Sprintf("%s_%s", time.Now().UTC().Format("20060102-150405"), uuid.NewString()[:8])
	s.mu.Unlock()
	s.logger.Infof("Build ID: %s", s.buildID)

	if customerName != "" {
		s.logger.Infof("New salad order received for %q: %v", customerName, value)
	} else {
		s.logger.Infof("New salad order received: %v", value)
	}

	// TODO: Move to a state machine function.
	defer func() {
		s.mu.Lock()
		s.opCancelFunc = nil
		s.customerName = ""
		s.buildID = ""
		close(s.opDone)
		s.opDone = nil
		s.mu.Unlock()
	}()

	var result map[string]interface{}
	var err error
	if s.simulate {
		s.logger.Infof("Simulate mode: skipping robot commands for build")
		s.updateStatus("complete", 100)
		result = map[string]interface{}{
			"success":   true,
			"message":   "Salad built and delivered successfully (simulated)",
			"simulated": true,
		}
	} else {
		result, err = s.executeBuild(buildCtx, value)
	}
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

	buildFailed := false
	var failMsg string
	if err != nil {
		buildFailed = true
		failMsg = err.Error()
	} else if result != nil {
		if success, ok := result["success"].(bool); !ok || !success {
			buildFailed = true
			if msg, ok := result["message"].(string); ok {
				failMsg = msg
			}
		}
	}
	if buildFailed {
		s.mu.Lock()
		s.status = "failed"
		s.errorMsg = failMsg
		s.mu.Unlock()
		if result != nil {
			return result, nil
		}
		return map[string]interface{}{
			"success": false,
			"message": failMsg,
		}, nil
	}

	if err == nil && s.textToSpeech != nil {
		msg := "Your salad is ready!"
		if customerName != "" {
			msg = customerName + "'s salad is ready!"
		}
		if _, ttsErr := s.textToSpeech.DoCommand(buildCtx, map[string]interface{}{"say": msg}); ttsErr != nil {
			s.logger.Errorw("text-to-speech announcement failed", "err", ttsErr)
		}
	}

	return result, err
}

func (s *buildCoordinator) doSetupStation() (map[string]interface{}, error) {
	if s.imagingCamera == nil {
		return nil, fmt.Errorf("setup_station requires 'imaging-camera' to be configured")
	}

	s.mu.Lock()
	if s.opCancelFunc != nil {
		s.mu.Unlock()
		return map[string]interface{}{
			"success": false,
			"message": "An operation is already in progress, use 'stop' to cancel it first",
		}, nil
	}
	setupCtx, setupCancelFunc := context.WithCancel(s.cancelCtx)
	s.opCancelFunc = setupCancelFunc
	s.opDone = make(chan struct{})
	s.status = "setting_up_station"
	s.progress = 0
	s.errorMsg = ""
	s.mu.Unlock()

	s.logger.Infof("Starting station setup")

	defer func() {
		s.mu.Lock()
		s.opCancelFunc = nil
		close(s.opDone)
		s.opDone = nil
		s.mu.Unlock()
	}()

	err := s.executeSetup(setupCtx)
	if setupCtx.Err() != nil {
		s.updateStatus("stopped", 0)
		return map[string]interface{}{
			"success": false,
			"message": "Setup stopped",
		}, nil
	}
	if err != nil {
		s.mu.Lock()
		s.status = "failed"
		s.errorMsg = err.Error()
		s.mu.Unlock()
		return map[string]interface{}{
			"success": false,
			"message": err.Error(),
		}, nil
	}

	s.updateStatus("idle", 0)
	s.logger.Infof("Station setup complete")
	return map[string]interface{}{
		"success": true,
		"message": "Station setup complete",
	}, nil
}

func (s *buildCoordinator) executeSetup(ctx context.Context) error {
	s.logger.Infof("Capturing point cloud from imaging camera")
	pc, err := s.imagingCamera.NextPointCloud(ctx, nil)
	if err != nil {
		return fmt.Errorf("failed to capture point cloud: %w", err)
	}

	if err := os.MkdirAll(s.cfg.CaptureDir, 0o750); err != nil {
		return fmt.Errorf("failed to create capture directory %q: %w", s.cfg.CaptureDir, err)
	}

	ts := time.Now().Format("20060102-150405")

	// Write the PCD to the stable path first so meshification uses a
	// consistent location. Also write to the capture dir so it syncs to cloud.
	stablePCDPath := s.assetsDir + "/merged.pcd"
	stableFilteredPCDPath := s.assetsDir + "/filtered_merged.pcd"
	stableZonesPath := s.assetsDir + "/zones.json"
	if err := os.MkdirAll(s.assetsDir, 0o750); err != nil {
		return fmt.Errorf("failed to create assets directory %q: %w", s.assetsDir, err)
	}
	if err := saladutils.WritePCD(pc, stablePCDPath); err != nil {
		return fmt.Errorf("failed to write stable PCD: %w", err)
	}
	s.logger.Infof("Wrote %s (%d points)", stablePCDPath, pc.Size())

	pcdPath := filepath.Join(s.cfg.CaptureDir, fmt.Sprintf("setup-%s.pcd", ts))
	if err := saladutils.WritePCD(pc, pcdPath); err != nil {
		return fmt.Errorf("failed to write point cloud: %w", err)
	}
	s.logger.Infof("Wrote %s", pcdPath)

	filterOpts := s.cfg.Filter.toOptions()
	s.logger.Infof("Filtering merged PCD (voxel=%vmm, neighbor-radius=%d, min-neighbors=%d, min-component-voxels=%d)",
		filterOpts.VoxelMM, filterOpts.NeighborRadius, filterOpts.MinNeighbors, filterOpts.MinComponentVoxels)
	filteredPC, _, err := filter.Apply(pc, filterOpts, s.logger)
	if err != nil {
		return fmt.Errorf("filter failed: %w", err)
	}
	if err := saladutils.WritePCD(filteredPC, stableFilteredPCDPath); err != nil {
		return fmt.Errorf("failed to write stable filtered PCD: %w", err)
	}
	s.logger.Infof("Wrote %s (%d points)", stableFilteredPCDPath, filteredPC.Size())

	filteredPCDPath := filepath.Join(s.cfg.CaptureDir, fmt.Sprintf("setup-%s-filtered.pcd", ts))
	if err := saladutils.WritePCD(filteredPC, filteredPCDPath); err != nil {
		return fmt.Errorf("failed to write filtered point cloud: %w", err)
	}
	s.logger.Infof("Wrote %s", filteredPCDPath)

	s.logger.Infof("Running meshifier on %s", stableFilteredPCDPath)
	meshPath := filepath.Join(s.cfg.CaptureDir, fmt.Sprintf("setup-%s-mesh.ply", ts))
	targetTriangles := defaultMeshTargetTriangles
	if s.cfg.MeshTargetTriangles != nil {
		targetTriangles = *s.cfg.MeshTargetTriangles
	}
	if err := saladutils.ExecMeshifier(ctx, stableFilteredPCDPath, meshPath, 30, 50, 0, targetTriangles); err != nil {
		return fmt.Errorf("meshification failed: %w", err)
	}
	s.logger.Infof("Wrote mesh %s", meshPath)

	stableMeshPath := s.assetsDir + "/mesh.ply"
	meshBytes, err := os.ReadFile(meshPath) //nolint:gosec // path built from config-controlled CaptureDir
	if err != nil {
		return fmt.Errorf("failed to read mesh for stable copy: %w", err)
	}
	if err := os.WriteFile(stableMeshPath, meshBytes, 0o600); err != nil {
		return fmt.Errorf("failed to write stable mesh: %w", err)
	}
	s.logger.Infof("Wrote stable mesh to %s", stableMeshPath)

	s.logger.Infof("Segmenting mesh %s", meshPath)
	segOpts := segmentation.DefaultOptions()
	if c := s.cfg.Segmentation; c != nil {
		if c.CellSizeMM != nil {
			segOpts.CellSizeMM = *c.CellSizeMM
		}
		if c.DividerZPercentile != nil {
			segOpts.DividerZPercentile = *c.DividerZPercentile
		}
		if c.DividerGradientMM != nil {
			segOpts.DividerGradientMM = *c.DividerGradientMM
		}
		if c.DividerDilation != nil {
			segOpts.DividerDilation = *c.DividerDilation
		}
		if c.MinZoneAreaMM2 != nil {
			segOpts.MinZoneAreaMM2 = *c.MinZoneAreaMM2
		}
		if c.MaxZoneAreaMM2 != nil {
			segOpts.MaxZoneAreaMM2 = *c.MaxZoneAreaMM2
		}
	}
	result, _, err := segmentation.SegmentFridgeBins(meshPath, segOpts)
	if err != nil {
		return fmt.Errorf("segmentation failed: %w", err)
	}
	zonesPath := filepath.Join(s.cfg.CaptureDir, fmt.Sprintf("setup-%s-zones.json", ts))
	if err := segmentation.SaveZones(result, zonesPath); err != nil {
		return fmt.Errorf("failed to save zones: %w", err)
	}
	s.logger.Infof("Wrote %d zone(s) to %s", len(result.Zones), zonesPath)

	if err := segmentation.SaveZones(result, stableZonesPath); err != nil {
		return fmt.Errorf("failed to write stable zones: %w", err)
	}
	s.logger.Infof("Wrote stable zones to %s", stableZonesPath)

	return nil
}

func (s *buildCoordinator) getSetupResult() (map[string]interface{}, error) {
	if err := s.checkAssets(); err != nil {
		return nil, err
	}

	pcdPath := s.assetsDir + "/merged.pcd"
	zonesPath := s.assetsDir + "/zones.json"

	pcdBytes, err := os.ReadFile(pcdPath) //nolint:gosec // path derived from our own assetsDir
	if err != nil {
		return nil, fmt.Errorf("failed to read PCD file %q: %w", pcdPath, err)
	}

	zonesBytes, err := os.ReadFile(zonesPath) //nolint:gosec // path derived from our own assetsDir
	if err != nil {
		return nil, fmt.Errorf("failed to read zones file %q: %w", zonesPath, err)
	}

	var zones interface{}
	if err := json.Unmarshal(zonesBytes, &zones); err != nil {
		return nil, fmt.Errorf("failed to parse zones: %w", err)
	}

	return map[string]interface{}{
		"pcd":   base64.StdEncoding.EncodeToString(pcdBytes),
		"zones": zones,
	}, nil
}

// checkAssets verifies that setup assets exist and that every configured
// ingredient has a zone ID present in zones.json.
func (s *buildCoordinator) checkAssets() error {
	if _, err := os.Stat(s.assetsDir + "/merged.pcd"); err != nil {
		return fmt.Errorf("setup asset missing merged.pcd")
	}
	if _, err := os.Stat(s.assetsDir + "/zones.json"); err != nil {
		return fmt.Errorf("setup asset missing zones.json")
	}
	if _, err := os.Stat(s.assetsDir + "/mesh.ply"); err != nil {
		return fmt.Errorf("setup asset missing mesh.ply")
	}

	// check all zones exist
	zones, err := segmentation.LoadZones(s.assetsDir + "/zones.json")
	if err != nil {
		return fmt.Errorf("failed to load zones: %w", err)
	}
	availableZoneIDs := make(map[int]bool, len(zones.Zones))
	for _, z := range zones.Zones {
		availableZoneIDs[z.ID] = true
	}
	for _, ing := range s.ingredients {
		if ing.ZoneID == nil {
			continue
		}
		if !availableZoneIDs[*ing.ZoneID] {
			return fmt.Errorf("ingredient %q has zone-id %d which is not in zones.json (zones: %v)", ing.Name, *ing.ZoneID, zones.Zones)
		}
	}
	return nil
}

func (s *buildCoordinator) executeBuild(ctx context.Context, value interface{}) (map[string]interface{}, error) {
	// check setup was run before executing salad build
	if err := s.checkAssets(); err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Please run setup_station before building a salad: %v", err),
		}, nil
	}

	// reset to initial positions
	err := s.resetAll(ctx)
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to reset all controls: %v", err),
		}, nil
	}
	if _, err := s.readScaleWeight(ctx); err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Scale is not available, cannot start build: %v", err),
		}, nil
	}

	ingredientMap, ok := value.(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("build_salad value must be a map of ingredient name to servings count")
	}
	if len(ingredientMap) == 0 {
		return nil, fmt.Errorf("build_salad requires at least one ingredient")
	}

	lilArmControls, _ := s.bowlControls.(*bowlControls)
	lilArmEnabled := !s.skipLilArm && lilArmControls != nil && lilArmControls.lilArmGripper != nil
	if s.skipLilArm {
		s.logger.Infof("skip-lil-arm is set; skipping lil-arm grab_bowl/grab_lid steps")
	}

	if lilArmEnabled {
		_, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
			"grab_bowl": true,
			"target":    60,
		})
		if err != nil {
			return map[string]interface{}{
				"success": false,
				"message": fmt.Sprintf("Failed to grab bowl: %v", err),
			}, nil
		}
	}

	if s.bowlControls != nil {
		_, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
			"reset":        true,
			"skip_lil_arm": s.skipLilArm,
		})
		if err != nil {
			return map[string]interface{}{
				"success": false,
				"message": fmt.Sprintf("Failed to reset bowl controls after preparing: %v", err),
			}, nil
		}
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
			targetGrams: s.ingredients[name].GramsPerServing * servings,
			category:    s.ingredients[name].Category,
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

	// kick off background pre-planning for each dressing while ingredients are grabbed
	for _, t := range targets {
		if t.category != categoryDressing {
			continue
		}
		name := t.name
		buildID := s.getBuildID()
		go func() {
			if _, err := s.dressingControls.DoCommand(ctx, map[string]interface{}{
				"pre_plan_dressing": name,
				"build_id":          buildID,
			}); err != nil {
				s.logger.Warnf("Pre-planning dressing %q failed, will plan on demand: %v", name, err)
			}
		}()
	}

	for _, target := range targets {
		if ctx.Err() != nil {
			return nil, ctx.Err()
		}
		s.updateStatus(fmt.Sprintf("adding %s", target.name), completedServings/totalSteps*100)
		s.logger.Infof("Adding ingredient %q: target %.1fg", target.name, target.targetGrams)
		if target.category == categoryDressing {
			continue
		}
		if err := s.addIngredient(ctx, target.name, target.targetGrams); err != nil {
			s.logger.Errorf("Failed to add ingredient %q: %v", target.name, err)
			return map[string]interface{}{
				"success": false,
				"message": fmt.Sprintf("Failed to add ingredient %q: %v", target.name, err),
			}, nil
		}
		completedServings += target.servings
	}

	_, err = s.grabberControls.DoCommand(ctx, map[string]interface{}{
		"reset": true,
	})
	if err != nil {
		return map[string]interface{}{
			"success": false,
			"message": fmt.Sprintf("Failed to reset grabber controls: %v", err),
		}, nil
	}

	// commenting out for demo, not reliable yet
	// if lilArmEnabled {
	// 	result, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
	// 		"grab_lid": true,
	// 		"target":   80,
	// 	})
	// 	if err != nil {
	// 		return map[string]interface{}{
	// 			"success": false,
	// 			"message": fmt.Sprintf("Failed to grab lid: %v", err),
	// 		}, nil
	// 	}
	// }

	s.updateStatus("delivering salad", completedServings/totalSteps*100)
	s.logger.Infof("All ingredients added; skipping deliver_bowl step")

	if s.bowlControls != nil {
		_, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
			"reset":        true,
			"skip_lil_arm": s.skipLilArm,
		})
		if err != nil {
			return map[string]interface{}{
				"success": false,
				"message": fmt.Sprintf("Failed to reset bowl controls: %v", err),
			}, nil
		}
	}

	// if target contains dressing item:
	// Dressing pours over the bowl at the delivery position, after both
	// the grabber and bowl-controls arms have gone home.
	for _, target := range targets {
		if target.category == categoryDressing {
			s.updateStatus(fmt.Sprintf("adding %s", target.name), 100)
			if err := s.addDressing(ctx, target.name); err != nil {
				s.logger.Errorf("Failed to add dressing %q: %v", target.name, err)
			}
		}
	}

	s.updateStatus("complete", 100)

	// chefs kiss
	if _, err := s.chefsKissControls.DoCommand(ctx, map[string]interface{}{
		"chefs_kiss": true,
	}); err != nil {
		// TODO: Should not fail silently
		s.logger.Errorf("Failed to perform chefs kiss: %v", err)
	}

	return map[string]interface{}{
		"success": true,
		"message": "Salad built and delivered successfully",
	}, nil
}

const (
	zeroChangeTolerance     = 0.5 // grams
	defaultDepthStepMM      = 20.0
	defaultMaxDepthOffsetMM = 80.0
)

func (s *buildCoordinator) depthProbeParams() (step, max float64) {
	step = defaultDepthStepMM
	if s.cfg.DepthStepMM != nil {
		step = *s.cfg.DepthStepMM
	}
	max = defaultMaxDepthOffsetMM
	if s.cfg.MaxDepthOffsetMM != nil {
		max = *s.cfg.MaxDepthOffsetMM
	}
	if step <= 0 || max <= 0 {
		return 0, 0
	}
	return step, max
}

func isMotionPlanningFailure(err error) bool {
	if err == nil {
		return false
	}
	msg := err.Error()
	return strings.Contains(msg, "physically unreachable") ||
		strings.Contains(msg, "zero IK solutions") ||
		strings.Contains(msg, "no plan found") ||
		strings.Contains(msg, "fatal early collision")
}

func (s *buildCoordinator) addDressing(ctx context.Context, name string) error {
	_, err := s.dressingControls.DoCommand(ctx, map[string]interface{}{
		"pour_dressing": name,
		"build_id":      s.getBuildID(),
	})
	if err != nil {
		return fmt.Errorf("failed to pour dressing %q: %w", name, err)
	}
	return nil
}

func (s *buildCoordinator) addIngredient(ctx context.Context, name string, targetGrams float64) error {
	var totalAdded float64
	var zeroChangeStreak int
	var depthOffset float64

	depthStep, maxDepth := s.depthProbeParams()

	for totalAdded < targetGrams {
		if ctx.Err() != nil {
			return ctx.Err()
		}
		weightBefore, err := s.readScaleWeight(ctx)
		if err != nil {
			return fmt.Errorf("failed to read scale before grab: %w", err)
		}

		s.logger.Infof("Grabbing %q (added so far: %.1fg / %.1fg, depth-offset %.1fmm)",
			name, totalAdded, targetGrams, depthOffset)

		zoneID := s.ingredients[name].ZoneID
		if zoneID == nil {
			return fmt.Errorf("ingredient %q has no zone-id", name)
		}
		result, err := s.grabberControls.DoCommand(ctx, map[string]interface{}{
			"get_from_bin":    *zoneID,
			"depth-offset-mm": depthOffset,
			"build_id":        s.getBuildID(),
		})
		if err != nil {
			if isMotionPlanningFailure(err) && depthOffset > 0 {
				newOffset := depthOffset / 2
				if newOffset < 0.5 {
					newOffset = 0
				}
				s.logger.Warnf("Motion planning failed for %q at depth-offset %.1fmm; halving to %.1fmm",
					name, depthOffset, newOffset)
				depthOffset = newOffset
				continue
			}
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
			if depthStep > 0 && zeroChangeStreak >= 2 && depthOffset < maxDepth {
				depthOffset += depthStep
				if depthOffset > maxDepth {
					depthOffset = maxDepth
				}
				s.logger.Infof("Probing deeper for %q: depth-offset now %.1fmm", name, depthOffset)
			}
		} else {
			zeroChangeStreak = 0
			totalAdded += change
		}
	}

	s.logger.Infof("Ingredient %q complete: added %.1fg (target: %.1fg)", name, totalAdded, targetGrams)
	return nil
}

func (s *buildCoordinator) readScaleWeight(ctx context.Context) (float64, error) {
	readings, err := s.scaleSensor.Readings(ctx, nil)
	if err != nil {
		return 0, fmt.Errorf("failed to read scale sensor: %w", err)
	}

	v, ok := readings["weight"]
	if !ok {
		return 0, fmt.Errorf("scale readings missing 'weight' field: %+v", readings)
	}

	weight, err := toFloat64(v)
	if err != nil {
		return 0, fmt.Errorf("'weight' field is not numeric: %w", err)
	}

	return weight, nil
}

func (s *buildCoordinator) resetAll(ctx context.Context) error {
	if err := s.leftHome.SetPosition(ctx, 2, nil); err != nil {
		return fmt.Errorf("failed to set left-home switch to position 2: %w", err)
	}
	if err := s.rightHome.SetPosition(ctx, 2, nil); err != nil {
		return fmt.Errorf("failed to set right-home switch to position 2: %w", err)
	}
	_, err := s.grabberControls.DoCommand(ctx, map[string]interface{}{
		"reset": true,
	})
	if err != nil {
		return fmt.Errorf("failed to reset grabber controls: %w", err)
	}
	if s.bowlControls != nil {
		_, err = s.bowlControls.DoCommand(ctx, map[string]interface{}{
			"reset":        true,
			"skip_lil_arm": s.skipLilArm,
		})
		if err != nil {
			return fmt.Errorf("failed to reset bowl controls: %w", err)
		}
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
