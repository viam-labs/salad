package salad

import (
	"context"
	"fmt"
	"math"
	"sync"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	sw "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"salad/lib/fileio"
	"salad/segmentation"
)

const defaultPlanCaptureDir = "/root/.viam/capture"

var GrabberControls = resource.NewModel("ncs", "salad", "grabber-controls")

const defaultBowlHoverHeightMM = 150.0

func init() {
	resource.RegisterService(genericservice.API, GrabberControls,
		resource.Registration[resource.Resource, *GrabberControlsConfig]{
			Constructor: newGrabberControls,
		},
	)
}

// GrabberControlsBinConfig represents a single bin configuration with switches.
type GrabberControlsBinConfig struct {
	Name   string `json:"name"`
	ZoneID int    `json:"zone-id"`
	// ServingDepthMM is how far below the detected food surface the gripper
	// descends when grabbing a serving from this bin. Defaults to
	// defaultServingDepthMM when zero/unset.
	ServingDepthMM float64 `json:"serving-depth-mm,omitempty"`
	// HoverXOffsetMM and HoverYOffsetMM shift the bin hover XY position from
	// the zone plane point (world frame, mm).
	HoverXOffsetMM  float64 `json:"hover-x-offset-mm,omitempty"`
	HoverYOffsetMM  float64 `json:"hover-y-offset-mm,omitempty"`
	GramsPerServing float64 `json:"grams-per-serving,omitempty"`
	Category        string  `json:"category,omitempty"`
}

type BowlDropPose struct {
	X           float64                               `json:"x"`
	Y           float64                               `json:"y"`
	Z           float64                               `json:"z"`
	Orientation *spatialmath.OrientationVectorDegrees `json:"orientation"`
}

type GrabberControlsConfig struct {
	Bins                              []GrabberControlsBinConfig            `json:"bins"`
	BinHoverHeightMM                  float64                               `json:"bin-hover-height-mm"`
	BinHoverOrientation               *spatialmath.OrientationVectorDegrees `json:"bin-hover-orientation,omitempty"`
	EnableBinClearance                bool                                  `json:"enable-bin-clearance,omitempty"`
	BinClearanceXOffsetMM             float64                               `json:"bin-clearance-x-offset-mm,omitempty"`
	BinClearanceZOffsetMM             float64                               `json:"bin-clearance-z-offset-mm,omitempty"`
	ClearanceLineToleranceMM          float64                               `json:"clearance-line-tolerance-mm,omitempty"`
	ClearanceOrientationToleranceDegs float64                               `json:"clearance-orientation-tolerance-degs,omitempty"`
	GrabHeightMM                      float64                               `json:"grab-height-mm"`
	DroppingPose                      *BowlDropPose                         `json:"dropping-pose"`
	BowlHoverHeightMM                 float64                               `json:"bowl-hover-height-mm,omitempty"`
	Arm                               string                                `json:"arm"`
	Gripper                           string                                `json:"gripper"`
	MotionService                     string                                `json:"motion-service"`
	LeftHome                          string                                `json:"left-home"`
	ShakeArmService                   *string                               `json:"shake-arm-service,omitempty"`
	ScoopShakeService                 *string                               `json:"scoop-shake-service,omitempty"`
	AssetsDir                         string                                `json:"assets-dir"`
	XOffsetMM                         float64                               `json:"x-offset-mm,omitempty"`
	YOffsetMM                         float64                               `json:"y-offset-mm,omitempty"`
	GrabLineToleranceMM               float64                               `json:"grab-line-tolerance-mm,omitempty"`
	GrabOrientationToleranceDegs      float64                               `json:"grab-orientation-tolerance-degs,omitempty"`
	BinImagingCam                     string                                `json:"bin-imaging-cam"`
}

func (cfg *GrabberControlsConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.Bins) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bins")
	}

	if cfg.BinHoverHeightMM == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bin-hover-height-mm")
	}

	if cfg.EnableBinClearance && cfg.BinClearanceXOffsetMM == 0 && cfg.BinClearanceZOffsetMM == 0 {
		return nil, nil, fmt.Errorf("%s: at least one of 'bin-clearance-x-offset-mm' or 'bin-clearance-z-offset-mm' must be set when 'enable-bin-clearance' is true", path)
	}

	if cfg.BinHoverOrientation == nil {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bin-hover-orientation")
	}

	if cfg.MotionService == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "motion-service")
	}

	if cfg.DroppingPose == nil {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "dropping-pose")
	}
	if cfg.DroppingPose.Orientation == nil {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "dropping-pose.orientation")
	}

	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}

	if cfg.Gripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "gripper")
	}

	if cfg.BinImagingCam == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bin-imaging-cam")
	}

	if cfg.LeftHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-home")
	}

	requiredDeps := []string{}

	requiredDeps = append(requiredDeps, cfg.Arm)
	requiredDeps = append(requiredDeps, cfg.Gripper)
	requiredDeps = append(requiredDeps, cfg.BinImagingCam)
	requiredDeps = append(requiredDeps, cfg.MotionService)
	requiredDeps = append(requiredDeps, cfg.LeftHome)
	requiredDeps = append(requiredDeps, framesystem.PublicServiceName.String())
	if cfg.ShakeArmService != nil && *cfg.ShakeArmService != "" {
		requiredDeps = append(requiredDeps, *cfg.ShakeArmService)
	}
	if cfg.ScoopShakeService != nil && *cfg.ScoopShakeService != "" {
		requiredDeps = append(requiredDeps, *cfg.ScoopShakeService)
	}
	for i, bin := range cfg.Bins {
		if bin.Name == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'name' field is required", path, i)
		}
		if bin.ServingDepthMM < 0 {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'serving-depth-mm' must be non-negative, got %v", path, i, bin.ServingDepthMM)
		}
		if bin.GramsPerServing <= 0 {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'grams-per-serving' must be positive, got %v", path, i, bin.GramsPerServing)
		}
		if bin.Category == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'category' field is required", path, i)
		}
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

	bins              map[int]*grabberBinSwitches
	droppingPose      spatialmath.Pose
	bowlHoverPose     spatialmath.Pose
	arm               arm.Arm
	gripper           gripper.Gripper
	binImagingCam     camera.Camera
	leftHome          sw.Switch
	shakeArmService   genericservice.Service
	scoopShakeService genericservice.Service
	motionService     motion.Service
	fsService         framesystem.Service

	assetsMu   sync.Mutex
	assetsDir  string
	binMesh    *spatialmath.Mesh
	worldState *referenceframe.WorldState
	zones      *segmentation.ZonesResult

	calibrationMu         sync.Mutex
	gripperCalibrated     bool
	closedGripperHeightMM float64
	openGripperWidthMM    float64

	fileSaver *fileio.FileSaver
}

type grabberBinSwitches struct {
	name           string
	hoverPose      spatialmath.Pose
	servingDepthMM float64
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
		bins:       make(map[int]*grabberBinSwitches),
	}

	droppingPt := r3.Vector{X: conf.DroppingPose.X, Y: conf.DroppingPose.Y, Z: conf.DroppingPose.Z}
	s.droppingPose = spatialmath.NewPose(droppingPt, conf.DroppingPose.Orientation)
	hoverHeight := conf.BowlHoverHeightMM
	if hoverHeight == 0 {
		hoverHeight = defaultBowlHoverHeightMM
	}
	s.bowlHoverPose = spatialmath.NewPose(
		r3.Vector{X: droppingPt.X, Y: droppingPt.Y, Z: droppingPt.Z + hoverHeight},
		conf.DroppingPose.Orientation,
	)

	armComponent, err := arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, fmt.Errorf("failed to get left arm '%s': %w", conf.Arm, err)
	}
	s.arm = armComponent

	gripperComponent, err := gripper.FromProvider(deps, conf.Gripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get left gripper '%s': %w", conf.Gripper, err)
	}
	s.gripper = gripperComponent

	binImagingCam, err := camera.FromProvider(deps, conf.BinImagingCam)
	if err != nil {
		return nil, fmt.Errorf("failed to get bin-imaging-cam '%s': %w", conf.BinImagingCam, err)
	}
	s.binImagingCam = binImagingCam

	leftHomeSwitch, err := sw.FromProvider(deps, conf.LeftHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get left-home switch '%s': %w", conf.LeftHome, err)
	}
	s.leftHome = leftHomeSwitch

	for _, binCfg := range conf.Bins {
		servingDepth := binCfg.ServingDepthMM
		if servingDepth == 0 {
			servingDepth = DefaultServingDepthMM
		}
		s.bins[binCfg.ZoneID] = &grabberBinSwitches{
			name:           binCfg.Name,
			servingDepthMM: servingDepth,
		}
	}

	motionSvc, err := motion.FromProvider(deps, conf.MotionService)
	if err != nil {
		return nil, fmt.Errorf("failed to get motion service '%s': %w", conf.MotionService, err)
	}
	s.motionService = motionSvc

	fsSvc, err := framesystem.FromProvider(deps)
	if err != nil {
		return nil, fmt.Errorf("failed to get framesystem service: %w", err)
	}
	s.fsService = fsSvc

	if conf.ShakeArmService != nil && *conf.ShakeArmService != "" {
		shakeArmService, err := genericservice.FromProvider(deps, *conf.ShakeArmService)
		if err != nil {
			return nil, fmt.Errorf("failed to get shake-arm-service '%s': %w", *conf.ShakeArmService, err)
		}
		s.shakeArmService = shakeArmService
	}

	if conf.ScoopShakeService != nil && *conf.ScoopShakeService != "" {
		scoopShakeService, err := genericservice.FromProvider(deps, *conf.ScoopShakeService)
		if err != nil {
			return nil, fmt.Errorf("failed to get scoop-shake-service '%s': %w", *conf.ScoopShakeService, err)
		}
		s.scoopShakeService = scoopShakeService
	}
	if conf.AssetsDir != "" {
		s.assetsDir = conf.AssetsDir
	} else {
		s.assetsDir = "/home/viam/assets"
	}

	s.fileSaver = fileio.NewFileSaver(logger, defaultPlanCaptureDir)

	calibration, err := s.runGripperCalibration(ctx)
	if err != nil {
		return nil, fmt.Errorf("startup gripper calibration: %w", err)
	}
	s.setGripperCalibration(calibration)
	s.logger.Infof(
		"Grabber controls initialized with %d bins; closed gripper height = %.2f mm, open gripper width = %.2f mm",
		len(s.bins), calibration.closedHeightMM, calibration.openWidthMM,
	)
	return s, nil
}

func (s *grabberControls) Name() resource.Name {
	return s.name
}

func (s *grabberControls) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *grabberControls) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["get_from_bin"]; ok {
		return s.doGetFromBin(ctx, cmd)
	}

	if _, ok := cmd["reset"]; ok {
		return s.reset(ctx)
	}

	if _, ok := cmd["get_ingredients"]; ok {
		ingredients := make([]map[string]any, 0, len(s.cfg.Bins))
		for _, bin := range s.cfg.Bins {
			ingredients = append(ingredients, map[string]any{
				"name":              bin.Name,
				"grams_per_serving": bin.GramsPerServing,
				"category":          bin.Category,
				"zone_id":           bin.ZoneID,
			})
		}
		return map[string]any{"ingredients": ingredients}, nil
	}

	if _, ok := cmd["calibrate"]; ok {
		return s.calibrateClosedGripperHeight(ctx)
	}

	if _, ok := cmd["get_gripper_calibration"]; ok {
		return s.getGripperCalibration()
	}

	return nil, fmt.Errorf("unknown command, expected 'get_from_bin', 'reset', 'get_ingredients', 'calibrate', or 'get_gripper_calibration' field")
}

func (s *grabberControls) getGripperCalibration() (map[string]interface{}, error) {
	s.calibrationMu.Lock()
	calibrated := s.gripperCalibrated
	closedHeightMM := s.closedGripperHeightMM
	openWidthMM := s.openGripperWidthMM
	s.calibrationMu.Unlock()
	if !calibrated {
		return nil, fmt.Errorf("gripper not calibrated, run calibrate command first")
	}
	orient := s.cfg.BinHoverOrientation
	bins := make([]map[string]interface{}, 0, len(s.cfg.Bins))
	for _, binCfg := range s.cfg.Bins {
		bins = append(bins, map[string]interface{}{
			"zone_id":           binCfg.ZoneID,
			"hover_x_offset_mm": binCfg.HoverXOffsetMM,
			"hover_y_offset_mm": binCfg.HoverYOffsetMM,
		})
	}
	return map[string]interface{}{
		"closed_gripper_to_arm_base_height_mm": closedHeightMM,
		"open_gripper_width_mm":                openWidthMM,
		"bin_hover_height_mm":                  s.cfg.BinHoverHeightMM,
		"bin_hover_orientation": map[string]float64{
			"ox":    orient.OX,
			"oy":    orient.OY,
			"oz":    orient.OZ,
			"theta": orient.Theta,
		},
		"bins":        bins,
		"x_offset_mm": s.cfg.XOffsetMM,
		"y_offset_mm": s.cfg.YOffsetMM,
	}, nil
}

func (s *grabberControls) calibrateClosedGripperHeight(ctx context.Context) (map[string]interface{}, error) {
	calibration, err := s.runGripperCalibration(ctx)
	if err != nil {
		return nil, err
	}

	s.setGripperCalibration(calibration)
	s.logger.Infof(
		"calibration: closed gripper height = %.2f mm, open gripper width = %.2f mm",
		calibration.closedHeightMM, calibration.openWidthMM,
	)
	return map[string]interface{}{
		"closed_gripper_to_arm_base_height_mm": calibration.closedHeightMM,
		"open_gripper_width_mm":                calibration.openWidthMM,
	}, nil
}

type gripperCalibration struct {
	closedHeightMM float64
	openWidthMM    float64
}

func (s *grabberControls) runGripperCalibration(ctx context.Context) (gripperCalibration, error) {
	closedHeightMM, err := s.measureClosedGripperHeight(ctx)
	if err != nil {
		return gripperCalibration{}, err
	}
	openWidthMM, err := s.measureOpenGripperWidth(ctx)
	if err != nil {
		return gripperCalibration{}, err
	}
	return gripperCalibration{
		closedHeightMM: closedHeightMM,
		openWidthMM:    openWidthMM,
	}, nil
}

func (s *grabberControls) setGripperCalibration(calibration gripperCalibration) {
	s.calibrationMu.Lock()
	s.closedGripperHeightMM = calibration.closedHeightMM
	s.openGripperWidthMM = calibration.openWidthMM
	s.gripperCalibrated = true
	s.calibrationMu.Unlock()
}

func (s *grabberControls) measureClosedGripperHeight(ctx context.Context) (float64, error) {
	if _, err := s.gripper.Grab(ctx, nil); err != nil {
		return 0, fmt.Errorf("close gripper: %w", err)
	}

	geoms, err := s.gripper.Geometries(ctx, nil)
	if err != nil {
		return 0, fmt.Errorf("read gripper geometries: %w", err)
	}
	if len(geoms) == 0 {
		return 0, fmt.Errorf("gripper returned no geometries")
	}

	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return 0, fmt.Errorf("build frame system: %w", err)
	}

	startInputs := referenceframe.NewZeroInputs(fs)

	armCurrentInputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		return 0, fmt.Errorf("getting arm current inputs: %w", err)
	}
	if len(armCurrentInputs) > 0 {
		startInputs[s.cfg.Arm] = armCurrentInputs
	}

	gf := referenceframe.NewGeometriesInFrame(s.cfg.Gripper, geoms)
	tf, err := fs.Transform(startInputs.ToLinearInputs(), gf, s.cfg.Arm)
	if err != nil {
		return 0, fmt.Errorf("transform gripper geometries to arm frame: %w", err)
	}
	geomsInArm, ok := tf.(*referenceframe.GeometriesInFrame)
	if !ok {
		return 0, fmt.Errorf("expected GeometriesInFrame after transform, got %T", tf)
	}

	return maxGeometryZ(geomsInArm.Geometries())
}

func (s *grabberControls) measureOpenGripperWidth(ctx context.Context) (float64, error) {
	if err := s.gripper.Open(ctx, nil); err != nil {
		return 0, fmt.Errorf("open gripper: %w", err)
	}

	geoms, err := s.gripper.Geometries(ctx, nil)
	if err != nil {
		return 0, fmt.Errorf("read gripper geometries: %w", err)
	}
	if len(geoms) == 0 {
		return 0, fmt.Errorf("gripper returned no geometries")
	}

	return geometryXSpan(geoms)
}

func (s *grabberControls) calibratedClosedGripperHeight() (float64, error) {
	s.calibrationMu.Lock()
	defer s.calibrationMu.Unlock()
	if !s.gripperCalibrated {
		return 0, fmt.Errorf("gripper not calibrated, run calibrate command first")
	}
	return s.closedGripperHeightMM, nil
}

func maxGeometryZ(geoms []spatialmath.Geometry) (float64, error) {
	maxZ := math.Inf(-1)
	for _, g := range geoms {
		for _, pt := range g.ToPoints(1.0) {
			if pt.Z > maxZ {
				maxZ = pt.Z
			}
		}
	}
	if math.IsInf(maxZ, -1) {
		return 0, fmt.Errorf("geometries produced no points")
	}
	return maxZ, nil
}

func geometryXSpan(geoms []spatialmath.Geometry) (float64, error) {
	minX := math.Inf(1)
	maxX := math.Inf(-1)
	for _, g := range geoms {
		for _, pt := range g.ToPoints(1.0) {
			if pt.X < minX {
				minX = pt.X
			}
			if pt.X > maxX {
				maxX = pt.X
			}
		}
	}
	if math.IsInf(maxX, -1) {
		return 0, fmt.Errorf("geometries produced no points")
	}
	return math.Abs(maxX - minX), nil
}

func (s *grabberControls) applyXYOffset(pose spatialmath.Pose) spatialmath.Pose {
	if s.cfg.XOffsetMM == 0 && s.cfg.YOffsetMM == 0 {
		return pose
	}
	pt := pose.Point()
	return spatialmath.NewPose(
		r3.Vector{X: pt.X + s.cfg.XOffsetMM, Y: pt.Y + s.cfg.YOffsetMM, Z: pt.Z},
		pose.Orientation(),
	)
}

// loadAssets lazy-loads the bin mesh and zones from disk. Safe to call on every grab
// since it is a no-op once both are loaded.
func (s *grabberControls) loadAssets() error {
	s.assetsMu.Lock()
	defer s.assetsMu.Unlock()

	if s.binMesh == nil {
		mesh, err := spatialmath.NewMeshFromPLYFile(s.assetsDir + "/mesh.ply")
		if err != nil {
			return fmt.Errorf("bin mesh not available at %s, run setup first: %w", s.assetsDir+"/mesh.ply", err)
		}
		s.binMesh = mesh

		octree, err := pointcloud.NewFromMesh(s.binMesh)
		if err != nil {
			return fmt.Errorf("failed to convert bin mesh to octree: %w", err)
		}

		ws, err := referenceframe.NewWorldState(
			[]*referenceframe.GeometriesInFrame{
				referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{octree}),
			},
			nil,
		)
		if err != nil {
			return fmt.Errorf("failed to build world state: %w", err)
		}
		s.worldState = ws
	}

	if s.zones == nil {
		zones, err := segmentation.LoadZones(s.assetsDir + "/zones.json")
		if err != nil {
			return fmt.Errorf("zones not available at %s, run setup first: %w", s.assetsDir+"/zones.json", err)
		}
		s.zones = zones
		for _, binCfg := range s.cfg.Bins {
			z, ok := zones.ZoneByID(binCfg.ZoneID)
			if !ok {
				return fmt.Errorf("zone %d not found for bin %q", binCfg.ZoneID, binCfg.Name)
			}
			hoverPose, err := BinHoverPose(
				z,
				s.zones.ZMean,
				s.cfg.BinHoverHeightMM,
				binCfg.HoverXOffsetMM,
				binCfg.HoverYOffsetMM,
				s.cfg.BinHoverOrientation,
			)
			if err != nil {
				return fmt.Errorf("bin %q: %w", binCfg.Name, err)
			}
			s.bins[binCfg.ZoneID].hoverPose = hoverPose
		}
	}

	return nil
}

func (s *grabberControls) getZone(zoneID int) (*segmentation.Zone, error) {
	for i := range s.zones.Zones {
		if s.zones.Zones[i].ID == zoneID {
			return &s.zones.Zones[i], nil
		}
	}
	return nil, fmt.Errorf("zone %d not found in loaded zones", zoneID)
}

func (s *grabberControls) computeGrabPose(zone *segmentation.Zone, foodLevelMM, servingDepthMM float64) (spatialmath.Pose, error) {
	closedGripperHeightMM, err := s.calibratedClosedGripperHeight()
	if err != nil {
		return nil, err
	}
	pose, err := ComputeGrabPose(zone, foodLevelMM, servingDepthMM, closedGripperHeightMM, s.cfg.BinHoverOrientation)
	if err != nil {
		return nil, err
	}
	return s.applyXYOffset(pose), nil
}

// logBinImagingCamPlaneFit snapshots the bin-imaging camera point cloud, culls
// it to the zone's XY rect, and logs the mean distance from those points to
// the zone's fitted floor plane.
//
// Output format (single info line, mm units):
//
//	zone N plane-fit: K/T pts (P%) in zone XY rect; distance-to-floor-plane:
//	  mean|d|=A.AA mean_signed=S.SS (+above/-below) range=[MIN, MAX] tilt=T.TTdeg; capture+stats E.EEs
//
// Failure modes (camera read, empty cloud, no in-bounds points) are logged at
// warn level and do not propagate: this is observational only.
func (s *grabberControls) getBinFoodLevel(ctx context.Context, zone *segmentation.Zone) (float64, error) {
	if s.binImagingCam == nil {
		return 0, fmt.Errorf("bin-imaging-cam is not set")
	}
	start := time.Now()
	camPc, err := s.binImagingCam.NextPointCloud(ctx, nil)
	if err != nil {
		s.logger.Warnf("zone %d: bin-imaging-cam NextPointCloud failed after %.2fs: %v",
			zone.ID, time.Since(start).Seconds(), err)
		return 0, fmt.Errorf("bin-imaging-cam NextPointCloud failed after %.2fs: %w", time.Since(start).Seconds(), err)
	}

	camName := s.binImagingCam.Name().ShortName()
	pc, err := s.fsService.TransformPointCloud(ctx, camPc, camName, referenceframe.World)
	if err != nil {
		s.logger.Warnf("zone %d: failed to transform bin-imaging-cam point cloud from %q to world frame: %v",
			zone.ID, camName, err)
		return 0, fmt.Errorf("failed to transform bin-imaging-cam point cloud from %q to world frame: %w", camName, err)
	}

	stats, _ := segmentation.ZonePlaneFitStats(pc, zone, s.logger)
	if stats.PointsInBounds == 0 {
		s.logger.Warnf("zone %d: 0/%d bin-imaging-cam points fell inside zone XY rect [%.1f,%.1f]x[%.1f,%.1f] (in_x=%d, in_y=%d) -- camera pointed away or frames don't match",
			zone.ID, stats.PointsTotal, zone.MinX, zone.MaxX, zone.MinY, zone.MaxY,
			stats.PointsInsideX, stats.PointsInsideY)
		return 0, fmt.Errorf("0/%d bin-imaging-cam points fell inside zone XY rect [%.1f,%.1f]x[%.1f,%.1f] (in_x=%d, in_y=%d) -- camera pointed away or frames don't match",
			stats.PointsTotal, zone.MinX, zone.MaxX, zone.MinY, zone.MaxY, stats.PointsInsideX, stats.PointsInsideY)
	}
	pct := 0.0
	if stats.PointsTotal > 0 {
		pct = 100 * float64(stats.PointsInBounds) / float64(stats.PointsTotal)
	}
	s.logger.Debugf(
		"zone %d plane-fit: %d/%d pts (%.2f%%) in zone XY rect; distance-to-floor-plane (mm): "+
			"mean|d|=%.2f mean_signed=%+.2f (+above/-below) range=[%+.2f, %+.2f] tilt=%.2fdeg; capture+stats %.2fs",
		zone.ID, stats.PointsInBounds, stats.PointsTotal, pct,
		stats.MeanAbsDistanceMM, stats.MeanSignedDistanceMM,
		stats.MinSignedDistanceMM, stats.MaxSignedDistanceMM,
		zone.Plane.TiltDeg(),
		time.Since(start).Seconds(),
	)
	foodLevelMM, err := FoodLevelMMFromPlaneFitStats(zone, stats)
	if err != nil {
		s.logger.Errorf("distance to plane is wrong: %v, pointsInBounds: %d, planePoint: %v", err, stats.PointsInBounds, r3.Vector{X: zone.Plane.Point[0], Y: zone.Plane.Point[1], Z: zone.Plane.Point[2]})
		return 0, err
	}
	return foodLevelMM, nil
}

func (s *grabberControls) doGetFromBin(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if err := s.loadAssets(); err != nil {
		return nil, err
	}

	zoneIDVal := cmd["get_from_bin"]
	var zoneID int
	switch v := zoneIDVal.(type) {
	case int:
		zoneID = v
	case float64:
		zoneID = int(v)
	default:
		return nil, fmt.Errorf("'get_from_bin' must be an int zone ID, got %T", zoneIDVal)
	}

	var depthOffsetMM float64
	if v, ok := cmd["depth-offset-mm"]; ok {
		switch x := v.(type) {
		case float64:
			depthOffsetMM = x
		case int:
			depthOffsetMM = float64(x)
		default:
			return nil, fmt.Errorf("'depth-offset-mm' must be a number, got %T", v)
		}
		if depthOffsetMM < 0 {
			return nil, fmt.Errorf("'depth-offset-mm' must be non-negative, got %v", depthOffsetMM)
		}
	}

	buildID, _ := cmd["build_id"].(string)

	bin, ok := s.bins[zoneID]
	if !ok {
		return nil, fmt.Errorf("zone %d not found in configuration", zoneID)
	}

	zone, err := s.getZone(zoneID)
	if err != nil {
		return nil, err
	}

	binFoodLevelMM, err := s.getBinFoodLevel(ctx, zone)
	if err != nil {
		return nil, err
	}

	s.logger.Infof("Planning get_from_bin for bin '%s' (zone %d, depth-offset %.1fmm)", bin.name, zoneID, depthOffsetMM)
	plan, err := s.planGrab(ctx, bin, zoneID, zone, binFoodLevelMM, buildID)
	if err != nil {
		return nil, err
	}
	s.logger.Infof("Executing get_from_bin for bin '%s' (zone %d, %d steps)", bin.name, zoneID, len(plan.Steps))

	if err := s.executeGrab(ctx, plan); err != nil {
		return nil, err
	}
	s.logger.Infof("Successfully completed get_from_bin for bin '%s' (zone %d)", bin.name, zoneID)

	return map[string]interface{}{
		"success":         true,
		"bin":             bin.name,
		"depth-offset-mm": depthOffsetMM,
		"message":         fmt.Sprintf("Successfully grabbed from bin '%s' and moved to bowl", bin.name),
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
	if err := s.fileSaver.Close(); err != nil {
		s.logger.Warnw("FileSaver close error", "err", err)
	}
	return nil
}
