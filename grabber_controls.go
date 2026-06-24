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

// GrabberControlsZoneConfig describes a single grabbable zone: its zone ID and
// any hover-position offsets. All food/ingredient metadata (name, category,
// grams-per-serving, serving depth) lives in the build coordinator and is
// passed in at grab time, not stored here.
type GrabberControlsZoneConfig struct {
	ZoneID int `json:"zone-id"`
	// HoverXOffsetMM and HoverYOffsetMM shift the zone hover XY position from
	// the zone plane point (world frame, mm).
	HoverXOffsetMM float64 `json:"hover-x-offset-mm,omitempty"`
	HoverYOffsetMM float64 `json:"hover-y-offset-mm,omitempty"`
}

type BowlDropPose struct {
	X           float64                               `json:"x"`
	Y           float64                               `json:"y"`
	Z           float64                               `json:"z"`
	Orientation *spatialmath.OrientationVectorDegrees `json:"orientation"`
}

type GrabberControlsConfig struct {
	Zones                             []GrabberControlsZoneConfig           `json:"zones"`
	BinHoverHeightMM                  float64                               `json:"bin-hover-height-mm"`
	BinHoverOrientation               *spatialmath.OrientationVectorDegrees `json:"bin-hover-orientation,omitempty"`
	EnableBinClearance                bool                                  `json:"enable-bin-clearance,omitempty"`
	BinClearanceXOffsetMM             float64                               `json:"bin-clearance-x-offset-mm,omitempty"`
	BinClearanceZOffsetMM             float64                               `json:"bin-clearance-z-offset-mm,omitempty"`
	ClearanceLineToleranceMM          float64                               `json:"clearance-line-tolerance-mm,omitempty"`
	ClearanceOrientationToleranceDegs float64                               `json:"clearance-orientation-tolerance-degs,omitempty"`
	DroppingPose                      *BowlDropPose                         `json:"dropping-pose"`
	BowlHoverHeightMM                 float64                               `json:"bowl-hover-height-mm,omitempty"`
	Arm                               string                                `json:"arm"`
	Gripper                           string                                `json:"gripper"`
	MotionService                     string                                `json:"motion-service"`
	LeftHome                          string                                `json:"left-home"`
	ShakeArmService                   *string                               `json:"shake-arm-service,omitempty"`
	ScoopShakeService                 *string                               `json:"scoop-shake-service,omitempty"`
	AssetsDir                         string                                `json:"assets-dir"`
	GrabLineToleranceMM               float64                               `json:"grab-line-tolerance-mm,omitempty"`
	GrabOrientationToleranceDegs      float64                               `json:"grab-orientation-tolerance-degs,omitempty"`
	BinImagingCam                     string                                `json:"bin-imaging-cam"`
}

func (cfg *GrabberControlsConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.Zones) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "zones")
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
	seenZoneIDs := make(map[int]bool, len(cfg.Zones))
	for i, zone := range cfg.Zones {
		if seenZoneIDs[zone.ZoneID] {
			return nil, nil, fmt.Errorf("%s.zones[%d]: duplicate zone-id %d", path, i, zone.ZoneID)
		}
		seenZoneIDs[zone.ZoneID] = true
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

	zones             map[int]*grabberZone
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

	assetsMu    sync.Mutex
	assetsDir   string
	binMesh     *spatialmath.Mesh
	worldState  *referenceframe.WorldState
	loadedZones *segmentation.ZonesResult

	calibrationMu         sync.Mutex
	gripperCalibrated     bool
	closedGripperHeightMM float64
	openGripperWidthMM    float64
	openGripperDepthMM    float64

	fileSaver *fileio.FileSaver
}

type grabberZone struct {
	hoverPose spatialmath.Pose
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
		zones:      make(map[int]*grabberZone),
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

	for _, zoneCfg := range conf.Zones {
		s.zones[zoneCfg.ZoneID] = &grabberZone{}
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
		"Grabber controls initialized with %d zones; closed gripper height = %.2f mm, open gripper width = %.2f mm, open gripper depth = %.2f mm",
		len(s.zones), calibration.closedHeightMM, calibration.openWidthMM, calibration.openDepthMM,
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

	if _, ok := cmd["calibrate"]; ok {
		calibration, err := s.runGripperCalibration(ctx)
		if err != nil {
			return nil, err
		}
		s.setGripperCalibration(calibration)
		return map[string]any{
			"closed_gripper_to_arm_base_height_mm": calibration.closedHeightMM,
			"open_gripper_width_mm":                calibration.openWidthMM,
			"open_gripper_depth_mm":                calibration.openDepthMM,
		}, nil
	}

	if _, ok := cmd["get_gripper_calibration"]; ok {
		return s.getGripperCalibration()
	}

	if _, ok := cmd["get_zone_config"]; ok {
		return s.getZoneConfig(), nil
	}

	return nil, fmt.Errorf("unknown command, expected 'get_from_bin', 'reset', 'calibrate', 'get_gripper_calibration', or 'get_zone_config' field")
}

func (s *grabberControls) getGripperCalibration() (map[string]interface{}, error) {
	s.calibrationMu.Lock()
	calibrated := s.gripperCalibrated
	closedHeightMM := s.closedGripperHeightMM
	openWidthMM := s.openGripperWidthMM
	openDepthMM := s.openGripperDepthMM
	s.calibrationMu.Unlock()
	if !calibrated {
		return nil, fmt.Errorf("gripper not calibrated, run calibrate command first")
	}
	return map[string]interface{}{
		"closed_gripper_to_arm_base_height_mm": closedHeightMM,
		"open_gripper_width_mm":                openWidthMM,
		"open_gripper_depth_mm":                openDepthMM,
	}, nil
}

func (s *grabberControls) getZoneConfig() map[string]interface{} {
	orient := s.cfg.BinHoverOrientation
	zones := make([]map[string]interface{}, 0, len(s.cfg.Zones))
	for _, zoneCfg := range s.cfg.Zones {
		zones = append(zones, map[string]interface{}{
			"zone_id":           zoneCfg.ZoneID,
			"hover_x_offset_mm": zoneCfg.HoverXOffsetMM,
			"hover_y_offset_mm": zoneCfg.HoverYOffsetMM,
		})
	}
	return map[string]interface{}{
		"bin_hover_height_mm": s.cfg.BinHoverHeightMM,
		"bin_hover_orientation": map[string]float64{
			"ox":    orient.OX,
			"oy":    orient.OY,
			"oz":    orient.OZ,
			"theta": orient.Theta,
		},
		"zones": zones,
	}
}

type gripperCalibration struct {
	closedHeightMM float64
	openWidthMM    float64
	openDepthMM    float64
}

func (s *grabberControls) runGripperCalibration(ctx context.Context) (gripperCalibration, error) {
	closedHeightMM, err := s.measureClosedGripperHeight(ctx)
	if err != nil {
		return gripperCalibration{}, err
	}
	openWidthMM, openDepthMM, err := s.measureOpenGripper(ctx)
	if err != nil {
		return gripperCalibration{}, err
	}
	return gripperCalibration{
		closedHeightMM: closedHeightMM,
		openWidthMM:    openWidthMM,
		openDepthMM:    openDepthMM,
	}, nil
}

func (s *grabberControls) setGripperCalibration(calibration gripperCalibration) {
	s.calibrationMu.Lock()
	s.closedGripperHeightMM = calibration.closedHeightMM
	s.openGripperWidthMM = calibration.openWidthMM
	s.openGripperDepthMM = calibration.openDepthMM
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

func (s *grabberControls) measureOpenGripper(ctx context.Context) (widthMM, depthMM float64, err error) {
	if err := s.gripper.Open(ctx, nil); err != nil {
		return 0, 0, fmt.Errorf("open gripper: %w", err)
	}

	geoms, err := s.gripper.Geometries(ctx, nil)
	if err != nil {
		return 0, 0, fmt.Errorf("read gripper geometries: %w", err)
	}
	if len(geoms) == 0 {
		return 0, 0, fmt.Errorf("gripper returned no geometries")
	}

	widthMM, err = geometryXSpan(geoms)
	if err != nil {
		return 0, 0, err
	}
	depthMM, err = geometryYSpan(geoms)
	if err != nil {
		return 0, 0, err
	}
	return widthMM, depthMM, nil
}

func (s *grabberControls) calibratedClosedGripperHeight() (float64, error) {
	s.calibrationMu.Lock()
	defer s.calibrationMu.Unlock()
	if !s.gripperCalibrated {
		return 0, fmt.Errorf("gripper not calibrated, run calibrate command first")
	}
	return s.closedGripperHeightMM, nil
}

func (s *grabberControls) calibratedOpenGripperDims() (widthMM, depthMM float64, err error) {
	s.calibrationMu.Lock()
	defer s.calibrationMu.Unlock()
	if !s.gripperCalibrated {
		return 0, 0, fmt.Errorf("gripper not calibrated, run calibrate command first")
	}
	return s.openGripperWidthMM, s.openGripperDepthMM, nil
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

func geometryYSpan(geoms []spatialmath.Geometry) (float64, error) {
	minY := math.Inf(1)
	maxY := math.Inf(-1)
	for _, g := range geoms {
		for _, pt := range g.ToPoints(1.0) {
			if pt.Y < minY {
				minY = pt.Y
			}
			if pt.Y > maxY {
				maxY = pt.Y
			}
		}
	}
	if math.IsInf(maxY, -1) {
		return 0, fmt.Errorf("geometries produced no points")
	}
	return math.Abs(maxY - minY), nil
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

	if s.loadedZones == nil {
		zones, err := segmentation.LoadZones(s.assetsDir + "/zones.json")
		if err != nil {
			return fmt.Errorf("zones not available at %s, run setup first: %w", s.assetsDir+"/zones.json", err)
		}
		s.loadedZones = zones
		for _, zoneCfg := range s.cfg.Zones {
			z, ok := zones.ZoneByID(zoneCfg.ZoneID)
			if !ok {
				return fmt.Errorf("zone %d not found in loaded zones", zoneCfg.ZoneID)
			}
			hoverPose, err := BinHoverPose(
				z,
				s.loadedZones.ZMean,
				s.cfg.BinHoverHeightMM,
				zoneCfg.HoverXOffsetMM,
				zoneCfg.HoverYOffsetMM,
				s.cfg.BinHoverOrientation,
			)
			if err != nil {
				return fmt.Errorf("zone %d: %w", zoneCfg.ZoneID, err)
			}
			s.zones[zoneCfg.ZoneID].hoverPose = hoverPose
		}
	}

	return nil
}

func (s *grabberControls) getZone(zoneID int) (*segmentation.Zone, error) {
	for i := range s.loadedZones.Zones {
		if s.loadedZones.Zones[i].ID == zoneID {
			return &s.loadedZones.Zones[i], nil
		}
	}
	return nil, fmt.Errorf("zone %d not found in loaded zones", zoneID)
}

func (s *grabberControls) computeGrabPose(zone *segmentation.Zone, foodPoint r3.Vector, servingDepthMM float64) (spatialmath.Pose, error) {
	closedGripperHeightMM, err := s.calibratedClosedGripperHeight()
	if err != nil {
		return nil, err
	}
	return ComputeGrabPose(zone, foodPoint, servingDepthMM, closedGripperHeightMM, s.cfg.BinHoverOrientation), nil
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
func (s *grabberControls) getBinFoodPoint(ctx context.Context, zone *segmentation.Zone) (FoodPoint, error) {
	if s.binImagingCam == nil {
		return FoodPoint{}, fmt.Errorf("bin-imaging-cam is not set")
	}
	start := time.Now()
	camPc, err := s.binImagingCam.NextPointCloud(ctx, nil)
	if err != nil {
		s.logger.Warnf("zone %d: bin-imaging-cam NextPointCloud failed after %.2fs: %v",
			zone.ID, time.Since(start).Seconds(), err)
		return FoodPoint{}, fmt.Errorf("bin-imaging-cam NextPointCloud failed after %.2fs: %w", time.Since(start).Seconds(), err)
	}

	camName := s.binImagingCam.Name().ShortName()
	pc, err := s.fsService.TransformPointCloud(ctx, camPc, camName, referenceframe.World)
	if err != nil {
		s.logger.Warnf("zone %d: failed to transform bin-imaging-cam point cloud from %q to world frame: %v",
			zone.ID, camName, err)
		return FoodPoint{}, fmt.Errorf("failed to transform bin-imaging-cam point cloud from %q to world frame: %w", camName, err)
	}

	stats, _ := segmentation.ZonePlaneFitStats(pc, zone, s.logger)
	if stats.PointsInBounds == 0 {
		s.logger.Warnf("zone %d: 0/%d bin-imaging-cam points fell inside zone XY rect [%.1f,%.1f]x[%.1f,%.1f] (in_x=%d, in_y=%d) -- camera pointed away or frames don't match",
			zone.ID, stats.PointsTotal, zone.MinX, zone.MaxX, zone.MinY, zone.MaxY,
			stats.PointsInsideX, stats.PointsInsideY)
		return FoodPoint{}, fmt.Errorf("0/%d bin-imaging-cam points fell inside zone XY rect [%.1f,%.1f]x[%.1f,%.1f] (in_x=%d, in_y=%d) -- camera pointed away or frames don't match",
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

	// Restrict the search to cells where the open gripper fits inside the zone
	// (plus padding), then take the highest remaining cell as the grab target.
	// The calibrated open dimensions are in the gripper's local frame, so map
	// them onto world X/Y using the grab orientation before masking (the
	// gripper may be rotated so its width runs along world Y, not world X).
	// See GripperWorldExtents for the assumptions this relies on.
	openWidthMM, openDepthMM, err := s.calibratedOpenGripperDims()
	if err != nil {
		return FoodPoint{}, err
	}
	xExtentMM, yExtentMM := GripperWorldExtents(s.cfg.BinHoverOrientation, openWidthMM, openDepthMM)
	stats.HeightMap.MaskGripperOverflow(xExtentMM, yExtentMM)

	foodPoint, err := FoodPointFromPlaneFitStats(zone, stats)
	if err != nil {
		s.logger.Errorf("no valid food point in zone %d: %v, pointsInBounds: %d, planePoint: %v",
			zone.ID, err, stats.PointsInBounds, r3.Vector{X: zone.Plane.Point[0], Y: zone.Plane.Point[1], Z: zone.Plane.Point[2]})
		return FoodPoint{}, err
	}
	s.logger.Infof("zone %d highest food point: (%.1f, %.1f, %.1f) level=%.1f mm above floor (%.1f, %.1f, %.1f)",
		zone.ID, foodPoint.Point.X, foodPoint.Point.Y, foodPoint.Point.Z, foodPoint.LevelMM,
		foodPoint.FloorPoint.X, foodPoint.FloorPoint.Y, foodPoint.FloorPoint.Z)
	return foodPoint, nil
}

func (s *grabberControls) doGetFromBin(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if err := s.loadAssets(); err != nil {
		return nil, err
	}

	zoneID, name, servingDepthMM, err := parseGetFromBinArgs(cmd["get_from_bin"])
	if err != nil {
		return nil, err
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

	zoneCfg, ok := s.zones[zoneID]
	if !ok {
		return nil, fmt.Errorf("zone %d not found in configuration", zoneID)
	}

	label := name
	if label == "" {
		label = fmt.Sprintf("zone %d", zoneID)
	}

	zone, err := s.getZone(zoneID)
	if err != nil {
		return nil, err
	}

	binFoodPoint, err := s.getBinFoodPoint(ctx, zone)
	if err != nil {
		return nil, err
	}

	s.logger.Infof("Planning get_from_bin for '%s' (zone %d, serving-depth %.1fmm, depth-offset %.1fmm)", label, zoneID, servingDepthMM, depthOffsetMM)
	plan, err := s.planGrab(ctx, zoneCfg, label, zoneID, zone, binFoodPoint.Point, servingDepthMM, buildID)
	if err != nil {
		return nil, err
	}
	s.logger.Infof("Executing get_from_bin for '%s' (zone %d, %d steps)", label, zoneID, len(plan.Steps))

	if err := s.executeGrab(ctx, plan); err != nil {
		return nil, err
	}
	s.logger.Infof("Successfully completed get_from_bin for '%s' (zone %d)", label, zoneID)

	return map[string]interface{}{
		"success":         true,
		"bin":             label,
		"zone-id":         zoneID,
		"depth-offset-mm": depthOffsetMM,
		"message":         fmt.Sprintf("Successfully grabbed from '%s' and moved to bowl", label),
	}, nil
}

// parseGetFromBinArgs reads the value of the "get_from_bin" command. It accepts
// either a bare zone ID (legacy form) or a map carrying the zone ID along with
// optional ingredient metadata supplied by the build coordinator:
//
//	{"zone-id": <int>, "name": <string>, "serving-depth-mm": <number>}
//
// name is used only for logging. serving-depth-mm defaults to
// DefaultServingDepthMM when omitted or zero.
func parseGetFromBinArgs(v interface{}) (zoneID int, name string, servingDepthMM float64, err error) {
	servingDepthMM = DefaultServingDepthMM
	switch val := v.(type) {
	case int:
		zoneID = val
	case float64:
		zoneID = int(val)
	case map[string]interface{}:
		zid, ok := val["zone-id"]
		if !ok {
			return 0, "", 0, fmt.Errorf("'get_from_bin' map is missing required 'zone-id' field")
		}
		switch z := zid.(type) {
		case int:
			zoneID = z
		case float64:
			zoneID = int(z)
		default:
			return 0, "", 0, fmt.Errorf("'get_from_bin.zone-id' must be a number, got %T", zid)
		}
		if n, ok := val["name"].(string); ok {
			name = n
		}
		if sd, ok := val["serving-depth-mm"]; ok {
			switch d := sd.(type) {
			case float64:
				servingDepthMM = d
			case int:
				servingDepthMM = float64(d)
			default:
				return 0, "", 0, fmt.Errorf("'get_from_bin.serving-depth-mm' must be a number, got %T", sd)
			}
		}
	default:
		return 0, "", 0, fmt.Errorf("'get_from_bin' must be a zone ID or a map of grab arguments, got %T", v)
	}
	if servingDepthMM <= 0 {
		servingDepthMM = DefaultServingDepthMM
	}
	return zoneID, name, servingDepthMM, nil
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
