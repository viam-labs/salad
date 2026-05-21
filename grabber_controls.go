package salad

import (
	"context"
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
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

	"salad/segmentation"
)

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
	AssetsDir                         string                                `json:"assets-dir"`
	XOffsetMM                         float64                               `json:"x-offset-mm,omitempty"`
	YOffsetMM                         float64                               `json:"y-offset-mm,omitempty"`
	SavePlans                         bool                                  `json:"save-plans,omitempty"`
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

	for i, bin := range cfg.Bins {
		if bin.Name == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'name' field is required", path, i)
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

	bins            map[int]*grabberBinSwitches
	droppingPose    spatialmath.Pose
	bowlHoverPose   spatialmath.Pose
	arm             arm.Arm
	gripper         gripper.Gripper
	binImagingCam   camera.Camera
	leftHome        sw.Switch
	shakeArmService genericservice.Service
	motionService   motion.Service
	fsService       framesystem.Service

	assetsMu   sync.Mutex
	assetsDir  string
	binMesh    *spatialmath.Mesh
	worldState *referenceframe.WorldState
	zones      *segmentation.ZonesResult
}

type grabberBinSwitches struct {
	name      string
	hoverPose spatialmath.Pose
}

type grabPlanStep struct {
	Step          string `json:"step"`
	TrajectoryLen int    `json:"trajectory_len"`
	PlanningDurMS int64  `json:"planning_dur_ms"`
	ExecError     string `json:"exec_error,omitempty"`
}

type grabPlanRecord struct {
	StartedAt string         `json:"started_at"`
	BinName   string         `json:"bin_name"`
	ZoneID    int            `json:"zone_id"`
	Steps     []grabPlanStep `json:"steps"`
	Success   bool           `json:"success"`
	Error     string         `json:"error,omitempty"`
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
		s.bins[binCfg.ZoneID] = &grabberBinSwitches{name: binCfg.Name}
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

	if conf.AssetsDir != "" {
		s.assetsDir = conf.AssetsDir
	} else {
		s.assetsDir = "/home/viam/assets"
	}

	s.logger.Infof("Grabber controls initialized with %d bins", len(s.bins))
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

	return nil, fmt.Errorf("unknown command, expected 'get_from_bin' or 'reset' field")
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
			cx, cy, err := z.Centroid()
			if err != nil {
				return fmt.Errorf("bin %q: %w", binCfg.Name, err)
			}
			s.bins[binCfg.ZoneID].hoverPose = spatialmath.NewPose(
				r3.Vector{X: cx, Y: cy, Z: s.zones.ZMean + s.cfg.BinHoverHeightMM},
				s.cfg.BinHoverOrientation,
			)
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

func (s *grabberControls) computeGrabPose(ctx context.Context, zone *segmentation.Zone, foodLevelMM float64) (spatialmath.Pose, error) {
	zonePlane := zone.Plane
	zoneCenterVec := r3.Vector{
		X: zonePlane.Point[0],
		Y: zonePlane.Point[1],
		Z: zonePlane.Point[2],
	}

	zoneNormalVec := r3.Vector{
		X: zonePlane.Normal[0],
		Y: zonePlane.Normal[1],
		Z: zonePlane.Normal[2],
	}
	if foodLevelMM < 35.0 {
		return nil, fmt.Errorf("food level is too low: %.2f mm", foodLevelMM)
	}
	foodHeightPosition := zoneCenterVec.Add(zoneNormalVec.Mul(foodLevelMM))

	// hardcoding for now but want to detect this at some point
	closedGripperToArmBaseHeightMM := 365.0

	// food grab position is 30mm beneath food height
	grabBasePoint := foodHeightPosition.Add(zoneNormalVec.Mul(-30.0))

	idealArmBasePosition := grabBasePoint.Add(zoneNormalVec.Mul(closedGripperToArmBaseHeightMM))

	currentGripperPose, err := s.fsService.GetPose(ctx, s.gripper.Name().Name, referenceframe.World, nil, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get current gripper pose: %w", err)
	}
	gripperBasePoint := currentGripperPose.Pose().Point()

	// Signed distance from the gripper to the zone plane along the (unit) normal.
	// Positive means the gripper is on the +normal side of the plane.
	signedDist := gripperBasePoint.Sub(zoneCenterVec).Dot(zoneNormalVec)
	// Vector pointing from gripperBasePoint to its projection on the plane.
	gripperToPlaneDirection := zoneNormalVec.Mul(-signedDist).Normalize()
	gripperOrientationVec := spatialmath.OrientationVectorDegrees{
		Theta: s.cfg.BinHoverOrientation.Theta,
		OX:    gripperToPlaneDirection.X,
		OY:    gripperToPlaneDirection.Y,
		OZ:    gripperToPlaneDirection.Z,
	}

	return spatialmath.NewPose(idealArmBasePosition, &gripperOrientationVec), nil
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
		return 0, fmt.Errorf("bin-imaging-cam NextPointCloud failed after %.2fs: %v", time.Since(start).Seconds(), err)
	}

	camName := s.binImagingCam.Name().ShortName()
	pc, err := s.fsService.TransformPointCloud(ctx, camPc, camName, referenceframe.World)
	if err != nil {
		s.logger.Warnf("zone %d: failed to transform bin-imaging-cam point cloud from %q to world frame: %v",
			zone.ID, camName, err)
		return 0, fmt.Errorf("failed to transform bin-imaging-cam point cloud from %q to world frame: %v", camName, err)
	}

	stats, _ := segmentation.ZonePlaneFitStats(pc, zone, s.logger)
	if stats.PointsInBounds == 0 {
		s.logger.Warnf("zone %d: 0/%d bin-imaging-cam points fell inside zone XY rect [%.1f,%.1f]x[%.1f,%.1f] (in_x=%d, in_y=%d) -- camera pointed away or frames don't match",
			zone.ID, stats.PointsTotal, zone.MinX, zone.MaxX, zone.MinY, zone.MaxY,
			stats.PointsInsideX, stats.PointsInsideY)
		return 0, fmt.Errorf("0/%d bin-imaging-cam points fell inside zone XY rect [%.1f,%.1f]x[%.1f,%.1f] (in_x=%d, in_y=%d) -- camera pointed away or frames don't match",
			zone.ID, stats.PointsTotal, zone.MinX, zone.MaxX, zone.MinY, zone.MaxY, stats.PointsInsideX, stats.PointsInsideY)
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
	if stats.MeanSignedDistanceMM < 0 {
		return 0, fmt.Errorf("mean signed distance to plane is negative: %.2f mm", stats.MeanSignedDistanceMM)
	}
	return stats.MeanSignedDistanceMM, nil
}

const grabPlansDir = "/root/.viam/capture"

func (s *grabberControls) savePlan(plan *grabPlanRecord) error {
	if err := os.MkdirAll(grabPlansDir, 0o755); err != nil {
		return fmt.Errorf("creating grab-plans dir: %w", err)
	}
	ts := time.Now().UTC().Format("20060102-150405.000")
	fname := filepath.Join(grabPlansDir, fmt.Sprintf("grab-%s-zone%d.json", ts, plan.ZoneID))
	data, err := json.MarshalIndent(plan, "", "  ")
	if err != nil {
		return fmt.Errorf("marshaling plan: %w", err)
	}
	return os.WriteFile(fname, data, 0o644)
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
	plan, err := s.planGrab(ctx, bin, zoneID, zone, binFoodLevelMM)
	if err != nil {
		return nil, err
	}
	s.logger.Infof("Executing get_from_bin for bin '%s' (zone %d, %d steps)", bin.name, zoneID, len(plan.Steps))

	if err := s.executeGrab(ctx, plan); err != nil {
		return nil, err
	}

	if s.shakeArmService != nil {
		if _, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{"shake_arm": true}); err != nil {
			return nil, fmt.Errorf("failed to shake arm: %w", err)
		}
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
	return nil
}
