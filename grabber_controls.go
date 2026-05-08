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
	"go.viam.com/rdk/components/gripper"
	sw "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

var GrabberControls = resource.NewModel("ncs", "salad", "grabber-controls")

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

type GrabberControlsConfig struct {
	Bins                []GrabberControlsBinConfig            `json:"bins"`
	AboveBinExtra       float64                               `json:"above-bin-extra"`
	AboveBinOrientation *spatialmath.OrientationVectorDegrees `json:"above-bin-orientation,omitempty"`
	GrabHeightMM        float64                               `json:"grab-height-mm"`
	GrabOrientation     *spatialmath.OrientationVectorDegrees `json:"grab-orientation,omitempty"`
	HighAboveBowl       string                                `json:"high-above-bowl"`
	InBowl              string                                `json:"in-bowl"`
	Arm                 string                                `json:"arm"`
	Gripper             string                                `json:"gripper"`
	MotionService       string                                `json:"motion-service"`
	LeftHome            string                                `json:"left-home"`
	ShakeArmService     *string                               `json:"shake-arm-service,omitempty"`
	AssetsDir           string                                `json:"assets-dir"`
	SavePlans           bool                                  `json:"save-plans,omitempty"`
}

func (cfg *GrabberControlsConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.Bins) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bins")
	}

	if cfg.AboveBinOrientation == nil {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "above-bin-orientation")
	}

	if cfg.GrabOrientation == nil {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "grab-orientation")
	}

	if cfg.MotionService == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "motion-service")
	}

	if cfg.HighAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "high-above-bowl")
	}

	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}

	if cfg.Gripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "gripper")
	}

	if cfg.LeftHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-home")
	}

	if cfg.InBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "in-bowl")
	}

	requiredDeps := []string{}

	requiredDeps = append(requiredDeps, cfg.HighAboveBowl)
	requiredDeps = append(requiredDeps, cfg.Arm)
	requiredDeps = append(requiredDeps, cfg.Gripper)
	requiredDeps = append(requiredDeps, cfg.MotionService)
	requiredDeps = append(requiredDeps, cfg.LeftHome)
	requiredDeps = append(requiredDeps, cfg.InBowl)
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
	highAboveBowl   sw.Switch
	arm             arm.Arm
	gripper         gripper.Gripper
	leftInBowl      sw.Switch
	leftHome        sw.Switch
	shakeArmService genericservice.Service
	motionService   motion.Service

	assetsMu  sync.Mutex
	assetsDir string
	binMesh   *spatialmath.Mesh
	zones     *segmentation.ZonesResult
}

type grabberBinSwitches struct {
	name         string
	aboveBinPose spatialmath.Pose
}

type grabPlanStep struct {
	Step   string  `json:"step"`
	X      float64 `json:"x"`
	Y      float64 `json:"y"`
	Z      float64 `json:"z"`
	OX     float64 `json:"ox"`
	OY     float64 `json:"oy"`
	OZ     float64 `json:"oz"`
	Theta  float64 `json:"theta"`
	Linear bool    `json:"linear"`
	Error  string  `json:"error,omitempty"`
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

	highAboveBowlSwitch, err := sw.FromProvider(deps, conf.HighAboveBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get high-above-bowl switch '%s': %w", conf.HighAboveBowl, err)
	}
	s.highAboveBowl = highAboveBowlSwitch

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

	leftHomeSwitch, err := sw.FromProvider(deps, conf.LeftHome)
	if err != nil {
		return nil, fmt.Errorf("failed to get left-home switch '%s': %w", conf.LeftHome, err)
	}
	s.leftHome = leftHomeSwitch

	leftInBowlSwitch, err := sw.FromProvider(deps, conf.InBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get in-bowl switch '%s': %w", conf.InBowl, err)
	}
	s.leftInBowl = leftInBowlSwitch

	for _, binCfg := range conf.Bins {
		s.bins[binCfg.ZoneID] = &grabberBinSwitches{name: binCfg.Name}
	}

	motionSvc, err := motion.FromProvider(deps, conf.MotionService)
	if err != nil {
		return nil, fmt.Errorf("failed to get motion service '%s': %w", conf.MotionService, err)
	}
	s.motionService = motionSvc

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

func (s *grabberControls) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if _, ok := cmd["get_from_bin"]; ok {
		return s.doGetFromBin(ctx, cmd)
	}

	if _, ok := cmd["reset"]; ok {
		return s.reset(ctx)
	}

	if _, ok := cmd["bin_hover"]; ok {
		return s.doHover(ctx, cmd)
	}

	return nil, fmt.Errorf("unknown command, expected 'get_from_bin' or 'bin_hover' field")
}
func (s *grabberControls) moveArm(ctx context.Context, dest spatialmath.Pose, useLinearConstraints bool) error {
	var worldState *referenceframe.WorldState
	if s.binMesh != nil {
		var err error
		worldState, err = referenceframe.NewWorldState(
			[]*referenceframe.GeometriesInFrame{
				referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{s.binMesh}),
			},
			nil,
		)
		if err != nil {
			return fmt.Errorf("failed to build world state: %w", err)
		}
	}
	var constraints *motionplan.Constraints
	if useLinearConstraints {
		constraints = &motionplan.Constraints{
			LinearConstraint: []motionplan.LinearConstraint{{
				LineToleranceMm:          1.0,
				OrientationToleranceDegs: 1.0,
			}},
		}
	}
	pt := dest.Point()
	s.logger.Infof("moving arm to x=%.2f y=%.2f z=%.2f", pt.X, pt.Y, pt.Z)
	_, err := s.motionService.Move(ctx, motion.MoveReq{
		ComponentName: s.arm.Name().ShortName(),
		Destination:   referenceframe.NewPoseInFrame(referenceframe.World, dest),
		WorldState:    worldState,
		Constraints:   constraints,
	})
	return err
}

func (s *grabberControls) doHover(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if zoneIDVal, ok := cmd["bin_hover"]; ok {
		var zoneID int
		switch v := zoneIDVal.(type) {
		case int:
			zoneID = v
		case float64:
			zoneID = int(v)
		default:
			return nil, fmt.Errorf("'bin_hover' must be an int zone ID, got %T", zoneIDVal)
		}

		bin, ok := s.bins[zoneID]
		if !ok {
			return nil, fmt.Errorf("zone %d not found in configuration", zoneID)
		}
		return nil, s.moveArm(ctx, bin.aboveBinPose, false)
	}
	return nil, fmt.Errorf("unknown command")
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
			point := r3.Vector{
				X: cx,
				Y: cy,
				Z: s.zones.ZMean + s.cfg.AboveBinExtra,
			}

			s.bins[binCfg.ZoneID].aboveBinPose = spatialmath.NewPose(point, s.cfg.AboveBinOrientation)
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

func (s *grabberControls) computeGrabPose(zone *segmentation.Zone) (spatialmath.Pose, error) {
	cx, cy, err := zone.Centroid()
	if err != nil {
		return nil, err
	}
	point := r3.Vector{
		X: cx,
		Y: cy,
		Z: zone.MinZ() + s.cfg.GrabHeightMM,
	}
	return spatialmath.NewPose(point, s.cfg.AboveBinOrientation), nil
}

const grabPlansDir = "/root/.viam/capture/grab-plans"

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

func poseToStep(name string, pose spatialmath.Pose, linear bool, stepErr error) grabPlanStep {
	pt := pose.Point()
	ov := pose.Orientation().OrientationVectorDegrees()
	step := grabPlanStep{
		Step:   name,
		X:      pt.X,
		Y:      pt.Y,
		Z:      pt.Z,
		OX:     ov.OX,
		OY:     ov.OY,
		OZ:     ov.OZ,
		Theta:  ov.Theta,
		Linear: linear,
	}
	if stepErr != nil {
		step.Error = stepErr.Error()
	}
	return step
}

func (s *grabberControls) doGetFromBin(ctx context.Context, cmd map[string]interface{}) (_ map[string]interface{}, retErr error) {
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

	bin, ok := s.bins[zoneID]
	if !ok {
		return nil, fmt.Errorf("zone %d not found in configuration", zoneID)
	}

	zone, err := s.getZone(zoneID)
	if err != nil {
		return nil, err
	}
	grabPose, err := s.computeGrabPose(zone)
	if err != nil {
		return nil, err
	}

	var plan *grabPlanRecord
	if s.cfg.SavePlans {
		plan = &grabPlanRecord{
			StartedAt: time.Now().UTC().Format(time.RFC3339Nano),
			BinName:   bin.name,
			ZoneID:    zoneID,
		}
		defer func() {
			plan.Success = retErr == nil
			if retErr != nil {
				plan.Error = retErr.Error()
			}
			if saveErr := s.savePlan(plan); saveErr != nil {
				s.logger.Warnf("failed to save grab plan: %v", saveErr)
			}
		}()
	}

	recordMove := func(stepName string, pose spatialmath.Pose, linear bool, moveErr error) {
		if plan != nil {
			plan.Steps = append(plan.Steps, poseToStep(stepName, pose, linear, moveErr))
		}
	}

	s.logger.Infof("Executing get_from_bin for bin '%s' (zone %d)", bin.name, zoneID)

	err = s.moveArm(ctx, bin.aboveBinPose, false)
	recordMove("above_bin", bin.aboveBinPose, false, err)
	if err != nil {
		return nil, fmt.Errorf("failed to move arm above bin: %w", err)
	}
	s.logger.Debugf("Moved arm above bin")

	if err := s.gripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open gripper: %w", err)
	}
	s.logger.Debugf("Opened gripper")

	err = s.moveArm(ctx, grabPose, true)
	recordMove("descend", grabPose, true, err)
	if err != nil {
		return nil, fmt.Errorf("failed to descend into bin: %w", err)
	}
	s.logger.Debugf("Descended into bin")

	tiltedGrabPose := spatialmath.NewPose(grabPose.Point(), s.cfg.GrabOrientation)
	err = s.moveArm(ctx, tiltedGrabPose, false)
	recordMove("tilt", tiltedGrabPose, false, err)
	if err != nil {
		return nil, fmt.Errorf("failed to tilt gripper: %w", err)
	}
	s.logger.Debugf("Tilted gripper")

	if _, err := s.gripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to grab: %w", err)
	}
	s.logger.Debugf("Grabbed")

	err = s.moveArm(ctx, grabPose, false)
	recordMove("untilt", grabPose, false, err)
	if err != nil {
		return nil, fmt.Errorf("failed to untilt gripper: %w", err)
	}
	s.logger.Debugf("Untilted gripper")

	err = s.moveArm(ctx, bin.aboveBinPose, true)
	recordMove("ascend", bin.aboveBinPose, true, err)
	if err != nil {
		return nil, fmt.Errorf("failed to ascend from bin: %w", err)
	}
	s.logger.Debugf("Ascended from bin")

	if err := s.highAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set high-above-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set high-above-bowl switch to position 2")

	if err := s.leftInBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set in-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set in-bowl switch to position 2")

	if err := s.gripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open left gripper: %w", err)
	}
	s.logger.Debugf("Opened left gripper")

	if err := s.highAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set high-above-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set high-above-bowl switch to position 2")

	if s.shakeArmService != nil {
		_, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{
			"shake_arm": true,
		})
		if err != nil {
			return nil, fmt.Errorf("failed to shake arm: %w", err)
		}
	}
	s.logger.Infof("Successfully completed get_from_bin for bin '%s' (zone %d)", bin.name, zoneID)

	return map[string]interface{}{
		"success": true,
		"bin":     bin.name,
		"message": fmt.Sprintf("Successfully grabbed from bin '%s' and moved to bowl", bin.name),
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
