package salad

import (
	"context"
	"fmt"
	"sync"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	sw "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	genericservice "go.viam.com/rdk/services/generic"
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
	InBin  string `json:"in-bin"`
}

type GrabberControlsConfig struct {
	Bins                []GrabberControlsBinConfig            `json:"bins"`
	AboveBin            string                                `json:"above-bin"`
	AboveBinExtra       float64                               `json:"above-bin-extra"`
	AboveBinOrientation *spatialmath.OrientationVectorDegrees `json:"above-bin-orientation,omitempty"`
	HighAboveBowl       string                                `json:"high-above-bowl"`
	InBowl              string                                `json:"in-bowl"`
	LeftArm             string                                `json:"left-arm"`
	LeftGripper         string                                `json:"left-gripper"`
	LeftHome            string                                `json:"left-home"`
	ShakeArmService     *string                               `json:"shake-arm-service,omitempty"`
	AssetsDir           string                                `json:"assets-dir"`
}

func (cfg *GrabberControlsConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.Bins) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bins")
	}

	if cfg.AboveBin == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "above-bin")
	}

	if cfg.HighAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "high-above-bowl")
	}

	if cfg.LeftArm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-arm")
	}

	if cfg.LeftGripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-gripper")
	}

	if cfg.LeftHome == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "left-home")
	}

	if cfg.InBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "in-bowl")
	}

	requiredDeps := []string{}

	requiredDeps = append(requiredDeps, cfg.AboveBin)
	requiredDeps = append(requiredDeps, cfg.HighAboveBowl)
	requiredDeps = append(requiredDeps, cfg.LeftArm)
	requiredDeps = append(requiredDeps, cfg.LeftGripper)
	requiredDeps = append(requiredDeps, cfg.LeftHome)
	requiredDeps = append(requiredDeps, cfg.InBowl)
	if cfg.ShakeArmService != nil && *cfg.ShakeArmService != "" {
		requiredDeps = append(requiredDeps, *cfg.ShakeArmService)
	}

	for i, bin := range cfg.Bins {
		if bin.Name == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'name' field is required", path, i)
		}
		if bin.InBin == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'in-bin' field is required", path, i)
		}

		requiredDeps = append(requiredDeps, bin.InBin)
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
	leftArm         arm.Arm
	leftGripper     gripper.Gripper
	leftInBowl      sw.Switch
	leftHome        sw.Switch
	shakeArmService genericservice.Service

	assetsMu  sync.Mutex
	assetsDir string
	binMesh   *spatialmath.Mesh
	zones     *segmentation.ZonesResult
}

type grabberBinSwitches struct {
	name         string
	// aboveBin     sw.Switch
	inBin        sw.Switch
	aboveBinPose spatialmath.Pose
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

	leftArmComponent, err := arm.FromProvider(deps, conf.LeftArm)
	if err != nil {
		return nil, fmt.Errorf("failed to get left arm '%s': %w", conf.LeftArm, err)
	}
	s.leftArm = leftArmComponent

	leftGripperComponent, err := gripper.FromProvider(deps, conf.LeftGripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get left gripper '%s': %w", conf.LeftGripper, err)
	}
	s.leftGripper = leftGripperComponent

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
		inBinSwitch, err := sw.FromProvider(deps, binCfg.InBin)
		if err != nil {
			return nil, fmt.Errorf("failed to get in-bin switch '%s' for bin '%s': %w", binCfg.InBin, binCfg.Name, err)
		}
		s.bins[binCfg.ZoneID] = &grabberBinSwitches{name: binCfg.Name, inBin: inBinSwitch}
	}
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
	return nil, fmt.Errorf("unknown command, expected 'get_from_bin' or 'deliver_bowl' field")
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
			point := r3.Vector{
				X: (z.MinX + z.MaxX) / 2,
				Y: (z.MinY + z.MaxY) / 2,
				Z: s.zones.ZMean + s.cfg.AboveBinExtra,
			}
			var orientation spatialmath.Orientation
			if s.cfg.AboveBinOrientation != nil {
				orientation = s.cfg.AboveBinOrientation
			} else {
				orientation = spatialmath.NewZeroOrientation()
			}
			s.bins[binCfg.ZoneID].aboveBinPose = spatialmath.NewPose(point, orientation)
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

func (s *grabberControls) doGetFromBin(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if err := s.loadAssets(); err != nil {
		return nil, err
	}

	zoneIDVal, ok := cmd["get_from_bin"]
	zoneID, ok := zoneIDVal.(int)
	if !ok {
		return nil, fmt.Errorf("'get_from_bin' must be an int zone ID, got %T", zoneIDVal)
	}

	bin, ok := s.bins[zoneID]
	if !ok {
		return nil, fmt.Errorf("zone %d not found in configuration", zoneID)
	}

	s.logger.Infof("Executing get_from_bin for bin '%s' (zone %d)", bin.name, zoneID)

	if err := s.leftArm.MoveToPosition(ctx, bin.aboveBinPose, nil); err != nil {
		return nil, fmt.Errorf("failed to move arm above bin: %w", err)
	}
	s.logger.Debugf("Moved arm above bin")

	if err := s.leftGripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to close left gripper: %w", err)
	}
	s.logger.Debugf("Closed left gripper")

	if err := bin.inBin.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set in-bin switch to position 2: %w", err)
	}
	s.logger.Debugf("Set in-bin switch to position 2")

	if _, err := s.leftGripper.Grab(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to close left gripper: %w", err)
	}
	s.logger.Debugf("Closed left gripper")

	if err := s.leftArm.MoveToPosition(ctx, bin.aboveBinPose, nil); err != nil {
		return nil, fmt.Errorf("failed to move arm above bin (second time): %w", err)
	}
	s.logger.Debugf("Moved arm above bin (second time)")

	if err := s.highAboveBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set high-above-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set high-above-bowl switch to position 2")

	if err := s.leftInBowl.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set in-bowl switch to position 2: %w", err)
	}
	s.logger.Debugf("Set in-bowl switch to position 2")

	if err := s.leftGripper.Open(ctx, nil); err != nil {
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
