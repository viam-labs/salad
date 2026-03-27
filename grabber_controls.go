package salad

import (
	"context"
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	sw "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"
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
	Name     string `json:"name"`
	AboveBin string `json:"above-bin"`
	InBin    string `json:"in-bin"`
}

type GrabberControlsConfig struct {
	Bins            []GrabberControlsBinConfig `json:"bins"`
	HighAboveBowl   string                     `json:"high-above-bowl"`
	InBowl          string                     `json:"in-bowl"`
	LeftGripper     string                     `json:"left-gripper"`
	LeftHome        string                     `json:"left-home"`
	ShakeArmService *string                    `json:"shake-arm-service,omitempty"`

	// Dynamic grab fields (optional — required only when using add_ingredient_dynamic).
	MeshFile      string  `json:"mesh_file,omitempty"`
	ArmName       string  `json:"arm,omitempty"`
	MotionService string  `json:"motion_service,omitempty"`
	ScaleName     string  `json:"scale,omitempty"`
	BowlHoverX    float64 `json:"bowl_hover_x,omitempty"`
	BowlHoverY    float64 `json:"bowl_hover_y,omitempty"`
	BowlHoverZ    float64 `json:"bowl_hover_z,omitempty"`
	BowlDropX     float64 `json:"bowl_drop_x,omitempty"`
	BowlDropY     float64 `json:"bowl_drop_y,omitempty"`
	BowlDropZ     float64 `json:"bowl_drop_z,omitempty"`
	StepMM        float64 `json:"step_mm,omitempty"`
}

func (cfg *GrabberControlsConfig) Validate(path string) ([]string, []string, error) {
	if len(cfg.Bins) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "bins")
	}

	if cfg.HighAboveBowl == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "high-above-bowl")
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

	requiredDeps = append(requiredDeps, cfg.HighAboveBowl)
	requiredDeps = append(requiredDeps, cfg.LeftGripper)
	requiredDeps = append(requiredDeps, cfg.LeftHome)
	requiredDeps = append(requiredDeps, cfg.InBowl)
	if cfg.ShakeArmService != nil && *cfg.ShakeArmService != "" {
		requiredDeps = append(requiredDeps, *cfg.ShakeArmService)
	}
	if cfg.MotionService != "" {
		requiredDeps = append(requiredDeps, cfg.MotionService)
	}
	if cfg.ScaleName != "" {
		requiredDeps = append(requiredDeps, cfg.ScaleName)
	}

	for i, bin := range cfg.Bins {
		if bin.Name == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'name' field is required", path, i)
		}
		if bin.AboveBin == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'above-bin' field is required", path, i)
		}
		if bin.InBin == "" {
			return nil, nil, fmt.Errorf("%s.bins[%d]: 'in-bin' field is required", path, i)
		}

		requiredDeps = append(requiredDeps, bin.AboveBin, bin.InBin)
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

	bins            map[string]*grabberBinSwitches
	highAboveBowl   sw.Switch
	leftGripper     gripper.Gripper
	leftInBowl      sw.Switch
	leftHome        sw.Switch
	shakeArmService genericservice.Service

	// Dynamic grab fields (nil if not configured).
	meshPts   []r3.Vector
	motionSvc motion.Service
	scale     sensor.Sensor
}

type grabberBinSwitches struct {
	aboveBin sw.Switch
	inBin    sw.Switch
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
		bins:       make(map[string]*grabberBinSwitches),
	}

	highAboveBowlSwitch, err := sw.FromProvider(deps, conf.HighAboveBowl)
	if err != nil {
		return nil, fmt.Errorf("failed to get high-above-bowl switch '%s': %w", conf.HighAboveBowl, err)
	}
	s.highAboveBowl = highAboveBowlSwitch

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
		aboveBinSwitch, err := sw.FromProvider(deps, binCfg.AboveBin)
		if err != nil {
			return nil, fmt.Errorf("failed to get above-bin switch '%s' for bin '%s': %w", binCfg.AboveBin, binCfg.Name, err)
		}

		inBinSwitch, err := sw.FromProvider(deps, binCfg.InBin)
		if err != nil {
			return nil, fmt.Errorf("failed to get in-bin switch '%s' for bin '%s': %w", binCfg.InBin, binCfg.Name, err)
		}

		s.bins[binCfg.Name] = &grabberBinSwitches{
			aboveBin: aboveBinSwitch,
			inBin:    inBinSwitch,
		}
	}
	if conf.ShakeArmService != nil && *conf.ShakeArmService != "" {
		shakeArmService, err := genericservice.FromProvider(deps, *conf.ShakeArmService)
		if err != nil {
			return nil, fmt.Errorf("failed to get shake-arm-service '%s': %w", *conf.ShakeArmService, err)
		}
		s.shakeArmService = shakeArmService
	}

	if conf.MeshFile != "" {
		mesh, err := spatialmath.NewMeshFromPLYFile(conf.MeshFile)
		if err != nil {
			return nil, fmt.Errorf("failed to load mesh '%s': %w", conf.MeshFile, err)
		}
		s.meshPts = mesh.ToPoints(1)
		s.logger.Infof("Loaded mesh from %s (%d vertices)", conf.MeshFile, len(s.meshPts))
	}
	if conf.MotionService != "" {
		motionSvc, err := motion.FromProvider(deps, conf.MotionService)
		if err != nil {
			return nil, fmt.Errorf("failed to get motion service '%s': %w", conf.MotionService, err)
		}
		s.motionSvc = motionSvc
	}
	if conf.ScaleName != "" {
		scaleSensor, err := sensor.FromProvider(deps, conf.ScaleName)
		if err != nil {
			return nil, fmt.Errorf("failed to get scale sensor '%s': %w", conf.ScaleName, err)
		}
		s.scale = scaleSensor
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
	if _, ok := cmd["add_ingredient_dynamic"]; ok {
		return s.doAddIngredientDynamic(ctx, cmd)
	}
	return nil, fmt.Errorf("unknown command, expected 'get_from_bin', 'reset', or 'add_ingredient_dynamic'")
}

func (s *grabberControls) doGetFromBin(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	getFromBin := cmd["get_from_bin"]

	binName, ok := getFromBin.(string)
	if !ok {
		return nil, fmt.Errorf("'get_from_bin' must be a string, got %T", getFromBin)
	}

	bin, ok := s.bins[binName]
	if !ok {
		return nil, fmt.Errorf("bin '%s' not found in configuration", binName)
	}

	s.logger.Infof("Executing get_from_bin for bin '%s'", binName)

	if err := bin.aboveBin.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-bin switch to position 2: %w", err)
	}
	s.logger.Debugf("Set above-bin switch to position 2")

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

	if err := bin.aboveBin.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set above-bin switch to position 2 (second time): %w", err)
	}
	s.logger.Debugf("Set above-bin switch to position 2 (second time)")

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
	s.logger.Infof("Successfully completed get_from_bin for bin '%s'", binName)

	return map[string]interface{}{
		"success": true,
		"bin":     binName,
		"message": fmt.Sprintf("Successfully grabbed from bin '%s' and moved to bowl", binName),
	}, nil
}

func (s *grabberControls) reset(ctx context.Context) (map[string]interface{}, error) {
	if err := s.leftHome.SetPosition(ctx, 2, nil); err != nil {
		return nil, fmt.Errorf("failed to set left-home switch to position 2: %w", err)
	}
	s.logger.Debugf("Set left-home switch to position 2")

	return nil, nil
}

func (s *grabberControls) doAddIngredientDynamic(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	if len(s.meshPts) == 0 {
		return nil, fmt.Errorf("add_ingredient_dynamic requires 'mesh_file' to be configured")
	}
	if s.motionSvc == nil {
		return nil, fmt.Errorf("add_ingredient_dynamic requires 'motion_service' to be configured")
	}
	if s.scale == nil {
		return nil, fmt.Errorf("add_ingredient_dynamic requires 'scale' to be configured")
	}
	if s.cfg.ArmName == "" {
		return nil, fmt.Errorf("add_ingredient_dynamic requires 'arm' to be configured")
	}

	params, ok := cmd["add_ingredient_dynamic"].(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("'add_ingredient_dynamic' value must be a map with 'bin' and 'target_grams'")
	}
	binName, _ := params["bin"].(string)
	if binName == "" {
		return nil, fmt.Errorf("'add_ingredient_dynamic.bin' must be a non-empty string")
	}
	targetGrams, _ := params["target_grams"].(float64)
	if targetGrams <= 0 {
		return nil, fmt.Errorf("'add_ingredient_dynamic.target_grams' must be positive")
	}

	bin, ok := s.bins[binName]
	if !ok {
		return nil, fmt.Errorf("bin '%s' not found in configuration", binName)
	}

	stepMM := s.cfg.StepMM
	if stepMM <= 0 {
		stepMM = 80
	}

	var totalAdded float64
	var zeroChangeStreak int
	var xOffset float64

	for totalAdded < targetGrams {
		weightBefore, err := s.readScale(ctx)
		if err != nil {
			return nil, fmt.Errorf("failed to read scale before grab: %w", err)
		}

		s.logger.Infof("Moving to imaging switch for bin '%s' (added: %.1fg / %.1fg)", binName, totalAdded, targetGrams)
		if err := bin.aboveBin.SetPosition(ctx, 2, nil); err != nil {
			return nil, fmt.Errorf("failed to move to above-bin: %w", err)
		}
		time.Sleep(2 * time.Second)

		poseInWorld, err := s.motionSvc.GetPose(ctx, s.cfg.ArmName, referenceframe.World, nil, nil)
		if err != nil {
			return nil, fmt.Errorf("failed to get arm world pose: %w", err)
		}
		endPose := poseInWorld.Pose()
		imagingX := endPose.Point().X
		imagingY := endPose.Point().Y
		startZ := endPose.Point().Z
		descentX := imagingX + xOffset

		binCenterX, err := MeshBinCenterX(s.meshPts, imagingX, imagingY, s.logger)
		if err != nil {
			return nil, fmt.Errorf("failed to determine bin center X from mesh: %w", err)
		}
		meshFloorZ, err := MeshAvgLowestZ(s.meshPts, imagingX, imagingY, s.logger)
		if err != nil {
			return nil, fmt.Errorf("failed to determine target Z from mesh floor: %w", err)
		}
		targetZ := meshFloorZ + GrabAboveMeshFloorMM
		s.logger.Infof("Descent target: X=%.0f Y=%.0f Z=%.0f (mesh rim X=%.0f, xOffset=%.0f)", descentX, imagingY, targetZ, binCenterX, xOffset)

		if err := s.leftGripper.Open(ctx, nil); err != nil {
			return nil, fmt.Errorf("failed to open gripper: %w", err)
		}

		descentZs := DescentWaypoints(startZ, targetZ, stepMM)
		reachedZ := startZ
		for i, nextZ := range descentZs {
			s.logger.Infof("Descent step %d: X=%.0f Y=%.0f Z=%.0f", i+1, descentX, imagingY, nextZ)
			stepPose := spatialmath.NewPose(r3.Vector{X: descentX, Y: imagingY, Z: nextZ}, endPose.Orientation())
			if _, err := s.motionSvc.Move(ctx, motion.MoveReq{
				ComponentName: s.cfg.ArmName,
				Destination:   referenceframe.NewPoseInFrame(referenceframe.World, stepPose),
			}); err != nil {
				s.logger.Warnf("Descent step %d failed (Z=%.0f), stopping descent: %v", i+1, nextZ, err)
				break
			}
			reachedZ = nextZ
		}
		if len(descentZs) > 0 && reachedZ != descentZs[len(descentZs)-1] {
			s.logger.Warnf("Partial descent — stopped at Z=%.0f (target Z=%.0f), grabbing anyway", reachedZ, targetZ)
		}

		s.logger.Infof("Closing gripper at Z=%.0f", reachedZ)
		if _, err := s.leftGripper.Grab(ctx, nil); err != nil {
			return nil, fmt.Errorf("failed to grab: %w", err)
		}
		if err := WaitGripperSettledAfterGrab(ctx, s.leftGripper, s.logger); err != nil {
			return nil, err
		}

		s.logger.Infof("Ascending back to above-bin (from Z=%.0f)", reachedZ)
		for i, nextZ := range DescentWaypoints(reachedZ, startZ, stepMM) {
			s.logger.Infof("Ascent step %d: X=%.0f Y=%.0f Z=%.0f", i+1, descentX, imagingY, nextZ)
			stepPose := spatialmath.NewPose(r3.Vector{X: descentX, Y: imagingY, Z: nextZ}, endPose.Orientation())
			if _, err := s.motionSvc.Move(ctx, motion.MoveReq{
				ComponentName: s.cfg.ArmName,
				Destination:   referenceframe.NewPoseInFrame(referenceframe.World, stepPose),
			}); err != nil {
				return nil, fmt.Errorf("ascent step %d failed (Z=%.0f): %w", i+1, nextZ, err)
			}
		}

		bowlHoverPose := spatialmath.NewPose(
			r3.Vector{X: s.cfg.BowlHoverX, Y: s.cfg.BowlHoverY, Z: s.cfg.BowlHoverZ},
			endPose.Orientation(),
		)
		s.logger.Infof("Moving to bowl hover (X=%.0f Y=%.0f Z=%.0f)", s.cfg.BowlHoverX, s.cfg.BowlHoverY, s.cfg.BowlHoverZ)
		if _, err := s.motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: s.cfg.ArmName,
			Destination:   referenceframe.NewPoseInFrame(referenceframe.World, bowlHoverPose),
		}); err != nil {
			return nil, fmt.Errorf("failed to move to bowl hover: %w", err)
		}

		bowlDropPose := spatialmath.NewPose(
			r3.Vector{X: s.cfg.BowlDropX, Y: s.cfg.BowlDropY, Z: s.cfg.BowlDropZ},
			endPose.Orientation(),
		)
		s.logger.Infof("Moving to bowl drop (X=%.0f Y=%.0f Z=%.0f)", s.cfg.BowlDropX, s.cfg.BowlDropY, s.cfg.BowlDropZ)
		if _, err := s.motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: s.cfg.ArmName,
			Destination:   referenceframe.NewPoseInFrame(referenceframe.World, bowlDropPose),
		}); err != nil {
			return nil, fmt.Errorf("failed to move to bowl drop: %w", err)
		}

		if err := s.leftGripper.Open(ctx, nil); err != nil {
			return nil, fmt.Errorf("failed to open gripper over bowl: %w", err)
		}

		time.Sleep(1 * time.Second)

		weightAfter, err := s.readScale(ctx)
		if err != nil {
			return nil, fmt.Errorf("failed to read scale after grab: %w", err)
		}

		change := weightAfter - weightBefore
		s.logger.Infof("Scale change: %.1fg (before: %.1fg, after: %.1fg)", change, weightBefore, weightAfter)

		if change < ZeroChangeTolerance {
			zeroChangeStreak++
			if zeroChangeStreak >= 3 {
				return nil, fmt.Errorf("3 consecutive grabs with no weight change for bin '%s', possible empty bin", binName)
			}
			direction := 1.0
			if imagingX > binCenterX {
				direction = -1.0
			}
			if zeroChangeStreak == 1 {
				xOffset = direction * 5.0
			} else {
				xOffset = -direction * 5.0
			}
			s.logger.Warnf("No weight change (streak: %d/3) — retrying with X offset %.0fmm", zeroChangeStreak, xOffset)
		} else {
			zeroChangeStreak = 0
			xOffset = 0
			totalAdded += change
		}
	}

	s.logger.Infof("Done — added %.1fg (target: %.1fg)", totalAdded, targetGrams)
	return map[string]interface{}{
		"success":             true,
		"bin":                 binName,
		"total_added_grams":   totalAdded,
		"target_grams":        targetGrams,
	}, nil
}

func (s *grabberControls) readScale(ctx context.Context) (float64, error) {
	readings, err := s.scale.Readings(ctx, nil)
	if err != nil {
		return 0, fmt.Errorf("failed to read scale sensor: %w", err)
	}
	for _, v := range readings {
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
		}
	}
	return 0, fmt.Errorf("no numeric reading found from scale sensor")
}

func (s *grabberControls) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
