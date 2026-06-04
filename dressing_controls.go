package salad

import (
	"context"
	"fmt"
	"sync"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/robot/framesystem"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"

	"salad/lib/fileio"
)

var DressingControls = resource.NewModel("ncs", "salad", "dressing-controls")

func init() {
	resource.RegisterService(genericservice.API, DressingControls,
		resource.Registration[resource.Resource, *DressingControlsConfig]{
			Constructor: newDressingControls,
		},
	)
}

// DressingPoseConfig holds the target pose and motion constraints for a single arm step.
type DressingPoseConfig struct {
	Point       r3.Vector                            `json:"point"`
	Orientation spatialmath.OrientationVectorDegrees `json:"orientation"`
	Constraints *motionplan.Constraints              `json:"constraints,omitempty"`
}

func (c DressingPoseConfig) toPose() spatialmath.Pose {
	return spatialmath.NewPose(c.Point, &c.Orientation)
}

type DressingOptionConfig struct {
	ApproachGrab DressingPoseConfig `json:"approach-grab"`
	Grab         DressingPoseConfig `json:"grab"`
}

type CircularPourConfig struct {
	RadiusMm     float64                 `json:"radius-mm"`
	PointsPerRev int                     `json:"points-per-rev,omitempty"`
	Revolutions  int                     `json:"revolutions,omitempty"`
	Constraints  *motionplan.Constraints `json:"constraints,omitempty"`
}

type DressingControlsConfig struct {
	Arm                 string                          `json:"arm"`
	Gripper             string                          `json:"gripper"`
	AssetsDir           string                          `json:"assets-dir,omitempty"`
	PrepareDressing     DressingPoseConfig              `json:"prepare-dressing"`
	PourDressing        DressingPoseConfig              `json:"pour-dressing"`
	CircularPour        *CircularPourConfig             `json:"circular-pour,omitempty"`
	PostPourDressing    DressingPoseConfig              `json:"post-pour-dressing"`
	Home                DressingPoseConfig              `json:"home"`
	GrabSpeedDegsPerSec float64                         `json:"grab-speed-degs-per-sec,omitempty"`
	ShakeArmService     *string                         `json:"shake-arm-service,omitempty"`
	Dressings           map[string]DressingOptionConfig `json:"dressings"`
}

func (cfg *DressingControlsConfig) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "arm")
	}
	if cfg.Gripper == "" {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "gripper")
	}
	if len(cfg.Dressings) == 0 {
		return nil, nil, resource.NewConfigValidationFieldRequiredError(path, "dressings")
	}
	deps := []string{cfg.Arm, cfg.Gripper, framesystem.PublicServiceName.String()}
	if cfg.ShakeArmService != nil && *cfg.ShakeArmService != "" {
		deps = append(deps, *cfg.ShakeArmService)
	}
	return deps, []string{}, nil
}

type dressingControls struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *DressingControlsConfig

	cancelCtx  context.Context
	cancelFunc func()

	arm             arm.Arm
	gripper         gripper.Gripper
	shakeArmService genericservice.Service
	fsService       framesystem.Service

	assetsMu   sync.Mutex
	worldState *referenceframe.WorldState

	cachedPlansMu sync.Mutex
	cachedPlans   map[string]*dressingPlan

	fileSaver *fileio.FileSaver
}

func newDressingControls(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*DressingControlsConfig](rawConf)
	if err != nil {
		return nil, err
	}
	return NewDressingControls(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewDressingControls(ctx context.Context, deps resource.Dependencies, name resource.Name, conf *DressingControlsConfig, logger logging.Logger) (resource.Resource, error) {
	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &dressingControls{
		name:       name,
		logger:     logger,
		cfg:        conf,
		cancelCtx:  cancelCtx,
		cancelFunc: cancelFunc,
	}

	armComponent, err := arm.FromProvider(deps, conf.Arm)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm '%s': %w", conf.Arm, err)
	}
	s.arm = armComponent

	gripperComponent, err := gripper.FromProvider(deps, conf.Gripper)
	if err != nil {
		return nil, fmt.Errorf("failed to get gripper '%s': %w", conf.Gripper, err)
	}
	s.gripper = gripperComponent

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

	s.cachedPlans = make(map[string]*dressingPlan)
	s.fileSaver = fileio.NewFileSaver(logger, defaultPlanCaptureDir)

	s.logger.Infof("Dressing controls initialized")
	return s, nil
}

func (s *dressingControls) Name() resource.Name {
	return s.name
}

func (s *dressingControls) Status(ctx context.Context) (map[string]interface{}, error) {
	return map[string]interface{}{}, nil
}

func (s *dressingControls) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	buildID, _ := cmd["build_id"].(string)
	if v, ok := cmd["pour_dressing"]; ok {
		name, ok := v.(string)
		if !ok || name == "" {
			return nil, fmt.Errorf("pour_dressing requires a dressing name (string)")
		}
		return s.doPourDressing(ctx, name, buildID)
	}
	if v, ok := cmd["pre_plan_dressing"]; ok {
		name, ok := v.(string)
		if !ok || name == "" {
			return nil, fmt.Errorf("pre_plan_dressing requires a dressing name (string)")
		}
		return s.doPrePlanDressing(ctx, name, buildID)
	}
	if _, ok := cmd["get_dressings"]; ok {
		dressings := make([]map[string]any, 0, len(s.cfg.Dressings))
		for name := range s.cfg.Dressings {
			dressings = append(dressings, map[string]any{
				"name": name,
			})
		}
		return map[string]any{"dressings": dressings}, nil
	}
	if _, ok := cmd["reset"]; ok {
		return s.reset(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'pour_dressing', 'pre_plan_dressing', or 'reset' field")
}

func (s *dressingControls) loadWorldState() error {
	s.assetsMu.Lock()
	defer s.assetsMu.Unlock()

	if s.worldState != nil {
		return nil
	}

	assetsDir := s.cfg.AssetsDir
	if assetsDir == "" {
		assetsDir = "/home/viam/assets"
	}

	mesh, err := spatialmath.NewMeshFromPLYFile(assetsDir + "/mesh.ply")
	if err != nil {
		return fmt.Errorf("mesh not available at %s: %w", assetsDir+"/mesh.ply", err)
	}

	octree, err := pointcloud.NewFromMesh(mesh)
	if err != nil {
		return fmt.Errorf("failed to convert mesh to octree: %w", err)
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
	return nil
}

func (s *dressingControls) doPrePlanDressing(ctx context.Context, name, buildID string) (map[string]interface{}, error) {
	s.logger.Infof("Pre-planning dressing %q", name)
	plan, err := s.planDressing(ctx, name, buildID)
	if err != nil {
		return nil, fmt.Errorf("pre-planning dressing %q: %w", name, err)
	}
	s.cachedPlansMu.Lock()
	s.cachedPlans[name] = plan
	s.cachedPlansMu.Unlock()
	s.logger.Infof("Pre-planning dressing %q complete", name)
	return map[string]interface{}{"success": true}, nil
}

func (s *dressingControls) doPourDressing(ctx context.Context, name, buildID string) (map[string]interface{}, error) {
	s.cachedPlansMu.Lock()
	plan, cached := s.cachedPlans[name]
	if cached {
		delete(s.cachedPlans, name)
	}
	s.cachedPlansMu.Unlock()

	if cached {
		s.logger.Infof("Using pre-planned trajectories for %q", name)
	} else {
		s.logger.Infof("Planning pour_dressing for %q", name)
		var err error
		plan, err = s.planDressing(ctx, name, buildID)
		if err != nil {
			return nil, err
		}
	}

	s.logger.Infof("Executing pour_dressing for %q (%d steps)", name, len(plan.steps))
	if err := s.executeDressing(ctx, plan); err != nil {
		return nil, err
	}
	s.logger.Infof("Successfully completed pour_dressing for %q", name)
	return map[string]interface{}{"success": true, "message": "Successfully poured dressing"}, nil
}

func (s *dressingControls) reset(ctx context.Context) (map[string]interface{}, error) {
	if err := s.loadWorldState(); err != nil {
		return nil, err
	}

	fs, err := framesystem.NewFromService(ctx, s.fsService, nil)
	if err != nil {
		return nil, fmt.Errorf("building frame system: %w", err)
	}

	armCurrentInputs, err := s.arm.CurrentInputs(ctx)
	if err != nil {
		return nil, fmt.Errorf("getting arm current inputs: %w", err)
	}
	startInputs := referenceframe.NewZeroInputs(fs)
	if len(armCurrentInputs) > 0 {
		startInputs[s.cfg.Arm] = armCurrentInputs
	}

	goalState := armplanning.NewPlanState(
		referenceframe.FrameSystemPoses{
			s.cfg.Arm: referenceframe.NewPoseInFrame(referenceframe.World, s.cfg.Home.toPose()),
		},
		nil,
	)
	req := &armplanning.PlanRequest{
		FrameSystem: fs,
		WorldState:  s.worldState,
		StartState:  armplanning.NewPlanState(nil, startInputs),
		Goals:       []*armplanning.PlanState{goalState},
		Constraints: s.cfg.Home.Constraints,
	}

	plan, _, err := armplanning.PlanMotion(ctx, s.logger, req)
	if err != nil {
		return nil, fmt.Errorf("planning home: %w", err)
	}

	traj := plan.Trajectory()
	armInputs := make([][]referenceframe.Input, len(traj))
	for i, fsInputs := range traj {
		armInputs[i] = fsInputs[s.cfg.Arm]
	}

	if err := s.arm.MoveThroughJointPositions(ctx, armInputs, nil, nil); err != nil {
		return nil, fmt.Errorf("moving to home: %w", err)
	}
	return nil, nil
}

func (s *dressingControls) Close(context.Context) error {
	s.cancelFunc()
	if err := s.fileSaver.Close(); err != nil {
		s.logger.Warnw("FileSaver close error", "err", err)
	}
	return nil
}
