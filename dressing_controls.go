package salad

import (
	"context"
	"fmt"
	"sync"
	"time"

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

type DressingControlsConfig struct {
	Arm              string                          `json:"arm"`
	Gripper          string                          `json:"gripper"`
	AssetsDir        string                          `json:"assets-dir,omitempty"`
	PrepareDressing  DressingPoseConfig              `json:"prepare-dressing"`
	PourDressing     DressingPoseConfig              `json:"pour-dressing"`
	PourDressing2    DressingPoseConfig              `json:"pour-dressing2"`
	PostPourDressing DressingPoseConfig              `json:"post-pour-dressing"`
	Home             DressingPoseConfig              `json:"home"`
	ShakeArmService  *string                         `json:"shake-arm-service,omitempty"`
	Dressings        map[string]DressingOptionConfig `json:"dressings"`
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

type dressingStepSpec struct {
	name        string
	goal        spatialmath.Pose
	constraints *motionplan.Constraints
	postAction  GrabStepAction
	postShake   bool
}

type dressingStep struct {
	name         string
	trajectory   motionplan.Trajectory
	planningTime time.Duration
	postAction   GrabStepAction
	postShake    bool
}

type dressingPlan struct {
	dressingName string
	steps        []dressingStep
	plannedAt    time.Time
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
	if v, ok := cmd["pour_dressing"]; ok {
		name, ok := v.(string)
		if !ok || name == "" {
			return nil, fmt.Errorf("pour_dressing requires a dressing name (string)")
		}
		return s.doPourDressing(ctx, name)
	}
	if _, ok := cmd["reset"]; ok {
		return s.reset(ctx)
	}
	return nil, fmt.Errorf("unknown command, expected 'pour_dressing' or 'reset' field")
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

func (s *dressingControls) planDressing(ctx context.Context, name string) (*dressingPlan, error) {
	opt, ok := s.cfg.Dressings[name]
	if !ok {
		return nil, fmt.Errorf("unknown dressing %q", name)
	}

	if err := s.loadWorldState(); err != nil {
		return nil, err
	}

	specs := []dressingStepSpec{
		{name: "approach_grab",        goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "grab",                 goal: opt.Grab.toPose(),                constraints: opt.Grab.Constraints,               postAction: GrabStepActionClose},
		{name: "approach_grab_up",     goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "prepare_dressing",     goal: s.cfg.PrepareDressing.toPose(),  constraints: s.cfg.PrepareDressing.Constraints},
		{name: "pour",                 goal: s.cfg.PourDressing.toPose(),     constraints: s.cfg.PourDressing.Constraints,      postShake: true},
		{name: "pour_2",               goal: s.cfg.PourDressing2.toPose(),    constraints: s.cfg.PourDressing2.Constraints,     postShake: true},
		{name: "pour_back",            goal: s.cfg.PourDressing.toPose(),     constraints: s.cfg.PourDressing.Constraints,      postShake: true},
		{name: "post_pour",            goal: s.cfg.PostPourDressing.toPose(), constraints: s.cfg.PostPourDressing.Constraints},
		{name: "prepare_return",       goal: s.cfg.PrepareDressing.toPose(),  constraints: s.cfg.PrepareDressing.Constraints},
		{name: "approach_grab_return", goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "grab_return",          goal: opt.Grab.toPose(),               constraints: opt.Grab.Constraints,                postAction: GrabStepActionOpen},
		{name: "approach_grab_final",  goal: opt.ApproachGrab.toPose(),       constraints: opt.ApproachGrab.Constraints},
		{name: "home",                 goal: s.cfg.Home.toPose(),             constraints: s.cfg.Home.Constraints},
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
	startState := armplanning.NewPlanState(nil, startInputs)

	steps := make([]dressingStep, 0, len(specs))
	for _, spec := range specs {
		goalState := armplanning.NewPlanState(
			referenceframe.FrameSystemPoses{
				s.cfg.Arm: referenceframe.NewPoseInFrame(referenceframe.World, spec.goal),
			},
			nil,
		)
		req := &armplanning.PlanRequest{
			FrameSystem: fs,
			WorldState:  s.worldState,
			StartState:  startState,
			Goals:       []*armplanning.PlanState{goalState},
			Constraints: spec.constraints,
		}

		t := time.Now()
		plan, _, err := armplanning.PlanMotion(ctx, s.logger, req)
		planDur := time.Since(t)
		s.logger.Infof("planned step %q in %.2fs", spec.name, planDur.Seconds())
		if err != nil {
			return nil, fmt.Errorf("planning step %q: %w", spec.name, err)
		}

		traj := plan.Trajectory()
		steps = append(steps, dressingStep{
			name:         spec.name,
			trajectory:   traj,
			planningTime: planDur,
			postAction:   spec.postAction,
			postShake:    spec.postShake,
		})

		if len(traj) > 0 {
			startState = armplanning.NewPlanState(nil, traj[len(traj)-1])
		}
	}

	return &dressingPlan{dressingName: name, steps: steps, plannedAt: time.Now()}, nil
}

func (s *dressingControls) executeDressing(ctx context.Context, plan *dressingPlan) error {
	if err := s.gripper.Open(ctx, nil); err != nil {
		return fmt.Errorf("open gripper: %w", err)
	}

	for _, step := range plan.steps {
		armInputs := make([][]referenceframe.Input, len(step.trajectory))
		for i, fsInputs := range step.trajectory {
			armInputs[i] = fsInputs[s.cfg.Arm]
		}

		if err := s.arm.MoveThroughJointPositions(ctx, armInputs, nil, nil); err != nil {
			return fmt.Errorf("step %q: %w", step.name, err)
		}
		s.logger.Debugf("completed step %q", step.name)

		switch step.postAction {
		case GrabStepActionOpen:
			if err := s.gripper.Open(ctx, nil); err != nil {
				return fmt.Errorf("step %q: open gripper: %w", step.name, err)
			}
			s.logger.Debugf("opened gripper after %q", step.name)
		case GrabStepActionClose:
			if _, err := s.gripper.Grab(ctx, nil); err != nil {
				return fmt.Errorf("step %q: close gripper: %w", step.name, err)
			}
			s.logger.Debugf("closed gripper after %q", step.name)
		}

		if step.postShake && s.shakeArmService != nil {
			if _, err := s.shakeArmService.DoCommand(ctx, map[string]interface{}{"shake_arm": true}); err != nil {
				return fmt.Errorf("step %q: shake arm: %w", step.name, err)
			}
			s.logger.Debugf("shook arm after %q", step.name)
		}
	}
	return nil
}

func (s *dressingControls) doPourDressing(ctx context.Context, name string) (map[string]interface{}, error) {
	s.logger.Infof("Planning pour_dressing for %q", name)
	plan, err := s.planDressing(ctx, name)
	if err != nil {
		return nil, err
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
	return nil
}
