package main

import (
	"context"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sort"
	"time"

	"github.com/golang/geo/r3"
	"github.com/spf13/cobra"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/motionplan"
	"go.viam.com/rdk/motionplan/armplanning"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"

	"salad/segmentation"
)

type GrabFlags struct {
	ZonesPath         string
	MeshPath          string
	ZoneID            int
	ArmName           string
	EndEffectorFrame  string
	GripperName       string
	CameraName        string
	ScaleName         string
	SeparationMM      float64
	HoverDistanceMM   float64
	InitialDepthMM    float64
	DepthIncrementMM  float64
	MaxGrabZ          float64 // hard floor: grab Z will never go below this value (0 = no limit)
	MaxDepthPositions int
	GrabSettleMs      int
	ScaleSettleMs     int
	MinDepositG       float64
	WeightThresholdG  float64
	TargetWeightG     float64
	ScaleDropX           float64
	ScaleDropY           float64
	ScaleDropZ           float64
	ScaleApproachHoverMM float64 // transit height: arm travels here before descending to drop
	ScaleDropHoverMM     float64 // drop height: arm descends here before opening gripper
}

var grabFlags GrabFlags

var grabCmd = &cobra.Command{
	Use:   "grab",
	Short: "Autonomously grab ingredients from a bin using motion-planned trajectories",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runGrab(globalAddress, globalAPIKey, globalAPIKeyID, grabFlags)
	},
}

type grabTarget struct {
	x, y, z float64
}

func runGrab(address, apiKey, apiKeyID string, flags GrabFlags) error {
	if address == "" || apiKey == "" || apiKeyID == "" {
		return fmt.Errorf("--address, --api-key, and --api-key-id are required")
	}

	ctx := context.Background()
	logger := logging.NewLogger("grab")

	// Load zones
	logger.Infof("Loading zones from %s", flags.ZonesPath)
	zones, err := loadZonesFile(flags.ZonesPath)
	if err != nil {
		return fmt.Errorf("loading zones: %w", err)
	}
	zone, err := findZone(zones, flags.ZoneID)
	if err != nil {
		return err
	}
	logger.Infof("Zone %d: X[%.0f,%.0f] Y[%.0f,%.0f]", zone.ID, zone.MinX, zone.MaxX, zone.MinY, zone.MaxY)

	// Load mesh and build world state (mesh → octree for fast collision checking)
	logger.Infof("Loading mesh from %s", flags.MeshPath)
	mesh, err := spatialmath.NewMeshFromPLYFile(flags.MeshPath)
	if err != nil {
		return fmt.Errorf("loading mesh: %w", err)
	}
	worldState, err := buildWorldState(mesh)
	if err != nil {
		return fmt.Errorf("building world state: %w", err)
	}

	logger.Infof("Connecting to %s", address)
	robotClient, err := client.New(ctx, address, logger,
		client.WithDialOptions(
			rpc.WithEntityCredentials(apiKeyID, rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			}),
		),
	)
	if err != nil {
		return fmt.Errorf("connecting to robot: %w", err)
	}
	defer robotClient.Close(ctx)
	logger.Infof("Connected")

	armComp, err := arm.FromRobot(robotClient, flags.ArmName)
	if err != nil {
		names := robotClient.ResourceNames()
		logger.Infof("Available resources (%d):", len(names))
		for _, n := range names {
			logger.Infof("  %s", n)
		}
		return fmt.Errorf("arm %q: %w", flags.ArmName, err)
	}

	gripperComp, err := gripper.FromRobot(robotClient, flags.GripperName)
	if err != nil {
		return fmt.Errorf("gripper %q: %w", flags.GripperName, err)
	}

	cam, err := camera.FromRobot(robotClient, flags.CameraName)
	if err != nil {
		return fmt.Errorf("camera %q: %w", flags.CameraName, err)
	}

	scaleSensor, err := sensor.FromRobot(robotClient, flags.ScaleName)
	if err != nil {
		return fmt.Errorf("scale %q: %w", flags.ScaleName, err)
	}

	fsCfg, err := robotClient.FrameSystemConfig(ctx)
	if err != nil {
		return fmt.Errorf("getting frame system config: %w", err)
	}
	fs, err := referenceframe.NewFrameSystem("", fsCfg.Parts, nil)
	if err != nil {
		return fmt.Errorf("building frame system: %w", err)
	}

	endEffectorFrame := flags.EndEffectorFrame
	if endEffectorFrame == "" {
		endEffectorFrame = flags.ArmName
	}

	logger.Infof("Capturing point cloud from %q", flags.CameraName)
	pc, err := cam.NextPointCloud(ctx, nil)
	if err != nil {
		return fmt.Errorf("capturing point cloud: %w", err)
	}
	logger.Infof("Captured %d points", pc.Size())

	// Seed start state from current arm pose
	startState, err := currentArmPlanState(ctx, armComp, fs)
	if err != nil {
		return fmt.Errorf("reading arm state: %w", err)
	}

	// Find up to 3 collision-free, well-separated high points in the zone
	logger.Infof("Finding grab targets (separation=%.0fmm, hover=%.0fmm)", flags.SeparationMM, flags.HoverDistanceMM)
	targets, err := findGrabTargets(ctx, logger, pc, zone, fs, worldState, startState, endEffectorFrame, flags.SeparationMM, flags.HoverDistanceMM)
	if err != nil {
		return err
	}
	if len(targets) == 0 {
		return fmt.Errorf("no collision-free grab targets found in zone %d", flags.ZoneID)
	}
	logger.Infof("Found %d collision-free target(s):", len(targets))
	for i, t := range targets {
		logger.Infof("  [%d] (%.1f, %.1f, %.1f)", i, t.x, t.y, t.z)
	}

	fsGrab, err := buildGrabFrameSystem(fsCfg.Parts)
	if err != nil {
		return fmt.Errorf("building grab frame system: %w", err)
	}
	return executeGrabLoop(ctx, logger, targets, armComp, gripperComp, scaleSensor, fs, fsGrab, worldState, endEffectorFrame, flags)
}

func findGrabTargets(
	ctx context.Context,
	logger logging.Logger,
	pc pointcloud.PointCloud,
	zone *segmentation.Zone,
	fs *referenceframe.FrameSystem,
	worldState *referenceframe.WorldState,
	startState *armplanning.PlanState,
	endEffectorFrame string,
	separationMM float64,
	hoverDistanceMM float64,
) ([]grabTarget, error) {
	type pt struct{ x, y, z float64 }
	var pts []pt

	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		if p.X >= zone.MinX && p.X <= zone.MaxX && p.Y >= zone.MinY && p.Y <= zone.MaxY {
			pts = append(pts, pt{p.X, p.Y, p.Z})
		}
		return true
	})

	if len(pts) == 0 {
		return nil, fmt.Errorf("no point cloud points fall within zone %d bounds", zone.ID)
	}

	sort.Slice(pts, func(i, j int) bool { return pts[i].z > pts[j].z })

	var accepted []grabTarget

	for _, c := range pts {
		if len(accepted) >= 3 {
			break
		}

		tooClose := false
		for _, a := range accepted {
			dx, dy := c.x-a.x, c.y-a.y
			if math.Sqrt(dx*dx+dy*dy) < separationMM {
				tooClose = true
				break
			}
		}
		if tooClose {
			continue
		}

		hoverPose := spatialmath.NewPose(
			r3.Vector{X: c.x, Y: c.y, Z: c.z + hoverDistanceMM},
			&spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 90},
		)
		if _, err := planTo(ctx, logger, fs, worldState, startState, endEffectorFrame, hoverPose, nil); err != nil {
			logger.Debugf("Candidate (%.0f,%.0f,%.0f): plan failed (%v), skipping", c.x, c.y, c.z, err)
			continue
		}

		logger.Infof("Accepted target (%.0f, %.0f, %.0f)", c.x, c.y, c.z)
		accepted = append(accepted, grabTarget{c.x, c.y, c.z})
	}

	return accepted, nil
}

func executeGrabLoop(
	ctx context.Context,
	logger logging.Logger,
	targets []grabTarget,
	armComp arm.Arm,
	gripperComp gripper.Gripper,
	scaleSensor sensor.Sensor,
	fs *referenceframe.FrameSystem,
	fsGrab *referenceframe.FrameSystem,
	worldState *referenceframe.WorldState,
	endEffectorFrame string,
	flags GrabFlags,
) error {
	totalWeight := 0.0
	newGrabFailures := 0
	lastProductiveIdx := -1
	lastSuccessAttempt := 0
	nextNewIdx := 0

	for newGrabFailures < len(targets) {
		if flags.TargetWeightG > 0 && totalWeight >= flags.TargetWeightG {
			break
		}

		isRetry := lastProductiveIdx >= 0
		var targetIdx int
		if isRetry {
			targetIdx = lastProductiveIdx
		} else {
			if nextNewIdx >= len(targets) {
				break
			}
			targetIdx = nextNewIdx
			lastSuccessAttempt = 0
		}

		target := targets[targetIdx]
		logger.Infof("--- Target %d (%.0f,%.0f,%.0f) retry=%v startAttempt=%d totalWeight=%.1fg ---",
			targetIdx, target.x, target.y, target.z, isRetry, lastSuccessAttempt, totalWeight)

		weightDeposited, successAttempt, skipPosition, err := attemptGrabWithDepthStepping(
			ctx, logger, target, armComp, gripperComp, scaleSensor, fs, fsGrab, worldState, endEffectorFrame, flags, lastSuccessAttempt,
		)
		if err != nil {
			return err
		}

		switch {
		case skipPosition:
			logger.Infof("Target %d: unreachable — abandoning", targetIdx)
			lastProductiveIdx = -1
			lastSuccessAttempt = 0
			nextNewIdx++
			newGrabFailures++

		case weightDeposited > 0:
			// Any positive deposit: retry at the same depth.
			// Only count toward the target total if it meets the threshold.
			if weightDeposited >= flags.WeightThresholdG {
				totalWeight += weightDeposited
				logger.Infof("Target %d: deposited %.1fg (total %.1fg / target %.1fg) — retrying at same depth",
					targetIdx, weightDeposited, totalWeight, flags.TargetWeightG)
			} else {
				logger.Infof("Target %d: partial deposit %.1fg (below weight threshold %.1fg) — retrying at same depth",
					targetIdx, weightDeposited, flags.WeightThresholdG)
			}
			lastProductiveIdx = targetIdx
			lastSuccessAttempt = successAttempt
			newGrabFailures = 0

		default:
			logger.Infof("Target %d: exhausted all attempts without sufficient weight gain", targetIdx)
			lastProductiveIdx = -1
			lastSuccessAttempt = 0
			nextNewIdx++
			newGrabFailures++
		}
	}

	logger.Infof("Grab loop complete — total weight deposited: %.1fg", totalWeight)
	return nil
}

// attemptGrabWithDepthStepping steps through up to MaxDepthPositions distinct depth levels
// starting from startAttempt. At each depth it grabs once; if a meaningful deposit is made
// (>= MinDepositG) it returns immediately so the caller can retry at that same depth
// indefinitely. Only a zero/noise deposit advances to the next depth.
// Returns the weight deposited, the depth-position index that succeeded (-1 if none),
// whether the position should be skipped entirely, and any fatal error.
func attemptGrabWithDepthStepping(
	ctx context.Context,
	logger logging.Logger,
	target grabTarget,
	armComp arm.Arm,
	gripperComp gripper.Gripper,
	scaleSensor sensor.Sensor,
	fs *referenceframe.FrameSystem,
	fsGrab *referenceframe.FrameSystem,
	worldState *referenceframe.WorldState,
	endEffectorFrame string,
	flags GrabFlags,
	startAttempt int,
) (weightDeposited float64, successAttempt int, skipPosition bool, err error) {
	hoverPose := spatialmath.NewPose(
		r3.Vector{X: target.x, Y: target.y, Z: target.z + flags.HoverDistanceMM},
		&spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 90},
	)

	// Open gripper before approaching so it is safe to enter the bin area.
	if err := gripperComp.Open(ctx, nil); err != nil {
		return 0, -1, false, fmt.Errorf("opening gripper before approach: %w", err)
	}

	// Approach to hover above bin: unconstrained so the arm can swing freely from
	// wherever it starts (e.g. scale or home). The tongs at Theta=90 extend in ±Y
	// and would collide with the mesh of the narrow bin walls even at hover height,
	// so we use fsGrab (no static mesh obstacles).
	approachState, err := currentArmPlanState(ctx, armComp, fsGrab)
	if err != nil {
		return 0, -1, false, fmt.Errorf("reading arm state for approach: %w", err)
	}
	approachTraj, err := planTo(ctx, logger, fsGrab, nil, approachState, endEffectorFrame, hoverPose, nil)
	if err != nil {
		logger.Infof("Approach to hover (%.0f, %.0f, %.0f) failed (%v) — skipping position",
			target.x, target.y, target.z+flags.HoverDistanceMM, err)
		return 0, -1, true, nil
	}
	if err := executeTraj(ctx, armComp, approachTraj, flags.ArmName); err != nil {
		return 0, -1, false, fmt.Errorf("executing approach to hover: %w", err)
	}

	scaleApproachPose := spatialmath.NewPose(
		r3.Vector{X: flags.ScaleDropX, Y: flags.ScaleDropY, Z: flags.ScaleDropZ + flags.ScaleApproachHoverMM},
		&spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 90},
	)
	scaleDropPose := spatialmath.NewPose(
		r3.Vector{X: flags.ScaleDropX, Y: flags.ScaleDropY, Z: flags.ScaleDropZ + flags.ScaleDropHoverMM},
		&spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 90},
	)

	linC := linearConstraints()

	// Depth-stepping loop: starts at startAttempt so successful retries resume at the
	// same depth rather than restarting from the rim.
	for attempt := startAttempt; attempt < flags.MaxDepthPositions; attempt++ {
		depth := flags.InitialDepthMM + float64(attempt)*flags.DepthIncrementMM
		grabZ := target.z - depth
		if flags.MaxGrabZ != 0 && grabZ < flags.MaxGrabZ {
			logger.Infof("Attempt %d: grabZ=%.0f would go below floor buffer (maxGrabZ=%.0f) — stopping", attempt+1, grabZ, flags.MaxGrabZ)
			break
		}
		grabPose := spatialmath.NewPose(
			r3.Vector{X: target.x, Y: target.y, Z: grabZ},
			&spatialmath.OrientationVectorDegrees{OZ: -1, Theta: 90},
		)

		logger.Infof("Depth position %d/%d: descending to Z=%.0f (depth offset=%.0fmm)",
			attempt+1, flags.MaxDepthPositions, grabZ, depth)

		// Open gripper before descending (re-opens after each failed scale drop).
		if err := gripperComp.Open(ctx, nil); err != nil {
			return 0, -1, false, fmt.Errorf("opening gripper: %w", err)
		}

		// Descent: linear — straight down into the bin from hover.
		grabState, err := currentArmPlanState(ctx, armComp, fsGrab)
		if err != nil {
			return 0, -1, false, fmt.Errorf("reading arm state for grab: %w", err)
		}
		grabTraj, err := planTo(ctx, logger, fsGrab, nil, grabState, endEffectorFrame, grabPose, linC)
		if err != nil {
			logger.Infof("Attempt %d: descent unreachable — skipping position", attempt+1)
			return 0, -1, true, nil
		}
		if err := executeTraj(ctx, armComp, grabTraj, flags.ArmName); err != nil {
			return 0, -1, false, fmt.Errorf("executing descent: %w", err)
		}
		if flags.GrabSettleMs > 0 {
			time.Sleep(time.Duration(flags.GrabSettleMs) * time.Millisecond)
		}

		if _, err := gripperComp.Grab(ctx, nil); err != nil {
			return 0, -1, false, fmt.Errorf("grabbing: %w", err)
		}
		if flags.GrabSettleMs > 0 {
			time.Sleep(time.Duration(flags.GrabSettleMs) * time.Millisecond)
		}
		// Hold closed — gripper stays closed until we are over the scale.
		if err := gripperComp.Stop(ctx, nil); err != nil {
			return 0, -1, false, fmt.Errorf("stopping gripper before retreat: %w", err)
		}

		// Retreat: linear — straight up out of the bin back to hover.
		retreatState, err := currentArmPlanState(ctx, armComp, fsGrab)
		if err != nil {
			return 0, -1, false, fmt.Errorf("reading arm state for retreat: %w", err)
		}
		retreatTraj, err := planTo(ctx, logger, fsGrab, nil, retreatState, endEffectorFrame, hoverPose, linC)
		if err != nil {
			return 0, -1, false, fmt.Errorf("planning retreat to bin hover: %w", err)
		}

		// Pre-plan scale transit from the retreat end state so there is no planning
		// delay between retreat completion and the arm moving toward the scale.
		scaleTransitTraj, err := planTo(ctx, logger, fsGrab, nil,
			trajEndState(retreatTraj, flags.ArmName, fsGrab), endEffectorFrame, scaleApproachPose, linC)
		if err != nil {
			return 0, -1, false, fmt.Errorf("planning transit to scale approach hover: %w", err)
		}

		if err := executeTraj(ctx, armComp, retreatTraj, flags.ArmName); err != nil {
			return 0, -1, false, fmt.Errorf("executing retreat to bin hover: %w", err)
		}
		if err := executeTraj(ctx, armComp, scaleTransitTraj, flags.ArmName); err != nil {
			return 0, -1, false, fmt.Errorf("executing transit to scale approach hover: %w", err)
		}

		// Descend to drop height: unconstrained — forcing a straight vertical line at the
		// scale XY position can push the arm into awkward configurations that trigger the safety stop.
		scaleDescentState, err := currentArmPlanState(ctx, armComp, fsGrab)
		if err != nil {
			return 0, -1, false, fmt.Errorf("reading arm state for scale descent: %w", err)
		}
		scaleDescentTraj, err := planTo(ctx, logger, fsGrab, nil, scaleDescentState, endEffectorFrame, scaleDropPose, nil)
		if err != nil {
			return 0, -1, false, fmt.Errorf("planning descent to scale drop height: %w", err)
		}
		if err := executeTraj(ctx, armComp, scaleDescentTraj, flags.ArmName); err != nil {
			return 0, -1, false, fmt.Errorf("executing descent to scale drop height: %w", err)
		}

		// Read baseline, open gripper, wait for food to settle, then read again.
		weightBefore, err := readWeight(ctx, scaleSensor)
		if err != nil {
			return 0, -1, false, fmt.Errorf("reading scale before drop: %w", err)
		}
		if err := gripperComp.Open(ctx, nil); err != nil {
			return 0, -1, false, fmt.Errorf("opening gripper over scale: %w", err)
		}
		if flags.ScaleSettleMs > 0 {
			time.Sleep(time.Duration(flags.ScaleSettleMs) * time.Millisecond)
		}
		weightAfter, err := readWeight(ctx, scaleSensor)
		if err != nil {
			return 0, -1, false, fmt.Errorf("reading scale after drop: %w", err)
		}

		delta := weightAfter - weightBefore
		logger.Infof("Attempt %d: deposited %.1fg on scale (threshold %.1fg)", attempt+1, delta, flags.WeightThresholdG)
		// Rise back to scale approach hover before returning or looping: ensures the arm
		// is always at transit height when leaving the scale, whether the deposit succeeded or not.
		scaleRiseState, err := currentArmPlanState(ctx, armComp, fsGrab)
		if err != nil {
			return 0, -1, false, fmt.Errorf("reading arm state for scale rise: %w", err)
		}
		scaleRiseTraj, err := planTo(ctx, logger, fsGrab, nil, scaleRiseState, endEffectorFrame, scaleApproachPose, nil)
		if err != nil {
			return 0, -1, false, fmt.Errorf("planning rise to scale approach hover: %w", err)
		}
		if err := executeTraj(ctx, armComp, scaleRiseTraj, flags.ArmName); err != nil {
			return 0, -1, false, fmt.Errorf("executing rise to scale approach hover: %w", err)
		}

		if delta >= flags.MinDepositG {
			// Meaningful deposit: food is at this depth — return so executeGrabLoop
			// retries at the same depth. Arm is already at transit height.
			return delta, attempt, false, nil
		}

		// Return to bin hover: linear for the same reason.
		returnState, err := currentArmPlanState(ctx, armComp, fsGrab)
		if err != nil {
			return 0, -1, false, fmt.Errorf("reading arm state for return to bin hover: %w", err)
		}
		returnTraj, err := planTo(ctx, logger, fsGrab, nil, returnState, endEffectorFrame, hoverPose, linC)
		if err != nil {
			return 0, -1, false, fmt.Errorf("planning return to bin hover: %w", err)
		}
		if err := executeTraj(ctx, armComp, returnTraj, flags.ArmName); err != nil {
			return 0, -1, false, fmt.Errorf("executing return to bin hover: %w", err)
		}
	}

	return 0, -1, false, nil
}

func planTo(
	ctx context.Context,
	logger logging.Logger,
	fs *referenceframe.FrameSystem,
	worldState *referenceframe.WorldState,
	startState *armplanning.PlanState,
	frameName string,
	targetPose spatialmath.Pose,
	constraints *motionplan.Constraints,
) (motionplan.Trajectory, error) {
	goalState := armplanning.NewPlanState(
		referenceframe.FrameSystemPoses{
			frameName: referenceframe.NewPoseInFrame(referenceframe.World, targetPose),
		},
		nil,
	)
	plan, _, err := armplanning.PlanMotion(ctx, logger, &armplanning.PlanRequest{
		FrameSystem: fs,
		WorldState:  worldState,
		StartState:  startState,
		Goals:       []*armplanning.PlanState{goalState},
		Constraints: constraints,
	})
	if err != nil {
		return nil, err
	}
	return plan.Trajectory(), nil
}

// linearConstraints returns a Constraints with a single LinearConstraint using
// reasonable tolerances for straight-line cartesian moves.
func linearConstraints() *motionplan.Constraints {
	return &motionplan.Constraints{
		LinearConstraint: []motionplan.LinearConstraint{
			{LineToleranceMm: 1.0, OrientationToleranceDegs: 2.0},
		},
	}
}

// executeTraj extracts joint configurations for the arm from each trajectory step and
// sends them all at once via GoToInputs.
func executeTraj(ctx context.Context, armComp arm.Arm, traj motionplan.Trajectory, armName string) error {
	var steps [][]referenceframe.Input
	for _, step := range traj {
		inputs, ok := step[armName]
		if !ok {
			continue
		}
		steps = append(steps, inputs)
	}
	if len(steps) == 0 {
		return nil
	}
	return armComp.GoToInputs(ctx, steps...)
}

// trajEndState returns a PlanState representing the final joint configuration of a trajectory.
// Used to pre-plan the next move before executing the current one, eliminating planning delays.
func trajEndState(traj motionplan.Trajectory, armName string, fs *referenceframe.FrameSystem) *armplanning.PlanState {
	var last []referenceframe.Input
	for _, step := range traj {
		if inputs, ok := step[armName]; ok {
			last = inputs
		}
	}
	fsInputs := referenceframe.NewZeroInputs(fs)
	if last != nil {
		fsInputs[armName] = last
	}
	return armplanning.NewPlanState(nil, fsInputs)
}

// currentArmPlanState reads the arm's current joint positions and wraps them in a PlanState.
func currentArmPlanState(ctx context.Context, armComp arm.Arm, fs *referenceframe.FrameSystem) (*armplanning.PlanState, error) {
	armInputs, err := armComp.CurrentInputs(ctx)
	if err != nil {
		return nil, err
	}
	fsInputs := referenceframe.NewZeroInputs(fs)
	fsInputs[armComp.Name().Name] = armInputs
	return armplanning.NewPlanState(nil, fsInputs), nil
}

// buildGrabFrameSystem returns a frame system with geometry stripped from all non-kinematic frames
// (static world obstacles like fridge, table, bounds). Kinematic frames (robot arms with joints)
// are kept intact for self-collision checking. Used for descent planning so the arm can enter
// the bin without being blocked by the fridge geometry.
func buildGrabFrameSystem(parts []*referenceframe.FrameSystemPart) (*referenceframe.FrameSystem, error) {
	stripped := make([]*referenceframe.FrameSystemPart, len(parts))
	for i, p := range parts {
		if p.ModelFrame != nil && len(p.ModelFrame.DoF()) > 0 {
			// Kinematic model (arm with joints) — keep as-is.
			stripped[i] = p
			continue
		}
		// Static frame or link-only — strip geometry.
		fc := p.FrameConfig
		stripped[i] = &referenceframe.FrameSystemPart{
			FrameConfig: referenceframe.NewLinkInFrame(fc.Parent(), fc.Pose(), fc.Name(), nil),
		}
	}
	return referenceframe.NewFrameSystem("grab", stripped, nil)
}

// buildWorldState converts a PLY mesh into an octree-backed obstacle for efficient collision checking.
// Using an octree rather than the raw mesh is critical for motion planning performance.
func buildWorldState(mesh *spatialmath.Mesh) (*referenceframe.WorldState, error) {
	octree, err := pointcloud.NewFromMesh(mesh)
	if err != nil {
		return nil, fmt.Errorf("converting mesh to octree: %w", err)
	}
	return referenceframe.NewWorldState(
		[]*referenceframe.GeometriesInFrame{
			referenceframe.NewGeometriesInFrame(referenceframe.World, []spatialmath.Geometry{octree}),
		},
		[]*referenceframe.LinkInFrame{},
	)
}

// readWeight reads the current weight (grams) from the scale sensor's "weight" reading key.
func readWeight(ctx context.Context, s sensor.Sensor) (float64, error) {
	readings, err := s.Readings(ctx, nil)
	if err != nil {
		return 0, err
	}
	val, ok := readings["weight"]
	if !ok {
		return 0, fmt.Errorf("scale sensor missing 'weight' key (available: %v)", readings)
	}
	switch v := val.(type) {
	case float64:
		return v, nil
	case int:
		return float64(v), nil
	case int64:
		return float64(v), nil
	default:
		return 0, fmt.Errorf("unexpected weight value type %T", val)
	}
}

// loadZonesFile reads a zones.json file produced by the segment command.
func loadZonesFile(path string) ([]segmentation.Zone, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, err
	}
	defer f.Close()
	var result segmentation.ZonesResult
	if err := json.NewDecoder(f).Decode(&result); err != nil {
		return nil, err
	}
	return result.Zones, nil
}

// findZone returns the zone matching the given ID.
func findZone(zones []segmentation.Zone, id int) (*segmentation.Zone, error) {
	for i := range zones {
		if zones[i].ID == id {
			return &zones[i], nil
		}
	}
	ids := make([]int, len(zones))
	for i, z := range zones {
		ids[i] = z.ID
	}
	return nil, fmt.Errorf("zone %d not found (available IDs: %v)", id, ids)
}
