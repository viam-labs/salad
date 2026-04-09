package main

import (
	"context"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/components/sensor"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

const (
	sphereRadiusMM       = 25.0
	sphereStepMM         = 5.0
	grabAboveMeshFloorMM = 260.0 // end-effector Z above mesh-reported floor (lower = deeper into bin)
	// descentMeshClearanceMM: if any mesh vertex lies within this radius of the planned EE position at
	// end of descent, we skip that lateral X offset (bin wall / geometry too close).
	descentMeshClearanceMM = 40.0

	gripperIsMovingPoll       = 25 * time.Millisecond
	gripperGrabStopWait       = 15 * time.Second       // max wait for IsMoving to go false after Grab
	gripperSettleAfterStill   = 350 * time.Millisecond // mechanical slack after driver says stopped
	gripperSettleIfNoIsMoving = 800 * time.Millisecond // if IsMoving is unsupported / errors
)

// waitGripperSettledAfterGrab blocks until the gripper Actuator reports not moving, then waits a bit longer
// for the fingers to finish. Grab() is supposed to block until done, but many drivers return early.
func waitGripperSettledAfterGrab(ctx context.Context, g gripper.Gripper, logger logging.Logger) error {
	deadline := time.Now().Add(gripperGrabStopWait)
	for {
		if err := ctx.Err(); err != nil {
			return err
		}
		if time.Now().After(deadline) {
			return fmt.Errorf("timeout waiting for gripper to stop after grab (%s)", gripperGrabStopWait)
		}
		moving, err := g.IsMoving(ctx)
		if err != nil {
			logger.Warnf("gripper IsMoving after grab failed (%v); using fixed settle delay", err)
			time.Sleep(gripperSettleIfNoIsMoving)
			return nil
		}
		if !moving {
			time.Sleep(gripperSettleAfterStill)
			return nil
		}
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-time.After(gripperIsMovingPoll):
		}
	}
}

func newSpherePointCloud(center r3.Vector, radiusMM, stepMM float64) pointcloud.PointCloud {
	pc := pointcloud.NewBasicPointCloud(0)
	for dx := -radiusMM; dx <= radiusMM; dx += stepMM {
		for dy := -radiusMM; dy <= radiusMM; dy += stepMM {
			for dz := -radiusMM; dz <= radiusMM; dz += stepMM {
				if dx*dx+dy*dy+dz*dz <= radiusMM*radiusMM {
					_ = pc.Set(r3.Vector{X: center.X + dx, Y: center.Y + dy, Z: center.Z + dz}, pointcloud.NewBasicData())
				}
			}
		}
	}
	return pc
}

func saveSpherePCD(logger logging.Logger, path string, center r3.Vector, radiusMM, stepMM float64) error {
	pc := newSpherePointCloud(center, radiusMM, stepMM)
	if err := writePCD(pc, path); err != nil {
		return err
	}
	logger.Infof("Saved sphere to %s (X=%.0f Y=%.0f Z=%.0f, %d pts)", path, center.X, center.Y, center.Z, pc.Size())
	return nil
}

type GrabFlags struct {
	Bin       int
	ArmName   string
	CamName   string
	MeshFile  string
	OutputDir string
	StepMM    float64

	// add-ingredient fields
	GripperName string
	ScaleName   string
	TargetGrams float64
	BowlHoverX  float64 // world-frame X of hover position above bowl (mm)
	BowlHoverY  float64 // world-frame Y
	BowlHoverZ  float64 // world-frame Z
	BowlDropX   float64 // world-frame X at drop (mm)
	BowlDropY   float64
	BowlDropZ   float64
}

func runGrab(address, apiKey, apiKeyID string, flags GrabFlags) error {
	if address == "" || apiKey == "" || apiKeyID == "" {
		return fmt.Errorf("--address, --api-key, and --api-key-id are required")
	}

	ctx := context.Background()
	logger := logging.NewLogger("grab-test")

	logger.Infof("Loading mesh from %s", flags.MeshFile)
	mesh, err := spatialmath.NewMeshFromPLYFile(flags.MeshFile)
	if err != nil {
		return fmt.Errorf("loading mesh: %w", err)
	}
	meshPts := mesh.ToPoints(1)

	outputDir := flags.OutputDir
	if outputDir == "" {
		outputDir = filepath.Join("output", time.Now().Format("20060102-150405"))
	}

	logger.Infof("Connecting to robot at %s", address)
	robotClient, err := client.New(ctx, address, logger,
		client.WithDialOptions(
			rpc.WithEntityCredentials(apiKeyID, rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			}),
		),
	)
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}
	defer robotClient.Close(ctx)

	motionSvc, err := motion.FromRobot(robotClient, "builtin")
	if err != nil {
		return fmt.Errorf("failed to get motion service: %w", err)
	}

	cam, err := camera.FromRobot(robotClient, flags.CamName)
	if err != nil {
		return fmt.Errorf("failed to get camera %q: %w", flags.CamName, err)
	}

	aboveName := fmt.Sprintf("bin-%d-imaging", flags.Bin)
	aboveSw, err := toggleswitch.FromRobot(robotClient, aboveName)
	if err != nil {
		return fmt.Errorf("failed to get switch %q: %w", aboveName, err)
	}
	logger.Infof("Moving to %q", aboveName)
	if err := aboveSw.SetPosition(ctx, 2, nil); err != nil {
		return fmt.Errorf("failed to move to above-bin: %w", err)
	}
	time.Sleep(2 * time.Second)

	endPoseInWorld, err := motionSvc.GetPose(ctx, flags.ArmName, referenceframe.World, nil, nil)
	if err != nil {
		return fmt.Errorf("failed to get arm world pose: %w", err)
	}
	endPose := endPoseInWorld.Pose()

	switchX := endPose.Point().X
	switchY := endPose.Point().Y
	switchZ := endPose.Point().Z
	logger.Infof("Arm at bin imaging switch (world): X=%.0f Y=%.0f Z=%.0f", switchX, switchY, switchZ)

	binCenterX, err := meshBinCenterX(meshPts, switchX, switchY, logger)
	if err != nil {
		return fmt.Errorf("failed to determine bin center X from mesh: %w", err)
	}
	imagingX := endPose.Point().X
	imagingY := endPose.Point().Y
	startZ := endPose.Point().Z

	// Sample mesh floor under imaging XY so "bottom" matches where the arm descends. Using
	// binCenterX can hit the wrong region of a merged six-bin mesh and read a too-low Z.
	meshFloorZ, err := meshAvgLowestZ(meshPts, imagingX, imagingY, logger)
	if err != nil {
		return fmt.Errorf("failed to determine target Z from mesh floor: %w", err)
	}
	targetZ := meshFloorZ + grabAboveMeshFloorMM
	logger.Infof("Imaging pose (world): X=%.0f Y=%.0f Z=%.0f", imagingX, imagingY, startZ)

	// Lateral move to mesh rim X is often IK-unreachable at fixed imaging orientation;
	// descend at imaging XY. Mesh rim X is logged for alignment debugging.
	descentX := imagingX
	logger.Infof("Descent X=%.0f Y=%.0f (mesh rim X=%.0f, ΔX=%.0fmm — not used for motion)",
		descentX, imagingY, binCenterX, binCenterX-imagingX)

	imagingSpherePath := filepath.Join(outputDir, fmt.Sprintf("bin-%d-imaging-pose.pcd", flags.Bin))
	if err := os.MkdirAll(outputDir, 0o755); err != nil {
		logger.Warnf("Failed to create output directory %q: %v", outputDir, err)
	} else if err := saveSpherePCD(logger, imagingSpherePath, endPose.Point(), sphereRadiusMM, sphereStepMM); err != nil {
		logger.Warnf("Failed to save imaging pose sphere PCD: %v", err)
	}

	logger.Infof("Getting point cloud from %q", flags.CamName)
	pc, err := cam.NextPointCloud(ctx, nil)
	if err != nil {
		return fmt.Errorf("failed to get point cloud: %w", err)
	}
	logger.Infof("Got %d points", pc.Size())
	if pc.Size() == 0 {
		return fmt.Errorf("point cloud from %q is empty", flags.CamName)
	}

	pcdPath := filepath.Join(outputDir, fmt.Sprintf("bin-%d-capture.pcd", flags.Bin))
	if err := os.MkdirAll(outputDir, 0o755); err != nil {
		logger.Warnf("Failed to create output directory %q: %v", outputDir, err)
	} else if err := writePCD(pc, pcdPath); err != nil {
		logger.Warnf("Failed to save point cloud: %v", err)
	} else {
		logger.Infof("Saved point cloud to %s", pcdPath)
	}

	md := pc.MetaData()
	logger.Infof("Point cloud bbox: X[%.0f,%.0f] Y[%.0f,%.0f] Z[%.0f,%.0f]",
		md.MinX, md.MaxX, md.MinY, md.MaxY, md.MinZ, md.MaxZ)

	logger.Infof("Descending from X=%.0f Y=%.0f Z=%.0f to target Z=%.0f",
		descentX, imagingY, startZ, targetZ)

	stepMM := flags.StepMM
	if stepMM <= 0 {
		stepMM = 40
	}

	remaining := targetZ - startZ
	if remaining == 0 {
		logger.Infof("Done — already at target Z")
		if err := copyMeshIntoOutput(logger, flags.MeshFile, outputDir); err != nil {
			return fmt.Errorf("save mesh to output: %w", err)
		}
		return nil
	}
	descentZs := descentWaypoints(startZ, targetZ, stepMM)
	for i, nextZ := range descentZs {
		descentPCDPath := filepath.Join(outputDir, fmt.Sprintf("bin-%d-descent-%03d.pcd", flags.Bin, i+1))
		if err := saveSpherePCD(logger, descentPCDPath, r3.Vector{X: descentX, Y: imagingY, Z: nextZ}, sphereRadiusMM, sphereStepMM); err != nil {
			logger.Warnf("Failed to save descent sphere PCD: %v", err)
		}
	}
	reachedZ := startZ
	for stepCount, nextZ := range descentZs {
		logger.Infof("Descent step %d: X=%.0f Y=%.0f Z=%.0f", stepCount+1, descentX, imagingY, nextZ)
		stepPose := spatialmath.NewPose(
			r3.Vector{X: descentX, Y: imagingY, Z: nextZ},
			endPose.Orientation(),
		)
		if _, err := motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: flags.ArmName,
			Destination:   referenceframe.NewPoseInFrame(referenceframe.World, stepPose),
		}); err != nil {
			logger.Warnf("Descent step %d failed (Z=%.0f), stopping descent: %v", stepCount+1, nextZ, err)
			break
		}
		reachedZ = nextZ
	}
	descentComplete := len(descentZs) == 0 || reachedZ == descentZs[len(descentZs)-1]
	if descentComplete {
		logger.Infof("Descent complete — ascending vertically (same X/Y) to imaging Z=%.0f", startZ)
	} else {
		logger.Warnf("Partial descent (reached Z=%.0f, target was %.0f) — ascending to imaging Z=%.0f", reachedZ, targetZ, startZ)
	}
	ascentZs := descentWaypoints(reachedZ, startZ, stepMM)
	for stepCount, nextZ := range ascentZs {
		logger.Infof("Ascent step %d: X=%.0f Y=%.0f Z=%.0f", stepCount+1, descentX, imagingY, nextZ)
		stepPose := spatialmath.NewPose(
			r3.Vector{X: descentX, Y: imagingY, Z: nextZ},
			endPose.Orientation(),
		)
		if _, err := motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: flags.ArmName,
			Destination:   referenceframe.NewPoseInFrame(referenceframe.World, stepPose),
		}); err != nil {
			return fmt.Errorf("ascent step %d failed (Z=%.0f): %w", stepCount+1, nextZ, err)
		}
	}
	logger.Infof("Done — arm back at imaging height")
	if err := copyMeshIntoOutput(logger, flags.MeshFile, outputDir); err != nil {
		return fmt.Errorf("save mesh to output: %w", err)
	}
	return nil
}

// descentWaypoints matches the Z stepping used for motion.Move (same math as the previous inline loop).
func descentWaypoints(startZ, targetZ, stepMM float64) []float64 {
	rem := targetZ - startZ
	if rem == 0 {
		return nil
	}
	dir := 1.0
	if rem < 0 {
		dir = -1.0
	}
	var out []float64
	curZ := startZ
	for dir*(targetZ-curZ) > 0 {
		step := dir * stepMM
		if dir*step > dir*(targetZ-curZ) {
			step = targetZ - curZ
		}
		nextZ := curZ + step
		out = append(out, nextZ)
		curZ = nextZ
	}
	return out
}

func copyMeshIntoOutput(logger logging.Logger, meshSrc, outputDir string) error {
	if err := os.MkdirAll(outputDir, 0o755); err != nil {
		return err
	}
	dst := filepath.Join(outputDir, filepath.Base(meshSrc))
	src, err := os.Open(meshSrc)
	if err != nil {
		return err
	}
	defer src.Close()
	out, err := os.Create(dst)
	if err != nil {
		return err
	}
	defer out.Close()
	if _, err := io.Copy(out, src); err != nil {
		return err
	}
	logger.Infof("Saved mesh to %s", dst)
	return nil
}

// meshBinCenterX returns average rim X near the imaging pose (for visualization vs centroid).
func meshBinCenterX(pts []r3.Vector, nearX, nearY float64, logger logging.Logger) (float64, error) {
	if len(pts) == 0 {
		return 0, fmt.Errorf("mesh has no vertices")
	}

	maxZ := pts[0].Z
	minZ := pts[0].Z
	for _, p := range pts {
		if p.Z > maxZ {
			maxZ = p.Z
		}
		if p.Z < minZ {
			minZ = p.Z
		}
	}

	rimZThresh := minZ + 0.8*(maxZ-minZ)

	const rimSearchRadius = 300.0
	var sumX float64
	var n int
	for _, p := range pts {
		if p.Z < rimZThresh {
			continue
		}
		dx := p.X - nearX
		dy := p.Y - nearY
		if dx*dx+dy*dy > rimSearchRadius*rimSearchRadius {
			continue
		}
		sumX += p.X
		n++
	}
	if n < 5 {
		return 0, fmt.Errorf("only %d rim-level vertices within %.0fmm of imaging position", n, rimSearchRadius)
	}
	cx := sumX / float64(n)
	logger.Infof("Mesh rim avg X=%.0f at imaging Y=%.0f (%d rim vertices)", cx, nearY, n)
	return cx, nil
}

// meshAvgLowestZ returns the average Z of mesh vertices near the bin floor under (cx, nearY):
// points within floorSearchRadius in XY, then among those the average of vertices within floorBandMM of the local minimum Z.
func meshAvgLowestZ(pts []r3.Vector, cx, nearY float64, logger logging.Logger) (float64, error) {
	const floorSearchRadius = 150.0
	const floorBandMM = 3.0

	var inCylinder []r3.Vector
	for _, p := range pts {
		dx := p.X - cx
		dy := p.Y - nearY
		if dx*dx+dy*dy > floorSearchRadius*floorSearchRadius {
			continue
		}
		inCylinder = append(inCylinder, p)
	}
	if len(inCylinder) < 3 {
		return 0, fmt.Errorf("only %d mesh vertices within %.0fmm of (X=%.0f Y=%.0f)", len(inCylinder), floorSearchRadius, cx, nearY)
	}
	localMinZ := inCylinder[0].Z
	for _, p := range inCylinder {
		if p.Z < localMinZ {
			localMinZ = p.Z
		}
	}
	var sumZ float64
	var n int
	for _, p := range inCylinder {
		if p.Z <= localMinZ+floorBandMM {
			sumZ += p.Z
			n++
		}
	}
	if n == 0 {
		return localMinZ, nil
	}
	avgZ := sumZ / float64(n)
	logger.Infof("Mesh floor avg Z=%.0f (local min Z=%.0f, %d pts within %.0fmm of min under X=%.0f Y=%.0f)",
		avgZ, localMinZ, n, floorBandMM, cx, nearY)
	return avgZ, nil
}

// meshVertexWithinRadius reports whether any vertex lies within radiusMM of center (Euclidean, mm).
func meshVertexWithinRadius(pts []r3.Vector, center r3.Vector, radiusMM float64) bool {
	if radiusMM <= 0 {
		return false
	}
	r2 := radiusMM * radiusMM
	// Axis-aligned bounds reject most of a large mesh before distance checks.
	minX, maxX := center.X-radiusMM, center.X+radiusMM
	minY, maxY := center.Y-radiusMM, center.Y+radiusMM
	minZ, maxZ := center.Z-radiusMM, center.Z+radiusMM
	for _, p := range pts {
		if p.X < minX || p.X > maxX || p.Y < minY || p.Y > maxY || p.Z < minZ || p.Z > maxZ {
			continue
		}
		dx := p.X - center.X
		dy := p.Y - center.Y
		dz := p.Z - center.Z
		if dx*dx+dy*dy+dz*dz <= r2 {
			return true
		}
	}
	return false
}

// lateralOffsetForEmptyGrab returns the nominal ±40mm X offset for streak 1 (toward bin center) or 2 (away).
func lateralOffsetForEmptyGrab(imagingX, binCenterX float64, zeroChangeStreak int) float64 {
	direction := 1.0
	if imagingX > binCenterX {
		direction = -1.0
	}
	if zeroChangeStreak == 1 {
		return direction * 40.0
	}
	return -direction * 40.0
}

// pickSafeLateralXOffset returns an X offset (mm) for the next grab, or 0 if both ±40mm options would
// place the end-of-descent pose inside descentMeshClearanceMM of mesh geometry (or floor lookup fails).
func pickSafeLateralXOffset(
	meshPts []r3.Vector,
	imagingX, imagingY, binCenterX float64,
	zeroChangeStreak int,
	logger logging.Logger,
) float64 {
	preferred := lateralOffsetForEmptyGrab(imagingX, binCenterX, zeroChangeStreak)
	try := func(offsetMM float64) (safe bool, descentX, targetZ float64) {
		descentX = imagingX + offsetMM
		floorZ, err := meshAvgLowestZ(meshPts, descentX, imagingY, logger)
		if err != nil {
			logger.Warnf("skip lateral offset %.0fmm: mesh floor under X=%.0f: %v", offsetMM, descentX, err)
			return false, descentX, 0
		}
		tz := floorZ + grabAboveMeshFloorMM
		center := r3.Vector{X: descentX, Y: imagingY, Z: tz}
		if meshVertexWithinRadius(meshPts, center, descentMeshClearanceMM) {
			logger.Warnf("skip lateral offset %.0fmm: mesh within %.0fmm of descent pose (X=%.0f Y=%.0f Z=%.0f)",
				offsetMM, descentMeshClearanceMM, descentX, imagingY, tz)
			return false, descentX, tz
		}
		return true, descentX, tz
	}
	if ok, _, _ := try(preferred); ok {
		return preferred
	}
	alt := -preferred
	if ok, _, _ := try(alt); ok {
		logger.Warnf("preferred lateral offset unsafe — using opposite %.0fmm", alt)
		return alt
	}
	logger.Warnf("both lateral offsets unsafe vs mesh — retrying on imaging X")
	return 0
}

const zeroChangeTolerance = 0.5 // grams

func runAddIngredient(address, apiKey, apiKeyID string, flags GrabFlags) error {
	if address == "" || apiKey == "" || apiKeyID == "" {
		return fmt.Errorf("--address, --api-key, and --api-key-id are required")
	}
	if flags.TargetGrams <= 0 {
		return fmt.Errorf("--target-grams must be positive")
	}

	ctx := context.Background()
	logger := logging.NewLogger("add-ingredient")

	logger.Infof("Loading mesh from %s", flags.MeshFile)
	mesh, err := spatialmath.NewMeshFromPLYFile(flags.MeshFile)
	if err != nil {
		return fmt.Errorf("loading mesh: %w", err)
	}
	meshPts := mesh.ToPoints(1)

	logger.Infof("Connecting to robot at %s", address)
	robotClient, err := client.New(ctx, address, logger,
		client.WithDialOptions(
			rpc.WithEntityCredentials(apiKeyID, rpc.Credentials{
				Type:    rpc.CredentialsTypeAPIKey,
				Payload: apiKey,
			}),
		),
	)
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}
	defer robotClient.Close(ctx)

	motionSvc, err := motion.FromRobot(robotClient, "builtin")
	if err != nil {
		return fmt.Errorf("failed to get motion service: %w", err)
	}

	grpr, err := gripper.FromRobot(robotClient, flags.GripperName)
	if err != nil {
		return fmt.Errorf("failed to get gripper %q: %w", flags.GripperName, err)
	}

	scale, err := sensor.FromRobot(robotClient, flags.ScaleName)
	if err != nil {
		return fmt.Errorf("failed to get scale sensor %q: %w", flags.ScaleName, err)
	}

	aboveName := fmt.Sprintf("bin-%d-imaging", flags.Bin)
	aboveSw, err := toggleswitch.FromRobot(robotClient, aboveName)
	if err != nil {
		return fmt.Errorf("failed to get above-bin switch %q: %w", aboveName, err)
	}

	stepMM := flags.StepMM
	if stepMM <= 0 {
		stepMM = 40
	}

	var totalAdded float64
	var zeroChangeStreak int
	var xOffset float64 // lateral X offset applied on retries (mm, relative to imaging pose)

	for totalAdded < flags.TargetGrams {
		weightBefore, err := readScaleWeight(ctx, scale)
		if err != nil {
			return fmt.Errorf("failed to read scale before grab: %w", err)
		}

		logger.Infof("Moving to %q (added: %.1fg / %.1fg)", aboveName, totalAdded, flags.TargetGrams)
		if err := aboveSw.SetPosition(ctx, 2, nil); err != nil {
			return fmt.Errorf("failed to move to above-bin: %w", err)
		}
		time.Sleep(2 * time.Second)

		endPoseInWorld, err := motionSvc.GetPose(ctx, flags.ArmName, referenceframe.World, nil, nil)
		if err != nil {
			return fmt.Errorf("failed to get arm world pose: %w", err)
		}
		endPose := endPoseInWorld.Pose()

		imagingX := endPose.Point().X
		imagingY := endPose.Point().Y
		startZ := endPose.Point().Z
		descentX := imagingX + xOffset

		binCenterX, err := meshBinCenterX(meshPts, imagingX, imagingY, logger)
		if err != nil {
			return fmt.Errorf("failed to determine bin center X from mesh: %w", err)
		}

		// Floor and target Z must follow the actual descent column (includes lateral retry offset).
		meshFloorZ, err := meshAvgLowestZ(meshPts, descentX, imagingY, logger)
		if err != nil {
			return fmt.Errorf("failed to determine target Z from mesh floor: %w", err)
		}
		targetZ := meshFloorZ + grabAboveMeshFloorMM
		logger.Infof("Descent target: X=%.0f Y=%.0f Z=%.0f (mesh rim X=%.0f, xOffset=%.0f)", descentX, imagingY, targetZ, binCenterX, xOffset)

		if err := grpr.Open(ctx, nil); err != nil {
			return fmt.Errorf("failed to open gripper: %w", err)
		}

		descentZs := descentWaypoints(startZ, targetZ, stepMM)
		reachedZ := startZ
		for stepCount, nextZ := range descentZs {
			logger.Infof("Descent step %d: X=%.0f Y=%.0f Z=%.0f", stepCount+1, descentX, imagingY, nextZ)
			stepPose := spatialmath.NewPose(
				r3.Vector{X: descentX, Y: imagingY, Z: nextZ},
				endPose.Orientation(),
			)
			if _, err := motionSvc.Move(ctx, motion.MoveReq{
				ComponentName: flags.ArmName,
				Destination:   referenceframe.NewPoseInFrame(referenceframe.World, stepPose),
			}); err != nil {
				logger.Warnf("Descent step %d failed (Z=%.0f), stopping descent: %v", stepCount+1, nextZ, err)
				break
			}
			reachedZ = nextZ
		}
		descentComplete := len(descentZs) == 0 || reachedZ == descentZs[len(descentZs)-1]

		if !descentComplete {
			logger.Warnf("Partial descent — stopped at Z=%.0f (target Z=%.0f), grabbing anyway", reachedZ, targetZ)
		}

		logger.Infof("Closing gripper at Z=%.0f", reachedZ)
		if _, err := grpr.Grab(ctx, nil); err != nil {
			return fmt.Errorf("failed to grab: %w", err)
		}
		if err := waitGripperSettledAfterGrab(ctx, grpr, logger); err != nil {
			return err
		}

		logger.Infof("Ascending back to above-bin (from Z=%.0f)", reachedZ)
		ascentZs := descentWaypoints(reachedZ, startZ, stepMM)
		for stepCount, nextZ := range ascentZs {
			logger.Infof("Ascent step %d: X=%.0f Y=%.0f Z=%.0f", stepCount+1, descentX, imagingY, nextZ)
			stepPose := spatialmath.NewPose(
				r3.Vector{X: descentX, Y: imagingY, Z: nextZ},
				endPose.Orientation(),
			)
			if _, err := motionSvc.Move(ctx, motion.MoveReq{
				ComponentName: flags.ArmName,
				Destination:   referenceframe.NewPoseInFrame(referenceframe.World, stepPose),
			}); err != nil {
				return fmt.Errorf("ascent step %d failed (Z=%.0f): %w", stepCount+1, nextZ, err)
			}
		}

		// Move to hover position above bowl using free-space motion.Move (no linear constraints).
		bowlHoverPose := spatialmath.NewPose(
			r3.Vector{X: flags.BowlHoverX, Y: flags.BowlHoverY, Z: flags.BowlHoverZ},
			endPose.Orientation(),
		)
		logger.Infof("Moving to bowl hover (X=%.0f Y=%.0f Z=%.0f)", flags.BowlHoverX, flags.BowlHoverY, flags.BowlHoverZ)
		if _, err := motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: flags.ArmName,
			Destination:   referenceframe.NewPoseInFrame(referenceframe.World, bowlHoverPose),
		}); err != nil {
			return fmt.Errorf("failed to move to bowl hover: %w", err)
		}

		bowlDropPose := spatialmath.NewPose(
			r3.Vector{X: flags.BowlDropX, Y: flags.BowlDropY, Z: flags.BowlDropZ},
			endPose.Orientation(),
		)
		logger.Infof("Moving to bowl drop (X=%.0f Y=%.0f Z=%.0f)", flags.BowlDropX, flags.BowlDropY, flags.BowlDropZ)
		if _, err := motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: flags.ArmName,
			Destination:   referenceframe.NewPoseInFrame(referenceframe.World, bowlDropPose),
		}); err != nil {
			return fmt.Errorf("failed to move to bowl drop: %w", err)
		}

		if err := grpr.Open(ctx, nil); err != nil {
			return fmt.Errorf("failed to open gripper over bowl: %w", err)
		}

		logger.Infof("Retreating to bowl hover after drop")
		if _, err := motionSvc.Move(ctx, motion.MoveReq{
			ComponentName: flags.ArmName,
			Destination:   referenceframe.NewPoseInFrame(referenceframe.World, bowlHoverPose),
		}); err != nil {
			return fmt.Errorf("failed to retreat to bowl hover: %w", err)
		}

		time.Sleep(1 * time.Second)

		weightAfter, err := readScaleWeight(ctx, scale)
		if err != nil {
			return fmt.Errorf("failed to read scale after grab: %w", err)
		}

		change := weightAfter - weightBefore
		logger.Infof("Scale change: %.1fg (before: %.1fg, after: %.1fg)", change, weightBefore, weightAfter)

		if change < zeroChangeTolerance {
			zeroChangeStreak++
			if zeroChangeStreak >= 3 {
				return fmt.Errorf("3 consecutive grabs with no weight change for bin %d, possible empty bin", flags.Bin)
			}
			xOffset = pickSafeLateralXOffset(meshPts, imagingX, imagingY, binCenterX, zeroChangeStreak, logger)
			logger.Warnf("No weight change (streak: %d/3) — next X offset %.0fmm (mesh-checked)", zeroChangeStreak, xOffset)
		} else {
			zeroChangeStreak = 0
			xOffset = 0
			totalAdded += change
		}
	}

	logger.Infof("Done — added %.1fg (target: %.1fg)", totalAdded, flags.TargetGrams)
	return nil
}

func readScaleWeight(ctx context.Context, s sensor.Sensor) (float64, error) {
	readings, err := s.Readings(ctx, nil)
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
