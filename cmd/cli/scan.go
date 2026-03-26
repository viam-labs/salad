package main

import (
	"context"
	"fmt"
	"math"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	toggleswitch "go.viam.com/rdk/components/switch"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/robot/client"
	"go.viam.com/rdk/spatialmath"
	"go.viam.com/utils/rpc"
)

const scanArmName = "left-arm"

var camInArm = spatialmath.NewPose(
	r3.Vector{X: 83, Y: -30, Z: 19},
	&spatialmath.EulerAngles{
		Roll:  -1.74 * math.Pi / 180,
		Pitch: -0.24 * math.Pi / 180,
		Yaw:   88.91 * math.Pi / 180,
	},
)

const (
	gridYMinOffset = -750.0
	gridYMaxOffset = 750.0
	gridStepY      = 200.0
)

type angleVariant struct {
	pitch float64 // radians, negative = tilt down
	yaw   float64 // radians, negative = look left, positive = look right
	label string
}

// angleVariants covers 8 compass directions plus straight.
var angleVariants = []angleVariant{
	{0, 0, "straight"},
	{-25 * math.Pi / 180, 0, "down"},
	{25 * math.Pi / 180, 0, "up"},
	{0, -25 * math.Pi / 180, "left"},
	{0, 25 * math.Pi / 180, "right"},
	{-25 * math.Pi / 180, -25 * math.Pi / 180, "down-left"},
	{-25 * math.Pi / 180, 25 * math.Pi / 180, "down-right"},
	{25 * math.Pi / 180, -25 * math.Pi / 180, "up-left"},
	{25 * math.Pi / 180, 25 * math.Pi / 180, "up-right"},
}

func runScan(address, apiKey, apiKeyID string, flags ScanFlags) error {
	if address == "" || apiKey == "" || apiKeyID == "" {
		return fmt.Errorf("--address, --api-key, and --api-key-id are required")
	}

	ctx := context.Background()
	logger := logging.NewLogger("area-scanner")

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
		return fmt.Errorf("failed to connect to robot: %w", err)
	}
	defer robotClient.Close(ctx)
	logger.Infof("Connected")

	cam, err := camera.FromRobot(robotClient, flags.CameraName)
	if err != nil {
		return fmt.Errorf("failed to get camera %q: %w", flags.CameraName, err)
	}

	armComp, err := arm.FromRobot(robotClient, scanArmName)
	if err != nil {
		logger.Warnf("Could not get arm %q — tiled scan disabled: %v", scanArmName, err)
		armComp = nil
	}

	outputDir := flags.OutputDir
	if outputDir == "" {
		outputDir = filepath.Join("output", time.Now().Format("20060102-150405"))
	}
	if err := os.MkdirAll(outputDir, 0o755); err != nil {
		return fmt.Errorf("failed to create output directory %q: %w", outputDir, err)
	}

	var pcsInWorld []pointcloud.PointCloud

	if armComp != nil {
		camInArmInv := spatialmath.PoseInverse(camInArm)
		tileIdx := 0
		captured := 0
		unreachable := 0

		// Each named position is an anchor covering a different Y band of the bin row.
		// Tiles sweep Y only; anchor X and Z are held fixed.
		for _, posName := range imagingPositions {
			sw, err := waitAndGetSwitch(ctx, robotClient, posName, logger)
			if err != nil {
				logger.Warnf("Skipping %q: %v", posName, err)
				continue
			}
			logger.Infof("Moving to anchor %q", posName)
			if err := retryOnDisconnect(func() error { return sw.SetPosition(ctx, 2, nil) }, logger); err != nil {
				logger.Warnf("Skipping %q: move failed (%v)", posName, err)
				continue
			}
			time.Sleep(flags.SleepDuration())

			var endPose spatialmath.Pose
			if err := retryOnDisconnect(func() error {
				var e error
				endPose, e = armComp.EndPosition(ctx, nil)
				return e
			}, logger); err != nil {
				logger.Warnf("Skipping %q: could not read arm pose (%v)", posName, err)
				continue
			}
			anchorCamPose := spatialmath.Compose(endPose, camInArm)
			ap := anchorCamPose.Point()
			logger.Infof("  Camera pose: X=%.0f Y=%.0f Z=%.0f", ap.X, ap.Y, ap.Z)

			yPositions := tile1DFixedDistance(ap.Y+gridYMinOffset, ap.Y+gridYMaxOffset, gridStepY)
			logger.Infof("  Tiling %d Y positions (Y[%.0f,%.0f] X=%.0f Z=%.0f)",
				len(yPositions), ap.Y+gridYMinOffset, ap.Y+gridYMaxOffset, ap.X, ap.Z)

			for _, y := range yPositions {
				baseCamPose := spatialmath.NewPose(
					r3.Vector{X: ap.X, Y: y, Z: ap.Z},
					anchorCamPose.Orientation(),
				)

				for _, av := range angleVariants {
					anglePose := spatialmath.NewPoseFromOrientation(
						&spatialmath.EulerAngles{Pitch: av.pitch, Yaw: av.yaw},
					)
					camPose := spatialmath.Compose(baseCamPose, anglePose)
					armPose := spatialmath.Compose(camPose, camInArmInv)

					moveErr := retryOnDisconnect(func() error {
						return armComp.MoveToPosition(ctx, armPose, nil)
					}, logger)
					if moveErr != nil {
						unreachable++
						logger.Warnf("  Tile %d (Y=%.0f %s): not reachable (%v)", tileIdx, y, av.label, moveErr)
						tileIdx++
						continue
					}
					time.Sleep(flags.SleepDuration())

					var pc pointcloud.PointCloud
					captureErr := retryOnDisconnect(func() error {
						var e error
						pc, e = cam.NextPointCloud(ctx, nil)
						return e
					}, logger)
					if captureErr != nil {
						logger.Warnf("  Tile %d capture failed: %v", tileIdx, captureErr)
						tileIdx++
						continue
					}
					md := pc.MetaData()
					logger.Infof("  Tile %d (X=%.0f Y=%.0f Z=%.0f %s): %d pts  bbox X[%.0f,%.0f] Y[%.0f,%.0f] Z[%.0f,%.0f]",
						tileIdx, ap.X, y, ap.Z, av.label, pc.Size(),
						md.MinX, md.MaxX, md.MinY, md.MaxY, md.MinZ, md.MaxZ)

					tilePath := filepath.Join(outputDir, fmt.Sprintf("tile-%03d.pcd", tileIdx))
					if err := writePCD(pc, tilePath); err != nil {
						logger.Warnf("  Failed to write tile PCD: %v", err)
					} else {
						pcsInWorld = append(pcsInWorld, pc)
						captured++
					}
					tileIdx++
				}
			}
		}
		logger.Infof("Tiled scan complete: %d/%d positions captured (%d unreachable)",
			captured, tileIdx, unreachable)
	} else {
		logger.Warnf("Skipping tiled scan: arm not available")
	}

	// Concatenate all world-frame clouds.
	totalSize := 0
	for _, pc := range pcsInWorld {
		totalSize += pc.Size()
	}
	merged := pointcloud.NewBasicPointCloud(totalSize)
	for _, pc := range pcsInWorld {
		if err := pointcloud.ApplyOffset(pc, nil, merged); err != nil {
			return fmt.Errorf("failed to concatenate point clouds: %w", err)
		}
	}

	t0 := time.Now()
	vg := pointcloud.NewVoxelGridFromPointCloud(merged, 8.0, 0)
	deduped := pointcloud.NewBasicPointCloud(len(vg.Voxels))
	for _, vox := range vg.Voxels {
		if err := deduped.Set(vox.Center, pointcloud.NewBasicData()); err != nil {
			return fmt.Errorf("failed to build deduped cloud: %w", err)
		}
	}
	mergedPath := filepath.Join(outputDir, "merged.pcd")
	if err := writePCD(deduped, mergedPath); err != nil {
		return err
	}
	logger.Infof("Step 1/4 dedup+merge: %d → %d points, wrote %s (%s)", merged.Size(), deduped.Size(), mergedPath, time.Since(t0).Round(time.Millisecond))

	t0 = time.Now()
	filteredPath := filepath.Join(outputDir, "filtered.pcd")
	filtered, err := filterPointCloud(deduped, 10.0, 3)
	if err != nil {
		return fmt.Errorf("failed to filter point cloud: %w", err)
	}
	if err := writePCD(filtered, filteredPath); err != nil {
		return err
	}
	logger.Infof("Step 2/4 filter: %d → %d points, wrote %s (%s)", deduped.Size(), filtered.Size(), filteredPath, time.Since(t0).Round(time.Millisecond))

	t0 = time.Now()
	croppedPath := filepath.Join(outputDir, "cropped.pcd")
	cropped, err := cropPointCloud(filtered, CropBounds{
		MinX: 500, MaxX: 1000,
		MinY: -math.MaxFloat64, MaxY: math.MaxFloat64,
		MinZ: -math.MaxFloat64, MaxZ: math.MaxFloat64,
	})
	if err != nil {
		return fmt.Errorf("failed to crop point cloud: %w", err)
	}
	if err := writePCD(cropped, croppedPath); err != nil {
		return err
	}
	logger.Infof("Step 3/4 crop: %d → %d points, wrote %s (%s)", filtered.Size(), cropped.Size(), croppedPath, time.Since(t0).Round(time.Millisecond))

	t0 = time.Now()
	meshVerts, meshTris, voxelCount, err := pointCloudToMesh(cropped, 8.0)
	if err != nil {
		return fmt.Errorf("failed to build mesh: %w", err)
	}
	meshVerts = laplacianSmooth(meshVerts, meshTris, 5)
	triangles := make([]*spatialmath.Triangle, len(meshTris))
	for i, t := range meshTris {
		triangles[i] = spatialmath.NewTriangle(meshVerts[t[0]], meshVerts[t[1]], meshVerts[t[2]])
	}
	mesh := spatialmath.NewMesh(spatialmath.NewZeroPose(), triangles, "salad-scan")
	meshPath := filepath.Join(outputDir, "mesh.ply")
	if err := os.WriteFile(meshPath, mesh.TrianglesToPLYBytes(false), 0o644); err != nil {
		return fmt.Errorf("failed to write mesh: %w", err)
	}
	logger.Infof("Step 4/4 mesh: %d voxels → %d triangles, wrote %s (%s)", voxelCount, len(triangles), meshPath, time.Since(t0).Round(time.Millisecond))
	return nil
}

// retryOnDisconnect runs fn and, if it returns a disconnection error, waits
// for the connection to recover and retries up to 3 times before giving up.
func retryOnDisconnect(fn func() error, logger logging.Logger) error {
	const maxRetries = 3
	for attempt := range maxRetries {
		err := fn()
		if err == nil {
			return nil
		}
		if !isDisconnectError(err) {
			return err
		}
		if attempt < maxRetries-1 {
			logger.Warnf("  Connection lost, waiting to reconnect (attempt %d/%d)…", attempt+1, maxRetries)
			time.Sleep(5 * time.Second)
		}
	}
	return fmt.Errorf("failed after %d retries due to disconnection", maxRetries)
}

// waitAndGetSwitch retries fetching a switch component on disconnection errors.
func waitAndGetSwitch(ctx context.Context, robotClient *client.RobotClient, name string, logger logging.Logger) (toggleswitch.Switch, error) {
	const maxRetries = 3
	for attempt := range maxRetries {
		sw, err := toggleswitch.FromRobot(robotClient, name)
		if err == nil {
			return sw, nil
		}
		if !isDisconnectError(err) {
			return nil, err
		}
		if attempt < maxRetries-1 {
			logger.Warnf("  Connection lost getting switch %q, waiting to reconnect…", name)
			time.Sleep(5 * time.Second)
		}
	}
	return nil, fmt.Errorf("failed to get switch %q after retries", name)
}

func isDisconnectError(err error) bool {
	s := err.Error()
	return strings.Contains(s, "not connected") ||
		strings.Contains(s, "context canceled") ||
		strings.Contains(s, "SESSION_EXPIRED")
}

func writePCD(pc pointcloud.PointCloud, path string) error {
	f, err := os.Create(path)
	if err != nil {
		return fmt.Errorf("failed to create %q: %w", path, err)
	}
	defer f.Close()
	return pointcloud.ToPCD(pc, f, pointcloud.PCDBinary)
}
