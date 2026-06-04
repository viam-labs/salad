package main

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/spf13/cobra"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/referenceframe"

	"salad/segmentation"
	saladutils "salad/utils"
)

type PlaneFitFlags struct {
	CameraName string
	PCDPath    string
	SavePCD    string
	ZonesPath  string
	ZoneID     int
	OutputPath string
	Viz        bool
	VizURL     string
}

var planeFitFlags PlaneFitFlags

var planeFitCmd = &cobra.Command{
	Use:   "plane-fit",
	Short: "Pull a point cloud from a camera (or PCD file) and report mean distance to each zone's bin-floor plane",
	Long: `plane-fit loads a point cloud and a zones.json, then for each zone:
  - culls points outside the zone's axis-aligned XY rectangle
  - computes signed/absolute distance from each kept point to the zone's
    fitted bin-floor plane
  - prints mean |dist|, mean signed, min, max, and the zone's plane tilt

Source point cloud (pick one):
  --camera NAME           dial the machine (--address / --api-key /
                          --api-key-id, or VIAM_ADDRESS / VIAM_API_KEY /
                          VIAM_API_KEY_ID), call NextPointCloud on the
                          named camera component, and transform the result
                          to the world frame via the machine's frame system
  --pcd PATH              load from a saved PCD file (assumed world frame)

Use --save-pcd PATH to write the world-frame cloud to disk for later replay.
Use --zone-id N to restrict to a single zone (negative = all zones).
Use --output to write the culled points back to disk (file path for a single
zone, directory for multiple). Use --viz to display the source cloud,
per-zone culled clouds, and each zone's plane rectangle in motion-tools.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		return runPlaneFit(planeFitFlags)
	},
}

type planeFitZoneResult struct {
	zone   segmentation.Zone
	culled pointcloud.PointCloud
	stats  segmentation.PlaneFitStats
}

func runPlaneFit(flags PlaneFitFlags) error {
	logger := logging.NewLogger("plane-fit")

	if (flags.CameraName == "") == (flags.PCDPath == "") {
		return fmt.Errorf("exactly one of --camera or --pcd must be set")
	}

	ctx := context.Background()

	pc, sourceLabel, err := loadPlaneFitPointCloud(ctx, logger, flags)
	if err != nil {
		return err
	}
	logger.Infof("Source cloud %q: %d points (world frame)", sourceLabel, pc.Size())

	logger.Infof("Loading zones from %s", flags.ZonesPath)
	zonesResult, err := segmentation.LoadZones(flags.ZonesPath)
	if err != nil {
		return fmt.Errorf("failed to load zones %q: %w", flags.ZonesPath, err)
	}

	targets, err := selectZones(zonesResult, flags.ZoneID)
	if err != nil {
		return err
	}

	results := make([]planeFitZoneResult, 0, len(targets))
	for i := range targets {
		stats, culled := segmentation.ZonePlaneFitStats(pc, &targets[i], logger)
		results = append(results, planeFitZoneResult{zone: targets[i], culled: culled, stats: stats})
		logger.Infof("Zone %d: %d points in bounds, %d points outside bounds", targets[i].ID, stats.PointsInBounds, stats.PointsTotal-stats.PointsInBounds)
		logger.Infof("Zone %d: average distance to plane: %.2f mm", targets[i].ID, stats.MeanAbsDistanceMM)
	}

	if flags.OutputPath != "" {
		if err := writePlaneFitOutputs(flags.OutputPath, results, logger); err != nil {
			return err
		}
	}

	if flags.Viz {
		if err := vizPlaneFit(flags.VizURL, pc, results); err != nil {
			return err
		}
	}
	return nil
}

// loadPlaneFitPointCloud returns a point cloud in the world frame along with a
// short identifier string for logging. When --camera is used, the raw camera
// cloud is transformed to the world frame via the robot's frame system before
// being returned. When --pcd is used the file is assumed to already be in the
// world frame (e.g. previously saved via --save-pcd).
func loadPlaneFitPointCloud(ctx context.Context, logger logging.Logger, flags PlaneFitFlags) (pointcloud.PointCloud, string, error) {
	if flags.PCDPath != "" {
		logger.Infof("Loading PCD from %s (assumed world frame)", flags.PCDPath)
		pc, err := pointcloud.NewFromFile(flags.PCDPath, "")
		if err != nil {
			return nil, "", fmt.Errorf("failed to load point cloud %q: %w", flags.PCDPath, err)
		}
		return pc, flags.PCDPath, nil
	}

	rc, err := dialMachine(ctx, logger)
	if err != nil {
		return nil, "", err
	}
	defer func() {
		if cerr := rc.Close(ctx); cerr != nil {
			logger.Warnf("closing robot client: %v", cerr)
		}
	}()

	cam, err := camera.FromProvider(rc, flags.CameraName)
	if err != nil {
		return nil, "", fmt.Errorf("camera %q not found on machine: %w", flags.CameraName, err)
	}

	logger.Infof("Calling NextPointCloud on camera %q", flags.CameraName)
	t0 := time.Now()
	pc, err := cam.NextPointCloud(ctx, nil)
	if err != nil {
		return nil, "", fmt.Errorf("NextPointCloud on %q failed: %w", flags.CameraName, err)
	}
	logger.Infof("Captured %d points in %s", pc.Size(), time.Since(t0).Round(time.Millisecond))

	worldPc, err := rc.TransformPointCloud(ctx, pc, flags.CameraName, referenceframe.World)
	if err != nil {
		return nil, "", fmt.Errorf("failed to transform point cloud to world frame: %w", err)
	}
	pc = worldPc

	if flags.SavePCD != "" {
		if dir := filepath.Dir(flags.SavePCD); dir != "" && dir != "." {
			if err := os.MkdirAll(dir, 0o750); err != nil {
				return nil, "", fmt.Errorf("creating parent of --save-pcd %q: %w", flags.SavePCD, err)
			}
		}
		if err := saladutils.WritePCD(pc, flags.SavePCD); err != nil {
			return nil, "", fmt.Errorf("writing --save-pcd %q: %w", flags.SavePCD, err)
		}
		logger.Infof("Wrote captured cloud to %s", flags.SavePCD)
	}
	return pc, "camera:" + flags.CameraName, nil
}

func selectZones(result *segmentation.ZonesResult, zoneID int) ([]segmentation.Zone, error) {
	if zoneID < 0 {
		return result.Zones, nil
	}
	z, ok := result.ZoneByID(zoneID)
	if !ok {
		return nil, fmt.Errorf("zone %d not found", zoneID)
	}
	return []segmentation.Zone{*z}, nil
}

func writePlaneFitOutputs(outputPath string, results []planeFitZoneResult, logger logging.Logger) error {
	if len(results) == 0 {
		return nil
	}
	if len(results) == 1 {
		out := results[0]
		if dir := filepath.Dir(outputPath); dir != "" && dir != "." {
			if err := os.MkdirAll(dir, 0o750); err != nil {
				return fmt.Errorf("creating parent dir of %q: %w", outputPath, err)
			}
		}
		if err := saladutils.WritePCD(out.culled, outputPath); err != nil {
			return fmt.Errorf("writing culled pcd: %w", err)
		}
		logger.Infof("Wrote %s (%d points)", outputPath, out.culled.Size())
		return nil
	}
	if err := os.MkdirAll(outputPath, 0o750); err != nil {
		return fmt.Errorf("creating output directory %q: %w", outputPath, err)
	}
	for _, r := range results {
		p := filepath.Join(outputPath, fmt.Sprintf("zone-%d-culled.pcd", r.zone.ID))
		if err := saladutils.WritePCD(r.culled, p); err != nil {
			return fmt.Errorf("writing %q: %w", p, err)
		}
		logger.Infof("Wrote %s (%d points)", p, r.culled.Size())
	}
	return nil
}

func vizPlaneFit(vizURL string, source pointcloud.PointCloud, results []planeFitZoneResult) error {
	vizClient.SetURL(vizURL)
	if err := vizClient.RemoveAllSpatialObjects(); err != nil {
		return fmt.Errorf("clearing visualizer: %w", err)
	}
	if err := vizClient.DrawPointCloud("source", source, nil); err != nil {
		return fmt.Errorf("drawing source point cloud: %w", err)
	}
	time.Sleep(150 * time.Millisecond)
	for _, r := range results {
		color := zoneVizColors[r.zone.ID%len(zoneVizColors)]
		label := fmt.Sprintf("zone-%d-culled", r.zone.ID)
		if err := vizClient.DrawPointCloud(label, r.culled, nil); err != nil {
			return fmt.Errorf("drawing %s: %w", label, err)
		}
		rect := r.zone.PlaneRect
		if len(rect.Vertices) == 0 {
			rect = segmentation.PlaneRectMesh(r.zone.Plane, r.zone.MinX, r.zone.MaxX, r.zone.MinY, r.zone.MaxY)
		}
		planeMesh := rect.ToSpatialMesh(fmt.Sprintf("zone-%d-plane", r.zone.ID))
		if err := vizClient.DrawGeometry(planeMesh, color); err != nil {
			return fmt.Errorf("drawing zone %d plane: %w", r.zone.ID, err)
		}
		time.Sleep(100 * time.Millisecond)
	}
	return nil
}
