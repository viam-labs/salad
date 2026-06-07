package main

import (
	"context"
	"fmt"
	"os"
	"time"

	"github.com/spf13/cobra"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
	genericservice "go.viam.com/rdk/services/generic"
	"go.viam.com/rdk/spatialmath"

	"salad"
	"salad/segmentation"
)

type VizGrabPosesFlags struct {
	CameraName      string
	PCDPath         string
	SavePCD         string
	ZonesPath       string
	ZoneID          int
	ServingDepthMM  float64
	GrabberControls string
	VizURL          string
}

type binHoverOffsets struct {
	xOffsetMM float64
	yOffsetMM float64
}

type gripperCalibration struct {
	closedGripperHeightMM float64
	binHoverHeightMM      float64
	orientation           *spatialmath.OrientationVectorDegrees
	binHoverOffsets       map[int]binHoverOffsets
}

var vizGrabPosesFlags VizGrabPosesFlags

var vizGrabPosesCmd = &cobra.Command{
	Use:   "viz-grab-poses",
	Short: "Draw food-level, hover, and grab pose arrows in motion-tools",
	Long: `viz-grab-poses loads a point cloud and zones.json, fits each zone's
bin-floor plane, and draws pose arrows in the motion-tools visualizer:

  red    food-level grab base (from height-map median at plane center)
  blue   bin hover pose (requires --grabber-controls calibration)
  green  arm grab pose (requires --grabber-controls calibration)

Also draws the source point cloud, per-zone culled clouds, and zone plane
rectangles for context.

Source point cloud (pick one):
  --camera NAME           dial the machine and call NextPointCloud on the
                          named camera (world frame via frame system)
  --pcd PATH              load from a saved PCD file (assumed world frame)`,
	RunE: func(cmd *cobra.Command, args []string) error {
		return runVizGrabPoses(vizGrabPosesFlags)
	},
}

func runVizGrabPoses(flags VizGrabPosesFlags) error {
	logger := logging.NewLogger("viz-grab-poses")

	ctx := context.Background()
	pc, _, zonesResult, results, err := loadHeightMapZoneResults(ctx, logger, HeightMapFlags{
		CameraName:     flags.CameraName,
		PCDPath:        flags.PCDPath,
		SavePCD:        flags.SavePCD,
		ZonesPath:      flags.ZonesPath,
		ZoneID:         flags.ZoneID,
		ServingDepthMM: flags.ServingDepthMM,
	})
	if err != nil {
		return err
	}

	calibration, err := fetchGripperCalibration(ctx, logger, flags.GrabberControls)
	if err != nil {
		return err
	}

	return vizGrabPoses(flags.VizURL, pc, results, zonesResult.ZMean, flags.ServingDepthMM, calibration)
}

func fetchGripperCalibration(ctx context.Context, logger logging.Logger, serviceName string) (*gripperCalibration, error) {
	rc, err := dialMachine(ctx, logger)
	if err != nil {
		return nil, err
	}
	defer func() {
		if cerr := rc.Close(ctx); cerr != nil {
			logger.Warnf("closing robot client: %v", cerr)
		}
	}()

	svc, err := genericservice.FromProvider(rc, serviceName)
	if err != nil {
		return nil, fmt.Errorf("grabber controls service %q not found on machine: %w", serviceName, err)
	}

	calResp, err := svc.DoCommand(ctx, map[string]interface{}{"get_gripper_calibration": true})
	if err != nil {
		return nil, fmt.Errorf("get_gripper_calibration on %q: %w", serviceName, err)
	}

	closedHeight, err := floatFromDoCommand(calResp, "closed_gripper_to_arm_base_height_mm")
	if err != nil {
		return nil, fmt.Errorf("parsing gripper calibration from %q: %w", serviceName, err)
	}

	binResp, err := svc.DoCommand(ctx, map[string]interface{}{"get_bin_config": true})
	if err != nil {
		return nil, fmt.Errorf("get_bin_config on %q: %w", serviceName, err)
	}

	orientRaw, ok := binResp["bin_hover_orientation"].(map[string]interface{})
	if !ok {
		return nil, fmt.Errorf("parsing bin config from %q: missing bin_hover_orientation", serviceName)
	}
	ox, err := floatFromDoCommand(orientRaw, "ox")
	if err != nil {
		return nil, fmt.Errorf("parsing bin config from %q: %w", serviceName, err)
	}
	oy, err := floatFromDoCommand(orientRaw, "oy")
	if err != nil {
		return nil, fmt.Errorf("parsing bin config from %q: %w", serviceName, err)
	}
	oz, err := floatFromDoCommand(orientRaw, "oz")
	if err != nil {
		return nil, fmt.Errorf("parsing bin config from %q: %w", serviceName, err)
	}
	theta, err := floatFromDoCommand(orientRaw, "theta")
	if err != nil {
		return nil, fmt.Errorf("parsing bin config from %q: %w", serviceName, err)
	}

	binHoverHeight, err := floatFromDoCommand(binResp, "bin_hover_height_mm")
	if err != nil {
		return nil, fmt.Errorf("parsing bin config from %q: %w", serviceName, err)
	}

	binHoverOffsets, err := parseBinHoverOffsets(binResp)
	if err != nil {
		return nil, fmt.Errorf("parsing bin config from %q: %w", serviceName, err)
	}

	return &gripperCalibration{
		closedGripperHeightMM: closedHeight,
		binHoverHeightMM:      binHoverHeight,
		orientation: &spatialmath.OrientationVectorDegrees{
			OX:    ox,
			OY:    oy,
			OZ:    oz,
			Theta: theta,
		},
		binHoverOffsets: binHoverOffsets,
	}, nil
}

func parseBinHoverOffsets(resp map[string]interface{}) (map[int]binHoverOffsets, error) {
	rawBins, ok := resp["bins"].([]interface{})
	if !ok {
		return nil, fmt.Errorf("missing bins")
	}
	offsets := make(map[int]binHoverOffsets, len(rawBins))
	for i, raw := range rawBins {
		bin, ok := raw.(map[string]interface{})
		if !ok {
			return nil, fmt.Errorf("bins[%d] has unexpected type %T", i, raw)
		}
		zoneID, err := intFromDoCommand(bin, "zone_id")
		if err != nil {
			return nil, fmt.Errorf("bins[%d]: %w", i, err)
		}
		hoverX, _ := floatFromDoCommand(bin, "hover_x_offset_mm")
		hoverY, _ := floatFromDoCommand(bin, "hover_y_offset_mm")
		offsets[zoneID] = binHoverOffsets{xOffsetMM: hoverX, yOffsetMM: hoverY}
	}
	return offsets, nil
}

func intFromDoCommand(resp map[string]interface{}, key string) (int, error) {
	v, ok := resp[key]
	if !ok {
		return 0, fmt.Errorf("missing %q", key)
	}
	switch n := v.(type) {
	case float64:
		return int(n), nil
	case int:
		return n, nil
	case int64:
		return int(n), nil
	default:
		return 0, fmt.Errorf("%q has unexpected type %T", key, v)
	}
}

func floatFromDoCommand(resp map[string]interface{}, key string) (float64, error) {
	v, ok := resp[key]
	if !ok {
		return 0, fmt.Errorf("missing %q", key)
	}
	switch n := v.(type) {
	case float64:
		return n, nil
	case int:
		return float64(n), nil
	case int64:
		return float64(n), nil
	default:
		return 0, fmt.Errorf("%q has unexpected type %T", key, v)
	}
}

func vizGrabBasePose(zone *segmentation.Zone, foodLevelMM, servingDepthMM float64) (spatialmath.Pose, error) {
	point, err := salad.ComputeGrabBasePoint(zone, foodLevelMM, servingDepthMM)
	if err != nil {
		return nil, err
	}
	nx, ny, nz := zone.Plane.Normal[0], zone.Plane.Normal[1], zone.Plane.Normal[2]
	orient := &spatialmath.OrientationVector{OX: -nx, OY: -ny, OZ: -nz}
	return spatialmath.NewPose(point, orient), nil
}

func vizGrabPoses(vizURL string, source pointcloud.PointCloud, results []heightMapZoneResult, zMean, servingDepthMM float64, calibration *gripperCalibration) error {
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

		foodLevelMM, err := salad.FoodLevelMMFromPlaneFitStats(&r.zone, r.stats)
		if err != nil {
			return fmt.Errorf("zone %d food level: %w", r.zone.ID, err)
		}
		grabBasePose, err := vizGrabBasePose(&r.zone, foodLevelMM, servingDepthMM)
		if err != nil {
			return fmt.Errorf("zone %d grab base pose: %w", r.zone.ID, err)
		}
		if err := vizClient.DrawPoses([]spatialmath.Pose{grabBasePose}, []string{"red"}, true); err != nil {
			return fmt.Errorf("drawing zone-%d-grab-base: %w", r.zone.ID, err)
		}
		basePt := grabBasePose.Point()
		fmt.Fprintf(os.Stdout, "Zone %d grab base arrow at (%.1f, %.1f, %.1f)\n", r.zone.ID, basePt.X, basePt.Y, basePt.Z)

		hoverOffsets := calibration.binHoverOffsets[r.zone.ID]
		hoverPose, err := salad.BinHoverPose(
			&r.zone,
			zMean,
			calibration.binHoverHeightMM,
			hoverOffsets.xOffsetMM,
			hoverOffsets.yOffsetMM,
			calibration.orientation,
		)
		if err != nil {
			return fmt.Errorf("zone %d hover pose: %w", r.zone.ID, err)
		}
		if err := vizClient.DrawPoses([]spatialmath.Pose{hoverPose}, []string{"blue"}, true); err != nil {
			return fmt.Errorf("drawing zone-%d-hover: %w", r.zone.ID, err)
		}
		hoverPt := hoverPose.Point()
		fmt.Fprintf(os.Stdout, "Zone %d hover pose at (%.1f, %.1f, %.1f)\n",
			r.zone.ID, hoverPt.X, hoverPt.Y, hoverPt.Z)

		grabPose, err := salad.ComputeGrabPose(
			&r.zone,
			foodLevelMM,
			servingDepthMM,
			calibration.closedGripperHeightMM,
			calibration.orientation,
		)
		if err != nil {
			return fmt.Errorf("zone %d grab pose: %w", r.zone.ID, err)
		}
		if err := vizClient.DrawPoses([]spatialmath.Pose{grabPose}, []string{"green"}, true); err != nil {
			return fmt.Errorf("drawing zone-%d-grab: %w", r.zone.ID, err)
		}
		grabPt := grabPose.Point()
		fmt.Fprintf(os.Stdout, "Zone %d grab pose (arm base) at (%.1f, %.1f, %.1f)  closed gripper height=%.1f mm\n",
			r.zone.ID, grabPt.X, grabPt.Y, grabPt.Z, calibration.closedGripperHeightMM)

		time.Sleep(100 * time.Millisecond)
	}
	return nil
}
