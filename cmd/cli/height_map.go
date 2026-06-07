package main

import (
	"context"
	"fmt"
	"io"
	"math"
	"os"

	"github.com/spf13/cobra"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"

	"salad"
	"salad/segmentation"
)

type HeightMapFlags struct {
	CameraName     string
	PCDPath        string
	SavePCD        string
	ZonesPath      string
	ZoneID         int
	ServingDepthMM float64
}

var heightMapFlags HeightMapFlags

type heightMapZoneResult struct {
	zone   segmentation.Zone
	culled pointcloud.PointCloud
	stats  segmentation.PlaneFitStats
}

var heightMapCmd = &cobra.Command{
	Use:   "height-map",
	Short: "Print a per-zone median-distance height map from a camera or PCD file",
	Long: `height-map loads a point cloud and zones.json, computes per-cell median
signed distance to each zone's bin-floor plane, and prints a grid to the
terminal.

Source point cloud (pick one):
  --camera NAME           dial the machine and call NextPointCloud on the
                          named camera (world frame via frame system)
  --pcd PATH              load from a saved PCD file (assumed world frame)

The printed grid uses X as the vertical axis (increasing upward) and −Y as
the horizontal axis (Y decreases left to right). Each cell shows the median
signed distance in mm; empty cells show "---". Positive = above the plane.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		return runHeightMap(heightMapFlags)
	},
}

func runHeightMap(flags HeightMapFlags) error {
	logger := logging.NewLogger("height-map")

	ctx := context.Background()
	_, _, _, results, err := loadHeightMapZoneResults(ctx, logger, flags)
	if err != nil {
		return err
	}

	for i, r := range results {
		if i > 0 {
			fmt.Fprintln(os.Stdout)
		}
		printHeightMapGrid(os.Stdout, r.zone, r.stats, flags.ServingDepthMM)
	}
	return nil
}

func loadHeightMapZoneResults(ctx context.Context, logger logging.Logger, flags HeightMapFlags) (pointcloud.PointCloud, string, *segmentation.ZonesResult, []heightMapZoneResult, error) {
	if (flags.CameraName == "") == (flags.PCDPath == "") {
		return nil, "", nil, nil, fmt.Errorf("exactly one of --camera or --pcd must be set")
	}

	pc, sourceLabel, err := loadPlaneFitPointCloud(ctx, logger, PlaneFitFlags{
		CameraName: flags.CameraName,
		PCDPath:    flags.PCDPath,
		SavePCD:    flags.SavePCD,
	})
	if err != nil {
		return nil, "", nil, nil, err
	}
	logger.Infof("Source cloud %q: %d points (world frame)", sourceLabel, pc.Size())

	zonesResult, err := segmentation.LoadZones(flags.ZonesPath)
	if err != nil {
		return nil, "", nil, nil, fmt.Errorf("failed to load zones %q: %w", flags.ZonesPath, err)
	}

	targets, err := selectZones(zonesResult, flags.ZoneID)
	if err != nil {
		return nil, "", nil, nil, err
	}

	results := make([]heightMapZoneResult, 0, len(targets))
	for i := range targets {
		stats, culled := segmentation.ZonePlaneFitStats(pc, &targets[i], nil)
		results = append(results, heightMapZoneResult{zone: targets[i], culled: culled, stats: stats})
	}
	return pc, sourceLabel, zonesResult, results, nil
}

// printHeightMapGrid renders hm with X increasing upward and Y decreasing
// left-to-right (i.e. −Y is the horizontal axis). Internal storage uses
// row∝Y and col∝X (see segmentation.ZoneHeightMap).
const (
	ansiHighlight = "\033[7m"
	ansiReset     = "\033[0m"
)

func printHeightMapGrid(w io.Writer, zone segmentation.Zone, stats segmentation.PlaneFitStats, servingDepthMM float64) {
	hm := stats.HeightMap
	n := segmentation.ZoneHeightMapGridSize

	centroidRow, centroidCol := hm.CellXY(zone.Plane.Point[0], zone.Plane.Point[1])
	centroidDisplayRow := n - 1 - centroidCol
	centroidDisplayCol := n - 1 - centroidRow

	fmt.Fprintf(w, "Zone %d height map — median signed distance to bin-floor plane (mm)\n", zone.ID)
	if foodLevelMM, err := salad.FoodLevelMMFromPlaneFitStats(&zone, stats); err == nil {
		if grabBase, err := salad.ComputeGrabBasePoint(&zone, foodLevelMM, servingDepthMM); err == nil {
			fmt.Fprintf(w, "  expected grab base: (%.1f, %.1f, %.1f)  food level=%.1f mm  serving depth=%.1f mm\n",
				grabBase.X, grabBase.Y, grabBase.Z, foodLevelMM, servingDepthMM)
		}
	}
	fmt.Fprintf(w, "  vertical ↑ X  (%.1f at top → %.1f at bottom)\n", hm.MaxX, hm.MinX)
	fmt.Fprintf(w, "  horizontal → −Y  (left: Y=%.1f, right: Y=%.1f)\n", hm.MaxY, hm.MinY)
	fmt.Fprintf(w, "  plane center (%.1f, %.1f) → highlighted cell at row %d, col %d (0-based, top-left origin)\n",
		zone.Plane.Point[0], zone.Plane.Point[1], centroidDisplayRow, centroidDisplayCol)
	fmt.Fprintf(w, "  %d / %d points in zone bounds\n\n", stats.PointsInBounds, stats.PointsTotal)

	for displayRow := 0; displayRow < n; displayRow++ {
		for displayCol := 0; displayCol < n; displayCol++ {
			internalRow := n - 1 - displayCol
			internalCol := n - 1 - displayRow
			v := hm.MedianSignedDistanceMM[internalRow][internalCol]
			highlight := displayRow == centroidDisplayRow && displayCol == centroidDisplayCol
			fmt.Fprint(w, formatHeightMapCell(v, highlight))
		}
		fmt.Fprintln(w)
	}
}

func formatHeightMapCell(v float64, highlight bool) string {
	var s string
	if math.IsNaN(v) {
		s = "   ."
	} else {
		s = fmt.Sprintf("%4.0f", v)
	}
	if highlight {
		return ansiHighlight + s + ansiReset
	}
	return s
}

