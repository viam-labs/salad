package main

import (
	"fmt"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/spf13/cobra"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

type SegmentFlags struct {
	MeshPath  string
	OutputDir string
	Viz       bool
	VizURL             string
	CellSizeMM         float64
	DividerZPercentile float64
	DividerGradientMM  float64
	DividerDilation    int
	MinZoneAreaMM2     float64
	MaxZoneAreaMM2     float64
}

var segmentFlags SegmentFlags

var segmentCmd = &cobra.Command{
	Use:   "segment",
	Short: "Segment a fridge mesh into individual bin zones",
	RunE: func(cmd *cobra.Command, args []string) error {
		return runSegment(segmentFlags)
	},
}

func runSegment(flags SegmentFlags) error {
	logger := logging.NewLogger("segment")
	opts := segmentation.Options{
		CellSizeMM:         flags.CellSizeMM,
		DividerZPercentile: flags.DividerZPercentile,
		DividerGradientMM:  flags.DividerGradientMM,
		DividerDilation:    flags.DividerDilation,
		MinZoneAreaMM2:     flags.MinZoneAreaMM2,
		MaxZoneAreaMM2:     flags.MaxZoneAreaMM2,
	}

	logger.Infof("Segmenting %s", flags.MeshPath)

	t0 := time.Now()
	logger.Infof("Loading mesh and running segmentation...")
	result, stats, err := segmentation.SegmentFridgeBins(flags.MeshPath, opts)
	elapsed := time.Since(t0)
	if err != nil {
		return err
	}

	logger.Infof("Done in %s", elapsed.Round(time.Millisecond))
	logger.Infof("Grid:       %d × %d cells (%g mm resolution)", stats.GridCols, stats.GridRows, opts.CellSizeMM)
	logger.Infof("Triangles:  %d", stats.TriangleCount)
	logger.Infof("Z range:    [%.1f, %.1f] mm  threshold=%.1f mm (p%.0f)",
		stats.ZMin, stats.ZMax, stats.ZThreshold, opts.DividerZPercentile*100)
	logger.Infof("Gradient:   ≥%.1f mm rise above lowest neighbour", opts.DividerGradientMM)
	logger.Infof("Barriers:   %d raw → %d after dilation=%d  (%.0f%% of %d cells)",
		stats.BarrierCellsRaw, stats.BarrierCellsDilated, opts.DividerDilation,
		100*float64(stats.BarrierCellsDilated)/float64(stats.OccupiedCells),
		stats.OccupiedCells)
	logger.Infof("Components: %d total → %d after area filter [%.0f, %.0f] mm²",
		stats.ComponentsTotal, stats.ComponentsAfterFilter, opts.MinZoneAreaMM2, opts.MaxZoneAreaMM2)

	logger.Infof("Detected %d zone(s):", len(result.Zones))
	logger.Infof("%-4s  %-10s  %-10s  %-10s  %-10s  %-12s  %-12s  %-9s",
		"ID", "MinX", "MaxX", "MinY", "MaxY", "Width(mm)", "Depth(mm)", "Triangles")
	logger.Infof("%s", strings.Repeat("-", 87))
	for _, z := range result.Zones {
		logger.Infof("%-4d  %-10.1f  %-10.1f  %-10.1f  %-10.1f  %-12.1f  %-12.1f  %-9d",
			z.ID, z.MinX, z.MaxX, z.MinY, z.MaxY,
			z.MaxX-z.MinX, z.MaxY-z.MinY, len(z.Mesh.Faces))
	}
	logger.Info("")

	outputDir := flags.OutputDir
	if outputDir == "" {
		outputDir = filepath.Join("output", time.Now().Format("20060102-150405"))
	}
	if err := os.MkdirAll(outputDir, 0o755); err != nil {
		return fmt.Errorf("failed to create output directory %q: %w", outputDir, err)
	}

	outPath := filepath.Join(outputDir, "zones.json")
	logger.Infof("Writing %s...", outPath)
	tWrite := time.Now()
	if err := segmentation.SaveZones(result, outPath); err != nil {
		return err
	}
	logger.Infof("Wrote %s (%.1f s)", outPath, time.Since(tWrite).Seconds())

	if flags.Viz {
		return visualizeZones(logger, result, flags.VizURL, flags.MeshPath)
	}
	return nil
}

var zoneVizColors = []string{
	"red", "blue", "green", "orange", "purple",
	"cyan", "yellow", "pink", "teal", "chocolate",
}

func visualizeZones(logger logging.Logger, result *segmentation.ZonesResult, vizURL, meshPath string) error {
	vizClient.SetURL(vizURL)
	if err := vizClient.RemoveAllSpatialObjects(); err != nil {
		return fmt.Errorf("clearing visualizer: %w", err)
	}

	// Draw the source mesh in light grey as a reference background.
	srcMesh, err := spatialmath.NewMeshFromPLYFile(meshPath)
	if err != nil {
		return fmt.Errorf("loading mesh for viz: %w", err)
	}
	srcMesh.SetLabel("fridge-mesh")
	if err := vizClient.DrawGeometry(srcMesh, "lightgrey"); err != nil {
		return fmt.Errorf("drawing mesh: %w", err)
	}
	time.Sleep(200 * time.Millisecond)

	// Draw each zone as its actual sub-mesh in a distinct colour.
	for _, zone := range result.Zones {
		label := fmt.Sprintf("zone-%d", zone.ID)
		color := zoneVizColors[zone.ID%len(zoneVizColors)]
		logger.Infof("Drawing zone %d (%d triangles, color=%s)...", zone.ID, len(zone.Mesh.Faces), color)
		zoneMesh := zone.Mesh.ToSpatialMesh(label)
		if err := vizClient.DrawGeometry(zoneMesh, color); err != nil {
			return fmt.Errorf("drawing zone %d: %w", zone.ID, err)
		}
		time.Sleep(100 * time.Millisecond)
	}

	logger.Infof("Sent %d zone(s) to %s", len(result.Zones), vizURL)
	return nil
}
