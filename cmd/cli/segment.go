package main

import (
	"fmt"
	"path/filepath"
	"strings"
	"time"

	"github.com/spf13/cobra"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

type SegmentFlags struct {
	MeshPath           string
	OutputPath         string
	Viz                bool
	VizURL             string
	CellSizeMM         float64
	DividerZPercentile float64
	DividerDilation    int
	MinBinAreaMM2      float64
}

var segmentFlags SegmentFlags

var segmentCmd = &cobra.Command{
	Use:   "segment",
	Short: "Segment a fridge mesh into individual bin zones",
	Long: `Loads a PLY mesh of the fridge and segments it into individual bin zones.

Each zone is the section of the mesh that represents one bin's floor region. The
fridge is scanned from above; dividers appear as high-Z ridges between lower bin
surfaces. A height-map connected-component algorithm identifies each region.

Divider detection uses a Z-percentile threshold. Because Poisson reconstruction
smooths out thin dividers and may leave small gaps, the divider mask is dilated
before connected-component analysis to ensure bins are reliably separated.

The result is written to a JSON file (default: <mesh>.zones.json). Each zone
includes its XY footprint bounds and the actual mesh triangles from the source
mesh that belong to it.

If --viz is set, each zone is drawn in a distinct colour in the motion-tools
visualiser alongside the source mesh for visual verification.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		return runSegment(segmentFlags)
	},
}

func runSegment(flags SegmentFlags) error {
	opts := segmentation.Options{
		CellSizeMM:         flags.CellSizeMM,
		DividerZPercentile: flags.DividerZPercentile,
		DividerDilation:    flags.DividerDilation,
		MinBinAreaMM2:      flags.MinBinAreaMM2,
	}

	fmt.Printf("Segmenting %s\n", flags.MeshPath)
	result, err := segmentation.SegmentFridgeBins(flags.MeshPath, opts)
	if err != nil {
		return err
	}

	fmt.Printf("Detected %d zone(s):\n\n", len(result.Zones))
	fmt.Printf("  %-4s  %-10s  %-10s  %-10s  %-10s  %-12s  %-12s  %-9s\n",
		"ID", "MinX", "MaxX", "MinY", "MaxY", "Width(mm)", "Depth(mm)", "Triangles")
	fmt.Printf("  %s\n", strings.Repeat("-", 87))
	for _, z := range result.Zones {
		fmt.Printf("  %-4d  %-10.1f  %-10.1f  %-10.1f  %-10.1f  %-12.1f  %-12.1f  %-9d\n",
			z.ID, z.MinX, z.MaxX, z.MinY, z.MaxY,
			z.MaxX-z.MinX, z.MaxY-z.MinY, len(z.Mesh.Faces))
	}
	fmt.Println()

	outPath := flags.OutputPath
	if outPath == "" {
		ext := filepath.Ext(flags.MeshPath)
		outPath = strings.TrimSuffix(flags.MeshPath, ext) + ".zones.json"
	}
	if err := segmentation.SaveZones(result, outPath); err != nil {
		return err
	}
	fmt.Printf("Wrote %s\n", outPath)

	if flags.Viz {
		return visualizeZones(result, flags.VizURL, flags.MeshPath)
	}
	return nil
}

// zoneVizColors is the palette used to distinguish individual zones in motion-tools.
// Colors are chosen to be visually distinct against a grey mesh.
var zoneVizColors = []string{
	"red", "blue", "green", "orange", "purple",
	"cyan", "yellow", "pink", "teal", "chocolate",
}

func visualizeZones(result *segmentation.ZonesResult, vizURL, meshPath string) error {
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
		zoneMesh := zone.Mesh.ToSpatialMesh(label)
		if err := vizClient.DrawGeometry(zoneMesh, color); err != nil {
			return fmt.Errorf("drawing zone %d: %w", zone.ID, err)
		}
		time.Sleep(100 * time.Millisecond)
	}

	fmt.Printf("Sent %d zone(s) to %s\n", len(result.Zones), vizURL)
	return nil
}
