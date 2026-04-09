package main

import (
	"fmt"
	"image/color"
	"path/filepath"
	"strings"
	"time"

	"github.com/golang/geo/r3"
	"github.com/spf13/cobra"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/pointcloud"
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
	MinBinAreaMM2      float64
}

var segmentFlags SegmentFlags

var segmentCmd = &cobra.Command{
	Use:   "segment",
	Short: "Segment a fridge mesh into individual bin regions",
	Long: `Loads a PLY mesh of the fridge and segments it into individual ingredient
bin XY footprints using a height-map connected-component algorithm.

The fridge is scanned from above; dividers appear as high-Z ridges between
lower bin surfaces. Each detected region represents the stable XY footprint
of one bin, independent of ingredient fill level.

The result is written to a JSON sidecar file (default: <mesh>.segments.json)
for use by automated bin grabbing at runtime.

If --viz is set, each bin is rendered in a distinct color in the motion-tools
visualizer alongside the source mesh for visual verification.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		return runSegment(segmentFlags)
	},
}

func runSegment(flags SegmentFlags) error {
	opts := segmentation.Options{
		CellSizeMM:         flags.CellSizeMM,
		DividerZPercentile: flags.DividerZPercentile,
		MinBinAreaMM2:      flags.MinBinAreaMM2,
	}

	fmt.Printf("Segmenting %s\n", flags.MeshPath)
	result, err := segmentation.SegmentFridgeBins(flags.MeshPath, opts)
	if err != nil {
		return err
	}

	fmt.Printf("Detected %d bin(s):\n\n", len(result.Bins))
	fmt.Printf("  %-4s  %-10s  %-10s  %-10s  %-10s  %-12s  %-12s\n",
		"ID", "MinX", "MaxX", "MinY", "MaxY", "Width(mm)", "Depth(mm)")
	fmt.Printf("  %s\n", strings.Repeat("-", 76))
	for _, b := range result.Bins {
		fmt.Printf("  %-4d  %-10.1f  %-10.1f  %-10.1f  %-10.1f  %-12.1f  %-12.1f\n",
			b.ID, b.MinX, b.MaxX, b.MinY, b.MaxY,
			b.MaxX-b.MinX, b.MaxY-b.MinY)
	}
	fmt.Println()

	outPath := flags.OutputPath
	if outPath == "" {
		ext := filepath.Ext(flags.MeshPath)
		outPath = strings.TrimSuffix(flags.MeshPath, ext) + ".segments.json"
	}
	if err := segmentation.SaveResult(result, outPath); err != nil {
		return err
	}
	fmt.Printf("Wrote %s\n", outPath)

	if flags.Viz {
		return visualizeSegmentation(result, flags.VizURL, flags.MeshPath)
	}
	return nil
}

// binVizColors is the palette used to distinguish individual bins in motion-tools.
// Colors are chosen to be visually distinct and readable against a grey mesh.
var binVizColors = [][3]uint8{
	{255, 99, 71},   // tomato
	{30, 144, 255},  // dodger blue
	{50, 205, 50},   // lime green
	{255, 165, 0},   // orange
	{147, 112, 219}, // medium purple
	{0, 206, 209},   // dark turquoise
	{255, 215, 0},   // gold
	{255, 20, 147},  // deep pink
	{0, 128, 128},   // teal
	{210, 105, 30},  // chocolate
}

func visualizeSegmentation(result *segmentation.Result, vizURL, meshPath string) error {
	vizClient.SetURL(vizURL)
	if err := vizClient.RemoveAllSpatialObjects(); err != nil {
		return fmt.Errorf("clearing visualizer: %w", err)
	}

	// Draw the source mesh in light grey as a reference background.
	mesh, err := spatialmath.NewMeshFromPLYFile(meshPath)
	if err != nil {
		return fmt.Errorf("loading mesh for viz: %w", err)
	}
	mesh.SetLabel("fridge-mesh")
	if err := vizClient.DrawGeometry(mesh, "lightgrey"); err != nil {
		return fmt.Errorf("drawing mesh: %w", err)
	}
	time.Sleep(200 * time.Millisecond)

	// Draw each bin as a colored point cloud filling its XY footprint.
	for _, bin := range result.Bins {
		col := binVizColors[bin.ID%len(binVizColors)]
		pc, err := binFootprintPointCloud(bin)
		if err != nil {
			return fmt.Errorf("building point cloud for bin %d: %w", bin.ID, err)
		}
		label := fmt.Sprintf("bin-%d", bin.ID)
		if err := vizClient.DrawPointCloud(label, pc, &col); err != nil {
			return fmt.Errorf("drawing bin %d: %w", bin.ID, err)
		}
		time.Sleep(100 * time.Millisecond)
	}

	fmt.Printf("Sent %d bin(s) to %s\n", len(result.Bins), vizURL)
	return nil
}

// binFootprintPointCloud creates a uniform grid of points at the bin's mean surface
// height, covering the bin's XY footprint. This renders as a visible colored "tile"
// for each bin in the motion-tools visualizer.
func binFootprintPointCloud(bin segmentation.BinRegion) (pointcloud.PointCloud, error) {
	const gridSpacingMM = 8.0

	estCols := int((bin.MaxX-bin.MinX)/gridSpacingMM) + 2
	estRows := int((bin.MaxY-bin.MinY)/gridSpacingMM) + 2
	pc := pointcloud.NewBasicPointCloud(estCols * estRows)

	nrgba := color.NRGBA{R: 255, G: 255, B: 255, A: 255} // color overridden by DrawPointCloud caller
	for x := bin.MinX; x <= bin.MaxX; x += gridSpacingMM {
		for y := bin.MinY; y <= bin.MaxY; y += gridSpacingMM {
			p := r3.Vector{X: x, Y: y, Z: bin.Center.Z}
			if err := pc.Set(p, pointcloud.NewColoredData(nrgba)); err != nil {
				return nil, err
			}
		}
	}
	return pc, nil
}
