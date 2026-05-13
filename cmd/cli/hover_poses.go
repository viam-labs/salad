package main

import (
	"fmt"
	"time"

	"github.com/golang/geo/r3"
	"github.com/spf13/cobra"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

type HoverPosesFlags struct {
	ZonesPath     string
	MeshPath      string
	LocalFiles    string
	AboveBinExtra float64
	SphereRadius  float64
	VizURL        string
	ClearFirst    bool
}

var hoverPosesFlags HoverPosesFlags

var hoverPosesCmd = &cobra.Command{
	Use:   "hover-poses",
	Short: "Visualize computed hover poses alongside zones and mesh in motion-tools",
	Long: `Loads zones.json and mesh.ply and sends them to the motion-tools visualizer.
For each zone it draws two sphere markers at the hover height:
  - colored sphere: centroid of mesh vertices (current algorithm)
  - white sphere:   center of bounding box (alternative)
A comparison table is always printed to stdout even if the visualizer is unreachable.`,
	RunE: func(cmd *cobra.Command, args []string) error {
		return runHoverPoses(hoverPosesFlags)
	},
}

type hoverZoneRow struct {
	zone   segmentation.Zone
	cx, cy float64
	bboxCX float64
	bboxCY float64
}

func runHoverPoses(flags HoverPosesFlags) error {
	zonesPath := flags.ZonesPath
	if zonesPath == "" {
		p, err := findNewestZonesJSON(flags.LocalFiles)
		if err != nil {
			return fmt.Errorf("finding zones.json: %w", err)
		}
		zonesPath = p
	}

	result, err := segmentation.LoadZones(zonesPath)
	if err != nil {
		return fmt.Errorf("loading zones: %w", err)
	}
	if len(result.Zones) == 0 {
		return fmt.Errorf("no zones found in %q", zonesPath)
	}

	meshPath := flags.MeshPath
	if meshPath == "" {
		meshPath = resolveZoneMeshPath(flags.LocalFiles, result.SourceMesh)
	}

	hoverZ := result.ZMean + flags.AboveBinExtra

	// --- Compute everything first ---
	rows := make([]hoverZoneRow, 0, len(result.Zones))
	for _, zone := range result.Zones {
		cx, cy, err := zone.Centroid()
		if err != nil {
			return fmt.Errorf("zone %d centroid: %w", zone.ID, err)
		}
		rows = append(rows, hoverZoneRow{
			zone:   zone,
			cx:     cx,
			cy:     cy,
			bboxCX: (zone.MinX + zone.MaxX) / 2,
			bboxCY: (zone.MinY + zone.MaxY) / 2,
		})
	}

	// --- Print table (always, regardless of visualizer) ---
	fmt.Printf("\nZones file: %s\n", zonesPath)
	fmt.Printf("ZMean=%.1f  above-bin-extra=%.1f  hover_z=%.1f  sphere_r=%.0f\n\n",
		result.ZMean, flags.AboveBinExtra, hoverZ, flags.SphereRadius)
	fmt.Printf("%-6s  %-22s  %-22s  %-14s\n",
		"Zone", "Centroid (X, Y)", "BBox Center (X, Y)", "Diff (dX, dY)")
	fmt.Println("------  ----------------------  ----------------------  --------------")
	for _, r := range rows {
		fmt.Printf("%-6d  (%-7.1f, %-7.1f)   (%-7.1f, %-7.1f)   (%-5.1f, %-5.1f)\n",
			r.zone.ID, r.cx, r.cy, r.bboxCX, r.bboxCY, r.cx-r.bboxCX, r.cy-r.bboxCY)
	}
	fmt.Println("\nLegend: colored sphere = centroid (current algo)  |  white sphere = bbox center")

	// --- Send to visualizer ---
	vizClient.SetURL(flags.VizURL)
	if flags.ClearFirst {
		if err := vizClient.RemoveAllSpatialObjects(); err != nil {
			return fmt.Errorf("clearing visualizer: %w", err)
		}
	}

	if meshPath != "" {
		mesh, err := spatialmath.NewMeshFromPLYFile(meshPath)
		if err != nil {
			return fmt.Errorf("loading mesh %q: %w", meshPath, err)
		}
		mesh.SetLabel("bin-mesh")
		if err := vizClient.DrawGeometry(mesh, "lightgrey"); err != nil {
			return fmt.Errorf("drawing mesh: %w", err)
		}
		fmt.Printf("Drew mesh: %s\n", meshPath)
		time.Sleep(200 * time.Millisecond)
	}

	if err := appendZoneColoredMeshes(result); err != nil {
		return err
	}
	time.Sleep(200 * time.Millisecond)

	for _, r := range rows {
		color := zoneVizColors[r.zone.ID%len(zoneVizColors)]

		centroidPose := spatialmath.NewPoseFromPoint(r3.Vector{X: r.cx, Y: r.cy, Z: hoverZ})
		centroidSphere, err := spatialmath.NewSphere(centroidPose, flags.SphereRadius, fmt.Sprintf("hover-centroid-%d", r.zone.ID))
		if err != nil {
			return fmt.Errorf("creating centroid sphere for zone %d: %w", r.zone.ID, err)
		}
		if err := vizClient.DrawGeometry(centroidSphere, color); err != nil {
			return fmt.Errorf("drawing centroid sphere for zone %d: %w", r.zone.ID, err)
		}

		bboxPose := spatialmath.NewPoseFromPoint(r3.Vector{X: r.bboxCX, Y: r.bboxCY, Z: hoverZ})
		bboxSphere, err := spatialmath.NewSphere(bboxPose, flags.SphereRadius, fmt.Sprintf("hover-bbox-%d", r.zone.ID))
		if err != nil {
			return fmt.Errorf("creating bbox sphere for zone %d: %w", r.zone.ID, err)
		}
		if err := vizClient.DrawGeometry(bboxSphere, "white"); err != nil {
			return fmt.Errorf("drawing bbox sphere for zone %d: %w", r.zone.ID, err)
		}

		time.Sleep(100 * time.Millisecond)
	}

	fmt.Println("Visualizer:", flags.VizURL)
	return nil
}

// resolveZoneMeshPath finds the mesh PLY adjacent to the zones file or via the stored source path.
func resolveZoneMeshPath(localDir, sourceMeshInJSON string) string {
	p, ok := resolveZoneBackgroundMeshPath(localDir, "", sourceMeshInJSON)
	if ok {
		return p
	}
	return ""
}
