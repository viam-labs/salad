package main

import (
	"context"
	"fmt"
	"os"
	"path/filepath"
	"time"

	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/spatialmath"

	saladutils "salad/utils"
)

func runMeshify(flags MeshifyFlags) error {
	if flags.OutputPath == "" {
		dir := filepath.Join("output", time.Now().Format("20060102-150405"))
		if err := os.MkdirAll(dir, 0o750); err != nil {
			return fmt.Errorf("failed to create output directory %q: %w", dir, err)
		}
		flags.OutputPath = filepath.Join(dir, "mesh.ply")
	}
	fmt.Printf("Meshing %s → %s\n", flags.InputPath, flags.OutputPath)
	if err := saladutils.ExecMeshifier(context.Background(), flags.InputPath, flags.OutputPath, flags.KDTreeKNN, flags.OrientNN, flags.LODMultiplier, flags.TargetTriangles); err != nil {
		return err
	}
	if _, err := os.Stat(flags.OutputPath); err != nil {
		return fmt.Errorf("meshifier exited successfully but output file not found at %q", flags.OutputPath)
	}
	fmt.Printf("Wrote %s\n", flags.OutputPath)

	if flags.Viz {
		vizClient.SetURL(flags.VizURL)
		if err := vizClient.RemoveAllSpatialObjects(); err != nil {
			return fmt.Errorf("clearing visualizer: %w", err)
		}
		mesh, err := spatialmath.NewMeshFromPLYFile(flags.OutputPath)
		if err != nil {
			return fmt.Errorf("loading mesh for viz: %w", err)
		}
		mesh.SetLabel("mesh")
		if err := vizClient.DrawGeometry(mesh, "lightblue"); err != nil {
			return fmt.Errorf("drawing mesh: %w", err)
		}
	}
	return nil
}
