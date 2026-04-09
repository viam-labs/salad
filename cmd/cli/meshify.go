package main

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
	"time"

	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/spatialmath"
)

func runMeshify(flags MeshifyFlags) error {
	outputPath := flags.OutputPath
	if outputPath == "" {
		dir := filepath.Join("output", time.Now().Format("20060102-150405"))
		if err := os.MkdirAll(dir, 0o755); err != nil {
			return fmt.Errorf("failed to create output directory %q: %w", dir, err)
		}
		outputPath = filepath.Join(dir, "mesh.ply")
	}

	fmt.Printf("Meshing %s → %s\n", flags.InputPath, outputPath)
	if err := execMeshifier(flags.InputPath, outputPath, flags.KDTreeKNN, flags.OrientNN, flags.LODMultiplier, flags.VoxelSize, flags.SmoothIterations); err != nil {
		return err
	}
	if _, err := os.Stat(outputPath); err != nil {
		return fmt.Errorf("meshifier exited successfully but output file was not created at %q", outputPath)
	}
	fmt.Printf("Wrote %s\n", outputPath)

	vizURL := flags.VizURL
	if vizURL == "" && flags.Viz {
		vizURL = "http://localhost:3000"
	}
	if vizURL != "" {
		vizClient.SetURL(vizURL)
		mesh, err := spatialmath.NewMeshFromPLYFile(outputPath)
		if err != nil {
			return fmt.Errorf("failed to load mesh for display: %w", err)
		}
		mesh.SetLabel(filepath.Base(outputPath))
		if err := vizClient.DrawGeometry(mesh, "lightblue"); err != nil {
			return fmt.Errorf("failed to display mesh: %w", err)
		}
		fmt.Printf("Displayed mesh in visualizer at %s\n", vizURL)
	}

	return nil
}

func execMeshifier(pcdPath, meshPath string, kdTreeKNN, orientNN, lodMultiplier int, voxelSize float64, smoothIterations int) error {
	scriptPath, err := meshifierScriptPath()
	if err != nil {
		return err
	}
	if fi, err := os.Stat(scriptPath); err != nil || fi.IsDir() {
		return fmt.Errorf("meshifier script not found at %q — ensure meshifier/ directory is alongside the binary", scriptPath)
	}

	cmd := exec.Command("python3", scriptPath,
		pcdPath,
		meshPath,
		strconv.Itoa(kdTreeKNN),
		strconv.Itoa(orientNN),
		strconv.Itoa(lodMultiplier),
		strconv.FormatFloat(voxelSize, 'f', -1, 64),
		strconv.Itoa(smoothIterations),
	)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("meshifier failed: %w", err)
	}
	return nil
}

// meshifierScriptPath resolves main.py relative to the running executable.
// Binary lives at <repo>/bin/salad-cli, scripts live at <repo>/meshifier/main.py.
func meshifierScriptPath() (string, error) {
	exe, err := os.Executable()
	if err != nil {
		return "", fmt.Errorf("cannot resolve executable path: %w", err)
	}
	return filepath.Join(filepath.Dir(exe), "..", "meshifier", "main.py"), nil
}
