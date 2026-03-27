package main

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
)

func runMeshify(flags MeshifyFlags) error {
	fmt.Printf("Meshing %s → %s\n", flags.InputPath, flags.OutputPath)
	if err := execMeshifier(flags.InputPath, flags.OutputPath, flags.KDTreeKNN, flags.OrientNN, flags.LODMultiplier); err != nil {
		return err
	}
	fmt.Printf("Wrote %s\n", flags.OutputPath)
	return nil
}

// execMeshifier runs the Python BPA meshifier script.
func execMeshifier(pcdPath, meshPath string, kdTreeKNN, orientNN, lodMultiplier int) error {
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
