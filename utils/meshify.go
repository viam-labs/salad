package utils

import (
	"context"
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
)

// ExecMeshifier runs the Python meshifier script on the given PCD file and writes
// a PLY mesh to meshPath. The context is wired into the subprocess so cancellation
// (e.g. from a stop command) terminates the Python process.
//
// Python dependencies must be pre-installed into meshifier/deps/ at build time
// via `make module.tar.gz`; they are bundled into the module tarball.
func ExecMeshifier(ctx context.Context, pcdPath, meshPath string, kdTreeKNN, orientNN, lodMultiplier int) error {
	scriptPath, err := meshifierScriptPath()
	if err != nil {
		return err
	}
	if fi, err := os.Stat(scriptPath); err != nil || fi.IsDir() {
		return fmt.Errorf("meshifier script not found at %q — ensure meshifier/ directory is alongside the binary", scriptPath)
	}

	depsDir, err := ensureMeshifierDeps(scriptPath)
	if err != nil {
		return fmt.Errorf("meshifier dependencies missing: %w", err)
	}

	cmd := exec.CommandContext(ctx, "python3", scriptPath,
		pcdPath,
		meshPath,
		strconv.Itoa(kdTreeKNN),
		strconv.Itoa(orientNN),
		strconv.Itoa(lodMultiplier),
	)
	cmd.Env = append(os.Environ(), "PYTHONPATH="+depsDir)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("meshifier failed: %w", err)
	}
	return nil
}

// ensureMeshifierDeps verifies that the Python dependencies were pre-installed
// into deps/ at build time (via `make module.tar.gz`). Returns the deps path.
func ensureMeshifierDeps(scriptPath string) (string, error) {
	meshifierDir := filepath.Dir(scriptPath)
	depsDir := filepath.Join(meshifierDir, "deps")
	sentinel := filepath.Join(depsDir, ".installed")

	if _, err := os.Stat(sentinel); err != nil {
		return "", fmt.Errorf("meshifier Python dependencies not found at %q — rebuild the module with `make module.tar.gz`", depsDir)
	}
	return depsDir, nil
}

// meshifierScriptPath resolves main.py relative to the running executable.
// Binary lives at <root>/bin/salad (or bin/salad-cli), script at <root>/meshifier/main.py.
func meshifierScriptPath() (string, error) {
	exe, err := os.Executable()
	if err != nil {
		return "", fmt.Errorf("cannot resolve executable path: %w", err)
	}
	return filepath.Join(filepath.Dir(exe), "..", "meshifier", "main.py"), nil
}
