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
// targetTriangles, if > 0, decimates the Poisson output to roughly that
// triangle count using quadric error metrics. Pass 0 to disable.
//
// Python dependencies are installed on first use into meshifier/deps/ on the target
// machine, so they are always compiled for the local Python version.
func ExecMeshifier(ctx context.Context, pcdPath, meshPath string, kdTreeKNN, orientNN, lodMultiplier, targetTriangles int) error {
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
		strconv.Itoa(targetTriangles),
	)
	cmd.Env = append(os.Environ(), "PYTHONPATH="+depsDir)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("meshifier failed: %w", err)
	}
	return nil
}

// ensureMeshifierDeps installs Python dependencies into meshifier/deps/ on the
// target machine if not already present. Installing natively ensures the compiled
// C extensions match the local Python version and CPU architecture.
func ensureMeshifierDeps(scriptPath string) (string, error) {
	meshifierDir := filepath.Dir(scriptPath)
	depsDir := filepath.Join(meshifierDir, "deps")
	sentinel := filepath.Join(depsDir, ".installed")

	if _, err := os.Stat(sentinel); err == nil {
		return depsDir, nil
	}

	requirementsPath := filepath.Join(meshifierDir, "requirements.txt")
	if _, err := os.Stat(requirementsPath); err != nil {
		return "", fmt.Errorf("meshifier requirements.txt not found at %q", requirementsPath)
	}

	if err := os.MkdirAll(depsDir, 0o755); err != nil {
		return "", fmt.Errorf("failed to create meshifier deps dir: %w", err)
	}

	cmd := exec.Command("pip3", "install", "--target", depsDir, "-r", requirementsPath)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		return "", fmt.Errorf("failed to install meshifier Python dependencies: %w", err)
	}

	if err := os.WriteFile(sentinel, nil, 0o644); err != nil {
		return "", fmt.Errorf("failed to write meshifier deps sentinel: %w", err)
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
