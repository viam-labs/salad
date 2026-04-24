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
// On first call it installs the Python dependencies listed in requirements.txt into
// a local deps/ directory alongside the script, so no system-wide pip access is needed.
func ExecMeshifier(ctx context.Context, pcdPath, meshPath string, kdTreeKNN, orientNN, lodMultiplier int) error {
	scriptPath, err := meshifierScriptPath()
	if err != nil {
		return err
	}
	if fi, err := os.Stat(scriptPath); err != nil || fi.IsDir() {
		return fmt.Errorf("meshifier script not found at %q — ensure meshifier/ directory is alongside the binary", scriptPath)
	}

	depsDir, err := ensureMeshifierDeps(ctx, scriptPath)
	if err != nil {
		return fmt.Errorf("failed to install meshifier dependencies: %w", err)
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

// ensureMeshifierDeps installs Python dependencies from requirements.txt into a
// deps/ subdirectory next to the meshifier script. A sentinel file prevents
// reinstalling on every call. Returns the path to the deps directory.
func ensureMeshifierDeps(ctx context.Context, scriptPath string) (string, error) {
	meshifierDir := filepath.Dir(scriptPath)
	depsDir := filepath.Join(meshifierDir, "deps")
	sentinel := filepath.Join(depsDir, ".installed")

	if _, err := os.Stat(sentinel); err == nil {
		return depsDir, nil
	}

	reqPath := filepath.Join(meshifierDir, "requirements.txt")
	if _, err := os.Stat(reqPath); err != nil {
		return "", fmt.Errorf("requirements.txt not found at %q", reqPath)
	}

	cmd := exec.CommandContext(ctx, "python3", "-m", "pip", "install",
		"--target", depsDir,
		"-r", reqPath,
	)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		return "", fmt.Errorf("pip install failed: %w", err)
	}

	if err := os.WriteFile(sentinel, []byte("ok"), 0o644); err != nil {
		return "", fmt.Errorf("failed to write sentinel: %w", err)
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
