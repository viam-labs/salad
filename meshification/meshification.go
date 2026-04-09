package meshification

import (
	"fmt"
	"os"
	"os/exec"
	"path/filepath"
	"strconv"
)

type Config struct {
	KDTreeKNN     int
	OrientNN      int
	LODMultiplier int
}

func DefaultConfig() Config {
	return Config{KDTreeKNN: 30, OrientNN: 50, LODMultiplier: 0}
}

func Run(pcdPath, meshPath string, cfg Config) error {
	script, err := scriptPath()
	if err != nil {
		return err
	}
	if fi, err := os.Stat(script); err != nil || fi.IsDir() {
		return fmt.Errorf("meshifier script not found at %q — ensure meshifier/ is alongside the binary", script)
	}
	cmd := exec.Command("python3", script,
		pcdPath, meshPath,
		strconv.Itoa(cfg.KDTreeKNN),
		strconv.Itoa(cfg.OrientNN),
		strconv.Itoa(cfg.LODMultiplier),
	)
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr
	if err := cmd.Run(); err != nil {
		return fmt.Errorf("meshifier failed: %w", err)
	}
	return nil
}

func scriptPath() (string, error) {
	exe, err := os.Executable()
	if err != nil {
		return "", fmt.Errorf("cannot resolve executable path: %w", err)
	}
	return filepath.Join(filepath.Dir(exe), "..", "meshifier", "main.py"), nil
}
