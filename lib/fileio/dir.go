package fileio

import (
	"fmt"
	"os"
)

func EnsureDir(path string) error {
	info, err := os.Stat(path)
	if os.IsNotExist(err) {
		return os.MkdirAll(path, 0o750)
	}
	if err != nil {
		return fmt.Errorf("error checking directory: %w", err)
	}
	if !info.IsDir() {
		return fmt.Errorf("%s exists but is not a directory", path)
	}
	return nil
}
