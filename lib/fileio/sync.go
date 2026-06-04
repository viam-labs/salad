package fileio

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"
	"time"
)

const defaultSyncBaseDir = "/root/.viam/capture"

func SaveJsonToSync(data any, filename, buildID string, t time.Time) error {
	if buildID == "" {
		return nil
	}
	b, err := json.Marshal(data)
	if err != nil {
		return fmt.Errorf("marshal %s: %w", filename, err)
	}
	dir := filepath.Join(defaultSyncBaseDir, fmt.Sprintf("tag=%s", buildID))
	if err := EnsureDir(dir); err != nil {
		return err
	}
	path := filepath.Join(dir, fmt.Sprintf("%s_%s", t.Format(timestampLayout), filename))
	return os.WriteFile(path, b, 0o600)
}
