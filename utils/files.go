package utils

import (
	"fmt"
	"os"

	"go.viam.com/rdk/pointcloud"
)

// WritePCD writes a point cloud to a file in binary PCD format.
func WritePCD(pc pointcloud.PointCloud, path string) error {
	f, err := os.Create(path) //nolint:gosec // caller-controlled output path
	if err != nil {
		return fmt.Errorf("failed to create %q: %w", path, err)
	}
	defer f.Close() //nolint:errcheck // best-effort close after successful write
	return pointcloud.ToPCD(pc, f, pointcloud.PCDBinary)
}
