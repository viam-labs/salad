package utils

import (
	"fmt"
	"os"

	"go.viam.com/rdk/pointcloud"
)

// WritePCD writes a point cloud to a file in binary PCD format.
func WritePCD(pc pointcloud.PointCloud, path string) error {
	f, err := os.Create(path)
	if err != nil {
		return fmt.Errorf("failed to create %q: %w", path, err)
	}
	defer f.Close()
	return pointcloud.ToPCD(pc, f, pointcloud.PCDBinary)
}
