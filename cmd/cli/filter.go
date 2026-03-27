package main

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

func filterPointCloud(pc pointcloud.PointCloud, voxelMM float64, minNeighbors int) (pointcloud.PointCloud, error) {
	type voxelKey struct{ x, y, z int }

	toKey := func(p r3.Vector) voxelKey {
		return voxelKey{
			x: int(p.X / voxelMM),
			y: int(p.Y / voxelMM),
			z: int(p.Z / voxelMM),
		}
	}

	occupied := make(map[voxelKey]bool, pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		occupied[toKey(p)] = true
		return true
	})

	keep := make(map[voxelKey]bool, len(occupied))
	for k := range occupied {
		neighbors := 0
		for dx := -1; dx <= 1; dx++ {
			for dy := -1; dy <= 1; dy++ {
				for dz := -1; dz <= 1; dz++ {
					if dx == 0 && dy == 0 && dz == 0 {
						continue
					}
					if occupied[voxelKey{k.x + dx, k.y + dy, k.z + dz}] {
						neighbors++
					}
				}
			}
		}
		if neighbors >= minNeighbors {
			keep[k] = true
		}
	}

	out := pointcloud.NewBasicPointCloud(pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if keep[toKey(p)] {
			_ = out.Set(p, d)
		}
		return true
	})
	return out, nil
}

func runFilter(flags FilterFlags) error {
	fmt.Printf("Loading %s\n", flags.InputPath)
	pc, err := pointcloud.NewFromFile(flags.InputPath, "")
	if err != nil {
		return fmt.Errorf("failed to load point cloud: %w", err)
	}
	fmt.Printf("Loaded %d points\n", pc.Size())

	out, err := filterPointCloud(pc, flags.VoxelMM, flags.MinNeighbors)
	if err != nil {
		return err
	}
	fmt.Printf("Kept %d / %d points after filtering (removed %d)\n", out.Size(), pc.Size(), pc.Size()-out.Size())

	if err := writePCD(out, flags.OutputPath); err != nil {
		return err
	}
	fmt.Printf("Wrote %s\n", flags.OutputPath)
	return nil
}
