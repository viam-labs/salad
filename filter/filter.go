package filter

import (
	"fmt"
	"sort"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

type Options struct {
	VoxelMM            float64
	NeighborRadius     int
	MinNeighbors       int
	MinComponentVoxels int
}

func DefaultOptions() Options {
	return Options{
		VoxelMM:            10.0,
		NeighborRadius:     1,
		MinNeighbors:       8,
		MinComponentVoxels: 1000,
	}
}

type Logger interface {
	Infof(format string, args ...any)
}

type Stats struct {
	InputPoints        int
	OutputPoints       int
	OccupiedVoxels     int
	AfterNeighborPass  int
	AfterComponentPass int
	ComponentSizes     []int
}

type voxelKey struct{ x, y, z int }

func Apply(pc pointcloud.PointCloud, opts Options, log Logger) (pointcloud.PointCloud, Stats, error) {
	stats := Stats{InputPoints: pc.Size()}

	if opts.VoxelMM <= 0 {
		return nil, stats, fmt.Errorf("voxel-mm must be > 0, got %v", opts.VoxelMM)
	}
	if opts.NeighborRadius < 1 {
		return nil, stats, fmt.Errorf("neighbor-radius must be >= 1, got %d", opts.NeighborRadius)
	}
	maxNeighbors := (2*opts.NeighborRadius+1)*(2*opts.NeighborRadius+1)*(2*opts.NeighborRadius+1) - 1
	if opts.MinNeighbors < 0 || opts.MinNeighbors > maxNeighbors {
		return nil, stats, fmt.Errorf("min-neighbors must be in [0, %d] for neighbor-radius=%d, got %d",
			maxNeighbors, opts.NeighborRadius, opts.MinNeighbors)
	}
	if opts.MinComponentVoxels < 0 {
		return nil, stats, fmt.Errorf("min-component-voxels must be >= 0, got %d", opts.MinComponentVoxels)
	}

	toKey := func(p r3.Vector) voxelKey {
		return voxelKey{
			x: int(p.X / opts.VoxelMM),
			y: int(p.Y / opts.VoxelMM),
			z: int(p.Z / opts.VoxelMM),
		}
	}

	occupied := make(map[voxelKey]bool, pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		occupied[toKey(p)] = true
		return true
	})
	stats.OccupiedVoxels = len(occupied)

	keep := occupied
	if opts.MinNeighbors > 0 {
		keep = neighborCountFilter(occupied, opts.NeighborRadius, opts.MinNeighbors)
		stats.AfterNeighborPass = len(keep)
		logf(log, "filter: neighbor-count pass kept %d / %d voxels (radius=%d, min-neighbors=%d)",
			len(keep), len(occupied), opts.NeighborRadius, opts.MinNeighbors)
	}
	if opts.MinComponentVoxels > 0 {
		before := len(keep)
		var sizes []int
		keep, sizes = connectedComponentFilter(keep, opts.MinComponentVoxels)
		stats.AfterComponentPass = len(keep)
		stats.ComponentSizes = sizes
		preview := sizes
		if len(preview) > 10 {
			preview = preview[:10]
		}
		logf(log, "filter: connected-components pass kept %d / %d voxels; %d component(s); largest sizes: %v (threshold=%d)",
			len(keep), before, len(sizes), preview, opts.MinComponentVoxels)
	}

	out := pointcloud.NewBasicPointCloud(pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if keep[toKey(p)] {
			_ = out.Set(p, d) //nolint:errcheck // Set on freshly-allocated BasicPointCloud cannot fail
		}
		return true
	})
	stats.OutputPoints = out.Size()
	logf(log, "filter: kept %d / %d points (removed %d)",
		stats.OutputPoints, stats.InputPoints, stats.InputPoints-stats.OutputPoints)
	return out, stats, nil
}

func neighborCountFilter(occupied map[voxelKey]bool, neighborRadius, minNeighbors int) map[voxelKey]bool {
	keep := make(map[voxelKey]bool, len(occupied))
	for k := range occupied {
		neighbors := 0
		for dx := -neighborRadius; dx <= neighborRadius; dx++ {
			for dy := -neighborRadius; dy <= neighborRadius; dy++ {
				for dz := -neighborRadius; dz <= neighborRadius; dz++ {
					if dx == 0 && dy == 0 && dz == 0 {
						continue
					}
					if occupied[voxelKey{k.x + dx, k.y + dy, k.z + dz}] {
						neighbors++
						if neighbors >= minNeighbors {
							break
						}
					}
				}
				if neighbors >= minNeighbors {
					break
				}
			}
			if neighbors >= minNeighbors {
				break
			}
		}
		if neighbors >= minNeighbors {
			keep[k] = true
		}
	}
	return keep
}

func connectedComponentFilter(occupied map[voxelKey]bool, minComponentVoxels int) (map[voxelKey]bool, []int) {
	visited := make(map[voxelKey]bool, len(occupied))
	keep := make(map[voxelKey]bool, len(occupied))
	var sizes []int
	for start := range occupied {
		if visited[start] {
			continue
		}
		component := []voxelKey{start}
		visited[start] = true
		for i := 0; i < len(component); i++ {
			k := component[i]
			for dx := -1; dx <= 1; dx++ {
				for dy := -1; dy <= 1; dy++ {
					for dz := -1; dz <= 1; dz++ {
						if dx == 0 && dy == 0 && dz == 0 {
							continue
						}
						n := voxelKey{k.x + dx, k.y + dy, k.z + dz}
						if occupied[n] && !visited[n] {
							visited[n] = true
							component = append(component, n)
						}
					}
				}
			}
		}
		sizes = append(sizes, len(component))
		if len(component) >= minComponentVoxels {
			for _, k := range component {
				keep[k] = true
			}
		}
	}
	sort.Sort(sort.Reverse(sort.IntSlice(sizes)))
	return keep, sizes
}

func logf(log Logger, format string, args ...any) {
	if log == nil {
		return
	}
	log.Infof(format, args...)
}
