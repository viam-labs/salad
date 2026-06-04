package main

import (
	"fmt"
	"os"
	"path/filepath"
	"time"

	"github.com/golang/geo/r3"
	vizClient "github.com/viam-labs/motion-tools/client/client"
	"go.viam.com/rdk/pointcloud"

	"salad/filter"
	saladutils "salad/utils"
)

type stdoutLogger struct{}

func (stdoutLogger) Infof(format string, args ...any) {
	fmt.Printf(format+"\n", args...)
}

func runFilter(flags FilterFlags) error {
	if flags.OutputPath == "" {
		dir := filepath.Join("output", time.Now().Format("20060102-150405"))
		if err := os.MkdirAll(dir, 0o750); err != nil {
			return fmt.Errorf("failed to create output directory %q: %w", dir, err)
		}
		flags.OutputPath = filepath.Join(dir, "filtered.pcd")
	}

	fmt.Printf("Loading %s\n", flags.InputPath)
	pc, err := pointcloud.NewFromFile(flags.InputPath, "")
	if err != nil {
		return fmt.Errorf("failed to load point cloud: %w", err)
	}
	fmt.Printf("Loaded %d points\n", pc.Size())

	out, _, err := filter.Apply(pc, filter.Options{
		VoxelMM:            flags.VoxelMM,
		NeighborRadius:     flags.NeighborRadius,
		MinNeighbors:       flags.MinNeighbors,
		MinComponentVoxels: flags.MinComponentVoxels,
	}, stdoutLogger{})
	if err != nil {
		return err
	}

	if err := saladutils.WritePCD(out, flags.OutputPath); err != nil {
		return err
	}
	fmt.Printf("Wrote %s\n", flags.OutputPath)

	if flags.Viz {
		if err := vizFilterResult(pc, out, flags.VizURL); err != nil {
			return fmt.Errorf("visualizing filter result: %w", err)
		}
	}
	return nil
}

func vizFilterResult(input, kept pointcloud.PointCloud, vizURL string) error {
	keptKeys := make(map[r3.Vector]bool, kept.Size())
	kept.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		keptKeys[p] = true
		return true
	})

	removed := pointcloud.NewBasicPointCloud(input.Size() - kept.Size())
	input.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if !keptKeys[p] {
			_ = removed.Set(p, d) //nolint:errcheck // Set on freshly-allocated BasicPointCloud cannot fail
		}
		return true
	})

	vizClient.SetURL(vizURL)
	if err := vizClient.RemoveAllSpatialObjects(); err != nil {
		return fmt.Errorf("clearing visualizer: %w", err)
	}
	if err := vizClient.DrawPointCloud("filter-input", input, nil); err != nil {
		return fmt.Errorf("drawing input cloud: %w", err)
	}
	if err := vizClient.DrawPointCloud("filter-kept", kept, nil); err != nil {
		return fmt.Errorf("drawing kept cloud: %w", err)
	}
	red := [3]uint8{255, 0, 0}
	if err := vizClient.DrawPointCloud("filter-removed", removed, &red); err != nil {
		return fmt.Errorf("drawing removed cloud: %w", err)
	}
	fmt.Printf("Drew filter-input (%d), filter-kept (%d), filter-removed (%d) at %s\n",
		input.Size(), kept.Size(), removed.Size(), vizURL)
	return nil
}
