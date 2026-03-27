package main

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
)

type CropBounds struct {
	MinX, MaxX float64
	MinY, MaxY float64
	MinZ, MaxZ float64
}

func cropPointCloud(pc pointcloud.PointCloud, b CropBounds) (pointcloud.PointCloud, error) {
	out := pointcloud.NewBasicPointCloud(pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		if p.X >= b.MinX && p.X <= b.MaxX &&
			p.Y >= b.MinY && p.Y <= b.MaxY &&
			p.Z >= b.MinZ && p.Z <= b.MaxZ {
			_ = out.Set(p, d)
		}
		return true
	})
	return out, nil
}

func runCrop(flags CropFlags) error {
	fmt.Printf("Loading %s\n", flags.InputPath)
	pc, err := pointcloud.NewFromFile(flags.InputPath, "")
	if err != nil {
		return fmt.Errorf("failed to load point cloud: %w", err)
	}
	fmt.Printf("Loaded %d points\n", pc.Size())

	md := pc.MetaData()
	fmt.Printf("Bounds: X[%.1f, %.1f]  Y[%.1f, %.1f]  Z[%.1f, %.1f]\n",
		md.MinX, md.MaxX, md.MinY, md.MaxY, md.MinZ, md.MaxZ)

	b := CropBounds{
		MinX: flags.MinX, MaxX: flags.MaxX,
		MinY: flags.MinY, MaxY: flags.MaxY,
		MinZ: flags.MinZ, MaxZ: flags.MaxZ,
	}
	out, err := cropPointCloud(pc, b)
	if err != nil {
		return err
	}
	fmt.Printf("Kept %d / %d points after cropping (removed %d)\n", out.Size(), pc.Size(), pc.Size()-out.Size())

	if err := writePCD(out, flags.OutputPath); err != nil {
		return err
	}
	fmt.Printf("Wrote %s\n", flags.OutputPath)
	return nil
}
