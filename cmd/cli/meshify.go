package main

import (
	"fmt"
	"os"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/spatialmath"
)

func runMeshify(flags MeshifyFlags) error {
	fmt.Printf("Loading %s\n", flags.InputPath)
	pc, err := pointcloud.NewFromFile(flags.InputPath, "")
	if err != nil {
		return fmt.Errorf("failed to load point cloud: %w", err)
	}
	fmt.Printf("Loaded %d points\n", pc.Size())

	verts, tris, voxelCount, err := pointCloudToMesh(pc, flags.VoxelMM)
	if err != nil {
		return err
	}
	fmt.Printf("Built voxel mesh: %d voxels, %d verts, %d triangles\n", voxelCount, len(verts), len(tris))

	if flags.SmoothIters > 0 {
		verts = laplacianSmooth(verts, tris, flags.SmoothIters)
		fmt.Printf("Applied %d Laplacian smoothing iterations\n", flags.SmoothIters)
	}

	triangles := make([]*spatialmath.Triangle, len(tris))
	for i, t := range tris {
		triangles[i] = spatialmath.NewTriangle(verts[t[0]], verts[t[1]], verts[t[2]])
	}

	mesh := spatialmath.NewMesh(spatialmath.NewZeroPose(), triangles, "salad-scan")
	plyBytes := mesh.TrianglesToPLYBytes(false)
	if err := os.WriteFile(flags.OutputPath, plyBytes, 0o644); err != nil {
		return fmt.Errorf("failed to write %q: %w", flags.OutputPath, err)
	}
	fmt.Printf("Wrote %s (%d triangles)\n", flags.OutputPath, len(triangles))
	return nil
}

func pointCloudToMesh(pc pointcloud.PointCloud, voxelMM float64) (verts []r3.Vector, tris [][3]int, voxelCount int, err error) {
	type ivec3 struct{ x, y, z int }

	occupied := make(map[ivec3]bool, pc.Size())
	pc.Iterate(0, 0, func(p r3.Vector, _ pointcloud.Data) bool {
		occupied[ivec3{
			x: int(p.X / voxelMM),
			y: int(p.Y / voxelMM),
			z: int(p.Z / voxelMM),
		}] = true
		return true
	})

	type cornerKey struct{ x, y, z int }
	vertIdx := make(map[cornerKey]int)
	getVert := func(cx, cy, cz int) int {
		k := cornerKey{cx, cy, cz}
		if i, ok := vertIdx[k]; ok {
			return i
		}
		i := len(verts)
		vertIdx[k] = i
		verts = append(verts, r3.Vector{
			X: float64(cx) * voxelMM,
			Y: float64(cy) * voxelMM,
			Z: float64(cz) * voxelMM,
		})
		return i
	}

	addFace := func(a, b, c, d int) {
		tris = append(tris, [3]int{a, b, c})
		tris = append(tris, [3]int{a, c, d})
	}

	for k := range occupied {
		x0, y0, z0 := k.x, k.y, k.z
		x1, y1, z1 := k.x+1, k.y+1, k.z+1

		if !occupied[ivec3{k.x - 1, k.y, k.z}] {
			addFace(getVert(x0, y0, z0), getVert(x0, y1, z0), getVert(x0, y1, z1), getVert(x0, y0, z1))
		}
		if !occupied[ivec3{k.x + 1, k.y, k.z}] {
			addFace(getVert(x1, y0, z0), getVert(x1, y0, z1), getVert(x1, y1, z1), getVert(x1, y1, z0))
		}
		if !occupied[ivec3{k.x, k.y - 1, k.z}] {
			addFace(getVert(x0, y0, z0), getVert(x0, y0, z1), getVert(x1, y0, z1), getVert(x1, y0, z0))
		}
		if !occupied[ivec3{k.x, k.y + 1, k.z}] {
			addFace(getVert(x0, y1, z0), getVert(x1, y1, z0), getVert(x1, y1, z1), getVert(x0, y1, z1))
		}
		if !occupied[ivec3{k.x, k.y, k.z - 1}] {
			addFace(getVert(x0, y0, z0), getVert(x1, y0, z0), getVert(x1, y1, z0), getVert(x0, y1, z0))
		}
		if !occupied[ivec3{k.x, k.y, k.z + 1}] {
			addFace(getVert(x0, y0, z1), getVert(x0, y1, z1), getVert(x1, y1, z1), getVert(x1, y0, z1))
		}
	}

	return verts, tris, len(occupied), nil
}

func laplacianSmooth(verts []r3.Vector, tris [][3]int, iters int) []r3.Vector {
	adj := make([][]int, len(verts))
	seen := make(map[[2]int]bool, len(tris)*3)
	addEdge := func(a, b int) {
		if a > b {
			a, b = b, a
		}
		key := [2]int{a, b}
		if seen[key] {
			return
		}
		seen[key] = true
		adj[a] = append(adj[a], b)
		adj[b] = append(adj[b], a)
	}
	for _, t := range tris {
		addEdge(t[0], t[1])
		addEdge(t[1], t[2])
		addEdge(t[2], t[0])
	}

	cur := make([]r3.Vector, len(verts))
	copy(cur, verts)
	next := make([]r3.Vector, len(verts))

	for range iters {
		for i, v := range cur {
			nbrs := adj[i]
			if len(nbrs) == 0 {
				next[i] = v
				continue
			}
			var sx, sy, sz float64
			for _, j := range nbrs {
				sx += cur[j].X
				sy += cur[j].Y
				sz += cur[j].Z
			}
			n := float64(len(nbrs))
			next[i] = r3.Vector{X: sx / n, Y: sy / n, Z: sz / n}
		}
		cur, next = next, cur
	}
	return cur
}
