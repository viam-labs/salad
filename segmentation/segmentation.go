package segmentation

import (
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sort"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
)

// ZoneMesh holds the triangle mesh data for one bin zone.
// Vertices are in millimetres, matching the coordinate system used throughout the project.
// Faces index into Vertices; each face is a triangle [i0, i1, i2].
type ZoneMesh struct {
	Vertices [][3]float64 `json:"vertices"`
	Faces    [][3]int     `json:"faces"`
}

// ToSpatialMesh converts a ZoneMesh into a *spatialmath.Mesh for use with the
// motion-tools visualizer and collision checking.
func (zm ZoneMesh) ToSpatialMesh(label string) *spatialmath.Mesh {
	triangles := make([]*spatialmath.Triangle, 0, len(zm.Faces))
	for _, f := range zm.Faces {
		p0 := r3.Vector{X: zm.Vertices[f[0]][0], Y: zm.Vertices[f[0]][1], Z: zm.Vertices[f[0]][2]}
		p1 := r3.Vector{X: zm.Vertices[f[1]][0], Y: zm.Vertices[f[1]][1], Z: zm.Vertices[f[1]][2]}
		p2 := r3.Vector{X: zm.Vertices[f[2]][0], Y: zm.Vertices[f[2]][1], Z: zm.Vertices[f[2]][2]}
		triangles = append(triangles, spatialmath.NewTriangle(p0, p1, p2))
	}
	return spatialmath.NewMesh(spatialmath.NewZeroPose(), triangles, label)
}

// Zone is one detected bin region together with its mesh section from the source mesh.
type Zone struct {
	ID   int      `json:"id"`
	MinX float64  `json:"min_x"`
	MaxX float64  `json:"max_x"`
	MinY float64  `json:"min_y"`
	MaxY float64  `json:"max_y"`
	Mesh ZoneMesh `json:"mesh"`
}

// ZonesResult is the output of a segmentation run.
type ZonesResult struct {
	SourceMesh  string    `json:"source_mesh"`
	GeneratedAt time.Time `json:"generated_at"`
	Zones       []Zone    `json:"zones"`
}

// Options controls segmentation behaviour.
type Options struct {
	// CellSizeMM is the resolution of the 2D height-map grid in millimetres.
	// Smaller values detect finer bin boundaries but increase memory and runtime.
	// Default: 5.0
	CellSizeMM float64

	// DividerZPercentile is the percentile of occupied-cell max-Z values used to
	// separate divider peaks (high Z) from bin content surfaces (low Z).
	// Must be in [0, 1]. Default: 0.75.
	// Increase toward 1.0 if bins are over-segmented. Decrease toward 0.5 if bins merge.
	DividerZPercentile float64

	// DividerDilation is the number of height-map cells by which the divider mask is
	// dilated before connected-component analysis. Dilation closes gaps left by
	// Poisson-reconstruction smoothing so that dividers form continuous barriers.
	// Default: 4.
	DividerDilation int

	// MinBinAreaMM2 is the minimum bin footprint area in mm².
	// Components smaller than this are discarded as noise.
	// Default: 5000.0 (roughly 50 mm × 100 mm).
	MinBinAreaMM2 float64
}

// DefaultOptions returns sensible defaults for fridge segmentation.
func DefaultOptions() Options {
	return Options{
		CellSizeMM:         5.0,
		DividerZPercentile: 0.75,
		DividerDilation:    4,
		MinBinAreaMM2:      5000.0,
	}
}

// SegmentFridgeBins loads a PLY mesh and segments it into bin zones.
//
// Algorithm:
//  1. Projects mesh triangle vertices onto a 2D grid, recording max Z per cell (height map).
//  2. Derives a Z threshold from DividerZPercentile of occupied-cell heights.
//  3. Marks cells above the threshold as dividers/walls.
//  4. Dilates the divider mask by DividerDilation cells to close gaps caused by Poisson
//     reconstruction smoothing.
//  5. Runs BFS connected-component labeling on the remaining open cells (4-connectivity).
//  6. Filters out components smaller than MinBinAreaMM2.
//  7. Assigns each mesh triangle to a zone based on which component contains its centroid.
//  8. Returns one Zone per remaining component, sorted left-to-right (X then Y).
func SegmentFridgeBins(meshPath string, opts Options) (*ZonesResult, error) {
	mesh, err := spatialmath.NewMeshFromPLYFile(meshPath)
	if err != nil {
		return nil, fmt.Errorf("loading mesh %q: %w", meshPath, err)
	}

	triangles := mesh.Triangles()
	if len(triangles) == 0 {
		return nil, fmt.Errorf("mesh %q has no triangles", meshPath)
	}

	zones, err := segmentTriangles(triangles, opts)
	if err != nil {
		return nil, err
	}

	return &ZonesResult{
		SourceMesh:  meshPath,
		GeneratedAt: time.Now(),
		Zones:       zones,
	}, nil
}

// SaveZones writes a ZonesResult to a JSON file.
func SaveZones(result *ZonesResult, path string) error {
	data, err := json.MarshalIndent(result, "", "  ")
	if err != nil {
		return fmt.Errorf("marshaling zones: %w", err)
	}
	if err := os.WriteFile(path, data, 0o644); err != nil {
		return fmt.Errorf("writing %q: %w", path, err)
	}
	return nil
}

// segmentTriangles performs height-map segmentation on the given mesh triangles.
func segmentTriangles(triangles []*spatialmath.Triangle, opts Options) ([]Zone, error) {
	if opts.CellSizeMM <= 0 {
		return nil, fmt.Errorf("CellSizeMM must be > 0, got %v", opts.CellSizeMM)
	}
	if opts.DividerZPercentile < 0 || opts.DividerZPercentile > 1 {
		return nil, fmt.Errorf("DividerZPercentile must be in [0,1], got %v", opts.DividerZPercentile)
	}

	// Extract vertices from triangles.
	type face struct{ verts [3]r3.Vector }
	faces := make([]face, len(triangles))

	minX, minY := math.MaxFloat64, math.MaxFloat64
	maxX, maxY := -math.MaxFloat64, -math.MaxFloat64

	for i, tri := range triangles {
		pts := tri.Points()
		faces[i] = face{verts: [3]r3.Vector{pts[0], pts[1], pts[2]}}
		for _, p := range pts {
			if p.X < minX {
				minX = p.X
			}
			if p.X > maxX {
				maxX = p.X
			}
			if p.Y < minY {
				minY = p.Y
			}
			if p.Y > maxY {
				maxY = p.Y
			}
		}
	}

	cols := int((maxX-minX)/opts.CellSizeMM) + 1
	rows := int((maxY-minY)/opts.CellSizeMM) + 1

	// --- Build 2D height map (max Z per cell) ---
	type cell struct {
		maxZ    float64
		hasData bool
	}
	grid := make([][]cell, rows)
	for r := range grid {
		grid[r] = make([]cell, cols)
		for c := range grid[r] {
			grid[r][c].maxZ = -math.MaxFloat64
		}
	}

	addPt := func(p r3.Vector) {
		c := int((p.X - minX) / opts.CellSizeMM)
		r := int((p.Y - minY) / opts.CellSizeMM)
		if c < 0 || c >= cols || r < 0 || r >= rows {
			return
		}
		if p.Z > grid[r][c].maxZ {
			grid[r][c].maxZ = p.Z
		}
		grid[r][c].hasData = true
	}

	for _, f := range faces {
		for _, p := range f.verts {
			addPt(p)
		}
	}

	// --- Compute Z threshold via percentile of occupied-cell max-Z values ---
	var zValues []float64
	for r := range grid {
		for c := range grid[r] {
			if grid[r][c].hasData {
				zValues = append(zValues, grid[r][c].maxZ)
			}
		}
	}
	if len(zValues) == 0 {
		return nil, fmt.Errorf("no occupied cells in height map")
	}
	sort.Float64s(zValues)
	threshIdx := int(float64(len(zValues)-1) * opts.DividerZPercentile)
	zThreshold := zValues[threshIdx]

	// --- Build divider mask: cells at or above the Z threshold ---
	isDivider := make([][]bool, rows)
	for r := range isDivider {
		isDivider[r] = make([]bool, cols)
		for c := range isDivider[r] {
			if grid[r][c].hasData && grid[r][c].maxZ >= zThreshold {
				isDivider[r][c] = true
			}
		}
	}

	// --- Dilate divider mask to close gaps from Poisson reconstruction smoothing ---
	if opts.DividerDilation > 0 {
		isDivider = dilateMask(isDivider, rows, cols, opts.DividerDilation)
	}

	// --- BFS connected-component labeling on open (non-divider, occupied) cells ---
	labels := make([][]int, rows)
	for r := range labels {
		labels[r] = make([]int, cols)
		for c := range labels[r] {
			labels[r][c] = -1
		}
	}

	isOpen := func(r, c int) bool {
		return r >= 0 && r < rows &&
			c >= 0 && c < cols &&
			grid[r][c].hasData &&
			!isDivider[r][c]
	}

	type coord struct{ r, c int }
	dirs := []coord{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}

	nextLabel := 0
	componentCells := map[int][]coord{}

	for startR := 0; startR < rows; startR++ {
		for startC := 0; startC < cols; startC++ {
			if !isOpen(startR, startC) || labels[startR][startC] >= 0 {
				continue
			}
			label := nextLabel
			nextLabel++
			queue := []coord{{startR, startC}}
			labels[startR][startC] = label
			for len(queue) > 0 {
				cur := queue[0]
				queue = queue[1:]
				componentCells[label] = append(componentCells[label], cur)
				for _, d := range dirs {
					nr, nc := cur.r+d.r, cur.c+d.c
					if isOpen(nr, nc) && labels[nr][nc] < 0 {
						labels[nr][nc] = label
						queue = append(queue, coord{nr, nc})
					}
				}
			}
		}
	}

	// --- Filter small components and compute XY bounds ---
	type zoneInfo struct {
		label            int
		minX, maxX       float64
		minY, maxY       float64
	}
	var zoneInfos []zoneInfo

	cellArea := opts.CellSizeMM * opts.CellSizeMM
	for label, cells := range componentCells {
		area := float64(len(cells)) * cellArea
		if area < opts.MinBinAreaMM2 {
			continue
		}

		minBinX, minBinY := math.MaxFloat64, math.MaxFloat64
		maxBinX, maxBinY := -math.MaxFloat64, -math.MaxFloat64
		for _, cc := range cells {
			x := minX + (float64(cc.c)+0.5)*opts.CellSizeMM
			y := minY + (float64(cc.r)+0.5)*opts.CellSizeMM
			if x < minBinX {
				minBinX = x
			}
			if x > maxBinX {
				maxBinX = x
			}
			if y < minBinY {
				minBinY = y
			}
			if y > maxBinY {
				maxBinY = y
			}
		}
		zoneInfos = append(zoneInfos, zoneInfo{
			label: label,
			minX:  minBinX, maxX: maxBinX,
			minY: minBinY, maxY: maxBinY,
		})
	}

	if len(zoneInfos) == 0 {
		return nil, fmt.Errorf(
			"no zones detected (z_threshold=%.1f, min_area=%.0f mm²); "+
				"try adjusting --divider-z-percentile, --divider-dilation, or --min-bin-area",
			zThreshold, opts.MinBinAreaMM2,
		)
	}

	// Sort left-to-right (X then Y) for stable numbering.
	sort.Slice(zoneInfos, func(i, j int) bool {
		ci := (zoneInfos[i].minX + zoneInfos[i].maxX) / 2
		cj := (zoneInfos[j].minX + zoneInfos[j].maxX) / 2
		if math.Abs(ci-cj) > opts.CellSizeMM {
			return ci < cj
		}
		return (zoneInfos[i].minY+zoneInfos[i].maxY)/2 < (zoneInfos[j].minY+zoneInfos[j].maxY)/2
	})

	// Map BFS label → sorted zone index for triangle assignment.
	labelToIdx := make(map[int]int, len(zoneInfos))
	for i, zi := range zoneInfos {
		labelToIdx[zi.label] = i
	}

	// --- Assign each triangle to a zone by the cell that contains its centroid ---
	zoneFaces := make([][][3]r3.Vector, len(zoneInfos))
	for _, f := range faces {
		cx := (f.verts[0].X + f.verts[1].X + f.verts[2].X) / 3
		cy := (f.verts[0].Y + f.verts[1].Y + f.verts[2].Y) / 3
		c := int((cx - minX) / opts.CellSizeMM)
		r := int((cy - minY) / opts.CellSizeMM)
		if c < 0 || c >= cols || r < 0 || r >= rows {
			continue
		}
		label := labels[r][c]
		if label < 0 {
			continue
		}
		idx, ok := labelToIdx[label]
		if !ok {
			continue
		}
		zoneFaces[idx] = append(zoneFaces[idx], f.verts)
	}

	// --- Build Zone structs with their sub-meshes ---
	zones := make([]Zone, len(zoneInfos))
	for i, zi := range zoneInfos {
		zones[i] = Zone{
			ID:   i,
			MinX: zi.minX,
			MaxX: zi.maxX,
			MinY: zi.minY,
			MaxY: zi.maxY,
			Mesh: buildZoneMesh(zoneFaces[i]),
		}
	}

	return zones, nil
}

// dilateMask expands every true cell in mask by radius cells in all directions.
// Used to close gaps in divider ridges left by Poisson reconstruction smoothing.
func dilateMask(mask [][]bool, rows, cols, radius int) [][]bool {
	result := make([][]bool, rows)
	for r := range result {
		result[r] = make([]bool, cols)
	}
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			if !mask[r][c] {
				continue
			}
			for dr := -radius; dr <= radius; dr++ {
				for dc := -radius; dc <= radius; dc++ {
					nr, nc := r+dr, c+dc
					if nr >= 0 && nr < rows && nc >= 0 && nc < cols {
						result[nr][nc] = true
					}
				}
			}
		}
	}
	return result
}

// buildZoneMesh creates a ZoneMesh from a slice of triangle vertex triplets.
// Duplicate vertices are deduplicated so that the mesh is compact.
func buildZoneMesh(faces [][3]r3.Vector) ZoneMesh {
	type key [3]float64
	vertIdx := make(map[key]int)
	var vertices [][3]float64
	var faceIndices [][3]int

	getOrAdd := func(v r3.Vector) int {
		k := key{v.X, v.Y, v.Z}
		if idx, ok := vertIdx[k]; ok {
			return idx
		}
		idx := len(vertices)
		vertIdx[k] = idx
		vertices = append(vertices, [3]float64{v.X, v.Y, v.Z})
		return idx
	}

	for _, f := range faces {
		i0 := getOrAdd(f[0])
		i1 := getOrAdd(f[1])
		i2 := getOrAdd(f[2])
		faceIndices = append(faceIndices, [3]int{i0, i1, i2})
	}

	return ZoneMesh{
		Vertices: vertices,
		Faces:    faceIndices,
	}
}
