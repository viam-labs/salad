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

// BinRegion represents the XY footprint of a single ingredient bin.
//
// The footprint is derived from divider geometry and is stable regardless of
// ingredient fill level. For automated grabbing, Z surface height should be
// queried from the live mesh at scoop-planning time rather than stored here.
type BinRegion struct {
	ID     int       `json:"id"`
	MinX   float64   `json:"min_x"`
	MaxX   float64   `json:"max_x"`
	MinY   float64   `json:"min_y"`
	MaxY   float64   `json:"max_y"`
	Center r3.Vector `json:"center"` // XY centroid; Z is the mean surface height of the bin
}

// Result holds the output of a segmentation run.
type Result struct {
	SourceMesh  string      `json:"source_mesh"`
	GeneratedAt time.Time   `json:"generated_at"`
	Bins        []BinRegion `json:"bins"`
}

// Options controls segmentation behavior.
type Options struct {
	// CellSizeMM is the resolution of the 2D height-map grid in millimeters.
	// Smaller values detect finer bin boundaries but increase memory and runtime.
	// Default: 5.0
	CellSizeMM float64

	// DividerZPercentile is the percentile of occupied-cell max-Z values used
	// to separate divider peaks (high Z) from bin content surfaces (low Z).
	// Must be in [0, 1]. Default: 0.7 — cells above the 70th percentile Z are
	// treated as dividers or walls; cells below are treated as bin content.
	// Increase toward 1.0 if bins are being over-segmented (threshold too low).
	// Decrease toward 0.5 if bins are being merged (threshold too high).
	DividerZPercentile float64

	// MinBinAreaMM2 is the minimum bin footprint area in mm².
	// Connected components smaller than this are discarded as noise.
	// Default: 5000.0 (roughly 50 mm × 100 mm)
	MinBinAreaMM2 float64
}

// DefaultOptions returns sensible defaults for fridge segmentation.
func DefaultOptions() Options {
	return Options{
		CellSizeMM:         5.0,
		DividerZPercentile: 0.7,
		MinBinAreaMM2:      5000.0,
	}
}

// SegmentFridgeBins loads a PLY mesh and segments it into individual bin XY footprints.
//
// The algorithm:
//  1. Projects mesh vertices onto a 2D grid, recording max Z per cell (height map).
//  2. Derives a Z threshold from the DividerZPercentile of occupied-cell heights.
//  3. Marks cells below the threshold as "open" (bin content) and above as barriers
//     (dividers or walls).
//  4. Runs BFS connected-component labeling on open cells (4-connectivity).
//  5. Filters out components smaller than MinBinAreaMM2.
//  6. Returns one BinRegion per remaining component, sorted by position.
func SegmentFridgeBins(meshPath string, opts Options) (*Result, error) {
	mesh, err := spatialmath.NewMeshFromPLYFile(meshPath)
	if err != nil {
		return nil, fmt.Errorf("loading mesh %q: %w", meshPath, err)
	}

	pts := mesh.ToPoints(1)
	if len(pts) == 0 {
		return nil, fmt.Errorf("mesh %q has no points", meshPath)
	}

	bins, err := segmentPoints(pts, opts)
	if err != nil {
		return nil, err
	}

	return &Result{
		SourceMesh:  meshPath,
		GeneratedAt: time.Now(),
		Bins:        bins,
	}, nil
}

// SaveResult writes a segmentation result to a JSON file.
func SaveResult(result *Result, path string) error {
	data, err := json.MarshalIndent(result, "", "  ")
	if err != nil {
		return fmt.Errorf("marshaling result: %w", err)
	}
	if err := os.WriteFile(path, data, 0o644); err != nil {
		return fmt.Errorf("writing %q: %w", path, err)
	}
	return nil
}

// segmentPoints performs height-map connected-component segmentation on a point set.
func segmentPoints(pts []r3.Vector, opts Options) ([]BinRegion, error) {
	if opts.CellSizeMM <= 0 {
		return nil, fmt.Errorf("CellSizeMM must be > 0, got %v", opts.CellSizeMM)
	}
	if opts.DividerZPercentile < 0 || opts.DividerZPercentile > 1 {
		return nil, fmt.Errorf("DividerZPercentile must be in [0,1], got %v", opts.DividerZPercentile)
	}

	// --- Build 2D height map ---
	minX, minY := math.MaxFloat64, math.MaxFloat64
	maxX, maxY := -math.MaxFloat64, -math.MaxFloat64
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

	cols := int((maxX-minX)/opts.CellSizeMM) + 1
	rows := int((maxY-minY)/opts.CellSizeMM) + 1

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

	for _, p := range pts {
		c := int((p.X - minX) / opts.CellSizeMM)
		r := int((p.Y - minY) / opts.CellSizeMM)
		if c < 0 || c >= cols || r < 0 || r >= rows {
			continue
		}
		if p.Z > grid[r][c].maxZ {
			grid[r][c].maxZ = p.Z
		}
		grid[r][c].hasData = true
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

	// --- BFS connected-component labeling on open cells ---
	// A cell is open if it has data and its max Z is below the threshold.
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
			grid[r][c].maxZ < zThreshold
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

	// --- Build BinRegions, filtering out components smaller than MinBinAreaMM2 ---
	cellArea := opts.CellSizeMM * opts.CellSizeMM
	var bins []BinRegion

	for label, cells := range componentCells {
		area := float64(len(cells)) * cellArea
		if area < opts.MinBinAreaMM2 {
			continue
		}

		minBinX, minBinY := math.MaxFloat64, math.MaxFloat64
		maxBinX, maxBinY := -math.MaxFloat64, -math.MaxFloat64
		sumZ := 0.0

		for _, cc := range cells {
			// Cell center in world coordinates
			x := minX + (float64(cc.c)+0.5)*opts.CellSizeMM
			y := minY + (float64(cc.r)+0.5)*opts.CellSizeMM
			z := grid[cc.r][cc.c].maxZ
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
			sumZ += z
		}

		bins = append(bins, BinRegion{
			ID:   label, // re-assigned after sorting
			MinX: minBinX,
			MaxX: maxBinX,
			MinY: minBinY,
			MaxY: maxBinY,
			Center: r3.Vector{
				X: (minBinX + maxBinX) / 2,
				Y: (minBinY + maxBinY) / 2,
				Z: sumZ / float64(len(cells)),
			},
		})
	}

	if len(bins) == 0 {
		return nil, fmt.Errorf(
			"no bin regions detected (z_threshold=%.1f, min_area=%.0f mm²); "+
				"try adjusting --divider-z-percentile or --min-bin-area",
			zThreshold, opts.MinBinAreaMM2,
		)
	}

	// Sort by X then Y for stable, left-to-right ordering.
	sort.Slice(bins, func(i, j int) bool {
		if math.Abs(bins[i].Center.X-bins[j].Center.X) > opts.CellSizeMM {
			return bins[i].Center.X < bins[j].Center.X
		}
		return bins[i].Center.Y < bins[j].Center.Y
	})
	for i := range bins {
		bins[i].ID = i
	}

	return bins, nil
}
