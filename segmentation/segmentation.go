package segmentation

import (
	"bufio"
	"encoding/json"
	"fmt"
	"math"
	"os"
	"sort"
	"time"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"
	"gonum.org/v1/gonum/mat"
)

type ZoneMesh struct {
	Vertices [][3]float64 `json:"vertices"`
	Faces    [][3]int     `json:"faces"`
}

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

type Zone struct {
	ID    int      `json:"id"`
	MinX  float64  `json:"min_x"`
	MaxX  float64  `json:"max_x"`
	MinY  float64  `json:"min_y"`
	MaxY  float64  `json:"max_y"`
	Mesh  ZoneMesh `json:"mesh"`
	Plane Plane    `json:"plane"`
}

// Plane represents a best-fit infinite plane in 3D as a point + unit normal.
// Normal is forced to point "up" (Normal[2] > 0) by the fitter so callers can
// use it directly as the local +Z of a bin-floor frame.
type Plane struct {
	Point  [3]float64 `json:"point"`
	Normal [3]float64 `json:"normal"`
}

// IsZero reports whether the plane is the zero value (no fit performed).
func (p Plane) IsZero() bool {
	return p.Normal == [3]float64{}
}

// TiltDeg returns the angle in degrees between the plane normal and world +Z.
// 0° means perfectly horizontal; 90° means vertical (and never expected for a
// bin floor).
func (p Plane) TiltDeg() float64 {
	nz := p.Normal[2]
	if nz > 1 {
		nz = 1
	}
	if nz < -1 {
		nz = -1
	}
	return math.Acos(nz) * 180 / math.Pi
}

// ZAt returns the plane's Z coordinate at world (x, y). For a near-vertical or
// zero-value plane the result is degenerate, so we fall back to Point[2].
func (p Plane) ZAt(x, y float64) float64 {
	nz := p.Normal[2]
	if math.Abs(nz) < 1e-6 {
		return p.Point[2]
	}
	return p.Point[2] - (p.Normal[0]*(x-p.Point[0])+p.Normal[1]*(y-p.Point[1]))/nz
}

// SignedDistance returns the signed distance from world point (x, y, z) to the
// plane along the plane normal. Positive means "above" the plane (in the
// direction the normal points).
func (p Plane) SignedDistance(x, y, z float64) float64 {
	return p.Normal[0]*(x-p.Point[0]) + p.Normal[1]*(y-p.Point[1]) + p.Normal[2]*(z-p.Point[2])
}

// minZPercentile is the vertex Z percentile used as the bin floor.
// Using a percentile (rather than the absolute min) discards mesh
// reconstruction outliers (stray vertices that spike below the true
// floor) which would otherwise pull descent depth too low.
const minZPercentile = 0.05

func (z *Zone) MinZ() float64 {
	if len(z.Mesh.Vertices) == 0 {
		return math.Inf(1)
	}
	zs := make([]float64, len(z.Mesh.Vertices))
	for i, v := range z.Mesh.Vertices {
		zs[i] = v[2]
	}
	sort.Float64s(zs)
	idx := int(float64(len(zs)) * minZPercentile)
	if idx >= len(zs) {
		idx = len(zs) - 1
	}
	return zs[idx]
}

// Centroid returns the XY centroid of the zone computed from its mesh vertices.
func (z *Zone) Centroid() (x, y float64, err error) {
	if len(z.Mesh.Vertices) == 0 {
		return 0, 0, fmt.Errorf("zone %d has no mesh vertices", z.ID)
	}
	var sumX, sumY float64
	for _, v := range z.Mesh.Vertices {
		sumX += v[0]
		sumY += v[1]
	}
	n := float64(len(z.Mesh.Vertices))
	return sumX / n, sumY / n, nil
}

func (zr *ZonesResult) ZoneByID(id int) (*Zone, bool) {
	for i := range zr.Zones {
		if zr.Zones[i].ID == id {
			return &zr.Zones[i], true
		}
	}
	return nil, false
}

type ZonesResult struct {
	SourceMesh  string    `json:"source_mesh"`
	GeneratedAt time.Time `json:"generated_at"`
	Zones       []Zone    `json:"zones"`
	ZMean       float64   `json:"z_mean"`
}

type SegmentStats struct {
	TriangleCount         int
	GridRows, GridCols    int
	OccupiedCells         int
	ZMin, ZMax            float64
	ZThreshold            float64
	BarrierCellsRaw       int
	BarrierCellsDilated   int
	ComponentsTotal       int
	ComponentsAfterFilter int
}

type Options struct {
	CellSizeMM         float64
	DividerZPercentile float64
	DividerGradientMM  float64
	DividerDilation    int
	MinZoneAreaMM2     float64
	MaxZoneAreaMM2     float64

	// FloorBandMM is the vertical slab thickness (mm) above each zone's local
	// minZ from which we sample candidate floor vertices. Should be generous
	// enough to capture the whole tilted floor: for a bin of width W tilted by
	// θ, the floor spans up to ~W·sin(θ) in world Z.
	FloorBandMM float64
	// FloorMaxTiltDeg is the max angle (in degrees) between a triangle's
	// normal and ±ẑ for it to count as a floor candidate. Larger values are
	// more permissive (good for tilted fridges) but risk including bin walls
	// whose normals are not perfectly horizontal.
	FloorMaxTiltDeg float64
	// FloorMinPoints is the minimum number of candidate vertices required to
	// run a real PCA fit. Below this we fall back to a horizontal plane at
	// the zone's MinZ percentile.
	FloorMinPoints int
}

func DefaultOptions() Options {
	return Options{
		CellSizeMM:         5.0,
		DividerZPercentile: 0.80,
		DividerGradientMM:  10.0,
		DividerDilation:    0,
		MinZoneAreaMM2:     5000.0,
		MaxZoneAreaMM2:     100000.0,
		FloorBandMM:        30.0,
		FloorMaxTiltDeg:    25.0,
		FloorMinPoints:     30,
	}
}

func SegmentFridgeBins(meshPath string, opts Options) (*ZonesResult, SegmentStats, error) {
	mesh, err := spatialmath.NewMeshFromPLYFile(meshPath)
	if err != nil {
		return nil, SegmentStats{}, fmt.Errorf("loading mesh %q: %w", meshPath, err)
	}

	triangles := mesh.Triangles()
	if len(triangles) == 0 {
		return nil, SegmentStats{}, fmt.Errorf("mesh %q has no triangles", meshPath)
	}

	zones, stats, err := segmentTriangles(triangles, opts)
	if err != nil {
		return nil, stats, err
	}

	return &ZonesResult{
		SourceMesh:  meshPath,
		GeneratedAt: time.Now(),
		Zones:       zones,
		ZMean:       stats.ZThreshold,
	}, stats, nil
}

func SaveZones(result *ZonesResult, path string) error {
	f, err := os.Create(path)
	if err != nil {
		return fmt.Errorf("creating %q: %w", path, err)
	}
	defer f.Close()

	bw := bufio.NewWriterSize(f, 1<<20)
	enc := json.NewEncoder(bw)
	if err := enc.Encode(result); err != nil {
		return fmt.Errorf("encoding zones: %w", err)
	}
	return bw.Flush()
}

// LoadZones reads a [ZonesResult] from JSON written by [SaveZones].
func LoadZones(path string) (*ZonesResult, error) {
	f, err := os.Open(path)
	if err != nil {
		return nil, fmt.Errorf("opening %q: %w", path, err)
	}
	defer f.Close()
	dec := json.NewDecoder(f)
	var z ZonesResult
	if err := dec.Decode(&z); err != nil {
		return nil, fmt.Errorf("decoding %q: %w", path, err)
	}
	return &z, nil
}

func segmentTriangles(triangles []*spatialmath.Triangle, opts Options) ([]Zone, SegmentStats, error) {
	var stats SegmentStats
	if opts.CellSizeMM <= 0 {
		return nil, stats, fmt.Errorf("CellSizeMM must be > 0, got %v", opts.CellSizeMM)
	}
	if opts.DividerZPercentile < 0 || opts.DividerZPercentile > 1 {
		return nil, stats, fmt.Errorf("DividerZPercentile must be in [0,1], got %v", opts.DividerZPercentile)
	}

	type face struct{ verts [3]r3.Vector }
	faces := make([]face, len(triangles))
	stats.TriangleCount = len(triangles)

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
	stats.GridCols = cols
	stats.GridRows = rows

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

	for _, f := range faces {
		for _, p := range f.verts {
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
	}

	var zValues []float64
	for r := range grid {
		for c := range grid[r] {
			if grid[r][c].hasData {
				zValues = append(zValues, grid[r][c].maxZ)
			}
		}
	}
	if len(zValues) == 0 {
		return nil, stats, fmt.Errorf("no occupied cells in height map")
	}
	sort.Float64s(zValues)
	stats.OccupiedCells = len(zValues)
	stats.ZMin = zValues[0]
	stats.ZMax = zValues[len(zValues)-1]

	threshIdx := int(float64(len(zValues)-1) * opts.DividerZPercentile)
	zThreshold := zValues[threshIdx]
	stats.ZThreshold = zThreshold

	dirs4 := [4][2]int{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}

	isDivider := make([][]bool, rows)
	for r := range isDivider {
		isDivider[r] = make([]bool, cols)
	}
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			if !grid[r][c].hasData {
				continue
			}
			if grid[r][c].maxZ >= zThreshold {
				isDivider[r][c] = true
				continue
			}
			if opts.DividerGradientMM > 0 {
				minNeighZ := math.MaxFloat64
				for _, d := range dirs4 {
					nr, nc := r+d[0], c+d[1]
					if nr < 0 || nr >= rows || nc < 0 || nc >= cols {
						continue
					}
					if grid[nr][nc].hasData && grid[nr][nc].maxZ < minNeighZ {
						minNeighZ = grid[nr][nc].maxZ
					}
				}
				if minNeighZ < math.MaxFloat64 && grid[r][c].maxZ-minNeighZ >= opts.DividerGradientMM {
					isDivider[r][c] = true
				}
			}
		}
	}

	for r := range isDivider {
		for c := range isDivider[r] {
			if isDivider[r][c] {
				stats.BarrierCellsRaw++
			}
		}
	}

	if opts.DividerDilation > 0 {
		isDivider = dilateMask(isDivider, rows, cols, opts.DividerDilation)
	}
	for r := range isDivider {
		for c := range isDivider[r] {
			if isDivider[r][c] {
				stats.BarrierCellsDilated++
			}
		}
	}

	type coord struct{ r, c int }
	dirs := [4]coord{{-1, 0}, {1, 0}, {0, -1}, {0, 1}}

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
	stats.ComponentsTotal = len(componentCells)

	type zoneInfo struct {
		label      int
		minX, maxX float64
		minY, maxY float64
	}
	var zoneInfos []zoneInfo

	cellArea := opts.CellSizeMM * opts.CellSizeMM
	for label, cells := range componentCells {
		area := float64(len(cells)) * cellArea
		if area < opts.MinZoneAreaMM2 {
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
		if opts.MaxZoneAreaMM2 > 0 {
			if (maxBinX-minBinX)*(maxBinY-minBinY) > opts.MaxZoneAreaMM2 {
				continue
			}
		}
		zoneInfos = append(zoneInfos, zoneInfo{
			label: label,
			minX:  minBinX, maxX: maxBinX,
			minY: minBinY, maxY: maxBinY,
		})
	}
	stats.ComponentsAfterFilter = len(zoneInfos)

	if len(zoneInfos) == 0 {
		return nil, stats, fmt.Errorf(
			"no zones detected (z_threshold=%.1fmm, gradient=%.1fmm, min_area=%.0fmm², max_area=%.0fmm²)\n"+
				"  Z range: [%.1f, %.1f]mm  barrier cells: %d/%d (%.0f%%)\n"+
				"  try: lower --divider-z-percentile, lower --divider-gradient, or adjust area bounds",
			zThreshold, opts.DividerGradientMM, opts.MinZoneAreaMM2, opts.MaxZoneAreaMM2,
			stats.ZMin, stats.ZMax,
			stats.BarrierCellsDilated, stats.OccupiedCells,
			100*float64(stats.BarrierCellsDilated)/float64(stats.OccupiedCells),
		)
	}

	var sumX, sumY float64
	for _, zi := range zoneInfos {
		sumX += (zi.minX + zi.maxX) / 2
		sumY += (zi.minY + zi.maxY) / 2
	}
	meanX := sumX / float64(len(zoneInfos))
	meanY := sumY / float64(len(zoneInfos))
	var varX, varY float64
	for _, zi := range zoneInfos {
		dx := (zi.minX+zi.maxX)/2 - meanX
		dy := (zi.minY+zi.maxY)/2 - meanY
		varX += dx * dx
		varY += dy * dy
	}
	sortByY := varY > varX
	sort.Slice(zoneInfos, func(i, j int) bool {
		if sortByY {
			ci := (zoneInfos[i].minY + zoneInfos[i].maxY) / 2
			cj := (zoneInfos[j].minY + zoneInfos[j].maxY) / 2
			if math.Abs(ci-cj) > opts.CellSizeMM {
				return ci < cj
			}
			return (zoneInfos[i].minX+zoneInfos[i].maxX)/2 < (zoneInfos[j].minX+zoneInfos[j].maxX)/2
		}
		ci := (zoneInfos[i].minX + zoneInfos[i].maxX) / 2
		cj := (zoneInfos[j].minX + zoneInfos[j].maxX) / 2
		if math.Abs(ci-cj) > opts.CellSizeMM {
			return ci < cj
		}
		return (zoneInfos[i].minY+zoneInfos[i].maxY)/2 < (zoneInfos[j].minY+zoneInfos[j].maxY)/2
	})

	labelToIdx := make(map[int]int, len(zoneInfos))
	for i, zi := range zoneInfos {
		labelToIdx[zi.label] = i
	}

	isHardBarrier := func(r, c int) bool {
		return !grid[r][c].hasData || grid[r][c].maxZ >= zThreshold
	}
	expandQueue := []coord{}
	for startR := 0; startR < rows; startR++ {
		for startC := 0; startC < cols; startC++ {
			if labels[startR][startC] >= 0 {
				expandQueue = append(expandQueue, coord{startR, startC})
			}
		}
	}
	for len(expandQueue) > 0 {
		cur := expandQueue[0]
		expandQueue = expandQueue[1:]
		for _, d := range dirs {
			nr, nc := cur.r+d.r, cur.c+d.c
			if nr < 0 || nr >= rows || nc < 0 || nc >= cols {
				continue
			}
			if labels[nr][nc] >= 0 || isHardBarrier(nr, nc) {
				continue
			}
			labels[nr][nc] = labels[cur.r][cur.c]
			expandQueue = append(expandQueue, coord{nr, nc})
		}
	}

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

	expandedBounds := make([][4]float64, len(zoneInfos))
	for i := range expandedBounds {
		expandedBounds[i] = [4]float64{math.MaxFloat64, -math.MaxFloat64, math.MaxFloat64, -math.MaxFloat64}
	}
	for r := 0; r < rows; r++ {
		for c := 0; c < cols; c++ {
			label := labels[r][c]
			if label < 0 {
				continue
			}
			idx, ok := labelToIdx[label]
			if !ok {
				continue
			}
			x := minX + (float64(c)+0.5)*opts.CellSizeMM
			y := minY + (float64(r)+0.5)*opts.CellSizeMM
			b := &expandedBounds[idx]
			if x < b[0] {
				b[0] = x
			}
			if x > b[1] {
				b[1] = x
			}
			if y < b[2] {
				b[2] = y
			}
			if y > b[3] {
				b[3] = y
			}
		}
	}

	zones := make([]Zone, len(zoneInfos))
	for i, zi := range zoneInfos {
		b := expandedBounds[i]
		minBX, maxBX, minBY, maxBY := b[0], b[1], b[2], b[3]
		if minBX > maxBX {
			minBX, maxBX = zi.minX, zi.maxX
			minBY, maxBY = zi.minY, zi.maxY
		}
		zones[i] = Zone{
			ID:    i,
			MinX:  minBX,
			MaxX:  maxBX,
			MinY:  minBY,
			MaxY:  maxBY,
			Mesh:  buildZoneMesh(zoneFaces[i]),
			Plane: fitFloorPlane(zoneFaces[i], opts),
		}
	}

	return zones, stats, nil
}

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
			rLo := max(0, r-radius)
			rHi := min(rows-1, r+radius)
			cLo := max(0, c-radius)
			cHi := min(cols-1, c+radius)
			for nr := rLo; nr <= rHi; nr++ {
				for nc := cLo; nc <= cHi; nc++ {
					result[nr][nc] = true
				}
			}
		}
	}
	return result
}

// fitFloorPlane estimates the bin-floor plane for a zone's triangle soup.
//
// Strategy: filter to "near-floor, upward-facing" triangles, then PCA-fit a
// plane through their vertices. If we don't have enough survivors we fall
// back to a horizontal plane at the zone's MinZ percentile.
//
// We accept a triangle if both:
//   - any vertex lies within FloorBandMM of the zone's MinZ percentile
//     (kills high horizontal surfaces like item tops or divider caps), and
//   - |normal.Z| ≥ cos(FloorMaxTiltDeg) (kills walls).
//
// We use abs(normal.Z) so input-mesh orientation (which Poisson does not
// strictly guarantee) does not matter; the final plane normal is forced
// upward at the end.
func fitFloorPlane(faces [][3]r3.Vector, opts Options) Plane {
	if len(faces) == 0 {
		return Plane{Normal: [3]float64{0, 0, 1}}
	}

	zs := make([]float64, 0, len(faces)*3)
	var cx, cy float64
	for _, f := range faces {
		zs = append(zs, f[0].Z, f[1].Z, f[2].Z)
		cx += (f[0].X + f[1].X + f[2].X) / 3
		cy += (f[0].Y + f[1].Y + f[2].Y) / 3
	}
	cx /= float64(len(faces))
	cy /= float64(len(faces))
	sort.Float64s(zs)
	idx := int(float64(len(zs)) * minZPercentile)
	if idx >= len(zs) {
		idx = len(zs) - 1
	}
	zoneMinZ := zs[idx]

	fallback := Plane{
		Point:  [3]float64{cx, cy, zoneMinZ},
		Normal: [3]float64{0, 0, 1},
	}

	zCutoff := zoneMinZ + opts.FloorBandMM
	cosTilt := math.Cos(opts.FloorMaxTiltDeg * math.Pi / 180)
	pts := make([]r3.Vector, 0, len(faces)*3)
	for _, f := range faces {
		e1 := f[1].Sub(f[0])
		e2 := f[2].Sub(f[0])
		n := e1.Cross(e2)
		nLen := n.Norm()
		if nLen < 1e-9 {
			continue
		}
		if math.Abs(n.Z/nLen) < cosTilt {
			continue
		}
		for _, v := range f {
			if v.Z <= zCutoff {
				pts = append(pts, v)
			}
		}
	}

	if len(pts) < opts.FloorMinPoints {
		return fallback
	}

	plane, ok := fitPlanePCA(pts)
	if !ok {
		return fallback
	}
	return plane
}

// fitPlanePCA fits a least-squares plane through points by taking the
// eigenvector of the smallest eigenvalue of their 3×3 covariance matrix as
// the plane normal, and the centroid as a point on the plane. The returned
// normal is unit-length and forced to satisfy Normal[2] > 0.
func fitPlanePCA(points []r3.Vector) (Plane, bool) {
	if len(points) < 3 {
		return Plane{}, false
	}
	n := float64(len(points))
	var cx, cy, cz float64
	for _, p := range points {
		cx += p.X
		cy += p.Y
		cz += p.Z
	}
	cx /= n
	cy /= n
	cz /= n

	var xx, xy, xz, yy, yz, zz float64
	for _, p := range points {
		dx := p.X - cx
		dy := p.Y - cy
		dz := p.Z - cz
		xx += dx * dx
		xy += dx * dy
		xz += dx * dz
		yy += dy * dy
		yz += dy * dz
		zz += dz * dz
	}

	cov := mat.NewSymDense(3, []float64{
		xx, xy, xz,
		0, yy, yz,
		0, 0, zz,
	})

	var eig mat.EigenSym
	if !eig.Factorize(cov, true) {
		return Plane{}, false
	}
	var vecs mat.Dense
	eig.VectorsTo(&vecs)
	nx, ny, nz := vecs.At(0, 0), vecs.At(1, 0), vecs.At(2, 0)
	nLen := math.Sqrt(nx*nx + ny*ny + nz*nz)
	if nLen < 1e-9 {
		return Plane{}, false
	}
	nx, ny, nz = nx/nLen, ny/nLen, nz/nLen
	if nz < 0 {
		nx, ny, nz = -nx, -ny, -nz
	}
	return Plane{
		Point:  [3]float64{cx, cy, cz},
		Normal: [3]float64{nx, ny, nz},
	}, true
}

// PlaneRectMesh builds a 2-triangle ZoneMesh for the quad whose XY corners
// are (minX|maxX) × (minY|maxY), with each corner's Z lying on the plane.
// Used to visualize a zone's bin-floor plane in the motion-tools visualizer.
func PlaneRectMesh(p Plane, minX, maxX, minY, maxY float64) ZoneMesh {
	v00 := [3]float64{minX, minY, p.ZAt(minX, minY)}
	v10 := [3]float64{maxX, minY, p.ZAt(maxX, minY)}
	v11 := [3]float64{maxX, maxY, p.ZAt(maxX, maxY)}
	v01 := [3]float64{minX, maxY, p.ZAt(minX, maxY)}
	return ZoneMesh{
		Vertices: [][3]float64{v00, v10, v11, v01},
		Faces:    [][3]int{{0, 1, 2}, {0, 2, 3}},
	}
}

func buildZoneMesh(faces [][3]r3.Vector) ZoneMesh {
	type key [3]float64
	vertIdx := make(map[key]int, len(faces)*2)
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
		faceIndices = append(faceIndices, [3]int{
			getOrAdd(f[0]),
			getOrAdd(f[1]),
			getOrAdd(f[2]),
		})
	}

	return ZoneMesh{Vertices: vertices, Faces: faceIndices}
}
