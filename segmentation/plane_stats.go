package segmentation

import (
	"math"
	"sort"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
)

// ZoneHeightMapGridSize is the number of rows and columns used when
// subdividing a zone's XY rectangle for per-cell plane-distance stats.
const ZoneHeightMapGridSize = 32

// ZoneHeightMap holds the median signed distance to the zone plane for each
// cell of a fixed ZoneHeightMapGridSize × ZoneHeightMapGridSize grid over the
// zone's axis-aligned XY bounds. Indices are [row][col] where row increases
// with Y and col increases with X. MedianSignedDistanceMM is math.NaN for
// cells with no in-bounds points.
type ZoneHeightMap struct {
	MinX, MaxX, MinY, MaxY float64
	MedianSignedDistanceMM [ZoneHeightMapGridSize][ZoneHeightMapGridSize]float64
	PointCount             [ZoneHeightMapGridSize][ZoneHeightMapGridSize]int
}

// NewZoneHeightMap returns a height map with XY bounds taken from zone. Median
// cells are filled by Populate.
func NewZoneHeightMap(zone *Zone) ZoneHeightMap {
	return ZoneHeightMap{
		MinX: zone.MinX,
		MaxX: zone.MaxX,
		MinY: zone.MinY,
		MaxY: zone.MaxY,
	}
}

// Populate computes per-cell median signed distances from accumulated samples.
func (hm *ZoneHeightMap) Populate(cellDistances [ZoneHeightMapGridSize][ZoneHeightMapGridSize][]float64) {
	for r := 0; r < ZoneHeightMapGridSize; r++ {
		for c := 0; c < ZoneHeightMapGridSize; c++ {
			dists := cellDistances[r][c]
			hm.PointCount[r][c] = len(dists)
			if len(dists) == 0 {
				hm.MedianSignedDistanceMM[r][c] = math.NaN()
				continue
			}
			hm.MedianSignedDistanceMM[r][c] = medianFloat64(dists)
		}
	}
}

// CellXY maps world (x, y) to height-map indices [row][col]. Values outside the
// map's stored XY bounds are clamped into the nearest edge cell (callers that
// need an out-of-bounds check should test bounds before calling).
func (hm ZoneHeightMap) CellXY(x, y float64) (row, col int) {
	spanX := hm.MaxX - hm.MinX
	spanY := hm.MaxY - hm.MinY
	if spanX <= 0 {
		col = 0
	} else {
		col = int((x - hm.MinX) / spanX * ZoneHeightMapGridSize)
		if col < 0 {
			col = 0
		} else if col >= ZoneHeightMapGridSize {
			col = ZoneHeightMapGridSize - 1
		}
	}
	if spanY <= 0 {
		row = 0
	} else {
		row = int((y - hm.MinY) / spanY * ZoneHeightMapGridSize)
		if row < 0 {
			row = 0
		} else if row >= ZoneHeightMapGridSize {
			row = ZoneHeightMapGridSize - 1
		}
	}
	return row, col
}

// MedianSignedDistanceAt returns the median signed distance (mm) stored for the
// height-map cell that contains the point's (X, Y) projection. Z is ignored.
// Returns nil if the point lies outside the map's XY bounds or the cell has no
// points. Points on the zone boundary are included.
func (hm ZoneHeightMap) MedianSignedDistanceAt(p r3.Vector) *float64 {
	if p.X < hm.MinX || p.X > hm.MaxX || p.Y < hm.MinY || p.Y > hm.MaxY {
		return nil
	}
	row, col := hm.CellXY(p.X, p.Y)
	if hm.PointCount[row][col] == 0 {
		return nil
	}
	v := hm.MedianSignedDistanceMM[row][col]
	return &v
}

// PlaneFitStats summarizes how well a point cloud aligns with a zone's
// bin-floor plane.
//
// All distances are in millimeters. Signed distances follow the plane normal
// (Plane.SignedDistance): positive = above the plane (in the direction the
// normal points, which the fitter forces to be roughly +Z).
type PlaneFitStats struct {
	// PointsTotal is the size of the input point cloud before culling.
	PointsTotal int
	// PointsInBounds is the number of points whose (X, Y) lies inside the
	// zone's axis-aligned XY rectangle.
	PointsInBounds int
	// PointsInsideX is the number of points whose X alone is inside the
	// zone's X range, ignoring Y. Useful for debugging which axis is
	// culling points when PointsInBounds is unexpectedly 0.
	PointsInsideX int
	// PointsInsideY is the same for the Y range, ignoring X.
	PointsInsideY int
	// MeanAbsDistanceMM is the mean |signed distance| over PointsInBounds.
	MeanAbsDistanceMM float64
	// MedianAbsDistanceMM is the median |signed distance| over PointsInBounds.
	MedianAbsDistanceMM float64
	// MeanSignedDistanceMM is the mean signed distance over PointsInBounds.
	// A consistent sign indicates the cloud sits above or below the plane.
	MeanSignedDistanceMM float64
	// MedianSignedDistanceMM is the median signed distance over PointsInBounds.
	MedianSignedDistanceMM float64
	// MinSignedDistanceMM / MaxSignedDistanceMM bracket the signed distances
	// of the in-bounds points. Both 0 when PointsInBounds == 0.
	MinSignedDistanceMM float64
	MaxSignedDistanceMM float64
	// HeightMap is a ZoneHeightMapGridSize×ZoneHeightMapGridSize grid over the
	// zone XY rectangle with per-cell median signed distance to the plane.
	HeightMap ZoneHeightMap
}

// ZonePlaneFitStats culls pc to the zone's XY rectangle and reports the
// signed/absolute distance from the surviving points to the zone's bin-floor
// plane (Zone.Plane). The culled subset is returned as a new point cloud so
// callers can save it or visualize it.
//
// Points exactly on the rectangle boundary are kept.
//
// If logger is non-nil, the function logs per-zone diagnostic counts (how
// many points passed the X / Y / both bounds checks, plus the cloud and
// zone bounding boxes) at info level. Pass nil from callers that don't want
// the extra log volume.
func ZonePlaneFitStats(pc pointcloud.PointCloud, zone *Zone, logger logging.Logger) (PlaneFitStats, pointcloud.PointCloud) {
	stats := PlaneFitStats{
		PointsTotal:         pc.Size(),
		MinSignedDistanceMM: math.Inf(1),
		MaxSignedDistanceMM: math.Inf(-1),
		HeightMap:           NewZoneHeightMap(zone),
	}
	culled := pointcloud.NewBasicPointCloud(0)

	var sumAbs, sumSigned float64
	absDistances := make([]float64, 0, pc.Size())
	signedDistances := make([]float64, 0, pc.Size())
	var cellDistances [ZoneHeightMapGridSize][ZoneHeightMapGridSize][]float64
	pc.Iterate(0, 0, func(p r3.Vector, d pointcloud.Data) bool {
		insideX := p.X >= zone.MinX && p.X <= zone.MaxX
		insideY := p.Y >= zone.MinY && p.Y <= zone.MaxY
		if insideX {
			stats.PointsInsideX++
		}
		if insideY {
			stats.PointsInsideY++
		}
		if !insideX || !insideY {
			return true
		}
		sd := zone.Plane.SignedDistance(p.X, p.Y, p.Z)
		stats.PointsInBounds++
		sumSigned += sd
		abs := math.Abs(sd)
		sumAbs += abs
		absDistances = append(absDistances, abs)
		signedDistances = append(signedDistances, sd)
		row, col := stats.HeightMap.CellXY(p.X, p.Y)
		cellDistances[row][col] = append(cellDistances[row][col], sd)
		if sd < stats.MinSignedDistanceMM {
			stats.MinSignedDistanceMM = sd
		}
		if sd > stats.MaxSignedDistanceMM {
			stats.MaxSignedDistanceMM = sd
		}
		_ = culled.Set(p, d) //nolint:errcheck // Set on freshly-allocated BasicPointCloud cannot fail
		return true
	})

	if stats.PointsInBounds > 0 {
		stats.MeanAbsDistanceMM = sumAbs / float64(stats.PointsInBounds)
		stats.MedianAbsDistanceMM = medianFloat64(absDistances)
		stats.MeanSignedDistanceMM = sumSigned / float64(stats.PointsInBounds)
		stats.MedianSignedDistanceMM = medianFloat64(signedDistances)
	}
	stats.HeightMap.Populate(cellDistances)
	if stats.PointsInBounds == 0 {
		stats.MinSignedDistanceMM = 0
		stats.MaxSignedDistanceMM = 0
	}

	if logger != nil {
		md := pc.MetaData()
		logger.Debugf(
			"zone %d cull: total=%d  in_x=%d  in_y=%d  in_both=%d  | zone x=[%.2f,%.2f] y=[%.2f,%.2f] | cloud x=[%.2f,%.2f] y=[%.2f,%.2f] z=[%.2f,%.2f]",
			zone.ID, stats.PointsTotal, stats.PointsInsideX, stats.PointsInsideY, stats.PointsInBounds,
			zone.MinX, zone.MaxX, zone.MinY, zone.MaxY,
			md.MinX, md.MaxX, md.MinY, md.MaxY, md.MinZ, md.MaxZ,
		)
	}
	return stats, culled
}

func medianFloat64(v []float64) float64 {
	n := len(v)
	if n == 0 {
		return 0
	}
	sort.Float64s(v)
	mid := n / 2
	if n%2 == 1 {
		return v[mid]
	}
	return (v[mid-1] + v[mid]) / 2
}
