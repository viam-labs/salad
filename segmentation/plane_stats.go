package segmentation

import (
	"math"
	"sort"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/pointcloud"
)

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
	}
	culled := pointcloud.NewBasicPointCloud(0)

	var sumAbs, sumSigned float64
	absDistances := make([]float64, 0, pc.Size())
	signedDistances := make([]float64, 0, pc.Size())
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
	} else {
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
