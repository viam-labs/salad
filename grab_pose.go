package salad

import (
	"fmt"
	"math"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

// DefaultServingDepthMM is how far below the detected food surface the gripper
// descends to scoop a serving when a bin does not specify a serving-depth-mm.
const DefaultServingDepthMM = 30.0

const MinimumFoodLevelMM = 25.0

// FoodPoint describes the highest detected food-surface location in a zone.
type FoodPoint struct {
	// Point is the world-frame coordinate of the food surface at the highest
	// height-map cell (the cell center XY, lifted LevelMM above the plane along
	// the plane normal).
	Point r3.Vector
	// LevelMM is the signed distance of Point above the zone's bin-floor plane,
	// i.e. how much food sits above the bin floor at that point. Used to decide
	// whether the bin has enough food (see MinimumFoodLevelMM).
	LevelMM float64
	// FloorPoint is the point on the bin-floor plane directly below Point (same
	// XY, projected onto the plane). It marks where the bottom of the bin is.
	FloorPoint r3.Vector
}

// FoodPointFromPlaneFitStats returns the highest food point in the zone: the
// height-map cell with the greatest median signed distance to the bin-floor
// plane, expressed as a world-frame coordinate. It errors if no cell has data
// or the highest point is below MinimumFoodLevelMM.
//
// Mask the stats' HeightMap (e.g. with MaskGripperOverflow) before calling to
// restrict the search to gripper-valid cells.
func FoodPointFromPlaneFitStats(zone *segmentation.Zone, stats segmentation.PlaneFitStats) (FoodPoint, error) {
	centerX, centerY, levelMM, ok := stats.HeightMap.HighestCell()
	if !ok {
		return FoodPoint{}, fmt.Errorf("no populated height-map cells in zone %d", zone.ID)
	}
	if levelMM < MinimumFoodLevelMM {
		return FoodPoint{}, fmt.Errorf("food level is too low: %.2f mm (minimum %.2f mm)", levelMM, MinimumFoodLevelMM)
	}
	normal := zonePlaneNormal(zone)
	floor := r3.Vector{X: centerX, Y: centerY, Z: zone.Plane.ZAt(centerX, centerY)}
	return FoodPoint{
		Point:      floor.Add(normal.Mul(levelMM)),
		LevelMM:    levelMM,
		FloorPoint: floor,
	}, nil
}

// GripperWorldExtents maps the gripper's calibrated open dimensions onto the
// world X and Y axes, returning extents suitable for
// segmentation.ZoneHeightMap.MaskGripperOverflow(xExtentMM, yExtentMM).
//
// The open gripper is wider on one axis than the other, so the bounding box we
// must keep inside the zone depends on how the gripper is rotated in world
// space at grab time. This decides whether the width or the depth runs along
// world X vs world Y.
//
// Assumptions:
//   - openWidthMM was measured along the gripper's local X axis and openDepthMM
//     along its local Y axis (see grabberControls.measureOpenGripper, which
//     uses geometryXSpan / geometryYSpan on the gripper-frame geometries).
//   - orientation is the gripper's world orientation at grab time (the grab
//     pose orientation, e.g. BinHoverOrientation). The columns of its rotation
//     matrix are therefore the world-frame directions of the gripper's local
//     axes.
//   - The zone's XY bounds (which MaskGripperOverflow checks against) are
//     axis-aligned with world X/Y, i.e. the zone plane's Y axis is the world Y
//     axis. The gripper's open bounding box is likewise treated as axis-aligned
//     in world XY.
//   - Exactly one of the gripper's local X/Y axes aligns with world Y and the
//     other with world X; we only need to decide which. The gripper axis whose
//     world-space unit vector has the larger |dot| with the world Y unit vector
//     is taken to be the world-Y-aligned axis.
func GripperWorldExtents(orientation spatialmath.Orientation, openWidthMM, openDepthMM float64) (xExtentMM, yExtentMM float64) {
	rm := orientation.RotationMatrix()
	gripperXInWorld := rm.Col(0) // world direction of gripper local +X (the width axis)
	gripperYInWorld := rm.Col(1) // world direction of gripper local +Y (the depth axis)
	worldY := r3.Vector{X: 0, Y: 1, Z: 0}
	if math.Abs(gripperXInWorld.Dot(worldY)) >= math.Abs(gripperYInWorld.Dot(worldY)) {
		// Gripper width (local X) runs along world Y; depth (local Y) runs along world X.
		return openWidthMM, openDepthMM
	}
	// Gripper depth (local Y) runs along world Y; width (local X) runs along world X.
	return openDepthMM, openWidthMM
}

// ComputeGrabBasePoint returns the world-frame point where the gripper should
// enter the food: servingDepthMM below the given food-surface point along the
// zone plane normal.
func ComputeGrabBasePoint(zone *segmentation.Zone, foodPoint r3.Vector, servingDepthMM float64) r3.Vector {
	normal := zonePlaneNormal(zone)
	return foodPoint.Add(normal.Mul(-servingDepthMM))
}

// ComputeGrabPose returns the arm-base pose that places the closed gripper tip
// at the grab base point derived from foodPoint.
func ComputeGrabPose(
	zone *segmentation.Zone,
	foodPoint r3.Vector,
	servingDepthMM, closedGripperHeightMM float64,
	orientation spatialmath.Orientation,
) spatialmath.Pose {
	grabBasePoint := ComputeGrabBasePoint(zone, foodPoint, servingDepthMM)
	orientVec := orientation.OrientationVectorRadians()
	armAxisVec := r3.Vector{
		X: orientVec.OX,
		Y: orientVec.OY,
		Z: orientVec.OZ,
	}
	idealArmBasePosition := grabBasePoint.Add(armAxisVec.Mul(-closedGripperHeightMM))
	return spatialmath.NewPose(idealArmBasePosition, orientation)
}

// BinHoverPose returns the configured hover pose for a bin zone: plane-point XY
// with Z at zMean, offset along -binHoverOrientation by binHoverHeightMM, plus
// optional per-bin hover XY offsets.
func BinHoverPose(
	zone *segmentation.Zone,
	zMean, binHoverHeightMM, hoverXOffsetMM, hoverYOffsetMM float64,
	orientation spatialmath.Orientation,
) (spatialmath.Pose, error) {
	orientVec := orientation.OrientationVectorRadians()
	axis := r3.Vector{X: orientVec.OX, Y: orientVec.OY, Z: orientVec.OZ}
	base := r3.Vector{
		X: zone.Plane.Point[0],
		Y: zone.Plane.Point[1],
		Z: zMean,
	}
	point := base.Add(axis.Mul(-binHoverHeightMM))
	point.X += hoverXOffsetMM
	point.Y += hoverYOffsetMM
	return spatialmath.NewPose(point, orientation), nil
}

func zonePlaneNormal(zone *segmentation.Zone) r3.Vector {
	return r3.Vector{
		X: zone.Plane.Normal[0],
		Y: zone.Plane.Normal[1],
		Z: zone.Plane.Normal[2],
	}
}
