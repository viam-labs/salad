package salad

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/spatialmath"

	"salad/segmentation"
)

// DefaultServingDepthMM is how far below the detected food surface the gripper
// descends to scoop a serving when a bin does not specify a serving-depth-mm.
const DefaultServingDepthMM = 30.0

const MinimumFoodLevelMM = 25.0

// FoodLevelMMFromPlaneFitStats returns the median signed distance to the zone
// plane at the plane center, matching getBinFoodLevel's height-map lookup.
func FoodLevelMMFromPlaneFitStats(zone *segmentation.Zone, stats segmentation.PlaneFitStats) (float64, error) {
	planePoint := r3.Vector{X: zone.Plane.Point[0], Y: zone.Plane.Point[1], Z: zone.Plane.Point[2]}
	heightAtPoint := stats.HeightMap.ApproximateHeightAround(planePoint, 3)
	if heightAtPoint == nil || *heightAtPoint < 0 {
		return 0, fmt.Errorf("distance to plane is wrong: %v mm", heightAtPoint)
	}
	return *heightAtPoint, nil
}

// ComputeGrabBasePoint returns the world-frame point where the gripper should
// enter the food: servingDepthMM below the detected food surface along the
// zone plane normal.
func ComputeGrabBasePoint(zone *segmentation.Zone, foodLevelMM, servingDepthMM float64) (r3.Vector, error) {
	if foodLevelMM < MinimumFoodLevelMM {
		return r3.Vector{}, fmt.Errorf("food level is too low: %.2f mm (minimum %.2f mm)", foodLevelMM, MinimumFoodLevelMM)
	}
	zoneCenter := r3.Vector{X: zone.Plane.Point[0], Y: zone.Plane.Point[1], Z: zone.Plane.Point[2]}
	normal := zonePlaneNormal(zone)
	foodHeight := zoneCenter.Add(normal.Mul(foodLevelMM))
	return foodHeight.Add(normal.Mul(-servingDepthMM)), nil
}

// ComputeGrabPose returns the arm-base pose that places the closed gripper tip
// at the grab base point.
func ComputeGrabPose(
	zone *segmentation.Zone,
	foodLevelMM, servingDepthMM, closedGripperHeightMM float64,
	orientation spatialmath.Orientation,
) (spatialmath.Pose, error) {
	grabBasePoint, err := ComputeGrabBasePoint(zone, foodLevelMM, servingDepthMM)
	if err != nil {
		return nil, err
	}
	orientVec := orientation.OrientationVectorRadians()
	armAxisVec := r3.Vector{
		X: orientVec.OX,
		Y: orientVec.OY,
		Z: orientVec.OZ,
	}
	idealArmBasePosition := grabBasePoint.Add(armAxisVec.Mul(-closedGripperHeightMM))
	return spatialmath.NewPose(idealArmBasePosition, orientation), nil
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
