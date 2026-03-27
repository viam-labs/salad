package main

import (
	"math"

	"github.com/golang/geo/r2"
)

const defaultStaggerPercentage = 0.5

// StaggerOptions offsets every other row or column by a fraction of the step size,
// producing a denser, more uniform coverage pattern (hexagonal-like).
type StaggerOptions struct {
	Axis       string   // "x" or "y"
	Percentage *float64 // fraction of step to offset; defaults to 0.5
}

// tile1D returns evenly-spaced values between start and stop,
// at most maxDistance apart, centered in each interval.
func tile1D(start, stop, maxDistance float64) []float64 {
	totalDistance := math.Abs(stop - start)
	numPoints := int(math.Ceil(totalDistance / maxDistance))
	stepSize := totalDistance / float64(numPoints)
	result := make([]float64, 0, numPoints)
	for i := range numPoints {
		result = append(result, start+stepSize*float64(1+2*i)/2.0)
	}
	return result
}

// tile2D returns a 2D grid of points from two independent 1D sweeps.
func tile2D(start, stop, maxDistance r2.Point) []r2.Point {
	xs := tile1D(start.X, stop.X, maxDistance.X)
	ys := tile1D(start.Y, stop.Y, maxDistance.Y)

	result := make([]r2.Point, 0, len(xs)*len(ys))
	for _, x := range xs {
		for _, y := range ys {
			result = append(result, r2.Point{X: x, Y: y})
		}
	}
	return result
}

// tile1DFixedDistance returns values from start to stop at fixed step intervals.
func tile1DFixedDistance(start, stop, distanceMM float64) []float64 {
	result := make([]float64, 0)
	for i := start; i <= stop; i += distanceMM {
		result = append(result, i)
	}
	return result
}

// tile2DFixedDistance produces a 2D grid of points with fixed step sizes.
// If staggerOpts is non-nil, every other row (axis="x") or column (axis="y")
// is offset by (step * percentage) for denser, more uniform coverage.
func tile2DFixedDistance(boxCorner, oppositeBoxCorner r2.Point, distanceXMM, distanceYMM float64, staggerOpts *StaggerOptions) []r2.Point {
	xs := tile1DFixedDistance(boxCorner.X, oppositeBoxCorner.X, distanceXMM)
	ys := tile1DFixedDistance(boxCorner.Y, oppositeBoxCorner.Y, distanceYMM)

	percentage := defaultStaggerPercentage
	if staggerOpts != nil && staggerOpts.Percentage != nil {
		percentage = *staggerOpts.Percentage
	}

	result := make([]r2.Point, 0, len(xs)*len(ys))
	for xi, x := range xs {
		for yi, y := range ys {
			px, py := x, y
			if staggerOpts != nil {
				if staggerOpts.Axis == "x" && yi%2 == 1 {
					px = clamp(px+distanceXMM*percentage, boxCorner.X, oppositeBoxCorner.X)
				}
				if staggerOpts.Axis == "y" && xi%2 == 1 {
					py = clamp(py+distanceYMM*percentage, boxCorner.Y, oppositeBoxCorner.Y)
				}
			}
			result = append(result, r2.Point{X: px, Y: py})
		}
	}
	return result
}

func clamp(value, min, max float64) float64 {
	if value < min {
		return min
	}
	if value > max {
		return max
	}
	return value
}
