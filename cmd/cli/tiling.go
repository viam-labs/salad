package main

// tile1DFixedDistance returns values from start to stop at fixed step intervals.
func tile1DFixedDistance(start, stop, distanceMM float64) []float64 {
	result := make([]float64, 0)
	for i := start; i <= stop; i += distanceMM {
		result = append(result, i)
	}
	return result
}
