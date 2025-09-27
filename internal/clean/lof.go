package clean

import (
	"fmt"
	"math"
	"runtime"
	"sort"
	"sync"
)

// lofData holds LOF computation data for a point
type lofData struct {
	neighbors []neighborInfo
	kDistance float64
	lrd       float64
	lof       float64
}

// neighborInfo holds information about a neighbor point
type neighborInfo struct {
	index    int
	distance float64
}

// lofOutlierDetection finds GPS outliers using Local Outlier Factor
func lofOutlierDetection(inputPoints []Point, k int, config Config) []int {
	fmt.Printf("🔬 LOF analysis with k=%d...\n", k)
	n := len(inputPoints)

	if n < k+1 {
		fmt.Printf("   ⚠️  Not enough points for LOF analysis (need >%d)\n", k)
		return make([]int, n)
	}

	// Detect pause regions to protect from filtering
	pauseRegions := detectPauseRegions(inputPoints, config)
	pauseCount := 0
	for _, isPause := range pauseRegions {
		if isPause {
			pauseCount++
		}
	}
	fmt.Printf("   🛑 Detecting pause regions (≥%.0fs at ≤%.1fm/s)...\n", config.MinPauseDuration, config.PauseSpeed)
	fmt.Printf("   Protected %d pause points from filtering\n", pauseCount)

	// Calculate distances and find neighbors
	// Step 1: Calculate k-distance and k-nearest neighbors for each point
	lofValues := make([]lofData, n)

	// Parallel computation of k-nearest neighbors with optimized chunking
	numWorkers := runtime.NumCPU()
	chunkSize := max(1, n/numWorkers)

	var wg sync.WaitGroup
	for start := 0; start < n; start += chunkSize {
		end := min(start+chunkSize, n)
		wg.Add(1)

		go func(startIdx, endIdx int) {
			defer wg.Done()
			window := neighborWindow(n, k)

			for i := startIdx; i < endIdx; i++ {
				windowStart := max(0, i-window/2)
				windowEnd := min(n, i+window/2+1)

				neighbors := findKNearestNeighbors(inputPoints, i, k, windowStart, windowEnd)
				lofValues[i].neighbors = neighbors
				if len(neighbors) > 0 {
					lofValues[i].kDistance = neighbors[len(neighbors)-1].distance
				}
			}
		}(start, end)
	}
	wg.Wait()

	// Step 2: Calculate Local Reachability Density (LRD)
	for i := 0; i < n; i++ {
		lofValues[i].lrd = calculateLRD(i, lofValues, inputPoints)
	}

	// Step 3: Calculate Local Outlier Factor (LOF)
	lofScores := make([]float64, n)
	for i := 0; i < n; i++ {
		lof := calculateLOF(i, lofValues)
		lofValues[i].lof = lof
		lofScores[i] = lof
	}

	// Step 4: Robust outlier detection using median and MAD
	validLofScores := make([]float64, 0, n)
	for i, lof := range lofScores {
		if !pauseRegions[i] { // Exclude pause points from threshold calculation
			validLofScores = append(validLofScores, lof)
		}
	}

	lofMedian := medianFloat(validLofScores)
	deviations := make([]float64, len(validLofScores))
	for i, lof := range validLofScores {
		deviations[i] = math.Abs(lof - lofMedian)
	}
	mad := medianFloat(deviations)

	// Threshold: median + 2.5 * MAD
	// This is much more stable on tracks with legitimate sharp turns
	robustThreshold := lofMedian + 2.5*mad

	// Ensure minimum threshold to catch obvious outliers
	threshold := math.Max(robustThreshold, config.LofMinThreshold)

	// Count outliers
	outliers := 0
	for _, lof := range validLofScores {
		if lof > threshold {
			outliers++
		}
	}

	fmt.Printf("   LOF statistics: median=%.2f, MAD=%.2f, min=%.2f, max=%.2f\n",
		lofMedian, mad, minFloat(lofScores), maxFloat(lofScores))
	fmt.Printf("   Threshold: %.2f (median + 2.5*MAD), outliers: %d (%.1f%%)\n",
		threshold, outliers, float64(outliers)/float64(len(validLofScores))*100)

	// Step 5: Filter points based on LOF scores
	validIndices := make([]int, 0, n)
	for i := 0; i < n; i++ {
		// Keep points that are not outliers OR are in pause regions
		if lofScores[i] <= threshold || pauseRegions[i] {
			validIndices = append(validIndices, i)
		}
	}

	return validIndices
}

// detectPauseRegions identifies regions where the user was stationary
func detectPauseRegions(points []Point, config Config) []bool {
	n := len(points)
	isPause := make([]bool, n)

	if n < 3 {
		return isPause
	}

	// Find regions with sustained low speed
	for i := 1; i < n-1; i++ {
		if points[i].Time.IsZero() {
			continue
		}

		// Look for sustained low speed periods
		lowSpeedStart := -1
		for j := max(0, i-10); j <= min(n-2, i+10); j++ {
			if j+1 >= n || points[j].Time.IsZero() || points[j+1].Time.IsZero() {
				continue
			}

			dt := points[j+1].Time.Sub(points[j].Time).Seconds()
			if dt <= 0 {
				continue
			}

			dist := haversineDistance3D(
				points[j].Lat, points[j].Lon, points[j].Elevation,
				points[j+1].Lat, points[j+1].Lon, points[j+1].Elevation)
			speed := dist / dt

			if speed <= config.PauseSpeed {
				if lowSpeedStart == -1 {
					lowSpeedStart = j
				}
			} else {
				if lowSpeedStart != -1 {
					duration := points[j].Time.Sub(points[lowSpeedStart].Time).Seconds()
					if duration >= config.MinPauseDuration {
						// Mark this region as a pause
						for k := lowSpeedStart; k <= j; k++ {
							if k < n {
								isPause[k] = true
							}
						}
					}
				}
				lowSpeedStart = -1
			}
		}
	}

	return isPause
}

// neighborWindow calculates search window for k-NN
func neighborWindow(n, k int) int {
	const (
		bigTrackWindowMid   = 400
		bigTrackWindowLarge = 200
	)

	switch {
	case n <= 10000:
		// Small/medium tracks
		return max(2*k+1, min(max(30, n/20), n))
	case n <= 50000:
		// Large tracks: limit to ±400 points
		return max(2*k+1, min(2*bigTrackWindowMid, n))
	default:
		// Very large tracks: limit to ±200 points
		return max(2*k+1, min(2*bigTrackWindowLarge, n))
	}
}

// findKNearestNeighbors finds k nearest neighbors within a search window
func findKNearestNeighbors(points []Point, targetIdx, k, windowStart, windowEnd int) []neighborInfo {
	target := points[targetIdx]
	var neighbors []neighborInfo

	for i := windowStart; i < windowEnd; i++ {
		if i == targetIdx {
			continue
		}

		distance := haversineDistance3D(
			target.Lat, target.Lon, target.Elevation,
			points[i].Lat, points[i].Lon, points[i].Elevation)

		neighbors = append(neighbors, neighborInfo{
			index:    i,
			distance: distance,
		})
	}

	// Sort by distance and take k nearest
	sort.Slice(neighbors, func(i, j int) bool {
		return neighbors[i].distance < neighbors[j].distance
	})

	if len(neighbors) > k {
		neighbors = neighbors[:k]
	}

	return neighbors
}

// calculateLRD computes Local Reachability Density
// LRD_k(A) = |N_k(A)| / Σ_{B∈N_k(A)} reach-dist_k(A,B)
// where reach-dist_k(A,B) = max(k-distance(B), d(A,B))
func calculateLRD(pointIndex int, lofValues []lofData, points []Point) float64 {
	neighbors := lofValues[pointIndex].neighbors
	if len(neighbors) == 0 {
		return 0.0
	}

	sumReachabilityDist := 0.0
	for _, neighbor := range neighbors {
		neighborKDist := lofValues[neighbor.index].kDistance
		reachabilityDist := math.Max(neighborKDist, neighbor.distance)
		sumReachabilityDist += reachabilityDist
	}

	if sumReachabilityDist == 0 {
		return math.Inf(1)
	}

	// Local Reachability Density = |N_k(A)| / Σ reach-dist
	return float64(len(neighbors)) / sumReachabilityDist
}

// calculateLOF computes Local Outlier Factor
// LOF_k(A) = (1/|N_k(A)|) Σ_{B∈N_k(A)} LRD_k(B)/LRD_k(A)
// This measures how much more/less dense point A is compared to its neighbors
func calculateLOF(pointIndex int, lofValues []lofData) float64 {
	neighbors := lofValues[pointIndex].neighbors
	if len(neighbors) == 0 {
		return 1.0
	}

	pointLRD := lofValues[pointIndex].lrd
	if pointLRD == 0 || math.IsInf(pointLRD, 1) {
		return 1.0
	}

	sumLRDRatio := 0.0
	validNeighbors := 0

	for _, neighbor := range neighbors {
		neighborLRD := lofValues[neighbor.index].lrd
		if neighborLRD > 0 && !math.IsInf(neighborLRD, 1) {
			sumLRDRatio += neighborLRD / pointLRD
			validNeighbors++
		}
	}

	if validNeighbors == 0 {
		return 1.0
	}

	return sumLRDRatio / float64(validNeighbors)
}

// Utility functions
func minFloat(values []float64) float64 {
	if len(values) == 0 {
		return 0.0
	}
	min := values[0]
	for _, v := range values[1:] {
		if v < min {
			min = v
		}
	}
	return min
}

func maxFloat(values []float64) float64 {
	if len(values) == 0 {
		return 0.0
	}
	max := values[0]
	for _, v := range values[1:] {
		if v > max {
			max = v
		}
	}
	return max
}