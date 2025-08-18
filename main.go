package main

import (
	"encoding/xml"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"path/filepath"
	"runtime"
	"sort"
	"strings"
	"sync"
	"time"
)

type gpxPoint struct {
	XMLName    xml.Name `xml:"trkpt"`
	Lat        float64  `xml:"lat,attr"`
	Lon        float64  `xml:"lon,attr"`
	Elevation  *float64 `xml:"ele,omitempty"`
	Time       string   `xml:"time,omitempty"`
	Extensions struct {
		Power int `xml:"power,omitempty"`
		TPX   struct {
			HR      int `xml:"gpxtpx:hr,omitempty"`
			Cadence int `xml:"gpxtpx:cad,omitempty"`
			Temp    int `xml:"gpxtpx:atemp,omitempty"`
		} `xml:"gpxtpx:TrackPointExtension,omitempty"`
	} `xml:"extensions,omitempty"`
}

type gpx struct {
	XMLName     xml.Name `xml:"gpx"`
	Xmlns       string   `xml:"xmlns,attr"`
	XmlnsXsi    string   `xml:"xmlns:xsi,attr,omitempty"`
	XmlnsGpxtpx string   `xml:"xmlns:gpxtpx,attr,omitempty"`
	Creator     string   `xml:"creator,attr"`
	Version     string   `xml:"version,attr"`
	SchemaLoc   string   `xml:"xsi:schemaLocation,attr,omitempty"`
	Metadata    struct {
		Time string `xml:"time,omitempty"`
	} `xml:"metadata,omitempty"`
	Tracks []struct {
		Name     string `xml:"name,omitempty"`
		Type     string `xml:"type,omitempty"`
		Segments []struct {
			Points []gpxPoint `xml:"trkpt"`
		} `xml:"trkseg"`
	} `xml:"trk"`
}

type point struct {
	Lat, Lon, Elevation     float64
	Time                    time.Time // Real timestamp from GPX for accurate speed calculation
	TrackIdx, SegIdx, PtIdx int       // Original indices for multi-segment GPX preservation
}

// UNIVERSAL DEFAULTS (no knobs) - battle-tested for all trail conditions
const (
	eleWindow         = 7     // median filter on elevation (smooths baro noise)
	maxHairpinDegrees = 160.0 // allow sharp trail switchbacks (vs 120° before)
	minSpeed          = 0.1   // m/s (0.36 km/h - allows extended stops)
	ultraMaxSpeed     = 12.0  // m/s (~43 km/h) conservative for trail running
	pauseSpeed        = 0.7   // m/s (slightly higher than before for robustness)
	minPauseDuration  = 120.0 // seconds (2 min - typical aid station stop)
	teleportMeters    = 200.0 // jump guard when timestamps missing
	lofMinK           = 10    // minimum LOF neighbors (was too low before)
	lofMaxK           = 20    // maximum LOF neighbors
	lofMinThreshold   = 1.5   // LOF floor
	removedPctGuard   = 20.0  // safety: never remove >20% of points
	distDropGuard     = 25.0  // safety: never drop >25% of distance
	bigTrackWindowLo  = 200   // ± window for >50k pts
	bigTrackWindowMid = 400   // ± window for >10k pts
)

type config struct {
	InputFile  string
	OutputFile string
	Workers    int
}

// chooseK picks optimal k for LOF based on track size (community-tested)
func chooseK(n int) int {
	k := n/1000 + 10 // base 10, +1 per 1000 points
	k = max(k, lofMinK)
	k = min(k, lofMaxK)
	return k
}

// neighborWindow calculates optimal search window for k-NN (performance vs accuracy)
func neighborWindow(n, k int) int {
	switch {
	case n <= 10000:
		// Small/medium tracks: use generous window
		return max(2*k+1, min(max(30, n/20), n))
	case n <= 50000:
		// Large tracks: limit to ±400 points
		return max(2*k+1, min(2*bigTrackWindowMid, n))
	default:
		// Very large tracks: limit to ±200 points
		return max(2*k+1, min(2*bigTrackWindowLo, n))
	}
}

func haversineDistance(lat1, lon1, lat2, lon2 float64) float64 {
	const earthRadius = 6371000 // meters

	if lat1 == lat2 && lon1 == lon2 {
		return 0
	}

	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLatRad := (lat2 - lat1) * math.Pi / 180
	deltaLonRad := (lon2 - lon1) * math.Pi / 180

	a := math.Sin(deltaLatRad/2)*math.Sin(deltaLatRad/2) +
		math.Cos(lat1Rad)*math.Cos(lat2Rad)*
			math.Sin(deltaLonRad/2)*math.Sin(deltaLonRad/2)

	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	return earthRadius * c
}

// haversineDistance3D calculates 3D distance including elevation difference
// Helps detect vertical GPS spikes and barometric glitches
func haversineDistance3D(lat1, lon1, elev1, lat2, lon2, elev2 float64) float64 {
	// Calculate horizontal distance using standard haversine
	horizontalDistance := haversineDistance(lat1, lon1, lat2, lon2)

	// Calculate vertical distance
	verticalDistance := math.Abs(elev2 - elev1)

	// Return 3D euclidean distance: sqrt(horizontal² + vertical²)
	return math.Sqrt(horizontalDistance*horizontalDistance + verticalDistance*verticalDistance)
}

// parseTimeSafe tries multiple timestamp formats for robust GPX parsing
func parseTimeSafe(s string) time.Time {
	layouts := []string{
		time.RFC3339Nano,
		time.RFC3339,
		"2006-01-02T15:04:05.000Z07:00",
		"2006-01-02T15:04:05Z07:00",
		"2006-01-02T15:04:05.000Z",
		"2006-01-02T15:04:05Z",
		"2006-01-02T15:04:05",
	}
	for _, layout := range layouts {
		if t, err := time.Parse(layout, s); err == nil {
			return t
		}
	}
	return time.Time{}
}

func parseGPX(filename string) ([]point, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to open file: %w", err)
	}
	defer func() {
		_ = file.Close()
	}()

	data, err := io.ReadAll(file)
	if err != nil {
		return nil, fmt.Errorf("failed to read file: %w", err)
	}

	var gpxData gpx
	if err := xml.Unmarshal(data, &gpxData); err != nil {
		return nil, fmt.Errorf("failed to parse GPX: %w", err)
	}

	var points []point
	for trackIdx, track := range gpxData.Tracks {
		for segIdx, segment := range track.Segments {
			for ptIdx, trkpt := range segment.Points {
				elevation := 0.0
				if trkpt.Elevation != nil {
					elevation = *trkpt.Elevation
				}

				// Parse GPX timestamp using robust parsing
				var pointTime time.Time
				if trkpt.Time != "" {
					pointTime = parseTimeSafe(trkpt.Time)
				}

				points = append(points, point{
					Lat:       trkpt.Lat,
					Lon:       trkpt.Lon,
					Elevation: elevation,
					Time:      pointTime,
					TrackIdx:  trackIdx,
					SegIdx:    segIdx,
					PtIdx:     ptIdx,
				})
			}
		}
	}

	return points, nil
}

func calculateDistance(points []point) float64 {
	if len(points) < 2 {
		return 0
	}

	total := 0.0
	for i := 1; i < len(points); i++ {
		total += haversineDistance(points[i-1].Lat, points[i-1].Lon, points[i].Lat, points[i].Lon)
	}
	return total
}

type neighbor struct {
	index    int
	distance float64
}

type lofData struct {
	kDistance float64
	neighbors []neighbor
	lrd       float64
	lof       float64
}

// lofOutlierDetection implements the Local Outlier Factor algorithm as described by
// Breunig, Kriegel, Ng, and Sander (2000) in "LOF: Identifying Density-Based Local Outliers".
// The algorithm computes outlier scores based on the local density deviation of data points
// with respect to their neighborhoods using mathematically rigorous formulations.
func lofOutlierDetection(inputPoints []point, k int) []int {
	fmt.Printf("🔬 LOF Analysis (Breunig et al. 2000) with k=%d...\n", k)
	n := len(inputPoints)

	if n < k+1 {
		fmt.Printf("   ⚠️  Not enough points for LOF analysis (need >%d)\n", k)
		result := make([]int, n)
		for i := range result {
			result[i] = i
		}
		return result
	}

	// Detect pause regions to protect from aggressive filtering
	fmt.Printf("   🛑 Detecting pause regions (≥%.0fs at ≤%.1fm/s)...\n", minPauseDuration, pauseSpeed)
	pauseRegions := detectPauseRegions(inputPoints, minPauseDuration, pauseSpeed)
	pauseCount := 0
	for _, isPause := range pauseRegions {
		if isPause {
			pauseCount++
		}
	}
	fmt.Printf("   Protected %d pause points from filtering\n", pauseCount)

	// Mathematical Implementation following Breunig et al. (2000)
	// Step 1: Calculate k-distance and k-nearest neighbors for each point
	lofValues := make([]lofData, n)

	// Parallel computation of k-nearest neighbors with optimized chunking
	var wg sync.WaitGroup
	chunkSize := max(n/runtime.NumCPU(), 50)

	for i := 0; i < n; i += chunkSize {
		start := i
		end := min(i+chunkSize, n)

		wg.Add(1)
		go func(s, e int) {
			defer wg.Done()
			for idx := s; idx < e; idx++ {
				// Find k-nearest neighbors and compute k-distance
				neighbors := findKNearestNeighbors(inputPoints, idx, k)
				lofValues[idx].neighbors = neighbors
				// k-distance is the distance to the k-th nearest neighbor
				if len(neighbors) >= k {
					lofValues[idx].kDistance = neighbors[k-1].distance
				} else if len(neighbors) > 0 {
					lofValues[idx].kDistance = neighbors[len(neighbors)-1].distance
				}
			}
		}(start, end)
	}
	wg.Wait()

	// Step 2: Calculate Local Reachability Density (LRD) for each point
	// LRD_k(A) = |N_k(A)| / Σ_{B∈N_k(A)} reach-dist_k(A,B)
	for i := range lofValues {
		lofValues[i].lrd = calculateLRD(i, lofValues)
	}

	// Step 3: Calculate Local Outlier Factor (LOF) for each point
	// LOF_k(A) = (1/|N_k(A)|) Σ_{B∈N_k(A)} LRD_k(B)/LRD_k(A)
	for i := range lofValues {
		lofValues[i].lof = calculateLOF(i, lofValues)
	}

	// Step 4: Robust outlier threshold based on LOF distribution
	// Use robust statistics: median + MAD for stability on trails with hairpins
	lofScores := make([]float64, n)
	for i, lofData := range lofValues {
		lofScores[i] = lofData.lof
	}

	// Calculate robust statistics (median + MAD)
	lofMedian := medianFloat(lofScores)

	// Calculate MAD (Median Absolute Deviation)
	deviations := make([]float64, n)
	for i, lof := range lofScores {
		deviations[i] = math.Abs(lof - lofMedian)
	}
	mad := medianFloat(deviations)

	// Robust threshold: median + 2.5 * MAD (conservative for trail running)
	// This is much more stable on tracks with legitimate sharp turns
	robustThreshold := lofMedian + 2.5*mad

	// Ensure minimum threshold to catch obvious outliers
	lofThreshold := math.Max(robustThreshold, lofMinThreshold)

	validIndices := []int{}
	outlierCount := 0

	for i, lofData := range lofValues {
		// Classic LOF interpretation: LOF ≈ 1 means normal, LOF >> 1 means outlier
		// However, protect pause regions from being filtered out
		if lofData.lof <= lofThreshold || pauseRegions[i] {
			validIndices = append(validIndices, i)
		} else {
			outlierCount++
		}
	}

	// Additional diagnostic info
	var maxLOF, minLOF float64 = 0, math.Inf(1)
	for _, lofData := range lofValues {
		maxLOF = math.Max(maxLOF, lofData.lof)
		minLOF = math.Min(minLOF, lofData.lof)
	}

	fmt.Printf("   LOF statistics: median=%.2f, MAD=%.2f, min=%.2f, max=%.2f\n",
		lofMedian, mad, minLOF, maxLOF)
	fmt.Printf("   Threshold: %.2f (median + 2.5*MAD), outliers: %d (%.1f%%)\n",
		lofThreshold, outlierCount, float64(outlierCount)/float64(n)*100)

	return validIndices
}

// findKNearestNeighbors uses temporal locality optimization for GPS tracks.
// Since GPS points are ordered by time, nearby points in the array are usually
// geographically close. This reduces complexity from O(n) to O(window_size).
func findKNearestNeighbors(points []point, targetIdx, k int) []neighbor {
	n := len(points)
	targetPoint := points[targetIdx]

	// Use optimized window calculation
	windowSize := neighborWindow(n, k)

	// Ensure window size is odd for symmetry
	if windowSize%2 == 0 {
		windowSize++
	}
	halfWindow := windowSize / 2

	// Define search window around target point
	startIdx := max(0, targetIdx-halfWindow)
	endIdx := min(n, targetIdx+halfWindow+1)

	// Calculate distances only within the window
	neighbors := make([]neighbor, 0, endIdx-startIdx)
	for i := startIdx; i < endIdx; i++ {
		if i != targetIdx {
			// Use 3D distance for more accurate neighbor detection
			dist := haversineDistance3D(targetPoint.Lat, targetPoint.Lon, targetPoint.Elevation,
				points[i].Lat, points[i].Lon, points[i].Elevation)
			neighbors = append(neighbors, neighbor{index: i, distance: dist})
		}
	}

	// Sort by distance and take k nearest
	sort.Slice(neighbors, func(i, j int) bool {
		return neighbors[i].distance < neighbors[j].distance
	})

	// Return up to k nearest neighbors
	if len(neighbors) > k {
		neighbors = neighbors[:k]
	}

	return neighbors
}

// calculateLRD computes Local Reachability Density following Breunig et al. (2000)
// LRD_k(A) = |N_k(A)| / Σ_{B∈N_k(A)} reach-dist_k(A,B)
// where reach-dist_k(A,B) = max(k-distance(B), d(A,B))
func calculateLRD(pointIndex int, lofValues []lofData) float64 {
	neighbors := lofValues[pointIndex].neighbors
	if len(neighbors) == 0 {
		return 1.0 // Default density for isolated points
	}

	// Calculate sum of reachability distances
	sumReachabilityDist := 0.0
	for _, neighbor := range neighbors {
		// Reachability distance: reach-dist_k(A,B) = max(k-distance(B), d(A,B))
		kDistanceB := lofValues[neighbor.index].kDistance
		actualDistance := neighbor.distance
		reachabilityDistance := math.Max(kDistanceB, actualDistance)
		sumReachabilityDist += reachabilityDistance
	}

	// LRD is the inverse of average reachability distance
	avgReachabilityDist := sumReachabilityDist / float64(len(neighbors))
	if avgReachabilityDist == 0 {
		// Handle edge case: if all neighbors are at same location
		return math.Inf(1) // Infinite density
	}

	// Local Reachability Density = |N_k(A)| / Σ reach-dist
	return float64(len(neighbors)) / sumReachabilityDist
}

// calculateLOF computes Local Outlier Factor following Breunig et al. (2000)
// LOF_k(A) = (1/|N_k(A)|) Σ_{B∈N_k(A)} LRD_k(B)/LRD_k(A)
// This measures how much more/less dense point A is compared to its neighbors
func calculateLOF(pointIndex int, lofValues []lofData) float64 {
	neighbors := lofValues[pointIndex].neighbors
	pointLRD := lofValues[pointIndex].lrd

	if len(neighbors) == 0 {
		return 1.0 // Default LOF for isolated points
	}

	if pointLRD == 0 || math.IsInf(pointLRD, 1) {
		// Point has infinite density (all neighbors at same location)
		return 1.0
	}

	// Calculate sum of LRD ratios: Σ_{B∈N_k(A)} LRD_k(B)/LRD_k(A)
	sumLRDRatio := 0.0
	validNeighbors := 0

	for _, neighbor := range neighbors {
		neighborLRD := lofValues[neighbor.index].lrd

		// Skip neighbors with infinite or zero density
		if math.IsInf(neighborLRD, 1) || neighborLRD == 0 {
			continue
		}

		// Add ratio of neighbor's density to point's density
		sumLRDRatio += neighborLRD / pointLRD
		validNeighbors++
	}

	if validNeighbors == 0 {
		return 1.0 // Default if no valid neighbors
	}

	// LOF is average density ratio: higher values indicate outliers
	return sumLRDRatio / float64(validNeighbors)
}

func advancedPrecisionFilter(inputPoints []point) []int {
	fmt.Printf("🔬 GPS outlier detection using LOF algorithm...\n")

	// Always use LOF algorithm - no reference tracks needed
	return fastLOFDetection(inputPoints)
}

func fastLOFDetection(inputPoints []point) []int {
	fmt.Printf("   Hybrid GPS outlier detection (velocity + LOF)...\n")

	n := len(inputPoints)

	// Step 1: Pre-filter with adaptive velocity-based outlier detection
	fmt.Printf("   Step 1: Adaptive velocity filter...\n")
	velocityFiltered := velocityOutlierFilter(inputPoints)
	filteredPoints := extractPoints(inputPoints, velocityFiltered)

	velocityRemovedCount := n - len(filteredPoints)
	fmt.Printf("   Removed %d obvious GPS jumps (%.1f%%) via velocity filter\n",
		velocityRemovedCount, float64(velocityRemovedCount)/float64(n)*100)

	if len(filteredPoints) == 0 {
		return []int{} // All points were outliers
	}

	// Step 2: Apply LOF to remaining points for subtle anomaly detection
	fmt.Printf("   Step 2: LOF analysis on %d velocity-filtered points...\n", len(filteredPoints))

	// Use universal k selection (community-tested)
	filteredN := len(filteredPoints)
	k := chooseK(filteredN)

	if filteredN > 500000 {
		// Extreme case fallback: use sampling only for massive files (>500k points)
		fmt.Printf("   Extreme size track (%d points). Using sampling for LOF analysis...\n", filteredN)
		sampleSize := 500000 // Sample 500k points for extreme cases
		sampleStep := filteredN / sampleSize
		sampleStep = max(sampleStep, 1)

		sampledPoints := make([]point, 0, sampleSize)
		for i := 0; i < filteredN; i += sampleStep {
			sampledPoints = append(sampledPoints, filteredPoints[i])
		}

		k = chooseK(len(sampledPoints))
		fmt.Printf("   Analyzing sample of %d points (every %d points)\n", len(sampledPoints), sampleStep)
		sampledLOF := lofOutlierDetection(sampledPoints, k)

		// Map sampled results back to original indices
		finalIndices := make([]int, 0, filteredN)
		validSampledSet := make(map[int]bool)
		for _, idx := range sampledLOF {
			validSampledSet[idx] = true
		}

		for i, originalIdx := range velocityFiltered {
			sampledIdx := i / sampleStep
			if i%sampleStep == 0 {
				// This point was sampled
				if validSampledSet[sampledIdx] {
					finalIndices = append(finalIndices, originalIdx)
				}
			} else {
				// Point between samples - keep by default (conservative)
				finalIndices = append(finalIndices, originalIdx)
			}
		}

		// SAFETY GUARDRAILS: Never remove too many points or distance (large track case)
		originalCount := len(inputPoints)
		finalCount := len(finalIndices)
		removalPercent := float64(originalCount-finalCount) / float64(originalCount) * 100

		if removalPercent > removedPctGuard {
			fmt.Printf("   ⚠️  Safety override: Would remove %.1f%% > %.0f%% limit. Using velocity filter only.\n",
				removalPercent, removedPctGuard)
			return velocityFiltered
		}

		// Check distance guardrail
		originalDistance := calculateDistance(inputPoints)
		finalDistance := calculateDistance(extractPoints(inputPoints, finalIndices))
		distanceReduction := 0.0
		if originalDistance > 0 {
			distanceReduction = (originalDistance - finalDistance) / originalDistance * 100
		}

		if distanceReduction > distDropGuard {
			fmt.Printf("   ⚠️  Safety override: Would drop %.1f%% > %.0f%% distance. Using velocity filter only.\n",
				distanceReduction, distDropGuard)
			return velocityFiltered
		}

		return finalIndices
	}

	lofFilteredIndices := lofOutlierDetection(filteredPoints, k)

	// Map LOF results back to original indices
	finalIndices := []int{}
	for _, lofIdx := range lofFilteredIndices {
		originalIdx := velocityFiltered[lofIdx]
		finalIndices = append(finalIndices, originalIdx)
	}

	// SAFETY GUARDRAILS: Never remove too many points or distance
	originalCount := len(inputPoints)
	finalCount := len(finalIndices)
	removalPercent := float64(originalCount-finalCount) / float64(originalCount) * 100

	if removalPercent > removedPctGuard {
		fmt.Printf("   ⚠️  Safety override: Would remove %.1f%% > %.0f%% limit. Using velocity filter only.\n",
			removalPercent, removedPctGuard)
		return velocityFiltered
	}

	// Check distance guardrail
	originalDistance := calculateDistance(inputPoints)
	finalDistance := calculateDistance(extractPoints(inputPoints, finalIndices))
	distanceReduction := 0.0
	if originalDistance > 0 {
		distanceReduction = (originalDistance - finalDistance) / originalDistance * 100
	}

	if distanceReduction > distDropGuard {
		fmt.Printf("   ⚠️  Safety override: Would drop %.1f%% > %.0f%% distance. Using velocity filter only.\n",
			distanceReduction, distDropGuard)
		return velocityFiltered
	}

	return finalIndices
}

// detectMaxReasonableSpeed analyzes the track to determine appropriate speed limits
// Automatically detects activity type: running/hiking (~10 m/s), cycling (~25 m/s), etc.
func detectMaxReasonableSpeed(points []point) float64 {
	if len(points) < 10 {
		return 15.0 // Default: 54 km/h for mixed activities
	}

	// Build robust time interval estimate from available timestamps
	deltas := make([]float64, 0, 512)
	for i := 1; i < len(points); i++ {
		if !points[i].Time.IsZero() && !points[i-1].Time.IsZero() {
			d := points[i].Time.Sub(points[i-1].Time).Seconds()
			if d > 0 && d < 300 { // Reasonable interval < 5 minutes
				deltas = append(deltas, d)
			}
		}
	}
	fallbackDt := medianFloat(deltas)
	if fallbackDt <= 0 {
		fallbackDt = 1.0 // Ultimate fallback
	}

	// Sample speeds from the track to determine activity type
	speeds := []float64{}
	sampleInterval := max(1, len(points)/1000) // Sample ~1000 points max

	for i := sampleInterval; i < len(points)-sampleInterval; i += sampleInterval {
		prev := points[i-sampleInterval]
		curr := points[i]
		next := points[i+sampleInterval]

		// Calculate 3D speeds using actual timestamps from GPX
		dist1 := haversineDistance3D(prev.Lat, prev.Lon, prev.Elevation,
			curr.Lat, curr.Lon, curr.Elevation)
		dist2 := haversineDistance3D(curr.Lat, curr.Lon, curr.Elevation,
			next.Lat, next.Lon, next.Elevation)

		// Use real time intervals from GPX timestamps
		timeInterval1 := curr.Time.Sub(prev.Time).Seconds()
		timeInterval2 := next.Time.Sub(curr.Time).Seconds()

		// Handle missing or invalid timestamps using robust median estimate
		if timeInterval1 <= 0 {
			timeInterval1 = fallbackDt
		}
		if timeInterval2 <= 0 {
			timeInterval2 = fallbackDt
		}

		speed1 := dist1 / timeInterval1
		speed2 := dist2 / timeInterval2

		// Only include reasonable speeds for analysis
		if speed1 > 0.5 && speed1 < 50.0 {
			speeds = append(speeds, speed1)
		}
		if speed2 > 0.5 && speed2 < 50.0 {
			speeds = append(speeds, speed2)
		}
	}

	if len(speeds) == 0 {
		return 15.0 // Default fallback
	}

	// Sort speeds to find percentiles
	sort.Float64s(speeds)

	// Use trimmed 95th percentile to avoid GPS spike contamination
	// Remove top 1% outliers before calculating P95
	trimmedEnd := int(float64(len(speeds)) * 0.99)
	if trimmedEnd >= len(speeds) {
		trimmedEnd = len(speeds) - 1
	}
	trimmedSpeeds := speeds[:trimmedEnd]

	var p95 float64
	if len(trimmedSpeeds) > 0 {
		p95 = trimmedSpeeds[int(float64(len(trimmedSpeeds))*0.95)]
	} else {
		p95 = speeds[int(float64(len(speeds))*0.95)] // Fallback to original
	}

	// Activity classification based on speed patterns
	var maxSpeed float64
	var activityType string

	if p95 <= 8.0 { // 28.8 km/h
		// Running/Hiking activity
		maxSpeed = 12.0 // 43.2 km/h (generous for running)
		activityType = "running/hiking"
	} else if p95 <= 20.0 { // 72 km/h
		// Cycling activity
		maxSpeed = 30.0 // 108 km/h (generous for cycling)
		activityType = "cycling"
	} else {
		// High-speed activity (skiing, motorsports)
		maxSpeed = 50.0 // 180 km/h
		activityType = "high-speed"
	}

	// Calculate average time interval for diagnostics
	avgInterval := 0.0
	intervalCount := 0
	for i := 1; i < len(points) && intervalCount < 100; i += sampleInterval {
		if !points[i].Time.IsZero() && !points[i-1].Time.IsZero() {
			interval := points[i].Time.Sub(points[i-1].Time).Seconds()
			if interval > 0 && interval < 300 { // Reasonable interval < 5 minutes
				avgInterval += interval
				intervalCount++
			}
		}
	}
	if intervalCount > 0 {
		avgInterval /= float64(intervalCount)
	}

	fmt.Printf("   Auto-detected activity: %s (P95: %.1f m/s, avg interval: %.1fs)\n",
		activityType, p95, avgInterval)
	fmt.Printf("   Speed limit: %.1f m/s (%.1f km/h)\n", maxSpeed, maxSpeed*3.6)

	return maxSpeed
}

// medianFloat calculates median of float64 slice for robust time interval estimation
func medianFloat(xs []float64) float64 {
	if len(xs) == 0 {
		return 0
	}
	ys := append([]float64(nil), xs...)
	sort.Float64s(ys)
	mid := len(ys) / 2
	if len(ys)%2 == 1 {
		return ys[mid]
	}
	return 0.5 * (ys[mid-1] + ys[mid])
}

// smoothElevation applies median filter to elevation data to reduce barometric noise
// This is critical for 3D distance calculations and speed-based filtering
func smoothElevation(points []point, windowSize int) {
	if len(points) < 3 || windowSize < 3 {
		return
	}

	// Ensure window size is odd
	if windowSize%2 == 0 {
		windowSize++
	}
	half := windowSize / 2

	// Create buffer for median calculation
	smoothed := make([]float64, len(points))

	for i := range points {
		// Collect elevation values in window
		elevations := make([]float64, 0, windowSize)
		start := max(0, i-half)
		end := min(len(points), i+half+1)

		for j := start; j < end; j++ {
			elevations = append(elevations, points[j].Elevation)
		}

		// Apply median filter
		smoothed[i] = medianFloat(elevations)
	}

	// Update points with smoothed elevation
	for i := range points {
		points[i].Elevation = smoothed[i]
	}
}

// bearing calculates the initial bearing from point A to point B using spherical trigonometry
func bearing(a, b point) float64 {
	φ1 := a.Lat * math.Pi / 180
	φ2 := b.Lat * math.Pi / 180
	λ1 := a.Lon * math.Pi / 180
	λ2 := b.Lon * math.Pi / 180

	y := math.Sin(λ2-λ1) * math.Cos(φ2)
	x := math.Cos(φ1)*math.Sin(φ2) - math.Sin(φ1)*math.Cos(φ2)*math.Cos(λ2-λ1)

	θ := math.Atan2(y, x)
	// Convert to degrees and normalize to 0-360
	deg := math.Mod((θ*180/math.Pi)+360, 360)
	return deg
}

// calculateTurnAngle calculates the turn angle at point B when moving from A to B to C
// Uses spherical trigonometry for accurate bearing calculation regardless of latitude
// Returns angle in degrees (0-180°). Sharp turns (>120°) often indicate GPS jumps.
func calculateTurnAngle(pointA, pointB, pointC point) float64 {
	// Check for identical points
	if (pointA.Lat == pointB.Lat && pointA.Lon == pointB.Lon) ||
		(pointB.Lat == pointC.Lat && pointB.Lon == pointC.Lon) {
		return 0.0 // No turn if points are identical
	}

	// Calculate bearings using spherical trigonometry
	bearing1 := bearing(pointA, pointB) // A → B
	bearing2 := bearing(pointB, pointC) // B → C

	// Calculate turn angle (difference between bearings)
	diff := math.Abs(bearing2 - bearing1)

	// Normalize to 0-180° (smallest angle between bearings)
	if diff > 180 {
		diff = 360 - diff
	}

	return diff
}

// detectPauseRegions identifies regions where the athlete was likely stopped or moving very slowly
// These regions should be protected from aggressive filtering to preserve aid station stops etc.
func detectPauseRegions(points []point, minPauseDuration float64, maxPauseSpeed float64) []bool {
	if len(points) < 3 {
		return make([]bool, len(points))
	}

	isPause := make([]bool, len(points))

	// Detect low-speed segments
	for i := 1; i < len(points)-1; i++ {
		prev := points[i-1]
		curr := points[i]
		next := points[i+1]

		// Calculate speeds to previous and next points
		var speedToPrev, speedToNext float64 = -1, -1 // -1 means no valid timestamp

		if !curr.Time.IsZero() && !prev.Time.IsZero() {
			timeToPrev := curr.Time.Sub(prev.Time).Seconds()
			if timeToPrev > 0 {
				distToPrev := haversineDistance(prev.Lat, prev.Lon, curr.Lat, curr.Lon)
				speedToPrev = distToPrev / timeToPrev
			}
		}

		if !next.Time.IsZero() && !curr.Time.IsZero() {
			timeToNext := next.Time.Sub(curr.Time).Seconds()
			if timeToNext > 0 {
				distToNext := haversineDistance(curr.Lat, curr.Lon, next.Lat, next.Lon)
				speedToNext = distToNext / timeToNext
			}
		}

		// Mark as potential pause if at least one speed is valid and low
		lowPrev := (speedToPrev >= 0 && speedToPrev <= maxPauseSpeed)
		lowNext := (speedToNext >= 0 && speedToNext <= maxPauseSpeed)
		if lowPrev || lowNext {
			isPause[i] = true
		}
	}

	// Extend pause regions based on duration
	if minPauseDuration > 0 {
		for i := 0; i < len(points); i++ {
			if !isPause[i] {
				continue
			}

			// Find the extent of this pause region
			start := i
			end := i

			// Extend backwards
			for start > 0 && isPause[start-1] {
				start--
			}

			// Extend forwards
			for end < len(points)-1 && isPause[end+1] {
				end++
			}

			// Check if pause duration meets minimum requirement
			if end > start && !points[start].Time.IsZero() && !points[end].Time.IsZero() {
				duration := points[end].Time.Sub(points[start].Time).Seconds()
				if duration < minPauseDuration {
					// Too short, not a real pause
					for j := start; j <= end; j++ {
						isPause[j] = false
					}
				}
			}

			// Skip to end of this region
			i = end
		}
	}

	return isPause
}

// velocityOutlierFilter removes GPS points with unrealistic speeds for outdoor activities
// Supports running, hiking, cycling with adaptive speed limits based on activity detection
func velocityOutlierFilter(inputPoints []point) []int {
	if len(inputPoints) < 3 {
		result := make([]int, len(inputPoints))
		for i := range result {
			result[i] = i
		}
		return result
	}

	// Adaptive speed detection with universal cap for safety
	detectedMaxSpeed := detectMaxReasonableSpeed(inputPoints)
	maxSpeed := math.Min(detectedMaxSpeed, ultraMaxSpeed) // Cap at 12 m/s for safety

	fmt.Printf("   Detected activity max speed: %.1f m/s (%.1f km/h)\n", maxSpeed, maxSpeed*3.6)

	validIndices := []int{0} // Always keep first point

	for i := 1; i < len(inputPoints)-1; i++ {
		curr := inputPoints[i]
		prev := inputPoints[i-1]
		next := inputPoints[i+1]

		// Calculate 3D speeds using real timestamps from GPX (includes elevation)
		distToPrev := haversineDistance3D(prev.Lat, prev.Lon, prev.Elevation,
			curr.Lat, curr.Lon, curr.Elevation)
		distToNext := haversineDistance3D(curr.Lat, curr.Lon, curr.Elevation,
			next.Lat, next.Lon, next.Elevation)

		// Calculate actual time intervals from GPX timestamps
		timeToPrev := curr.Time.Sub(prev.Time).Seconds()
		timeToNext := next.Time.Sub(curr.Time).Seconds()

		// Check which timestamps are valid to avoid misleading speed calculations
		validPrev := timeToPrev > 0
		validNext := timeToNext > 0

		// Calculate turn angle to detect GPS jumps (sharp turns)
		// Trail running allows for very sharp hairpins, especially in mountains
		turnAngle := calculateTurnAngle(prev, curr, next)
		directionOK := turnAngle <= maxHairpinDegrees // Allow sharp hairpins

		// Speed validation: check timestamps first, fallback to distance-based detection
		speedOK := true
		if validPrev && validNext {
			// Both timestamps valid: check both speeds
			speedFromPrev := distToPrev / timeToPrev
			speedToNext := distToNext / timeToNext
			speedOK = (speedFromPrev >= minSpeed && speedFromPrev <= maxSpeed) &&
				(speedToNext >= minSpeed && speedToNext <= maxSpeed)
		} else if validPrev {
			// Only previous timestamp valid
			speedFromPrev := distToPrev / timeToPrev
			speedOK = (speedFromPrev >= minSpeed && speedFromPrev <= maxSpeed)
		} else if validNext {
			// Only next timestamp valid
			speedToNext := distToNext / timeToNext
			speedOK = (speedToNext >= minSpeed && speedToNext <= maxSpeed)
		} else {
			// No valid timestamps: fallback to distance-based teleport detection
			// Large jumps (>200m between consecutive points) are likely GPS errors
			if distToPrev > teleportMeters || distToNext > teleportMeters {
				speedOK = false
			}
		}

		if speedOK && directionOK {
			validIndices = append(validIndices, i)
		} else {
			// Additional check for stops: allow if reasonable location but failed speed/direction
			if len(validIndices) > 0 && i < len(inputPoints)-2 {
				lastValidIdx := validIndices[len(validIndices)-1]
				nextNext := inputPoints[i+1]

				// Check if it's a legitimate stop (low speed + reasonable location)
				distFromLastValid := haversineDistance(curr.Lat, curr.Lon,
					inputPoints[lastValidIdx].Lat, inputPoints[lastValidIdx].Lon)
				distToNextNext := haversineDistance(curr.Lat, curr.Lon, nextNext.Lat, nextNext.Lon)

				// Allow stops: close to track AND not a sharp direction change
				isReasonableStop := distFromLastValid <= 50.0 && distToNextNext <= 50.0

				// Check for low speed if we have valid timestamps
				isLowSpeed := false
				if validPrev && validNext {
					speedFromPrev := distToPrev / timeToPrev
					speedToNext := distToNext / timeToNext
					isLowSpeed = speedFromPrev <= 2.0 && speedToNext <= 2.0
				}

				if isReasonableStop && (isLowSpeed || turnAngle <= 90.0) {
					validIndices = append(validIndices, i)
				}
			}
		}
	}

	// Always keep last point
	if len(inputPoints) > 1 {
		validIndices = append(validIndices, len(inputPoints)-1)
	}

	// Print diagnostic information about filtering effectiveness
	totalRejected := len(inputPoints) - len(validIndices)
	fmt.Printf("   Velocity+Direction filter removed %d points (includes speed + direction violations)\n",
		totalRejected)

	return validIndices
}

func extractPoints(inputPoints []point, indices []int) []point {
	result := make([]point, len(indices))
	for i, idx := range indices {
		result[i] = inputPoints[idx]
	}
	return result
}

func writeCleanedGPX(inputFile, outputFile string, validIndices []int) error {
	file, err := os.Open(inputFile)
	if err != nil {
		return err
	}
	defer func() {
		_ = file.Close()
	}()

	data, err := io.ReadAll(file)
	if err != nil {
		return err
	}

	var originalGPX gpx
	if unmarshalErr := xml.Unmarshal(data, &originalGPX); unmarshalErr != nil {
		return unmarshalErr
	}

	if len(originalGPX.Tracks) == 0 {
		return fmt.Errorf("invalid GPX structure: no tracks")
	}

	// Create a set of valid indices for fast lookup
	validSet := make(map[int]bool)
	for _, idx := range validIndices {
		validSet[idx] = true
	}

	// Rebuild GPX structure preserving multi-segment layout
	globalIdx := 0
	for trackIdx := range originalGPX.Tracks {
		for segIdx := range originalGPX.Tracks[trackIdx].Segments {
			originalPoints := originalGPX.Tracks[trackIdx].Segments[segIdx].Points
			cleanedPoints := make([]gpxPoint, 0)

			for ptIdx := range originalPoints {
				if validSet[globalIdx] {
					cleanedPoints = append(cleanedPoints, originalPoints[ptIdx])
				}
				globalIdx++
			}

			originalGPX.Tracks[trackIdx].Segments[segIdx].Points = cleanedPoints
		}

		// Add "(Cleaned)" to track name
		if len(originalGPX.Tracks[trackIdx].Name) > 0 {
			originalGPX.Tracks[trackIdx].Name += " (Cleaned)"
		}
	}

	// Preserve original GPX metadata and namespaces
	originalGPX.Creator = "GPX Precision Cleaner v2.1"
	if originalGPX.XmlnsGpxtpx == "" {
		originalGPX.XmlnsGpxtpx = "http://www.garmin.com/xmlschemas/TrackPointExtension/v1"
	}

	output, err := xml.MarshalIndent(originalGPX, "", "  ")
	if err != nil {
		return err
	}

	header := `<?xml version="1.0" encoding="UTF-8"?>` + "\n"

	return os.WriteFile(outputFile, []byte(header+string(output)), 0o644)
}

func cleanGPX(cfg *config) error {
	inputPoints, err := parseGPX(cfg.InputFile)
	if err != nil {
		return fmt.Errorf("failed to parse input file: %w", err)
	}

	fmt.Printf("📊 Processing %d GPS points\n", len(inputPoints))

	// Apply elevation smoothing to reduce barometric noise before 3D calculations
	fmt.Printf("🏔️ Applying elevation smoothing (median filter, window=%d)...\n", eleWindow)
	smoothElevation(inputPoints, eleWindow)

	validIndices := advancedPrecisionFilter(inputPoints)

	originalDistance := calculateDistance(inputPoints)
	cleanedPoints := make([]point, len(validIndices))
	for i, idx := range validIndices {
		cleanedPoints[i] = inputPoints[idx]
	}
	cleanedDistance := calculateDistance(cleanedPoints)

	if err := writeCleanedGPX(cfg.InputFile, cfg.OutputFile, validIndices); err != nil {
		return err
	}

	fmt.Printf("\n🏃 Track processed: %.1f km → %.1f km\n", originalDistance/1000, cleanedDistance/1000)

	removalPercentage := float64(len(inputPoints)-len(validIndices)) / float64(len(inputPoints)) * 100

	// Guard against divide-by-zero for degenerate tracks
	distanceReduction := 0.0
	if originalDistance > 0 {
		distanceReduction = (originalDistance - cleanedDistance) / originalDistance * 100
	}

	fmt.Printf("📋 Removed %d GPS outliers (%.1f%%)\n",
		len(inputPoints)-len(validIndices), removalPercentage)

	// Safety guardrails: warn about excessive filtering
	if removalPercentage > removedPctGuard {
		fmt.Printf("⚠️  WARNING: Removed >%.0f%% of points! Conservative ultra-trail thresholds exceeded.\n", removedPctGuard)
		fmt.Printf("    Track may have unusual GPS conditions. Verify results carefully.\n")
	}
	if originalDistance > 0 && distanceReduction > distDropGuard {
		fmt.Printf("⚠️  WARNING: Distance reduced by >%.0f%%! Verify results carefully.\n", distDropGuard)
		fmt.Printf("    This may indicate aggressive filtering on hairpin/switchback sections.\n")
	}

	outlierReduction := originalDistance - cleanedDistance
	if outlierReduction > 1000 { // More than 1km removed
		fmt.Printf("✅ Significant GPS errors removed (%.1f km phantom distance)\n", outlierReduction/1000)
	} else {
		fmt.Printf("✅ Track appears clean, minimal outliers detected\n")
	}

	fmt.Printf("📁 Saved: %s\n", cfg.OutputFile)

	return nil
}

func main() {
	if len(os.Args) < 2 || (os.Args[1] != "-i" && !strings.HasPrefix(os.Args[1], "--input")) {
		fmt.Println("usage: gfix -i /path/to/file.gpx")
		fmt.Println()
		fmt.Println("examples:")
		fmt.Println("  gfix -i track.gpx")
		fmt.Println("  gfix -i \"My Activity.gpx\"")
		fmt.Println("  ./gfix-linux-amd64 -i track.gpx")
		fmt.Println("  gfix-windows-amd64.exe -i \"C:\\Downloads\\track.gpx\"")
		os.Exit(2)
	}

	var in string
	if os.Args[1] == "-i" {
		if len(os.Args) < 3 {
			fmt.Println("missing file")
			os.Exit(2)
		}
		in = os.Args[2]
	} else {
		in = strings.TrimPrefix(os.Args[1], "--input=")
		if in == "" {
			fmt.Println("missing file")
			os.Exit(2)
		}
	}

	cfg := &config{InputFile: in}
	ext := filepath.Ext(in)
	base := strings.TrimSuffix(in, ext)
	cfg.OutputFile = base + "_cleaned" + ext
	cfg.Workers = runtime.NumCPU()

	if err := cleanGPX(cfg); err != nil {
		log.Fatal(err)
	}
}
