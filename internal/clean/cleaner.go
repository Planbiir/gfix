package clean

import (
	"fmt"
	"math"
	"time"
)

// Clean performs GPS track cleaning with the given configuration
func Clean(points []Point, config Config) (CleaningResult, error) {
	if len(points) < 3 {
		return CleaningResult{
			Points: points,
			Stats: Stats{
				OriginalPoints: len(points),
				FinalPoints:    len(points),
			},
		}, nil
	}
	
	startTime := time.Now()
	
	// Calculate original distance for statistics
	originalDistance := calculateDistance(points)
	
	// Smooth elevation to reduce barometric noise
	smoothElevation(points, config.ElevationWindow)
	
	// Step 1: Velocity and geometric filtering
	fmt.Printf("🔬 GPS outlier detection using LOF...\n")
	fmt.Printf("   Hybrid GPS outlier detection (velocity + LOF)...\n")
	fmt.Printf("   Step 1: Adaptive velocity filter...\n")
	
	velocityFiltered := velocityOutlierFilter(points, config)
	fmt.Printf("   Removed %d obvious GPS jumps (%.1f%%) via velocity filter\n",
		len(points)-len(velocityFiltered),
		float64(len(points)-len(velocityFiltered))/float64(len(points))*100)
	
	// Extract filtered points for LOF
	filteredPoints := make([]Point, len(velocityFiltered))
	for i, idx := range velocityFiltered {
		filteredPoints[i] = points[idx]
	}
	
	// Step 2: LOF analysis
	fmt.Printf("   Step 2: LOF analysis on %d velocity-filtered points...\n", len(filteredPoints))
	
	k := chooseK(len(filteredPoints))
	lofFiltered := lofOutlierDetection(filteredPoints, k, config)
	
	// Map LOF results back to original indices
	finalIndices := make([]int, len(lofFiltered))
	for i, lofIdx := range lofFiltered {
		finalIndices[i] = velocityFiltered[lofIdx]
	}
	
	// Apply safety limits
	finalIndices = applySafetyLimits(points, finalIndices, velocityFiltered, config)
	
	// Extract final points
	finalPoints := make([]Point, len(finalIndices))
	for i, idx := range finalIndices {
		finalPoints[i] = points[idx]
	}
	
	// Calculate final statistics
	finalDistance := calculateDistance(finalPoints)
	processingTime := time.Since(startTime)
	
	// Detect activity type for stats
	activityType, detectedMaxSpeed, p95Speed := detectActivityType(points)
	
	stats := Stats{
		OriginalPoints:   len(points),
		OriginalDistance: originalDistance / 1000, // convert to km
		VelocityFiltered: len(velocityFiltered),
		LofFiltered:      len(lofFiltered),
		FinalPoints:      len(finalPoints),
		PointsRemoved:    len(points) - len(finalPoints),
		PointsPercent:    float64(len(points)-len(finalPoints)) / float64(len(points)) * 100,
		FinalDistance:    finalDistance / 1000, // convert to km
		DistanceReduced:  (originalDistance - finalDistance) / 1000, // convert to km
		DistancePercent:  (originalDistance - finalDistance) / originalDistance * 100,
		ProcessingTime:   processingTime,
		ActivityType:     activityType,
		DetectedMaxSpeed: detectedMaxSpeed,
		P95Speed:         p95Speed,
	}
	
	// Print summary
	fmt.Printf("📊 Cleaning completed: %d→%d points (%.1f%% removed), %.1f→%.1f km (%.1f%% reduced) in %v\n",
		len(points), len(finalPoints), stats.PointsPercent,
		stats.OriginalDistance, stats.FinalDistance, stats.DistancePercent,
		processingTime)
	
	return CleaningResult{
		Points: finalPoints,
		Stats:  stats,
	}, nil
}

// chooseK picks k for LOF based on track size
func chooseK(n int) int {
	k := n/1000 + 10 // base 10, +1 per 1000 points
	k = max(k, 10)   // lofMinK
	k = min(k, 20)   // lofMaxK
	return k
}

// applySafetyLimits ensures we don't remove too many points or distance
func applySafetyLimits(inputPoints []Point, finalIndices, velocityFiltered []int, config Config) []int {
	originalCount := len(inputPoints)
	finalCount := len(finalIndices)
	removalPercent := float64(originalCount-finalCount) / float64(originalCount) * 100
	
	if removalPercent > config.MaxRemovedPercent {
		fmt.Printf("   ⚠️  Safety override: Would remove %.1f%% > %.0f%% limit. Using velocity filter only.\n",
			removalPercent, config.MaxRemovedPercent)
		return velocityFiltered
	}
	
	// Check distance guardrail
	originalDistance := calculateDistance(inputPoints)
	finalPoints := make([]Point, len(finalIndices))
	for i, idx := range finalIndices {
		finalPoints[i] = inputPoints[idx]
	}
	finalDistance := calculateDistance(finalPoints)
	distanceReduction := 0.0
	if originalDistance > 0 {
		distanceReduction = (originalDistance - finalDistance) / originalDistance * 100
	}
	
	if distanceReduction > config.MaxDistanceReduced {
		fmt.Printf("   ⚠️  Safety override: Would drop %.1f%% > %.0f%% distance. Using velocity filter only.\n",
			distanceReduction, config.MaxDistanceReduced)
		return velocityFiltered
	}
	
	return finalIndices
}

// calculateDistance computes total 3D distance of a track
func calculateDistance(points []Point) float64 {
	if len(points) < 2 {
		return 0.0
	}
	
	var total float64
	for i := 1; i < len(points); i++ {
		total += haversineDistance3D(
			points[i-1].Lat, points[i-1].Lon, points[i-1].Elevation,
			points[i].Lat, points[i].Lon, points[i].Elevation)
	}
	return total
}

// haversineDistance3D calculates 3D distance between two points
func haversineDistance3D(lat1, lon1, ele1, lat2, lon2, ele2 float64) float64 {
	const earthRadius = 6371000 // meters
	
	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLat := (lat2 - lat1) * math.Pi / 180
	deltaLon := (lon2 - lon1) * math.Pi / 180
	
	a := math.Sin(deltaLat/2)*math.Sin(deltaLat/2) +
		math.Cos(lat1Rad)*math.Cos(lat2Rad)*
		math.Sin(deltaLon/2)*math.Sin(deltaLon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	
	horizontalDist := earthRadius * c
	verticalDist := math.Abs(ele2 - ele1)
	
	return math.Sqrt(horizontalDist*horizontalDist + verticalDist*verticalDist)
}