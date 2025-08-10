package main

import (
	"math"
	"testing"
	"time"
)

func TestHaversineDistance(t *testing.T) {
	// Test known distance between two points
	lat1, lon1 := 46.0, 7.0 // Approximate coordinates
	lat2, lon2 := 46.1, 7.0 // 0.1 degree north

	dist := haversineDistance(lat1, lon1, lat2, lon2)

	// 0.1 degree ≈ 11.1 km at this latitude
	expected := 11100.0 // meters
	tolerance := 500.0  // 500m tolerance

	if math.Abs(dist-expected) > tolerance {
		t.Errorf("Haversine distance incorrect: got %.0fm, expected ~%.0fm", dist, expected)
	}
}

func TestCalculateDistance(t *testing.T) {
	// Test with simple 3-point track
	points := []point{
		{Lat: 46.0, Lon: 7.0, Elevation: 1000},
		{Lat: 46.01, Lon: 7.0, Elevation: 1100},  // ~1.1km north
		{Lat: 46.01, Lon: 7.01, Elevation: 1200}, // ~1.1km east
	}

	totalDist := calculateDistance(points)

	// Should be roughly 2.2km
	expected := 2200.0
	tolerance := 500.0

	if math.Abs(totalDist-expected) > tolerance {
		t.Errorf("Total distance incorrect: got %.0fm, expected ~%.0fm", totalDist, expected)
	}
}

func TestLOFWithOutliers(t *testing.T) {
	// Test LOF algorithm with some clear outliers
	inputPoints := make([]point, 20)

	// Create a linear track
	for i := 0; i < 18; i++ {
		inputPoints[i] = point{
			Lat:       46.0 + float64(i)*0.0001,
			Lon:       7.0 + float64(i)*0.0001,
			Elevation: 1000 + float64(i)*5,
		}
	}

	// Add 2 clear outliers
	inputPoints[18] = point{Lat: 46.1, Lon: 7.1, Elevation: 2000} // Far away
	inputPoints[19] = point{Lat: 45.9, Lon: 6.9, Elevation: 500}  // Far away

	k := chooseK(len(inputPoints))
	result := lofOutlierDetection(inputPoints, k)

	// Should detect and remove the outliers
	removedCount := len(inputPoints) - len(result)
	if removedCount < 1 {
		t.Errorf("LOF should detect outliers: removed only %d points", removedCount)
	}

	// But should keep most of the clean track
	if len(result) < 15 {
		t.Errorf("LOF removed too many points: only %d left from %d", len(result), len(inputPoints))
	}

	// Verify that the outliers (indices 18, 19) were specifically removed
	outlierIndicesFound := make(map[int]bool)
	for _, idx := range result {
		if idx == 18 || idx == 19 {
			outlierIndicesFound[idx] = true
		}
	}

	if outlierIndicesFound[18] || outlierIndicesFound[19] {
		t.Errorf("LOF failed to remove outliers: outlier indices %v still present", outlierIndicesFound)
	}
}

func TestLOFNotOverlyAggressive(t *testing.T) {
	// Test that LOF filtering doesn't remove too many points from a clean track
	inputPoints := make([]point, 100)

	// Create a simple track with mostly good points
	for i := range inputPoints {
		inputPoints[i] = point{
			Lat:       46.0 + float64(i)*0.0001, // Moving north
			Lon:       7.0 + float64(i)*0.0001,  // Moving east
			Elevation: 1000 + float64(i),        // Gradual climb
		}
	}

	// Test LOF algorithm
	k := chooseK(len(inputPoints))
	result := lofOutlierDetection(inputPoints, k)

	// Should not remove more than 10% of points for a clean track
	removedPercent := float64(len(inputPoints)-len(result)) / float64(len(inputPoints)) * 100

	if removedPercent > 10.0 {
		t.Errorf("LOF filter too aggressive: removed %.1f%% of points from clean track", removedPercent)
	}

	if len(result) < 80 {
		t.Errorf("LOF filter removed too many points: only %d left from %d", len(result), len(inputPoints))
	}
}

func TestAdvancedFilterNotTooAggressive(t *testing.T) {
	// Test that advanced filter works with reasonable GPS track
	inputPoints := []point{
		{Lat: 46.0, Lon: 7.0, Elevation: 1000},
		{Lat: 46.0001, Lon: 7.0001, Elevation: 1005}, // ~15m
		{Lat: 46.0002, Lon: 7.0002, Elevation: 1010}, // Normal progression
		{Lat: 46.0003, Lon: 7.0003, Elevation: 1015}, // Normal progression
		{Lat: 46.0004, Lon: 7.0004, Elevation: 1020}, // Normal progression
	}

	filtered := advancedPrecisionFilter(inputPoints)

	// Should keep most points for reasonable trail running track
	if len(filtered) < 4 {
		t.Errorf("Advanced filter too strict: only kept %d points from %d", len(filtered), len(inputPoints))
	}
}

func TestPausePreservation(t *testing.T) {
	// Test that pause regions (aid stations) are preserved from filtering
	baseTime := time.Now()

	// Create a track with a 3-4 minute pause in the middle (aid station scenario)
	inputPoints := []point{
		{Lat: 46.0, Lon: 7.0, Elevation: 1000, Time: baseTime},
		{Lat: 46.0001, Lon: 7.0001, Elevation: 1005, Time: baseTime.Add(10 * time.Second)}, // Moving
		{Lat: 46.0002, Lon: 7.0002, Elevation: 1010, Time: baseTime.Add(20 * time.Second)}, // Moving

		// Aid station pause: ~200 seconds (3+ minutes) near-static
		{Lat: 46.0002, Lon: 7.0002, Elevation: 1010, Time: baseTime.Add(30 * time.Second)},    // Stopped
		{Lat: 46.00021, Lon: 7.00021, Elevation: 1010, Time: baseTime.Add(60 * time.Second)},  // Very slow movement
		{Lat: 46.00022, Lon: 7.00022, Elevation: 1010, Time: baseTime.Add(120 * time.Second)}, // Very slow movement
		{Lat: 46.00023, Lon: 7.00023, Elevation: 1010, Time: baseTime.Add(180 * time.Second)}, // Very slow movement
		{Lat: 46.00024, Lon: 7.00024, Elevation: 1010, Time: baseTime.Add(230 * time.Second)}, // Leaving aid station

		{Lat: 46.0003, Lon: 7.0003, Elevation: 1015, Time: baseTime.Add(240 * time.Second)}, // Moving again
		{Lat: 46.0004, Lon: 7.0004, Elevation: 1020, Time: baseTime.Add(250 * time.Second)}, // Moving again
	}

	filtered := advancedPrecisionFilter(inputPoints)

	// Should preserve the pause region points (aid station)
	if len(filtered) < 8 {
		t.Errorf("Pause preservation failed: only kept %d points from %d, should preserve aid station", len(filtered), len(inputPoints))
	}

	// Verify that the pause region points are actually in the filtered result
	pausePointsPreserved := 0
	for _, idx := range filtered {
		// Check if this is one of the aid station points (indices 3-7)
		if idx >= 3 && idx <= 7 {
			pausePointsPreserved++
		}
	}

	if pausePointsPreserved < 3 {
		t.Errorf("Aid station points not preserved: only %d pause points kept from 5 expected", pausePointsPreserved)
	}
}
