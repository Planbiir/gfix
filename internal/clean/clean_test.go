package clean

import (
	"testing"
	"time"
)

func TestClean(t *testing.T) {
	// Create a simple test track
	points := []Point{
		{Lat: 46.0, Lon: 7.0, Elevation: 1000, Time: time.Now()},
		{Lat: 46.0001, Lon: 7.0001, Elevation: 1005, Time: time.Now().Add(time.Second)},
		{Lat: 46.0002, Lon: 7.0002, Elevation: 1010, Time: time.Now().Add(2 * time.Second)},
		{Lat: 46.0003, Lon: 7.0003, Elevation: 1015, Time: time.Now().Add(3 * time.Second)},
		{Lat: 46.0004, Lon: 7.0004, Elevation: 1020, Time: time.Now().Add(4 * time.Second)},
	}

	config := DefaultConfig()
	result, err := Clean(points, config)
	
	if err != nil {
		t.Fatalf("Clean failed: %v", err)
	}
	
	if len(result.Points) == 0 {
		t.Errorf("Clean removed all points")
	}
	
	if result.Stats.OriginalPoints != len(points) {
		t.Errorf("Expected %d original points, got %d", len(points), result.Stats.OriginalPoints)
	}
	
	if result.Stats.FinalPoints != len(result.Points) {
		t.Errorf("Expected %d final points, got %d", len(result.Points), result.Stats.FinalPoints)
	}
}

func TestLOFWithOutliers(t *testing.T) {
	// Test LOF with clear outliers
	inputPoints := make([]Point, 20)

	// Create a linear track
	for i := range 18 {
		inputPoints[i] = Point{
			Lat:       46.0 + float64(i)*0.0001,
			Lon:       7.0 + float64(i)*0.0001,
			Elevation: 1000 + float64(i)*5,
			Time:      time.Now().Add(time.Duration(i) * time.Second),
		}
	}

	// Add clear outliers
	inputPoints[18] = Point{Lat: 46.1, Lon: 7.1, Elevation: 2000, Time: time.Now().Add(18 * time.Second)} // Far outlier
	inputPoints[19] = Point{Lat: 46.2, Lon: 7.2, Elevation: 3000, Time: time.Now().Add(19 * time.Second)} // Another outlier

	// Test LOF
	k := chooseK(len(inputPoints))
	config := DefaultConfig()
	result := lofOutlierDetection(inputPoints, k, config)

	// Should detect and remove the 2 obvious outliers
	expectedOutliers := 2
	actualRemoved := len(inputPoints) - len(result)
	
	if actualRemoved != expectedOutliers {
		t.Errorf("Expected to remove %d outliers, but removed %d", expectedOutliers, actualRemoved)
	}
}

func TestHaversineDistance(t *testing.T) {
	// Test distance calculation between known points
	lat1, lon1 := 46.0, 7.0
	lat2, lon2 := 46.001, 7.001

	distance := haversineDistance(lat1, lon1, lat2, lon2)
	
	// Should be approximately 140 meters
	expected := 140.0
	tolerance := 10.0
	
	if distance < expected-tolerance || distance > expected+tolerance {
		t.Errorf("Expected distance ~%.0fm, got %.0fm", expected, distance)
	}
}

func TestCalculateDistance(t *testing.T) {
	points := []Point{
		{Lat: 46.0, Lon: 7.0, Elevation: 1000},
		{Lat: 46.001, Lon: 7.001, Elevation: 1000}, // ~140m
		{Lat: 46.002, Lon: 7.002, Elevation: 1000}, // ~140m more
	}

	totalDist := calculateDistance(points)
	expected := 280.0 // approximately 2 * 140m

	if totalDist < expected-20 || totalDist > expected+20 {
		t.Errorf("Total distance incorrect: got %.0fm, expected ~%.0fm", totalDist, expected)
	}
}

func TestDefaultConfig(t *testing.T) {
	config := DefaultConfig()
	
	if config.MinSpeed <= 0 {
		t.Errorf("MinSpeed should be > 0, got %f", config.MinSpeed)
	}
	
	if config.PauseSpeed <= 0 {
		t.Errorf("PauseSpeed should be > 0, got %f", config.PauseSpeed)
	}
	
	if config.LofMinK <= 0 {
		t.Errorf("LofMinK should be > 0, got %d", config.LofMinK)
	}
}