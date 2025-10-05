package merge

import (
	"testing"
	"time"

	"github.com/planbiir/gfix/internal/gpx"
)

// TestMergeNoZeroCoordinates verifies that merged tracks don't contain points with zero lat/lon
// This test catches the critical bug where segment copying creates zero-valued points
func TestMergeNoZeroCoordinates(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	// Create primary track with a large gap
	primary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0001, Lon: 7.0001, Time: base.Add(30 * time.Second)},
		{Lat: 46.0002, Lon: 7.0002, Time: base.Add(10 * time.Minute)}, // Large gap after this
	})

	// Create secondary track with many points to trigger the copy bug
	var secondaryPoints []gpx.Point
	for i := 0; i < 1000; i++ {
		lat := 46.0001 + float64(i)*0.0001
		lon := 7.0001 + float64(i)*0.0001
		secondaryPoints = append(secondaryPoints, gpx.Point{Lat: lat, Lon: lon})
	}
	secondary := buildSingleTrack(secondaryPoints)

	merged, _, err := MergeTracks(primary, secondary, Config{
		GapThreshold:          time.Minute,
		MaxDeviationMeters:    1000, // Large to allow matching
		EnableSpatialFallback: true,
	})
	if err != nil {
		t.Fatalf("MergeTracks failed: %v", err)
	}

	// Check that no point has zero coordinates
	mergedPoints := merged.FlattenPoints()
	for i, pt := range mergedPoints {
		if pt.Lat == 0.0 && pt.Lon == 0.0 {
			t.Fatalf("Found zero coordinates at index %d: %+v", i, pt)
		}
		if pt.Lat == 0.0 || pt.Lon == 0.0 {
			t.Fatalf("Found zero coordinate (lat or lon) at index %d: %+v", i, pt)
		}
	}

	t.Logf("Verified %d merged points have no zero coordinates", len(mergedPoints))
}

// TestStationaryContextDetection tests that estimateExpectedDistanceMeters 
// correctly handles stationary contexts by returning 0 (disabling validation)
func TestStationaryContextDetection(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	// Create truly stationary context (athlete not moving)
	stationaryContext := []gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0, Lon: 7.0, Time: base.Add(30 * time.Second)},
		{Lat: 46.0, Lon: 7.0, Time: base.Add(60 * time.Second)}, // no movement at all
	}

	// Create fast moving context (> MinReliableSpeedMPS)
	movingContext := []gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.01, Lon: 7.01, Time: base.Add(30 * time.Second)}, // fast movement
		{Lat: 46.02, Lon: 7.02, Time: base.Add(60 * time.Second)},
	}

	cfg := DefaultConfig()
	gap := 25 * time.Minute

	// Test: both contexts stationary -> should return 0 (disable validation)
	expectedDist1 := estimateExpectedDistanceMeters(stationaryContext, stationaryContext, gap, cfg)
	if expectedDist1 != 0 {
		t.Errorf("Expected 0 for stationary contexts, got %f", expectedDist1)
	}

	// Test: one stationary, one moving -> should use the moving one
	expectedDist2 := estimateExpectedDistanceMeters(stationaryContext, movingContext, gap, cfg)
	
	// Debug the speeds
	stationarySpeed, _ := averageSpeed(stationaryContext)
	movingSpeed, _ := averageSpeed(movingContext)
	t.Logf("Stationary speed: %.4f m/s, Moving speed: %.4f m/s", stationarySpeed, movingSpeed)
	
	if expectedDist2 == 0 {
		t.Error("Expected non-zero when one context is moving")
	}

	// Test: both moving -> should compute normally
	expectedDist3 := estimateExpectedDistanceMeters(movingContext, movingContext, gap, cfg)
	if expectedDist3 == 0 {
		t.Error("Expected non-zero when both contexts are moving")
	}

	t.Logf("Stationary+Stationary: %.2fm", expectedDist1)
	t.Logf("Stationary+Moving: %.2fm", expectedDist2)
	t.Logf("Moving+Moving: %.2fm", expectedDist3)
}