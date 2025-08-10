package main

import (
	"fmt"
	"path/filepath"
	"testing"
	"time"
)

// Benchmark performance with real Monte Rosa track
func BenchmarkMonteRosaTrack(b *testing.B) {
	trackPath := "/Users/timofeipermiakov/scripts/test/Tim_Monte_Rosa_Trail_2025.gpx"

	// Load the track once
	inputPoints, err := parseGPX(trackPath)
	if err != nil {
		b.Skipf("Could not load Monte Rosa track: %v", err)
	}

	if len(inputPoints) == 0 {
		b.Skip("Monte Rosa track is empty")
	}

	b.Logf("Monte Rosa track: %d GPS points", len(inputPoints))

	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		validIndices := advancedPrecisionFilter(inputPoints)

		if len(validIndices) == 0 {
			b.Fatal("Algorithm removed all points")
		}

		// Calculate results for reporting
		originalDistance := calculateDistance(inputPoints) / 1000
		cleanedPoints := extractPoints(inputPoints, validIndices)
		cleanedDistance := calculateDistance(cleanedPoints) / 1000
		removedPercent := float64(len(inputPoints)-len(validIndices)) / float64(len(inputPoints)) * 100

		b.Logf("Result: %.1f km → %.1f km (%.1f%% points removed)",
			originalDistance, cleanedDistance, removedPercent)
	}
}

// Benchmark LOF algorithm performance with different track sizes
func BenchmarkLOFSizes(b *testing.B) {
	sizes := []int{1000, 5000, 10000, 20000}

	for _, size := range sizes {
		b.Run(fmt.Sprintf("LOF-%d-points", size), func(b *testing.B) {
			// Generate synthetic track
			points := make([]point, size)
			for i := range points {
				points[i] = point{
					Lat:       46.0 + float64(i)*0.0001,
					Lon:       7.0 + float64(i)*0.0001,
					Elevation: 1000 + float64(i)*0.5,
				}
			}

			// Add some outliers (5% of points)
			outlierCount := size / 20
			for i := 0; i < outlierCount; i++ {
				idx := i * (size / outlierCount)
				if idx < len(points) {
					points[idx].Lat += 0.001 // Move 100m away
					points[idx].Lon += 0.001
				}
			}

			b.ResetTimer()

			for i := 0; i < b.N; i++ {
				result := fastLOFDetection(points)
				if len(result) == 0 {
					b.Fatal("LOF removed all points")
				}
			}
		})
	}
}

// Test accuracy with reference tracks vs standalone
func TestAccuracyComparison(t *testing.T) {
	trackPath := "/Users/timofeipermiakov/scripts/test/Tim_Monte_Rosa_Trail_2025.gpx"

	inputPoints, err := parseGPX(trackPath)
	if err != nil {
		t.Skipf("Could not load Monte Rosa track: %v", err)
	}

	if len(inputPoints) == 0 {
		t.Skip("Monte Rosa track is empty")
	}

	cfg := &config{
		InputFile: trackPath,
		Workers:   4,
	}

	t.Logf("Testing accuracy with %d GPS points", len(inputPoints))

	// Test with reference tracks
	start := time.Now()
	withRefsIndices := advancedPrecisionFilter(inputPoints)
	withRefsDuration := time.Since(start)

	// Test standalone (simulate no reference tracks by using different directory)
	cfg.InputFile = filepath.Join("/tmp", filepath.Base(trackPath))
	start = time.Now()
	standaloneIndices := fastLOFDetection(inputPoints)
	standaloneDuration := time.Since(start)

	// Calculate metrics
	originalDistance := calculateDistance(inputPoints) / 1000

	withRefsPoints := extractPoints(inputPoints, withRefsIndices)
	withRefsDistance := calculateDistance(withRefsPoints) / 1000
	withRefsRemoved := float64(len(inputPoints)-len(withRefsIndices)) / float64(len(inputPoints)) * 100

	standalonePoints := extractPoints(inputPoints, standaloneIndices)
	standaloneDistance := calculateDistance(standalonePoints) / 1000
	standaloneRemoved := float64(len(inputPoints)-len(standaloneIndices)) / float64(len(inputPoints)) * 100

	t.Logf("Original track: %.1f km", originalDistance)
	t.Logf("With reference tracks: %.1f km (%.1f%% removed) in %v",
		withRefsDistance, withRefsRemoved, withRefsDuration)
	t.Logf("Standalone LOF: %.1f km (%.1f%% removed) in %v",
		standaloneDistance, standaloneRemoved, standaloneDuration)

	// Accuracy assertions
	if withRefsRemoved > 15.0 {
		t.Errorf("Reference-guided method too aggressive: %.1f%% removed", withRefsRemoved)
	}

	if standaloneRemoved > 20.0 {
		t.Errorf("Standalone LOF too aggressive: %.1f%% removed", standaloneRemoved)
	}

	if withRefsDuration > 30*time.Second {
		t.Errorf("Reference-guided method too slow: %v", withRefsDuration)
	}

	if standaloneDuration > 30*time.Second {
		t.Errorf("Standalone LOF too slow: %v", standaloneDuration)
	}
}

// Reference track loading test removed - using LOF-only approach
