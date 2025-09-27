package merge

import (
	"testing"
	"time"

	"github.com/planbiir/gfix/internal/gpx"
)

func TestMergeTracksFillsGap(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	primary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0001, Lon: 7.0001, Time: base.Add(30 * time.Second)},
		{Lat: 46.0005, Lon: 7.0005, Time: base.Add(10 * time.Minute)},
	})

	secondary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0002, Lon: 7.0002, Time: base.Add(6 * time.Minute)},
		{Lat: 46.0003, Lon: 7.0003, Time: base.Add(7 * time.Minute)},
	})

	merged, stats, err := MergeTracks(primary, secondary, Config{
		GapThreshold:          time.Minute,
		MaxDeviationMeters:    100,
		EnableSpatialFallback: true,
	})
	if err != nil {
		t.Fatalf("MergeTracks failed: %v", err)
	}

	mergedPoints := merged.FlattenPoints()
	if len(mergedPoints) != 5 {
		t.Fatalf("expected 5 points after merge, got %d", len(mergedPoints))
	}

	if stats.GapsDetected != 1 {
		t.Fatalf("expected 1 detected gap, got %d", stats.GapsDetected)
	}
	if stats.GapsFilled != 1 {
		t.Fatalf("expected 1 filled gap, got %d", stats.GapsFilled)
	}
	if stats.InsertedPoints != 2 {
		t.Fatalf("expected 2 inserted points, got %d", stats.InsertedPoints)
	}

	// Ensure the inserted points fall between the surrounding timestamps.
	if !mergedPoints[2].Time.After(mergedPoints[1].Time) || !mergedPoints[2].Time.Before(mergedPoints[4].Time) {
		t.Fatalf("inserted point is not ordered between surrounding points")
	}
}

func TestMergeTracksSkipsTails(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	primary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0001, Lon: 7.0001, Time: base.Add(30 * time.Second)},
		{Lat: 46.0002, Lon: 7.0002, Time: base.Add(60 * time.Second)},
	})

	secondary := buildSingleTrack([]gpx.Point{
		{Lat: 45.9, Lon: 6.9, Time: base.Add(-2 * time.Minute)}, // before start
		{Lat: 46.1, Lon: 7.1, Time: base.Add(10 * time.Minute)}, // after end
	})

	merged, stats, err := MergeTracks(primary, secondary, Config{GapThreshold: 30 * time.Second})
	if err != nil {
		t.Fatalf("MergeTracks failed: %v", err)
	}

	mergedPoints := merged.FlattenPoints()
	if len(mergedPoints) != len(primary.FlattenPoints()) {
		t.Fatalf("expected no additional points, got %d", len(mergedPoints))
	}

	if stats.InsertedPoints != 0 {
		t.Fatalf("expected 0 inserted points, got %d", stats.InsertedPoints)
	}
}

func TestMergeTracksRespectsDeviation(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	primary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0001, Lon: 7.0001, Time: base.Add(30 * time.Second)},
		{Lat: 46.0002, Lon: 7.0002, Time: base.Add(6 * time.Minute)},
	})

	secondary := buildSingleTrack([]gpx.Point{
		{Lat: 47.0, Lon: 8.0, Time: base.Add(3 * time.Minute)}, // far away
	})

	merged, stats, err := MergeTracks(primary, secondary, Config{
		GapThreshold:          time.Minute,
		MaxDeviationMeters:    10,
		EnableSpatialFallback: false,
	})
	if err != nil {
		t.Fatalf("MergeTracks failed: %v", err)
	}

	mergedPoints := merged.FlattenPoints()
	if len(mergedPoints) != len(primary.FlattenPoints()) {
		t.Fatalf("expected no merge due to deviation, got %d points", len(mergedPoints))
	}

	if stats.GapsDetected != 1 {
		t.Fatalf("expected 1 detected gap, got %d", stats.GapsDetected)
	}
	if stats.GapsFilled != 0 {
		t.Fatalf("expected 0 filled gaps, got %d", stats.GapsFilled)
	}
	if stats.InsertedPoints != 0 {
		t.Fatalf("expected 0 inserted points, got %d", stats.InsertedPoints)
	}
}

func TestMergeTracksWithNoSecondaryCoverage(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	primary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0, Lon: 7.0, Time: base.Add(10 * time.Minute)},
	})

	secondary := buildSingleTrack([]gpx.Point{})

	merged, stats, err := MergeTracks(primary, secondary, Config{})
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	if len(merged.FlattenPoints()) != len(primary.FlattenPoints()) {
		t.Fatalf("expected primary track to remain unchanged")
	}

	if stats.InsertedPoints != 0 {
		t.Fatalf("expected zero inserted points, got %d", stats.InsertedPoints)
	}
}

func TestMergeTracksSpatialFallback(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	primary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0001, Lon: 7.0001, Time: base.Add(30 * time.Second)},
		{Lat: 46.0002, Lon: 7.0002, Time: base.Add(60 * time.Second)},
		{Lat: 46.0003, Lon: 7.0003, Time: base.Add(12 * time.Minute)},
		{Lat: 46.0004, Lon: 7.0004, Time: base.Add(12*time.Minute + 30*time.Second)},
	})

	secondaryPoints := []gpx.Point{
		{Lat: 46.0, Lon: 7.0},
		{Lat: 46.00005, Lon: 7.00005},
		{Lat: 46.0001, Lon: 7.0001},
		{Lat: 46.00015, Lon: 7.00015},
		{Lat: 46.0002, Lon: 7.0002},
		{Lat: 46.00025, Lon: 7.00025},
		{Lat: 46.0003, Lon: 7.0003},
		{Lat: 46.00035, Lon: 7.00035},
		{Lat: 46.0004, Lon: 7.0004},
	}

	secondary := buildSingleTrack(secondaryPoints)

	merged, stats, err := MergeTracks(primary, secondary, Config{
		GapThreshold:           2 * time.Minute,
		EnableSpatialFallback:  true,
		SpatialWindowSize:      3,
		SpatialMaxAvgDeviation: 10,
	})
	if err != nil {
		t.Fatalf("MergeTracks failed: %v", err)
	}

	if stats.GapsDetected != 1 {
		t.Fatalf("expected 1 detected gap, got %d", stats.GapsDetected)
	}
	if stats.SpatialMatches != 1 {
		t.Fatalf("expected spatial fallback to run once, got %d", stats.SpatialMatches)
	}
	if stats.InsertedPoints == 0 {
		t.Fatalf("expected gap to be filled by spatial fallback")
	}

	mergedPoints := merged.FlattenPoints()
	if len(mergedPoints) <= len(primary.FlattenPoints()) {
		t.Fatalf("expected additional points after spatial merge")
	}

	gapStart := primary.FlattenPoints()[2].Time
	gapEnd := primary.FlattenPoints()[3].Time
	for _, pt := range mergedPoints {
		if pt.Time.After(gapStart) && pt.Time.Before(gapEnd) && pt.Time.IsZero() {
			t.Fatalf("expected interpolated timestamps for inserted points")
		}
	}
}

func TestMergeTracksSpatialFallbackNoMatch(t *testing.T) {
	base := time.Date(2025, 1, 1, 7, 0, 0, 0, time.UTC)

	primary := buildSingleTrack([]gpx.Point{
		{Lat: 46.0, Lon: 7.0, Time: base},
		{Lat: 46.0001, Lon: 7.0001, Time: base.Add(30 * time.Second)},
		{Lat: 46.0002, Lon: 7.0002, Time: base.Add(60 * time.Second)},
		{Lat: 46.0003, Lon: 7.0003, Time: base.Add(12 * time.Minute)},
	})

	secondary := buildSingleTrack([]gpx.Point{
		{Lat: 47.0, Lon: 8.0},
		{Lat: 47.1, Lon: 8.1},
		{Lat: 47.2, Lon: 8.2},
	})

	merged, stats, err := MergeTracks(primary, secondary, Config{
		GapThreshold:           2 * time.Minute,
		EnableSpatialFallback:  true,
		SpatialWindowSize:      3,
		SpatialMaxAvgDeviation: 5,
	})
	if err != nil {
		t.Fatalf("MergeTracks failed: %v", err)
	}

	if stats.InsertedPoints != 0 {
		t.Fatalf("expected no inserted points when spatial match fails")
	}

	if stats.SpatialMatches != 0 {
		t.Fatalf("expected zero spatial matches")
	}

	if len(merged.FlattenPoints()) != len(primary.FlattenPoints()) {
		t.Fatalf("expected merged track to equal primary")
	}
}

func buildSingleTrack(points []gpx.Point) *gpx.GPX {
	track := gpx.Track{
		Segments: []gpx.TrackSegment{
			{Points: points},
		},
	}

	g := &gpx.GPX{
		Tracks: []gpx.Track{track},
	}

	return g
}
