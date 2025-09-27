package gpx

import (
	"strings"
	"testing"
	"time"
)

func TestParseReader(t *testing.T) {
	gpxContent := `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="test">
	<trk>
		<name>Test Track</name>
		<trkseg>
			<trkpt lat="46.0" lon="7.0">
				<ele>1000</ele>
				<time>2025-01-01T10:00:00Z</time>
			</trkpt>
			<trkpt lat="46.001" lon="7.001">
				<ele>1005</ele>
				<time>2025-01-01T10:00:01Z</time>
			</trkpt>
		</trkseg>
	</trk>
</gpx>`

	reader := strings.NewReader(gpxContent)
	gpxData, err := ParseReader(reader)
	
	if err != nil {
		t.Fatalf("ParseReader failed: %v", err)
	}
	
	if len(gpxData.Tracks) != 1 {
		t.Errorf("Expected 1 track, got %d", len(gpxData.Tracks))
	}
	
	if len(gpxData.Tracks[0].Segments) != 1 {
		t.Errorf("Expected 1 segment, got %d", len(gpxData.Tracks[0].Segments))
	}
	
	if len(gpxData.Tracks[0].Segments[0].Points) != 2 {
		t.Errorf("Expected 2 points, got %d", len(gpxData.Tracks[0].Segments[0].Points))
	}
	
	// Check first point
	point := gpxData.Tracks[0].Segments[0].Points[0]
	if point.Lat != 46.0 || point.Lon != 7.0 {
		t.Errorf("Expected lat=46.0, lon=7.0, got lat=%f, lon=%f", point.Lat, point.Lon)
	}
	
	if point.Elevation != 1000.0 {
		t.Errorf("Expected elevation=1000.0, got %f", point.Elevation)
	}
}

func TestFlattenPoints(t *testing.T) {
	gpx := &GPX{
		Tracks: []Track{
			{
				Segments: []TrackSegment{
					{
						Points: []Point{
							{Lat: 46.0, Lon: 7.0},
							{Lat: 46.001, Lon: 7.001},
						},
					},
					{
						Points: []Point{
							{Lat: 46.002, Lon: 7.002},
						},
					},
				},
			},
		},
	}
	
	points := gpx.FlattenPoints()
	
	if len(points) != 3 {
		t.Errorf("Expected 3 flattened points, got %d", len(points))
	}
	
	// Check indices are set correctly
	if points[0].TrackIdx != 0 || points[0].SegIdx != 0 || points[0].PtIdx != 0 {
		t.Errorf("Point 0 indices incorrect: track=%d, seg=%d, pt=%d", 
			points[0].TrackIdx, points[0].SegIdx, points[0].PtIdx)
	}
	
	if points[2].TrackIdx != 0 || points[2].SegIdx != 1 || points[2].PtIdx != 0 {
		t.Errorf("Point 2 indices incorrect: track=%d, seg=%d, pt=%d", 
			points[2].TrackIdx, points[2].SegIdx, points[2].PtIdx)
	}
}

func TestRebuildFromPoints(t *testing.T) {
	gpx := &GPX{
		Tracks: []Track{
			{
				Name: "Test Track",
				Segments: []TrackSegment{
					{
						Points: []Point{
							{Lat: 46.0, Lon: 7.0, TrackIdx: 0, SegIdx: 0, PtIdx: 0},
							{Lat: 46.001, Lon: 7.001, TrackIdx: 0, SegIdx: 0, PtIdx: 1},
							{Lat: 46.002, Lon: 7.002, TrackIdx: 0, SegIdx: 0, PtIdx: 2},
						},
					},
				},
			},
		},
	}
	
	// Simulate filtering that removes the middle point
	filteredPoints := []Point{
		{Lat: 46.0, Lon: 7.0, TrackIdx: 0, SegIdx: 0, PtIdx: 0},
		{Lat: 46.002, Lon: 7.002, TrackIdx: 0, SegIdx: 0, PtIdx: 2},
	}
	
	gpx.RebuildFromPoints(filteredPoints)
	
	if len(gpx.Tracks) != 1 {
		t.Errorf("Expected 1 track after rebuild, got %d", len(gpx.Tracks))
	}
	
	if len(gpx.Tracks[0].Segments) != 1 {
		t.Errorf("Expected 1 segment after rebuild, got %d", len(gpx.Tracks[0].Segments))
	}
	
	if len(gpx.Tracks[0].Segments[0].Points) != 2 {
		t.Errorf("Expected 2 points after rebuild, got %d", len(gpx.Tracks[0].Segments[0].Points))
	}
	
	// Check that track name is preserved
	if gpx.Tracks[0].Name != "Test Track" {
		t.Errorf("Expected track name 'Test Track', got '%s'", gpx.Tracks[0].Name)
	}
}

func TestStats(t *testing.T) {
	baseTime := time.Date(2025, 1, 1, 10, 0, 0, 0, time.UTC)
	
	gpx := &GPX{
		Tracks: []Track{
			{
				Segments: []TrackSegment{
					{
						Points: []Point{
							{Lat: 46.0, Lon: 7.0, Time: baseTime},
							{Lat: 46.001, Lon: 7.001, Time: baseTime.Add(time.Minute)},
							{Lat: 46.002, Lon: 7.002, Time: baseTime.Add(2 * time.Minute)},
						},
					},
				},
			},
		},
	}
	
	pointCount, trackCount, segmentCount, duration, distance := gpx.Stats()
	
	if pointCount != 3 {
		t.Errorf("Expected 3 points, got %d", pointCount)
	}
	
	if trackCount != 1 {
		t.Errorf("Expected 1 track, got %d", trackCount)
	}
	
	if segmentCount != 1 {
		t.Errorf("Expected 1 segment, got %d", segmentCount)
	}
	
	if duration != 2*time.Minute {
		t.Errorf("Expected 2 minutes duration, got %v", duration)
	}
	
	if distance <= 0 {
		t.Errorf("Expected positive distance, got %f", distance)
	}
}