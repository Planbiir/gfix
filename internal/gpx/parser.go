package gpx

import (
	"encoding/xml"
	"fmt"
	"io"
	"math"
	"os"
	"time"
)

// Parse reads and parses a GPX file, preserving all extensions and namespaces
func Parse(filename string) (*GPX, error) {
	file, err := os.Open(filename)
	if err != nil {
		return nil, fmt.Errorf("failed to open file: %w", err)
	}
	defer file.Close()
	
	return ParseReader(file)
}

// ParseReader parses GPX from an io.Reader
func ParseReader(r io.Reader) (*GPX, error) {
	decoder := xml.NewDecoder(r)
	
	var gpxData GPX
	if err := decoder.Decode(&gpxData); err != nil {
		return nil, fmt.Errorf("failed to parse GPX: %w", err)
	}
	
	// Set default namespaces if missing
	if gpxData.XMLNS == "" {
		gpxData.XMLNS = "http://www.topografix.com/GPX/1/1"
	}
	if gpxData.Version == "" {
		gpxData.Version = "1.1"
	}
	if gpxData.Creator == "" {
		gpxData.Creator = "gfix"
	}
	
	// Add internal indices for multi-segment preservation
	for trackIdx, track := range gpxData.Tracks {
		for segIdx, segment := range track.Segments {
			for ptIdx := range segment.Points {
				gpxData.Tracks[trackIdx].Segments[segIdx].Points[ptIdx].TrackIdx = trackIdx
				gpxData.Tracks[trackIdx].Segments[segIdx].Points[ptIdx].SegIdx = segIdx
				gpxData.Tracks[trackIdx].Segments[segIdx].Points[ptIdx].PtIdx = ptIdx
			}
		}
	}
	
	return &gpxData, nil
}

// Write saves GPX data to a file, preserving all extensions and structure
func (g *GPX) Write(filename string) error {
	file, err := os.Create(filename)
	if err != nil {
		return fmt.Errorf("failed to create file: %w", err)
	}
	defer file.Close()
	
	return g.WriteToWriter(file)
}

// WriteToWriter writes GPX data to an io.Writer
func (g *GPX) WriteToWriter(w io.Writer) error {
	// Write XML header
	if _, err := w.Write([]byte(xml.Header)); err != nil {
		return err
	}
	
	encoder := xml.NewEncoder(w)
	encoder.Indent("", "  ")
	
	if err := encoder.Encode(g); err != nil {
		return fmt.Errorf("failed to encode GPX: %w", err)
	}
	
	return nil
}

// FlattenPoints returns all points from all tracks and segments in order
func (g *GPX) FlattenPoints() []Point {
	var points []Point
	
	for trackIdx, track := range g.Tracks {
		for segIdx, segment := range track.Segments {
			for ptIdx, point := range segment.Points {
				point.TrackIdx = trackIdx
				point.SegIdx = segIdx
				point.PtIdx = ptIdx
				points = append(points, point)
			}
		}
	}
	
	return points
}

// RebuildFromPoints reconstructs GPX structure from a flat list of filtered points
func (g *GPX) RebuildFromPoints(filteredPoints []Point) {
	// Group points back into their original track/segment structure
	trackMap := make(map[int]map[int][]Point)
	
	for _, point := range filteredPoints {
		if trackMap[point.TrackIdx] == nil {
			trackMap[point.TrackIdx] = make(map[int][]Point)
		}
		trackMap[point.TrackIdx][point.SegIdx] = append(trackMap[point.TrackIdx][point.SegIdx], point)
	}
	
	// Rebuild tracks and segments
	var newTracks []Track
	for trackIdx := 0; trackIdx < len(g.Tracks); trackIdx++ {
		if segmentMap, exists := trackMap[trackIdx]; exists {
			var newSegments []TrackSegment
			
			for segIdx := 0; segIdx < len(g.Tracks[trackIdx].Segments); segIdx++ {
				if points, exists := segmentMap[segIdx]; exists && len(points) > 0 {
					newSegments = append(newSegments, TrackSegment{
						Points:     points,
						Extensions: g.Tracks[trackIdx].Segments[segIdx].Extensions,
					})
				}
			}
			
			if len(newSegments) > 0 {
				newTracks = append(newTracks, Track{
					Name:        g.Tracks[trackIdx].Name,
					Description: g.Tracks[trackIdx].Description,
					Segments:    newSegments,
					Extensions:  g.Tracks[trackIdx].Extensions,
				})
			}
		}
	}
	
	g.Tracks = newTracks
}

// Stats returns basic statistics about the GPX data
func (g *GPX) Stats() (pointCount int, trackCount int, segmentCount int, duration time.Duration, distance float64) {
	points := g.FlattenPoints()
	pointCount = len(points)
	trackCount = len(g.Tracks)
	
	for _, track := range g.Tracks {
		segmentCount += len(track.Segments)
	}
	
	if len(points) >= 2 {
		duration = points[len(points)-1].Time.Sub(points[0].Time)
		// Basic distance calculation (can be enhanced with haversine)
		for i := 1; i < len(points); i++ {
			distance += basicDistance(points[i-1], points[i])
		}
	}
	
	return
}

// basicDistance calculates rough distance between two points (km)
func basicDistance(p1, p2 Point) float64 {
	const earthRadius = 6371.0 // km
	
	lat1Rad := p1.Lat * math.Pi / 180
	lat2Rad := p2.Lat * math.Pi / 180
	deltaLat := (p2.Lat - p1.Lat) * math.Pi / 180
	deltaLon := (p2.Lon - p1.Lon) * math.Pi / 180
	
	a := math.Sin(deltaLat/2)*math.Sin(deltaLat/2) +
		math.Cos(lat1Rad)*math.Cos(lat2Rad)*
		math.Sin(deltaLon/2)*math.Sin(deltaLon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))
	
	return earthRadius * c
}