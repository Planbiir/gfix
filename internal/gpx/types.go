package gpx

import (
	"encoding/xml"
	"time"
)

// Point represents a GPS track point with all metadata
type Point struct {
	Lat       float64   `xml:"lat,attr"`
	Lon       float64   `xml:"lon,attr"`
	Elevation float64   `xml:"ele,omitempty"`
	Time      time.Time `xml:"time,omitempty"`
	
	// Extensions (Garmin, Strava, etc.) - preserve as raw XML
	Extensions []byte `xml:"extensions,omitempty"`
	
	// Internal tracking for multi-segment preservation
	TrackIdx, SegIdx, PtIdx int `xml:"-"`
}

// Track represents a GPX track with segments
type Track struct {
	Name        string         `xml:"name,omitempty"`
	Description string         `xml:"desc,omitempty"`
	Segments    []TrackSegment `xml:"trkseg"`
	Extensions  []byte `xml:"extensions,omitempty"`
}

// TrackSegment represents a track segment
type TrackSegment struct {
	Points     []Point        `xml:"trkpt"`
	Extensions []byte `xml:"extensions,omitempty"`
}

// GPX represents the full GPX file structure
type GPX struct {
	XMLName xml.Name `xml:"gpx"`
	Version string   `xml:"version,attr"`
	Creator string   `xml:"creator,attr"`
	
	// Namespaces - preserve all original namespaces
	XMLNS    string `xml:"xmlns,attr,omitempty"`
	XMLNSXSI string `xml:"xmlns:xsi,attr,omitempty"`
	XSI      string `xml:"xsi:schemaLocation,attr,omitempty"`
	
	// Garmin/Strava specific namespaces
	XMLNSGPXTPX string `xml:"xmlns:gpxtpx,attr,omitempty"`
	XMLNSGPXX   string `xml:"xmlns:gpxx,attr,omitempty"`
	
	Metadata   Metadata `xml:"metadata,omitempty"`
	Tracks     []Track  `xml:"trk"`
	Extensions []byte `xml:"extensions,omitempty"`
}

// Metadata represents GPX metadata
type Metadata struct {
	Name        string         `xml:"name,omitempty"`
	Description string         `xml:"desc,omitempty"`
	Author      string         `xml:"author,omitempty"`
	Time        time.Time      `xml:"time,omitempty"`
	Extensions  []byte `xml:"extensions,omitempty"`
}