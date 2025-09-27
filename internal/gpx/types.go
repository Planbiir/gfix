package gpx

import (
	"encoding/xml"
	"time"
)

// RawXML preserves nested extension blocks without re-parsing them.
// We store the inner XML bytes verbatim so we can round-trip extensions
// emitted by other tools (Garmin, Strava, etc.).
type RawXML []byte

func (r RawXML) MarshalXML(e *xml.Encoder, start xml.StartElement) error {
	if len(r) == 0 {
		return nil
	}

	type inner struct {
		Content string `xml:",innerxml"`
	}

	return e.EncodeElement(inner{Content: string(r)}, start)
}

func (r *RawXML) UnmarshalXML(d *xml.Decoder, start xml.StartElement) error {
	type inner struct {
		Content string `xml:",innerxml"`
	}

	var data inner
	if err := d.DecodeElement(&data, &start); err != nil {
		return err
	}

	if len(data.Content) == 0 {
		*r = nil
		return nil
	}

	*r = append((*r)[:0], data.Content...)
	return nil
}

// Point represents a GPS track point with all metadata
type Point struct {
	Lat       float64   `xml:"lat,attr"`
	Lon       float64   `xml:"lon,attr"`
	Elevation float64   `xml:"ele,omitempty"`
	Time      time.Time `xml:"time,omitempty"`

	// Extensions (Garmin, Strava, etc.) - preserve as raw XML
	Extensions RawXML `xml:"extensions,omitempty"`

	// Internal tracking for multi-segment preservation
	TrackIdx, SegIdx, PtIdx int `xml:"-"`
}

// Track represents a GPX track with segments
type Track struct {
	Name        string         `xml:"name,omitempty"`
	Description string         `xml:"desc,omitempty"`
	Segments    []TrackSegment `xml:"trkseg"`
	Extensions  RawXML         `xml:"extensions,omitempty"`
}

// TrackSegment represents a track segment
type TrackSegment struct {
	Points     []Point `xml:"trkpt"`
	Extensions RawXML  `xml:"extensions,omitempty"`
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
	Extensions RawXML   `xml:"extensions,omitempty"`
}

// Metadata represents GPX metadata
type Metadata struct {
	Name        string    `xml:"name,omitempty"`
	Description string    `xml:"desc,omitempty"`
	Author      string    `xml:"author,omitempty"`
	Time        time.Time `xml:"time,omitempty"`
	Extensions  RawXML    `xml:"extensions,omitempty"`
}
