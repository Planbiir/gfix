package gpx

import (
	"strings"
	"testing"
)

func TestParsePreservesExtensions(t *testing.T) {
	const gpxContent = `<?xml version="1.0" encoding="UTF-8"?>
<gpx version="1.1" creator="test" xmlns:gpxtpx="http://www.garmin.com/xmlschemas/TrackPointExtension/v1">
	<trk>
		<trkseg>
			<trkpt lat="46.0" lon="7.0">
				<extensions>
					<gpxtpx:TrackPointExtension>
						<gpxtpx:hr>145</gpxtpx:hr>
					</gpxtpx:TrackPointExtension>
				</extensions>
			</trkpt>
		</trkseg>
	</trk>
</gpx>`

	gpxData, err := ParseReader(strings.NewReader(gpxContent))
	if err != nil {
		t.Fatalf("ParseReader failed: %v", err)
	}

	point := gpxData.Tracks[0].Segments[0].Points[0]
	if len(point.Extensions) == 0 {
		t.Fatalf("expected extensions to be preserved")
	}

	// Ensure we can roundtrip without dropping the extensions block
	var buf strings.Builder
	if err := gpxData.WriteToWriter(&buf); err != nil {
		t.Fatalf("WriteToWriter failed: %v", err)
	}

	if !strings.Contains(buf.String(), "TrackPointExtension") {
		t.Fatalf("expected TrackPointExtension to appear in marshalled GPX")
	}
}
