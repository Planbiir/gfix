package clean

import (
	"time"
)

// Point represents a GPS point for cleaning algorithms
type Point struct {
	Lat       float64
	Lon       float64
	Elevation float64
	Time      time.Time
	
	// Original indices for reconstruction
	TrackIdx, SegIdx, PtIdx int
}

// Config holds cleaning algorithm parameters
type Config struct {
	// Speed thresholds
	MinSpeed      float64 // m/s - minimum valid speed
	MaxSpeed      float64 // m/s - maximum valid speed (auto-detected if 0)
	
	// Pause detection
	PauseSpeed       float64 // m/s - speed threshold for pauses
	MinPauseDuration float64 // seconds - minimum pause duration
	
	// Geometric filters
	MaxHairpinDegrees float64 // degrees - allow sharp trail switchbacks
	TeleportMeters    float64 // meters - jump guard for missing timestamps
	
	// LOF parameters
	LofMinK         int     // minimum LOF neighbors
	LofMaxK         int     // maximum LOF neighbors
	LofMinThreshold float64 // LOF floor threshold
	
	// Safety limits
	MaxRemovedPercent  float64 // never remove >X% of points
	MaxDistanceReduced float64 // never drop >X% of distance
	
	// Elevation smoothing
	ElevationWindow int // median filter window size
}

// DefaultConfig returns production-tested configuration
func DefaultConfig() Config {
	return Config{
		MinSpeed:           0.1,   // 0.36 km/h - allows extended stops
		MaxSpeed:           0,     // auto-detect based on activity type
		PauseSpeed:         0.7,   // slightly higher for robustness
		MinPauseDuration:   120.0, // 2 minutes - typical aid station stop
		MaxHairpinDegrees:  160.0, // allow sharp trail switchbacks
		TeleportMeters:     120.0, // tightened from 200m
		LofMinK:            10,    // minimum neighbors
		LofMaxK:            20,    // maximum neighbors
		LofMinThreshold:    1.5,   // LOF floor
		MaxRemovedPercent:  20.0,  // safety: never remove >20% of points
		MaxDistanceReduced: 25.0,  // safety: never drop >25% of distance
		ElevationWindow:    7,     // median filter window
	}
}

// Stats represents cleaning results and metrics
type Stats struct {
	// Input
	OriginalPoints   int     `json:"original_points"`
	OriginalDistance float64 `json:"original_distance_km"`
	
	// Processing steps
	VelocityFiltered int     `json:"velocity_filtered_points"`
	LofFiltered      int     `json:"lof_filtered_points"`
	FinalPoints      int     `json:"final_points"`
	
	// Results
	PointsRemoved    int     `json:"points_removed"`
	PointsPercent    float64 `json:"points_removed_percent"`
	FinalDistance    float64 `json:"final_distance_km"`
	DistanceReduced  float64 `json:"distance_reduced_km"`
	DistancePercent  float64 `json:"distance_reduced_percent"`
	
	// Performance
	ProcessingTime time.Duration `json:"processing_time_ms"`
	
	// Activity detection
	ActivityType     string  `json:"activity_type"`
	DetectedMaxSpeed float64 `json:"detected_max_speed_ms"`
	P95Speed         float64 `json:"p95_speed_ms"`
}

// CleaningResult contains the filtered points and statistics
type CleaningResult struct {
	Points []Point
	Stats  Stats
}