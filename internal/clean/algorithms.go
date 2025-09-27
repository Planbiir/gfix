package clean

import (
	"fmt"
	"math"
	"sort"
)

// smoothElevation applies median filter to reduce barometric noise
func smoothElevation(points []Point, windowSize int) {
	if len(points) < 3 || windowSize < 3 {
		return
	}

	// Ensure window size is odd
	if windowSize%2 == 0 {
		windowSize++
	}
	half := windowSize / 2

	// Create buffer for median calculation
	smoothed := make([]float64, len(points))

	for i := range points {
		// Collect elevation values in window
		elevations := make([]float64, 0, windowSize)
		start := max(0, i-half)
		end := min(len(points), i+half+1)

		for j := start; j < end; j++ {
			elevations = append(elevations, points[j].Elevation)
		}

		// Apply median filter
		smoothed[i] = medianFloat(elevations)
	}

	// Update points with smoothed elevation
	for i := range points {
		points[i].Elevation = smoothed[i]
	}
}

// velocityOutlierFilter removes points with impossible speeds and geometric spikes
func velocityOutlierFilter(inputPoints []Point, config Config) []int {
	if len(inputPoints) <= 2 {
		return []int{0, len(inputPoints) - 1}
	}

	// Auto-detect activity type and set appropriate speed limits
	activityType, maxSpeed, p95Speed := detectActivityType(inputPoints)
	
	fmt.Printf("   Auto-detected activity: %s (P95: %.1f m/s, avg interval: %.1f s)\n", 
		activityType, p95Speed, calculateAverageInterval(inputPoints))
	fmt.Printf("   Speed limit: %.1f m/s (%.1f km/h)\n", maxSpeed, maxSpeed*3.6)
	fmt.Printf("   Detected activity max speed: %.1f m/s (%.1f km/h)\n", maxSpeed, maxSpeed*3.6)

	validIndices := []int{0} // Always keep first point

	for i := 1; i < len(inputPoints)-1; i++ {
		curr := inputPoints[i]
		prev := inputPoints[i-1]
		next := inputPoints[i+1]

		// Calculate 3D speeds using real timestamps from GPX (includes elevation)
		distToPrev := haversineDistance3D(prev.Lat, prev.Lon, prev.Elevation,
			curr.Lat, curr.Lon, curr.Elevation)
		distToNext := haversineDistance3D(curr.Lat, curr.Lon, curr.Elevation,
			next.Lat, next.Lon, next.Elevation)

		// Calculate actual time intervals from GPX timestamps
		timeToPrev := curr.Time.Sub(prev.Time).Seconds()
		timeToNext := next.Time.Sub(curr.Time).Seconds()

		// Check which timestamps are valid to avoid misleading speed calculations
		validPrev := timeToPrev > 0
		validNext := timeToNext > 0

		// Calculate turn angle to detect GPS jumps (sharp turns)
		turnAngle := calculateTurnAngle(prev, curr, next)
		directionOK := turnAngle <= config.MaxHairpinDegrees

		// Speed validation: check timestamps first, fallback to distance-based detection
		speedOK := true
		if validPrev && validNext {
			// Both timestamps valid: check both speeds
			speedFromPrev := distToPrev / timeToPrev
			speedToNext := distToNext / timeToNext
			speedOK = (speedFromPrev >= config.MinSpeed && speedFromPrev <= maxSpeed) &&
				(speedToNext >= config.MinSpeed && speedToNext <= maxSpeed)

			// Strong geometric spike detector (two-leg "boomerang")
			if speedOK {
				base := haversineDistance(prev.Lat, prev.Lon, next.Lat, next.Lon)

				// Case A: classic boomerang (both legs long, base short, big turn)
				if distToPrev > 120 && distToNext > 120 && base < 40 && turnAngle > 100 {
					speedOK = false
				} else {
					// Case B: ratio-based spike (sum of legs >> base)
					ratio := (distToPrev + distToNext) / math.Max(base, 1)
					if ratio > 6 && turnAngle > 90 {
						speedOK = false
					}
				}
			}
		} else if validPrev {
			// Only previous timestamp valid
			speedFromPrev := distToPrev / timeToPrev
			speedOK = (speedFromPrev >= config.MinSpeed && speedFromPrev <= maxSpeed)
		} else if validNext {
			// Only next timestamp valid
			speedToNext := distToNext / timeToNext
			speedOK = (speedToNext >= config.MinSpeed && speedToNext <= maxSpeed)
		} else {
			// No valid timestamps: fallback to distance-based teleport detection
			if distToPrev > config.TeleportMeters || distToNext > config.TeleportMeters {
				speedOK = false
			}
		}

		if speedOK && directionOK {
			validIndices = append(validIndices, i)
		} else {
			// Only rescue if it's clearly a pause (very low speed) — not just geometry
			isLowSpeed := false
			if validPrev && validNext {
				speedFromPrev := distToPrev / timeToPrev
				speedToNext := distToNext / timeToNext
				isLowSpeed = speedFromPrev <= config.PauseSpeed && speedToNext <= config.PauseSpeed
			}
			if isLowSpeed {
				validIndices = append(validIndices, i)
			}
		}
	}

	// Always keep last point
	if len(inputPoints) > 1 {
		validIndices = append(validIndices, len(inputPoints)-1)
	}

	fmt.Printf("   Velocity+Direction filter removed %d points (includes speed + direction violations)\n",
		len(inputPoints)-len(validIndices))

	return validIndices
}

// detectActivityType automatically detects activity type and sets speed limits
func detectActivityType(points []Point) (string, float64, float64) {
	speeds := calculateAllSpeeds(points)
	if len(speeds) == 0 {
		return "unknown", 12.0, 0.0
	}

	// Calculate P95 speed for activity classification
	p95 := percentile(speeds, 95)

	var maxSpeed float64
	var activityType string

	if p95 <= 8.0 { // 28.8 km/h
		// Running/Hiking activity
		maxSpeed = 12.0 // 43.2 km/h for running
		activityType = "running/hiking"
	} else if p95 <= 20.0 { // 72 km/h
		// Cycling activity
		maxSpeed = 30.0 // 108 km/h for cycling
		activityType = "cycling"
	} else {
		// High-speed activity (skiing, motorsports)
		maxSpeed = 50.0 // 180 km/h
		activityType = "high-speed"
	}

	return activityType, maxSpeed, p95
}

// calculateAllSpeeds computes speeds between consecutive points
func calculateAllSpeeds(points []Point) []float64 {
	if len(points) < 2 {
		return nil
	}

	var speeds []float64
	for i := 1; i < len(points); i++ {
		if !points[i].Time.IsZero() && !points[i-1].Time.IsZero() {
			dt := points[i].Time.Sub(points[i-1].Time).Seconds()
			if dt > 0 {
				dist := haversineDistance3D(
					points[i-1].Lat, points[i-1].Lon, points[i-1].Elevation,
					points[i].Lat, points[i].Lon, points[i].Elevation)
				speed := dist / dt
				if speed > 0 && speed < 100 { // reasonable bounds
					speeds = append(speeds, speed)
				}
			}
		}
	}
	return speeds
}

// calculateAverageInterval computes average time interval between points
func calculateAverageInterval(points []Point) float64 {
	if len(points) < 2 {
		return 0.0
	}

	var totalInterval float64
	validIntervals := 0

	for i := 1; i < len(points); i++ {
		if !points[i].Time.IsZero() && !points[i-1].Time.IsZero() {
			interval := points[i].Time.Sub(points[i-1].Time).Seconds()
			if interval > 0 && interval < 3600 { // reasonable bounds (1 hour max)
				totalInterval += interval
				validIntervals++
			}
		}
	}

	if validIntervals > 0 {
		return totalInterval / float64(validIntervals)
	}
	return 1.0 // default 1 second
}

// calculateTurnAngle computes the turn angle between three consecutive points
func calculateTurnAngle(p1, p2, p3 Point) float64 {
	// Calculate bearings
	bearing1 := calculateBearing(p1.Lat, p1.Lon, p2.Lat, p2.Lon)
	bearing2 := calculateBearing(p2.Lat, p2.Lon, p3.Lat, p3.Lon)

	// Calculate turn angle
	turnAngle := math.Abs(bearing2 - bearing1)
	if turnAngle > 180.0 {
		turnAngle = 360.0 - turnAngle
	}

	return turnAngle
}

// calculateBearing computes bearing between two points
func calculateBearing(lat1, lon1, lat2, lon2 float64) float64 {
	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLonRad := (lon2 - lon1) * math.Pi / 180

	y := math.Sin(deltaLonRad) * math.Cos(lat2Rad)
	x := math.Cos(lat1Rad)*math.Sin(lat2Rad) - math.Sin(lat1Rad)*math.Cos(lat2Rad)*math.Cos(deltaLonRad)

	bearingRad := math.Atan2(y, x)
	bearingDeg := bearingRad * 180 / math.Pi

	return math.Mod(bearingDeg+360, 360)
}

// haversineDistance calculates 2D distance between two points
func haversineDistance(lat1, lon1, lat2, lon2 float64) float64 {
	const earthRadius = 6371000 // meters

	lat1Rad := lat1 * math.Pi / 180
	lat2Rad := lat2 * math.Pi / 180
	deltaLat := (lat2 - lat1) * math.Pi / 180
	deltaLon := (lon2 - lon1) * math.Pi / 180

	a := math.Sin(deltaLat/2)*math.Sin(deltaLat/2) +
		math.Cos(lat1Rad)*math.Cos(lat2Rad)*
		math.Sin(deltaLon/2)*math.Sin(deltaLon/2)
	c := 2 * math.Atan2(math.Sqrt(a), math.Sqrt(1-a))

	return earthRadius * c
}

// Utility functions
func medianFloat(values []float64) float64 {
	if len(values) == 0 {
		return 0.0
	}
	
	sorted := make([]float64, len(values))
	copy(sorted, values)
	sort.Float64s(sorted)
	
	if len(sorted)%2 == 0 {
		return (sorted[len(sorted)/2-1] + sorted[len(sorted)/2]) / 2
	}
	return sorted[len(sorted)/2]
}

func percentile(values []float64, p float64) float64 {
	if len(values) == 0 {
		return 0.0
	}
	
	sorted := make([]float64, len(values))
	copy(sorted, values)
	sort.Float64s(sorted)
	
	index := (p / 100.0) * float64(len(sorted)-1)
	lower := int(math.Floor(index))
	upper := int(math.Ceil(index))
	
	if lower == upper {
		return sorted[lower]
	}
	
	weight := index - float64(lower)
	return sorted[lower]*(1-weight) + sorted[upper]*weight
}