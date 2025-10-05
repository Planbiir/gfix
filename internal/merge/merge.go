package merge

import (
	"errors"
	"fmt"
	"math"
	"sort"
	"time"

	"github.com/planbiir/gfix/internal/gpx"
)

// Config controls how the merge operation behaves.
type Config struct {
	// GapThreshold defines the minimum pause duration (in wall-clock time)
	// that will be considered a gap worth filling with the secondary track.
	// If zero, DefaultConfig().GapThreshold is used.
	GapThreshold time.Duration

	// MaxDeviationMeters limits how far the secondary points are allowed to
	// deviate from the surrounding primary points. Set to a negative value to
	// disable the guard. A zero value means "use the default".
	MaxDeviationMeters float64

	// EnableSpatialFallback switches on geometry-based gap filling when the
	// secondary track lacks timestamps. Disabled gaps remain untouched.
	EnableSpatialFallback bool

	// SpatialWindowSize controls how many points from the primary track are
	// compared before/after a gap to align segments in the timestamp-free
	// secondary track. Zero means "use the default".
	SpatialWindowSize int

	// SpatialMaxAvgDeviation constrains the average distance (meters) between
	// the alignment windows. Zero means "use the default"; negative disables.
	SpatialMaxAvgDeviation float64

	// SpatialProgressTolerance bounds the difference between the progress
	// (cumulative distance ratio) of the primary gap and the matched segment
	// in the secondary track.
	SpatialProgressTolerance float64

	// MaxSearchDistanceMultiplier scales the direct distance between anchors
	// to determine how far we search along the secondary track.
	MaxSearchDistanceMultiplier float64

	// MaxSearchDistancePaddingMeters adds an absolute allowance to the search window.
	MaxSearchDistancePaddingMeters float64

	// MinReliableSpeedMPS sets the minimum average speed (m/s) required to
	// trust the expected-distance estimation derived from timestamps.
	MinReliableSpeedMPS float64

	// MaxSpeedRatio controls how much faster than the computed average speed we allow
	// the search to consider when matching the secondary track.
	MaxSpeedRatio float64

	// DistanceLowerRatio and DistanceUpperRatio describe acceptable bounds for
	// inserted segment length relative to the expected distance estimate.
	DistanceLowerRatio float64
	DistanceUpperRatio float64

	// Anchor tie-break threshold for distance score (as fraction of expected distance).
	AnchorTieBreakRatio float64
}

// Stats reports what happened during the merge so callers can surface it to users.
type Stats struct {
	GapsDetected   int
	GapsFilled     int
	InsertedPoints int
	SpatialMatches int

	// Debug information for spatial matching
	SpatialDebugInfo []SpatialMatchInfo
}

// SpatialMatchInfo provides debugging details for each spatial match attempt
type SpatialMatchInfo struct {
	GapIndex     int     // Which gap this was (0-based)
	BeforeAnchor string  // Description of before anchor point
	AfterAnchor  string  // Description of after anchor point
	SegmentStart int     // Start index in secondary track
	SegmentEnd   int     // End index in secondary track
	DistanceKm   float64 // Distance of inserted segment in km
	Valid        bool    // Whether this match was accepted
	Reason       string  // Why it was accepted/rejected
}

// DefaultConfig returns the recommended configuration for production use.
func DefaultConfig() Config {
	return Config{
		GapThreshold:                   2 * time.Minute,
		MaxDeviationMeters:             60,
		EnableSpatialFallback:          true,
		SpatialWindowSize:              15,
		SpatialMaxAvgDeviation:         35,
		SpatialProgressTolerance:       0.15,
		MaxSearchDistanceMultiplier:    5.0,
		MaxSearchDistancePaddingMeters: 2000.0,
		MinReliableSpeedMPS:            1.0,
		MaxSpeedRatio:                  1.6,
		DistanceLowerRatio:             0.4,
		DistanceUpperRatio:             1.6,
		AnchorTieBreakRatio:            0.05,
	}
}

// MergeTracks fills gaps in the primary track with matching points from the
// secondary track. Only time ranges that fall within the primary track window
// are considered, which prevents adding leading/trailing segments that the
// athlete did not record.
func MergeTracks(primary, secondary *gpx.GPX, cfg Config) (*gpx.GPX, Stats, error) {
	if primary == nil {
		return nil, Stats{}, errors.New("primary track is nil")
	}
	if secondary == nil {
		return nil, Stats{}, errors.New("secondary track is nil")
	}

	defaults := DefaultConfig()
	if cfg.GapThreshold <= 0 {
		cfg.GapThreshold = defaults.GapThreshold
	}
	if cfg.MaxDeviationMeters == 0 {
		cfg.MaxDeviationMeters = defaults.MaxDeviationMeters
	}
	if cfg.SpatialWindowSize <= 0 {
		cfg.SpatialWindowSize = defaults.SpatialWindowSize
	}
	if cfg.SpatialMaxAvgDeviation == 0 {
		cfg.SpatialMaxAvgDeviation = defaults.SpatialMaxAvgDeviation
	}
	if cfg.SpatialProgressTolerance <= 0 || cfg.SpatialProgressTolerance > 1 {
		cfg.SpatialProgressTolerance = defaults.SpatialProgressTolerance
	}
	if cfg.MaxSearchDistanceMultiplier <= 0 {
		cfg.MaxSearchDistanceMultiplier = defaults.MaxSearchDistanceMultiplier
	}
	if cfg.MaxSearchDistancePaddingMeters < 0 {
		cfg.MaxSearchDistancePaddingMeters = defaults.MaxSearchDistancePaddingMeters
	}
	if cfg.MinReliableSpeedMPS <= 0 {
		cfg.MinReliableSpeedMPS = defaults.MinReliableSpeedMPS
	}
	if cfg.MaxSpeedRatio <= 0 {
		cfg.MaxSpeedRatio = defaults.MaxSpeedRatio
	}
	if cfg.DistanceLowerRatio <= 0 || cfg.DistanceLowerRatio >= cfg.DistanceUpperRatio {
		cfg.DistanceLowerRatio = defaults.DistanceLowerRatio
		cfg.DistanceUpperRatio = defaults.DistanceUpperRatio
	}
	if cfg.AnchorTieBreakRatio <= 0 || cfg.AnchorTieBreakRatio >= 1 {
		cfg.AnchorTieBreakRatio = defaults.AnchorTieBreakRatio
	}

	primaryPoints := primary.FlattenPoints()
	if len(primaryPoints) == 0 {
		return nil, Stats{}, errors.New("primary track has no points")
	}

	secondaryPoints := secondary.FlattenPoints()
	if len(secondaryPoints) == 0 {
		merged := *primary
		return &merged, Stats{}, nil
	}

	primaryStart, primaryEnd := timeBounds(primaryPoints)
	if primaryStart.IsZero() || primaryEnd.IsZero() {
		return nil, Stats{}, errors.New("primary track lacks timestamped points")
	}

	stats := Stats{}

	if hasTimestamps(secondaryPoints) {
		filtered := filterSecondaryPoints(secondaryPoints, primaryStart, primaryEnd)
		if len(filtered) == 0 {
			merged := *primary
			return &merged, stats, nil
		}
		mergedPoints := mergeByTimestamps(primaryPoints, filtered, cfg, &stats)
		merged := *primary
		merged.RebuildFromPoints(mergedPoints)
		return &merged, stats, nil
	}

	if !cfg.EnableSpatialFallback {
		merged := *primary
		return &merged, stats, nil
	}

	mergedPoints := mergeByGeometry(primaryPoints, secondaryPoints, cfg, &stats)
	merged := *primary
	merged.RebuildFromPoints(mergedPoints)
	return &merged, stats, nil
}

func mergeByTimestamps(primaryPoints, secondaryPoints []gpx.Point, cfg Config, stats *Stats) []gpx.Point {
	mergedPoints := make([]gpx.Point, 0, len(primaryPoints)+len(secondaryPoints))
	secondaryIdx := 0

	for i := 0; i < len(primaryPoints); i++ {
		current := primaryPoints[i]
		mergedPoints = append(mergedPoints, current)

		if i == len(primaryPoints)-1 {
			continue
		}

		next := primaryPoints[i+1]

		if current.Time.IsZero() || next.Time.IsZero() {
			continue
		}

		gap := next.Time.Sub(current.Time)
		if gap <= cfg.GapThreshold {
			continue
		}

		stats.GapsDetected++

		for secondaryIdx < len(secondaryPoints) && !secondaryPoints[secondaryIdx].Time.After(current.Time) {
			secondaryIdx++
		}

		insertionStart := len(mergedPoints)
		idx := secondaryIdx

		for idx < len(secondaryPoints) {
			candidate := secondaryPoints[idx]
			if !candidate.Time.Before(next.Time) {
				break
			}

			if len(mergedPoints) > 0 && samePoint(mergedPoints[len(mergedPoints)-1], candidate) {
				idx++
				continue
			}

			if cfg.MaxDeviationMeters > 0 {
				if distanceMeters(current, candidate) > cfg.MaxDeviationMeters &&
					distanceMeters(candidate, next) > cfg.MaxDeviationMeters {
					idx++
					continue
				}
			}

			pointCopy := candidate
			pointCopy.TrackIdx = current.TrackIdx
			pointCopy.SegIdx = current.SegIdx
			pointCopy.PtIdx = 0
			mergedPoints = append(mergedPoints, pointCopy)
			idx++
		}

		inserted := len(mergedPoints) - insertionStart
		if inserted > 0 {
			stats.GapsFilled++
			stats.InsertedPoints += inserted
			secondaryIdx = idx
		}
	}

	return mergedPoints
}

func mergeByGeometry(primaryPoints, secondaryPoints []gpx.Point, cfg Config, stats *Stats) []gpx.Point {
	merged := make([]gpx.Point, 0, len(primaryPoints)+len(secondaryPoints))
	usedUntil := -1

	primaryProgress, primaryTotal := computeProgress(primaryPoints)
	secondaryProgress, secondaryTotal := computeProgress(secondaryPoints)

	if primaryTotal == 0 || secondaryTotal == 0 {
		// Nothing to interpolate — return a copy of the primary track.
		return clonePoints(primaryPoints)
	}

	for i := 0; i < len(primaryPoints); i++ {
		current := primaryPoints[i]
		merged = append(merged, current)

		if i == len(primaryPoints)-1 {
			continue
		}

		next := primaryPoints[i+1]
		if current.Time.IsZero() || next.Time.IsZero() {
			continue
		}

		gap := next.Time.Sub(current.Time)
		if gap <= cfg.GapThreshold {
			continue
		}

		stats.GapsDetected++

		beforeContext := windowBackwards(primaryPoints, i, cfg.SpatialWindowSize)
		afterContext := windowForward(primaryPoints, i+1, cfg.SpatialWindowSize)

		primaryBeforeProgress := primaryProgress[i] / primaryTotal
		primaryAfterProgress := primaryProgress[i+1] / primaryTotal

		segment, lastIdx, debugInfo := findSpatialSegment(
			beforeContext,
			afterContext,
			secondaryPoints,
			usedUntil,
			cfg,
			secondaryProgress,
			secondaryTotal,
			primaryBeforeProgress,
			primaryAfterProgress,
			stats.GapsDetected-1, // Current gap index (0-based)
			gap,
		)

		// Add debug information regardless of success/failure
		stats.SpatialDebugInfo = append(stats.SpatialDebugInfo, debugInfo)

		if len(segment) == 0 {
			continue
		}

		assignInterpolatedTimes(segment, current.Time, next.Time)

		for _, pt := range segment {
			pointCopy := pt
			pointCopy.TrackIdx = current.TrackIdx
			pointCopy.SegIdx = current.SegIdx
			pointCopy.PtIdx = 0
			merged = append(merged, pointCopy)
		}

		stats.GapsFilled++
		stats.InsertedPoints += len(segment)
		stats.SpatialMatches++
		usedUntil = lastIdx
	}

	return merged
}

func findSpatialSegment(
	beforeContext, afterContext []gpx.Point,
	secondary []gpx.Point,
	usedUntil int,
	cfg Config,
	secondaryProgress []float64,
	secondaryTotal float64,
	primaryBeforeProgress float64,
	primaryAfterProgress float64,
	gapIndex int,
	gapDuration time.Duration,
) ([]gpx.Point, int, SpatialMatchInfo) {
	debug := SpatialMatchInfo{
		GapIndex: gapIndex,
		Valid:    false,
	}

	if len(beforeContext) == 0 || len(afterContext) == 0 {
		debug.Reason = "empty context windows"
		return nil, usedUntil, debug
	}

	beforePoint := beforeContext[len(beforeContext)-1]
	afterPoint := afterContext[0]

	debug.BeforeAnchor = fmt.Sprintf("%.6f,%.6f", beforePoint.Lat, beforePoint.Lon)
	debug.AfterAnchor = fmt.Sprintf("%.6f,%.6f", afterPoint.Lat, afterPoint.Lon)

	expectedDistanceMeters := estimateExpectedDistanceMeters(beforeContext, afterContext, gapDuration, cfg)

	anchored, anchoredEnd, anchoredDebug := findAnchoredSegmentWithDebug(
		beforePoint,
		afterPoint,
		secondary,
		usedUntil,
		cfg,
		secondaryProgress,
		expectedDistanceMeters,
	)

	if len(anchored) > 0 {
		debug.SegmentStart = anchoredDebug.SegmentStart
		debug.SegmentEnd = anchoredDebug.SegmentEnd
		debug.DistanceKm = anchoredDebug.DistanceKm
		debug.Valid = true
		debug.Reason = "anchored match: " + anchoredDebug.Reason
		return anchored, anchoredEnd, debug
	}

	// If anchored matching failed, record why and try window matching
	debug.Reason = "anchored failed: " + anchoredDebug.Reason

	beforeMatch, beforeScore := matchWindow(
		beforeContext,
		secondary,
		usedUntil+1,
		len(secondary)-len(beforeContext),
		secondaryProgress,
		secondaryTotal,
		primaryBeforeProgress,
		cfg.SpatialProgressTolerance,
	)

	if beforeMatch < 0 {
		debug.Reason += "; no before window match"
		return nil, usedUntil, debug
	}

	if cfg.SpatialMaxAvgDeviation > 0 && beforeScore > cfg.SpatialMaxAvgDeviation {
		debug.Reason += fmt.Sprintf("; before score %.1fm > %.1fm", beforeScore, cfg.SpatialMaxAvgDeviation)
		return nil, usedUntil, debug
	}

	afterStart := beforeMatch + len(beforeContext)
	if afterStart >= len(secondary) {
		debug.Reason += "; no room for after window"
		return nil, usedUntil, debug
	}

	afterMatch, afterScore := matchWindow(
		afterContext,
		secondary,
		afterStart,
		len(secondary)-len(afterContext),
		secondaryProgress,
		secondaryTotal,
		primaryAfterProgress,
		cfg.SpatialProgressTolerance,
	)

	if afterMatch < 0 {
		debug.Reason += "; no after window match"
		return nil, usedUntil, debug
	}

	if cfg.SpatialMaxAvgDeviation > 0 && afterScore > cfg.SpatialMaxAvgDeviation {
		debug.Reason += fmt.Sprintf("; after score %.1fm > %.1fm", afterScore, cfg.SpatialMaxAvgDeviation)
		return nil, usedUntil, debug
	}

	if afterMatch <= beforeMatch+len(beforeContext)-1 {
		debug.Reason += "; overlapping windows"
		return nil, usedUntil, debug
	}

	segmentStart := beforeMatch + len(beforeContext)
	segmentEnd := afterMatch
	if segmentStart >= segmentEnd {
		debug.Reason += "; empty segment"
		return nil, usedUntil, debug
	}

	segmentDistanceKm := 0.0
	if len(secondaryProgress) > segmentEnd && len(secondaryProgress) > segmentStart {
		segmentDistanceKm = (secondaryProgress[segmentEnd] - secondaryProgress[segmentStart]) / 1000.0
	}

	if !validateSegmentDistance(expectedDistanceMeters, segmentDistanceKm, cfg, &debug) {
		return nil, usedUntil, debug
	}

	debug.SegmentStart = segmentStart
	debug.SegmentEnd = segmentEnd
	debug.DistanceKm = segmentDistanceKm
	debug.Valid = true
	debug.Reason = fmt.Sprintf("window match: before@%d(%.1fm) after@%d(%.1fm)", beforeMatch, beforeScore, afterMatch, afterScore)

	segment := make([]gpx.Point, segmentEnd-segmentStart)
	copy(segment, secondary[segmentStart:segmentEnd])

	return segment, segmentEnd - 1, debug
}

func findAnchoredSegmentWithDebug(
	beforePoint, afterPoint gpx.Point,
	secondary []gpx.Point,
	usedUntil int,
	cfg Config,
	secondaryProgress []float64,
	expectedDistanceMeters float64,
) ([]gpx.Point, int, SpatialMatchInfo) {
	debug := SpatialMatchInfo{}

	startIdx := usedUntil + 1
	if startIdx < 0 {
		startIdx = 0
	}

	maxDist := cfg.MaxDeviationMeters
	if maxDist <= 0 {
		maxDist = 100
	}
	type candidate struct {
		idx  int
		dist float64
	}

	candidates := make([]candidate, 0, 32)
	for idx := startIdx; idx < len(secondary)-1; idx++ {
		dist := distanceMeters(beforePoint, secondary[idx])
		if dist <= maxDist {
			candidates = append(candidates, candidate{idx: idx, dist: dist})
			if expectedDistanceMeters <= 0 && dist <= 1 {
				break
			}
			if len(candidates) >= 64 {
				break
			}
		}
	}

	if len(candidates) == 0 {
		debug.Reason = fmt.Sprintf("no before anchor within %.0fm", maxDist)
		return nil, startIdx - 1, debug
	}

	tieRatio := cfg.AnchorTieBreakRatio
	if tieRatio <= 0 {
		tieRatio = DefaultConfig().AnchorTieBreakRatio
	}

	baseDistance := distanceMeters(beforePoint, afterPoint)
	maxSearchDistance := cfg.MaxSearchDistanceMultiplier*baseDistance + cfg.MaxSearchDistancePaddingMeters

	bestStart := -1
	bestEnd := -1
	bestSegmentKm := 0.0
	bestStartDist := math.MaxFloat64
	bestScore := math.MaxFloat64
	bestAfterDist := math.MaxFloat64

	for _, cand := range candidates {
		startIdx := cand.idx
		startDist := cand.dist

		bestIdx := -1
		bestEndDist := math.MaxFloat64
		bestLocalScore := math.MaxFloat64

		for idx := startIdx + 1; idx < len(secondary); idx++ {
			dist := distanceMeters(afterPoint, secondary[idx])
			if len(secondaryProgress) <= idx || len(secondaryProgress) <= startIdx {
				continue
			}
			pathDistance := secondaryProgress[idx] - secondaryProgress[startIdx]
			if pathDistance < 0 {
				continue
			}

			if dist <= maxDist {
				var score float64
				if expectedDistanceMeters > 0 {
					score = math.Abs(pathDistance - expectedDistanceMeters)
					tieWindow := expectedDistanceMeters * tieRatio
					if bestIdx < 0 || score < bestLocalScore || (math.Abs(score-bestLocalScore) <= tieWindow && dist < bestEndDist) {
						bestLocalScore = score
						bestEndDist = dist
						bestIdx = idx
					}
				} else {
					score = pathDistance
					if bestIdx < 0 || score < bestLocalScore || (math.Abs(score-bestLocalScore) <= 1 && dist < bestEndDist) {
						bestLocalScore = score
						bestEndDist = dist
						bestIdx = idx
					}
				}
			}

			if maxSearchDistance > 0 && pathDistance > maxSearchDistance {
				break
			}
			if expectedDistanceMeters > 0 {
				upper := cfg.DistanceUpperRatio
				if upper <= 0 {
					upper = DefaultConfig().DistanceUpperRatio
				}
				if pathDistance > expectedDistanceMeters*upper {
					break
				}
			} else if dist <= 1 {
				break
			}
		}

		if bestIdx < 0 {
			continue
		}

		segmentMeters := 0.0
		if len(secondaryProgress) > bestIdx {
			segmentMeters = secondaryProgress[bestIdx] - secondaryProgress[startIdx]
		}
		segmentKm := segmentMeters / 1000.0

		if !validateSegmentDistance(expectedDistanceMeters, segmentKm, cfg, nil) {
			continue
		}

		score := bestLocalScore
		if expectedDistanceMeters <= 0 {
			score = segmentMeters
		}

		if bestStart < 0 || score < bestScore || (math.Abs(score-bestScore) <= expectedDistanceMeters*tieRatio && startDist < bestStartDist) {
			bestStart = startIdx
			bestEnd = bestIdx
			bestSegmentKm = segmentKm
			bestStartDist = startDist
			bestAfterDist = bestEndDist
			bestScore = score
		}
	}

	if bestStart < 0 || bestEnd <= bestStart {
		debug.Reason = fmt.Sprintf("no valid anchors within %.0fm", maxDist)
		return nil, startIdx - 1, debug
	}

	debug.SegmentStart = bestStart + 1
	debug.SegmentEnd = bestEnd
	debug.DistanceKm = bestSegmentKm
	debug.Reason = fmt.Sprintf("anchors %d(%.0fm)->%d(%.0fm), %.2fkm", bestStart, bestStartDist, bestEnd, bestAfterDist, bestSegmentKm)

	// Fix: ensure slice length matches source length to avoid zero-valued points
	segmentStart := bestStart + 1
	segmentEnd := bestEnd
	if segmentStart >= segmentEnd {
		debug.Reason = fmt.Sprintf("empty segment between anchors %d and %d", bestStart, bestEnd)
		return nil, startIdx - 1, debug
	}
	
	segment := make([]gpx.Point, segmentEnd-segmentStart)
	copy(segment, secondary[segmentStart:segmentEnd])

	return segment, bestEnd - 1, debug
}

func matchWindow(
	window []gpx.Point,
	secondary []gpx.Point,
	start, end int,
	progress []float64,
	total float64,
	targetProgress float64,
	tolerance float64,
) (int, float64) {
	if len(window) == 0 || len(secondary) < len(window) {
		return -1, 0
	}

	bestIdx := -1
	bestScore := math.MaxFloat64

	maxStart := len(secondary) - len(window)
	if end >= 0 {
		maxStart = min(maxStart, end)
	}
	for idx := max(start, 0); idx <= maxStart; idx++ {
		if total > 0 && tolerance > 0 && len(progress) > 0 {
			center := idx + (len(window)-1)/2
			if center < 0 {
				center = 0
			}
			if center >= len(progress) {
				center = len(progress) - 1
			}
			ratio := progress[center] / total
			if math.Abs(ratio-targetProgress) > tolerance {
				continue
			}
		}

		score := averageDistance(window, secondary[idx:idx+len(window)])
		if score < bestScore {
			bestScore = score
			bestIdx = idx
		}
	}

	return bestIdx, bestScore
}

func averageDistance(a, b []gpx.Point) float64 {
	count := min(len(a), len(b))
	if count == 0 {
		return math.MaxFloat64
	}
	sum := 0.0
	for i := 0; i < count; i++ {
		sum += distanceMeters(a[i], b[i])
	}
	return sum / float64(count)
}

func assignInterpolatedTimes(points []gpx.Point, start, end time.Time) {
	if len(points) == 0 {
		return
	}
	gap := end.Sub(start)
	if gap <= 0 {
		for i := range points {
			points[i].Time = start
		}
		return
	}
	interval := gap / time.Duration(len(points)+1)
	for i := range points {
		points[i].Time = start.Add(interval * time.Duration(i+1))
	}
}

func validateSegmentDistance(expectedMeters float64, segmentDistanceKm float64, cfg Config, debug *SpatialMatchInfo) bool {
	if expectedMeters <= 0 {
		return true
	}

	lower := cfg.DistanceLowerRatio
	upper := cfg.DistanceUpperRatio
	if lower <= 0 || upper <= lower {
		lower = DefaultConfig().DistanceLowerRatio
		upper = DefaultConfig().DistanceUpperRatio
	}

	minAllowed := expectedMeters * lower
	maxAllowed := expectedMeters * upper
	segmentMeters := segmentDistanceKm * 1000
	if segmentMeters < minAllowed || segmentMeters > maxAllowed {
		if debug != nil {
			dbg := fmt.Sprintf("segment distance %.2fkm outside %.2f–%.2fkm (estimated)", segmentDistanceKm, minAllowed/1000, maxAllowed/1000)
			if debug.Reason == "" {
				debug.Reason = dbg
			} else {
				debug.Reason += "; " + dbg
			}
		}
		return false
	}
	return true
}

func estimateExpectedDistanceMeters(beforeContext, afterContext []gpx.Point, gap time.Duration, cfg Config) float64 {
	if gap <= 0 {
		return 0
	}

	beforeSpeed, okBefore := averageSpeed(beforeContext)
	afterSpeed, okAfter := averageSpeed(afterContext)

	// Check if both contexts represent stationary periods (athlete standing still)
	const stationaryThreshold = 0.1 // 0.1 m/s = very slow movement
	beforeStationary := okBefore && beforeSpeed < stationaryThreshold
	afterStationary := okAfter && afterSpeed < stationaryThreshold
	
	// If both contexts are stationary (pause situation), disable distance validation
	// This allows proper detection of loops during pauses
	if beforeStationary && afterStationary {
		return 0 // Return 0 to disable distance validation
	}

	var selectedSpeed float64
	switch {
	case okBefore && okAfter:
		// If one context is stationary but the other shows movement, use the moving one
		if beforeStationary && !afterStationary {
			selectedSpeed = afterSpeed
		} else if afterStationary && !beforeStationary {
			selectedSpeed = beforeSpeed
		} else {
			// Both are moving - use existing logic
			slower := math.Min(beforeSpeed, afterSpeed)
			faster := math.Max(beforeSpeed, afterSpeed)
			if slower > 0 && faster/slower > cfg.MaxSpeedRatio {
				selectedSpeed = slower
			} else {
				selectedSpeed = (beforeSpeed + afterSpeed) / 2
			}
		}
	case okBefore && !beforeStationary:
		selectedSpeed = beforeSpeed
	case okAfter && !afterStationary:
		selectedSpeed = afterSpeed
	default:
		return 0 // Disable distance validation
	}

	if selectedSpeed < cfg.MinReliableSpeedMPS {
		return 0 // Disable distance validation for unreliable speeds
	}

	return selectedSpeed * gap.Seconds()
}

func averageSpeed(points []gpx.Point) (float64, bool) {
	if len(points) < 2 {
		return 0, false
	}

	var distanceSum float64
	var durationSum float64

	for i := 1; i < len(points); i++ {
		prev := points[i-1]
		curr := points[i]
		if prev.Time.IsZero() || curr.Time.IsZero() {
			continue
		}
		dt := curr.Time.Sub(prev.Time).Seconds()
		if dt <= 0 {
			continue
		}
		distanceSum += distanceMeters(prev, curr)
		durationSum += dt
	}

	if durationSum <= 0 {
		return 0, false
	}

	return distanceSum / durationSum, true
}

func computeProgress(points []gpx.Point) ([]float64, float64) {
	if len(points) == 0 {
		return nil, 0
	}

	progress := make([]float64, len(points))
	total := 0.0
	for i := 1; i < len(points); i++ {
		total += distanceMeters(points[i-1], points[i])
		progress[i] = total
	}
	return progress, total
}

func windowBackwards(points []gpx.Point, idx, size int) []gpx.Point {
	start := max(0, idx-size+1)
	return clonePoints(points[start : idx+1])
}

func windowForward(points []gpx.Point, idx, size int) []gpx.Point {
	end := min(len(points), idx+size)
	return clonePoints(points[idx:end])
}

func clonePoints(src []gpx.Point) []gpx.Point {
	out := make([]gpx.Point, len(src))
	copy(out, src)
	return out
}

func hasTimestamps(points []gpx.Point) bool {
	for _, pt := range points {
		if !pt.Time.IsZero() {
			return true
		}
	}
	return false
}

func filterSecondaryPoints(points []gpx.Point, start, end time.Time) []gpx.Point {
	const tolerance = time.Second
	filtered := make([]gpx.Point, 0, len(points))
	for _, pt := range points {
		if pt.Time.IsZero() {
			continue
		}
		if pt.Time.Before(start.Add(-tolerance)) {
			continue
		}
		if pt.Time.After(end.Add(tolerance)) {
			continue
		}
		filtered = append(filtered, pt)
	}

	sort.Slice(filtered, func(i, j int) bool {
		return filtered[i].Time.Before(filtered[j].Time)
	})

	return filtered
}

func timeBounds(points []gpx.Point) (time.Time, time.Time) {
	var start time.Time
	var end time.Time

	for _, pt := range points {
		if pt.Time.IsZero() {
			continue
		}
		if start.IsZero() || pt.Time.Before(start) {
			start = pt.Time
		}
		if end.IsZero() || pt.Time.After(end) {
			end = pt.Time
		}
	}

	return start, end
}

func distanceMeters(a, b gpx.Point) float64 {
	const earthRadius = 6371000.0

	lat1 := a.Lat * math.Pi / 180
	lat2 := b.Lat * math.Pi / 180
	dLat := (b.Lat - a.Lat) * math.Pi / 180
	dLon := (b.Lon - a.Lon) * math.Pi / 180

	sinLat := math.Sin(dLat / 2)
	sinLon := math.Sin(dLon / 2)

	h := sinLat*sinLat + math.Cos(lat1)*math.Cos(lat2)*sinLon*sinLon
	c := 2 * math.Atan2(math.Sqrt(h), math.Sqrt(1-h))

	return earthRadius * c
}

func samePoint(a, b gpx.Point) bool {
	const epsilon = 1e-9
	return math.Abs(a.Lat-b.Lat) < epsilon &&
		math.Abs(a.Lon-b.Lon) < epsilon &&
		a.Time.Equal(b.Time)
}

func max(a, b int) int {
	if a > b {
		return a
	}
	return b
}

func min(a, b int) int {
	if a < b {
		return a
	}
	return b
}
