package merge

import (
	"errors"
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
}

// Stats reports what happened during the merge so callers can surface it to users.
type Stats struct {
	GapsDetected   int
	GapsFilled     int
	InsertedPoints int
	SpatialMatches int
}

// DefaultConfig returns the recommended configuration for production use.
func DefaultConfig() Config {
	return Config{
		GapThreshold:             2 * time.Minute,
		MaxDeviationMeters:       60,
		EnableSpatialFallback:    true,
		SpatialWindowSize:        15,
		SpatialMaxAvgDeviation:   35,
		SpatialProgressTolerance: 0.15,
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

		segment, lastIdx := findSpatialSegment(
			beforeContext,
			afterContext,
			secondaryPoints,
			usedUntil,
			cfg,
			secondaryProgress,
			secondaryTotal,
			primaryBeforeProgress,
			primaryAfterProgress,
		)
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
) ([]gpx.Point, int) {
	if len(beforeContext) == 0 || len(afterContext) == 0 {
		return nil, usedUntil
	}

	anchored, anchoredEnd := findAnchoredSegment(
		beforeContext[len(beforeContext)-1],
		afterContext[0],
		secondary,
		usedUntil,
		cfg,
		secondaryProgress,
		secondaryTotal,
		primaryBeforeProgress,
		primaryAfterProgress,
	)
	if len(anchored) > 0 {
		return anchored, anchoredEnd
	}

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
	// debug: fmt.Printf("beforeMatch=%d (target %.3f, actual %.3f) score=%.2f\n", beforeMatch, primaryBeforeProgress, secondaryProgress[min(beforeMatch+len(beforeContext)/2, len(secondaryProgress)-1)]/secondaryTotal, beforeScore)
	if beforeMatch < 0 {
		return nil, usedUntil
	}

	if cfg.SpatialMaxAvgDeviation > 0 && beforeScore > cfg.SpatialMaxAvgDeviation {
		return nil, usedUntil
	}

	afterStart := beforeMatch + len(beforeContext)
	if afterStart >= len(secondary) {
		return nil, usedUntil
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
	// debug: fmt.Printf("afterMatch=%d (target %.3f, actual %.3f) score=%.2f\n", afterMatch, primaryAfterProgress, secondaryProgress[min(afterMatch+len(afterContext)/2, len(secondaryProgress)-1)]/secondaryTotal, afterScore)
	if afterMatch < 0 {
		return nil, usedUntil
	}

	if cfg.SpatialMaxAvgDeviation > 0 && afterScore > cfg.SpatialMaxAvgDeviation {
		return nil, usedUntil
	}

	if afterMatch <= beforeMatch+len(beforeContext)-1 {
		return nil, usedUntil
	}

	segmentStart := beforeMatch + len(beforeContext)
	segmentEnd := afterMatch
	if segmentStart >= segmentEnd {
		return nil, usedUntil
	}

	segment := make([]gpx.Point, segmentEnd-segmentStart)
	copy(segment, secondary[segmentStart:segmentEnd])

	return segment, segmentEnd - 1
}

func findAnchoredSegment(
	beforePoint, afterPoint gpx.Point,
	secondary []gpx.Point,
	usedUntil int,
	cfg Config,
	secondaryProgress []float64,
	secondaryTotal float64,
	primaryBeforeProgress float64,
	primaryAfterProgress float64,
) ([]gpx.Point, int) {
	startIdx := usedUntil + 1
	if startIdx < 0 {
		startIdx = 0
	}

	maxDist := cfg.MaxDeviationMeters
	if maxDist <= 0 {
		maxDist = 100
	}

	startCandidates := make([]int, 0)
	for idx := startIdx; idx < len(secondary); idx++ {
		if distanceMeters(beforePoint, secondary[idx]) <= maxDist {
			startCandidates = append(startCandidates, idx)
		}
	}

	if len(startCandidates) == 0 {
		return nil, startIdx - 1
	}

	endCandidates := make([]int, 0)
	for idx := startIdx + 1; idx < len(secondary); idx++ {
		if distanceMeters(afterPoint, secondary[idx]) <= maxDist {
			endCandidates = append(endCandidates, idx)
		}
	}

	if len(endCandidates) == 0 {
		return nil, startIdx - 1
	}

	bestStart, bestEnd := -1, -1
	bestScore := math.MaxFloat64
	bestLength := -1.0

	for _, sIdx := range startCandidates {
		for _, eIdx := range endCandidates {
			if eIdx <= sIdx || (usedUntil >= 0 && sIdx <= usedUntil) {
				continue
			}
			score := distanceMeters(beforePoint, secondary[sIdx]) + distanceMeters(afterPoint, secondary[eIdx])
			length := 0.0
			if len(secondaryProgress) > 0 {
				length = secondaryProgress[eIdx] - secondaryProgress[sIdx]
			}
			if length < 0 {
				continue
			}
			if length > bestLength || (math.Abs(length-bestLength) < 1e-6 && score < bestScore) {
				bestLength = length
				bestScore = score
				bestStart = sIdx
				bestEnd = eIdx
			}
		}
	}

	if bestStart < 0 || bestEnd <= bestStart {
		return nil, startIdx - 1
	}

	segment := make([]gpx.Point, bestEnd-bestStart)
	copy(segment, secondary[bestStart:bestEnd])

	return segment, bestEnd - 1
}

func withinProgress(idx int, progress []float64, total float64, target float64, tolerance float64) bool {
	if idx < 0 || idx >= len(progress) || total <= 0 || tolerance <= 0 {
		return true
	}
	ratio := progress[idx] / total
	return math.Abs(ratio-target) <= tolerance
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
