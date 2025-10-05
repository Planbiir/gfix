package main

import (
	"flag"
	"fmt"
	"log"
	"math"
	"os"
	"sort"
	"time"

	"github.com/planbiir/gfix/internal/gpx"
	"github.com/planbiir/gfix/internal/merge"
)

func main() {
	cfg := merge.DefaultConfig()

	gapFlag := flag.Duration("gap", cfg.GapThreshold, "Minimum gap duration to consider for merge (e.g. 2m)")
	maxDevFlag := flag.Float64("max-dev", cfg.MaxDeviationMeters, "Maximum allowed deviation for secondary points in meters (negative disables)")
	outFlag := flag.String("out", "", "Optional path to write the merged GPX")
	flag.Parse()

	args := flag.Args()
	if len(args) < 2 {
		log.Fatalf("usage: %s [flags] <primary.gpx> <secondary.gpx>", os.Args[0])
	}

	primaryPath := args[0]
	secondaryPath := args[1]

	cfg.GapThreshold = *gapFlag
	cfg.MaxDeviationMeters = *maxDevFlag

	primary, err := gpx.Parse(primaryPath)
	if err != nil {
		log.Fatalf("parse primary: %v", err)
	}
	secondary, err := gpx.Parse(secondaryPath)
	if err != nil {
		log.Fatalf("parse secondary: %v", err)
	}

	fmt.Printf("Primary: %s\n", primaryPath)
	primaryPoints := primary.FlattenPoints()
	printTrackStats(primaryPoints)

	fmt.Printf("\nSecondary: %s\n", secondaryPath)
	secondaryPoints := secondary.FlattenPoints()
	printTrackStats(secondaryPoints)

	fmt.Printf("\nMerge config: gap_threshold=%v, max_deviation=%.1fm\n", cfg.GapThreshold, cfg.MaxDeviationMeters)

	merged, stats, err := merge.MergeTracks(primary, secondary, cfg)
	if err != nil {
		log.Fatalf("merge failed: %v", err)
	}

	fmt.Printf("\nMerge stats: gaps_detected=%d gaps_filled=%d inserted_points=%d spatial_matches=%d\n",
		stats.GapsDetected, stats.GapsFilled, stats.InsertedPoints, stats.SpatialMatches)

	// Print spatial matching debug information
	if len(stats.SpatialDebugInfo) > 0 {
		fmt.Printf("\nSpatial matching details:\n")
		for _, info := range stats.SpatialDebugInfo {
			fmt.Printf("  Gap #%d: %s -> %s\n", info.GapIndex+1, info.BeforeAnchor, info.AfterAnchor)
			if info.Valid {
				fmt.Printf("    ✓ Inserted: indices %d-%d, distance %.2fkm\n", info.SegmentStart, info.SegmentEnd, info.DistanceKm)
				fmt.Printf("    ✓ Reason: %s\n", info.Reason)
			} else {
				fmt.Printf("    ✗ Rejected: %s\n", info.Reason)
			}
		}
	}

	mergedPoints := merged.FlattenPoints()
	fmt.Printf("\nMerged track summary:\n")
	printTrackStats(mergedPoints)

	inserted := diffPoints(primaryPoints, mergedPoints)
	fmt.Printf("Inserted points (in merged but not primary): %d\n", len(inserted))
	if len(inserted) > 0 {
		sort.Slice(inserted, func(i, j int) bool { return inserted[i].Time.Before(inserted[j].Time) })
		first := inserted[0]
		last := inserted[len(inserted)-1]
		fmt.Printf("  Time span: %s – %s (%v)\n", first.Time, last.Time, last.Time.Sub(first.Time))
		fmt.Printf("  Distance covered: %.3f km\n", trackDistance(inserted)/1000)
	}

	gapDetails := analyzeGaps(primaryPoints, inserted, cfg.GapThreshold)
	fmt.Printf("\nGap analysis (threshold %v):\n", cfg.GapThreshold)
	if len(gapDetails) == 0 {
		fmt.Println("  no gaps exceeding threshold")
	}
	for idx, gap := range gapDetails {
		fmt.Printf("  Gap #%d: %s – %s (duration %v)\n", idx+1, gap.startTime, gap.endTime, gap.duration)
		fmt.Printf("    inserted points: %d\n", len(gap.inserted))
		if len(gap.inserted) > 0 {
			first := gap.inserted[0]
			last := gap.inserted[len(gap.inserted)-1]
			fmt.Printf("    coverage: %s – %s (duration %v)\n", first.Time, last.Time, last.Time.Sub(first.Time))
			fmt.Printf("    pre-gap buffer: %v, post-gap buffer: %v\n", first.Time.Sub(gap.startTime), gap.endTime.Sub(last.Time))
			fmt.Printf("    inserted distance: %.3f km\n", trackDistance(gap.inserted)/1000)
		} else {
			fmt.Printf("    gap left empty\n")
		}
	}

	if *outFlag != "" {
		if err := merged.Write(*outFlag); err != nil {
			log.Fatalf("write merged gpx: %v", err)
		}
		fmt.Printf("\nMerged GPX written to %s\n", *outFlag)
	}
}

type gapInfo struct {
	startTime time.Time
	endTime   time.Time
	duration  time.Duration
	inserted  []gpx.Point
}

func analyzeGaps(primary []gpx.Point, inserted []gpx.Point, threshold time.Duration) []gapInfo {
	result := []gapInfo{}
	if len(primary) < 2 {
		return result
	}
	insertedIdx := 0
	for i := 0; i < len(primary)-1; i++ {
		a := primary[i]
		b := primary[i+1]
		if a.Time.IsZero() || b.Time.IsZero() {
			continue
		}
		gap := b.Time.Sub(a.Time)
		if gap <= threshold {
			continue
		}
		info := gapInfo{startTime: a.Time, endTime: b.Time, duration: gap}
		for insertedIdx < len(inserted) && !inserted[insertedIdx].Time.After(b.Time) {
			pt := inserted[insertedIdx]
			if pt.Time.After(a.Time) && pt.Time.Before(b.Time) {
				info.inserted = append(info.inserted, pt)
			}
			insertedIdx++
		}
		result = append(result, info)
	}
	return result
}

func diffPoints(primary, merged []gpx.Point) []gpx.Point {
	set := make(map[string]struct{}, len(primary))
	for _, pt := range primary {
		set[pointKey(pt)] = struct{}{}
	}
	out := []gpx.Point{}
	for _, pt := range merged {
		if _, exists := set[pointKey(pt)]; !exists {
			out = append(out, pt)
		}
	}
	return out
}

func pointKey(p gpx.Point) string {
	return fmt.Sprintf("%.7f|%.7f|%s", p.Lat, p.Lon, p.Time.UTC().Format(time.RFC3339Nano))
}

func printTrackStats(points []gpx.Point) {
	if len(points) == 0 {
		fmt.Printf("  points: 0\n")
		return
	}
	duration := trackDuration(points)
	distance := trackDistance(points)
	start, end := timeBounds(points)
	fmt.Printf("  points: %d\n", len(points))
	fmt.Printf("  time span: %s – %s (duration %v)\n", start, end, duration)
	fmt.Printf("  distance: %.3f km\n", distance/1000)
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

func trackDuration(points []gpx.Point) time.Duration {
	start, end := timeBounds(points)
	if start.IsZero() || end.IsZero() {
		return 0
	}
	return end.Sub(start)
}

func trackDistance(points []gpx.Point) float64 {
	if len(points) < 2 {
		return 0
	}
	total := 0.0
	for i := 1; i < len(points); i++ {
		total += haversine(points[i-1], points[i])
	}
	return total
}

func haversine(a, b gpx.Point) float64 {
	const earthRadius = 6371000.0
	lat1 := radians(a.Lat)
	lat2 := radians(b.Lat)
	dlat := radians(b.Lat - a.Lat)
	dlon := radians(b.Lon - a.Lon)

	sinLat := math.Sin(dlat / 2)
	sinLon := math.Sin(dlon / 2)

	h := sinLat*sinLat + math.Cos(lat1)*math.Cos(lat2)*sinLon*sinLon
	c := 2 * math.Atan2(math.Sqrt(h), math.Sqrt(1-h))
	return earthRadius * c
}

func radians(d float64) float64 {
	return d * math.Pi / 180
}
