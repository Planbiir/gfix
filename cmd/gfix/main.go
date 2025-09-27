package main

import (
	"encoding/json"
	"flag"
	"fmt"
	"os"
	"path/filepath"
	"strings"

	"github.com/planbiir/gfix/internal/clean"
	"github.com/planbiir/gfix/internal/gpx"
)

func main() {
	var (
		inputFile  = flag.String("i", "", "Input GPX file")
		outputFile = flag.String("o", "", "Output GPX file (default: <input>_cleaned.gpx)")
		maxSpeed   = flag.Float64("max-speed", 0, "Maximum speed in m/s (auto-detect if 0)")
		minPause   = flag.Float64("min-pause", 120, "Minimum pause duration in seconds")
		dryRun     = flag.Bool("dry-run", false, "Show statistics without writing output file")
		showStats  = flag.Bool("stats", false, "Show detailed statistics")
		statsJSON  = flag.Bool("stats-json", false, "Output statistics as JSON")
		version    = flag.Bool("version", false, "Show version information")
	)

	flag.Usage = func() {
		fmt.Printf("gfix - Fix GPS errors in GPX tracks\n\n")
		fmt.Printf("usage: gfix -i /path/to/file.gpx\n\n")
		fmt.Printf("examples:\n")
		fmt.Printf("  gfix -i track.gpx\n")
		fmt.Printf("  gfix -i \"My Activity.gpx\"\n")
		fmt.Printf("  ./gfix-linux-amd64 -i track.gpx\n")
		fmt.Printf("  gfix-windows-amd64.exe -i \"C:\\Downloads\\track.gpx\"\n\n")
		fmt.Printf("options:\n")
		flag.PrintDefaults()
	}

	flag.Parse()

	if *version {
		fmt.Println("gfix v1.1.0 - GPS track cleaner")
		fmt.Println("https://github.com/planbiir/gfix")
		os.Exit(0)
	}

	if *inputFile == "" {
		flag.Usage()
		os.Exit(2)
	}

	// Generate output filename if not provided
	if *outputFile == "" {
		ext := filepath.Ext(*inputFile)
		base := strings.TrimSuffix(*inputFile, ext)
		*outputFile = base + "_cleaned" + ext
	}

	// Parse GPX file
	fmt.Printf("📖 Reading GPX file: %s\n", *inputFile)
	gpxData, err := gpx.Parse(*inputFile)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error reading GPX file: %v\n", err)
		os.Exit(1)
	}

	// Get all points from GPX
	points := gpxData.FlattenPoints()
	if len(points) == 0 {
		fmt.Printf("❌ No GPS points found in file\n")
		os.Exit(1)
	}

	fmt.Printf("📊 Original track: %d points across %d tracks\n", len(points), len(gpxData.Tracks))

	// Convert GPX points to cleaning points
	cleanPoints := make([]clean.Point, len(points))
	for i, p := range points {
		cleanPoints[i] = clean.Point{
			Lat:       p.Lat,
			Lon:       p.Lon,
			Elevation: p.Elevation,
			Time:      p.Time,
			TrackIdx:  p.TrackIdx,
			SegIdx:    p.SegIdx,
			PtIdx:     p.PtIdx,
		}
	}

	// Configure cleaning parameters
	config := clean.DefaultConfig()
	if *maxSpeed > 0 {
		config.MaxSpeed = *maxSpeed
	}
	if *minPause > 0 {
		config.MinPauseDuration = *minPause
	}

	// Clean the track
	result, err := clean.Clean(cleanPoints, config)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error cleaning track: %v\n", err)
		os.Exit(1)
	}

	// Show statistics
	if *showStats || *statsJSON || *dryRun {
		if *statsJSON {
			jsonData, err := json.MarshalIndent(result.Stats, "", "  ")
			if err != nil {
				fmt.Fprintf(os.Stderr, "Error marshaling stats: %v\n", err)
				os.Exit(1)
			}
			fmt.Println(string(jsonData))
		} else {
			printStats(result.Stats)
		}
	}

	// Exit if dry run
	if *dryRun {
		fmt.Printf("🔍 Dry run completed - no files written\n")
		os.Exit(0)
	}

	// Convert cleaned points back to GPX points
	cleanedGPXPoints := make([]gpx.Point, len(result.Points))
	for i, p := range result.Points {
		cleanedGPXPoints[i] = gpx.Point{
			Lat:       p.Lat,
			Lon:       p.Lon,
			Elevation: p.Elevation,
			Time:      p.Time,
			TrackIdx:  p.TrackIdx,
			SegIdx:    p.SegIdx,
			PtIdx:     p.PtIdx,
		}
	}

	// Rebuild GPX structure
	gpxData.RebuildFromPoints(cleanedGPXPoints)

	// Write cleaned GPX
	fmt.Printf("💾 Writing cleaned track: %s\n", *outputFile)
	if err := gpxData.Write(*outputFile); err != nil {
		fmt.Fprintf(os.Stderr, "Error writing GPX file: %v\n", err)
		os.Exit(1)
	}

	fmt.Printf("✅ Track cleaned successfully!\n")
	fmt.Printf("   %d → %d points (%.1f%% removed)\n", 
		result.Stats.OriginalPoints, result.Stats.FinalPoints, result.Stats.PointsPercent)
	fmt.Printf("   %.1f → %.1f km (%.1f%% reduced)\n", 
		result.Stats.OriginalDistance, result.Stats.FinalDistance, result.Stats.DistancePercent)
}

func printStats(stats clean.Stats) {
	fmt.Printf("\n📊 Cleaning Statistics:\n")
	fmt.Printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
	fmt.Printf("🎯 Activity Type: %s\n", stats.ActivityType)
	fmt.Printf("📍 Points: %d → %d (%d removed, %.1f%%)\n", 
		stats.OriginalPoints, stats.FinalPoints, stats.PointsRemoved, stats.PointsPercent)
	fmt.Printf("📏 Distance: %.2f → %.2f km (%.2f km reduced, %.1f%%)\n", 
		stats.OriginalDistance, stats.FinalDistance, stats.DistanceReduced, stats.DistancePercent)
	fmt.Printf("⚡ Speed Detection: P95=%.1f m/s, Max=%.1f m/s\n", 
		stats.P95Speed, stats.DetectedMaxSpeed)
	fmt.Printf("🔄 Processing Steps:\n")
	fmt.Printf("   • Velocity filter: %d points\n", stats.VelocityFiltered)
	fmt.Printf("   • LOF filter: %d points\n", stats.LofFiltered)
	fmt.Printf("   • Final result: %d points\n", stats.FinalPoints)
	fmt.Printf("⏱️  Processing Time: %v\n", stats.ProcessingTime)
	fmt.Printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n")
}