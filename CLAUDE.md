# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

This is a Go CLI tool that fixes GPS tracking errors in ultra trail running and hiking tracks using mathematically sound outlier detection algorithms. The tool employs Local Outlier Factor (LOF) algorithm with proven mathematical foundations to identify and remove GPS outliers while preserving all performance data (heart rate, power, cadence, etc.).

## Development Commands

### Build and Test
```bash
# Build the application
make build

# Run all tests
make test
go test -v ./...

# Run a specific test
go test -run TestHaversineDistance -v

# Build for all platforms (cross-compilation)
make all

# Clean build artifacts
make clean
```

### Running the Tool
```bash
# Basic usage (from build directory)
./build/gpx-cleaner -i your-track.gpx

# Basic filtering mode (faster, less aggressive)
./build/gpx-cleaner -i your-track.gpx --basic

# Show help
./build/gpx-cleaner --help
```

## Architecture

### Core Algorithm Pipeline

The tool uses mathematically sound outlier detection algorithms:

1. **Local Outlier Factor (LOF)**: Detects GPS outliers based on local density deviation from neighbors with mathematical proof
2. **Basic velocity filtering**: Simple speed-based filtering for cases where computational resources are limited
3. **No distance fitting**: Algorithm works purely on geometric outlier detection without target distance adjustment

### Key Components

**Data Structures:**
- `gpxPoint`: Full GPX track point with extensions (HR, power, cadence, temperature)
- `point`: Simplified coordinate for processing (lat, lon, elevation)
- `neighbor`: k-nearest neighbor data for LOF calculations
- `lofData`: Local Outlier Factor calculation results (k-distance, LRD, LOF score)
- `config`: Application configuration with basic parameters

**Core Functions:**
- `lofOutlierDetection()`: Local Outlier Factor algorithm implementation with parallel processing
- `findKNearestNeighbors()`: k-NN search for LOF density calculations
- `calculateLRD()`: Local Reachability Density calculation (mathematical component of LOF)
- `calculateLOF()`: Local Outlier Factor score calculation (final outlier score)
- `advancedPrecisionFilter()`: LOF-based outlier detection with adaptive k parameter
- `adaptiveFilter()`: Simple velocity-based filtering for basic mode
- `haversineDistance()`: GPS coordinate distance calculation

### Algorithm Behavior

**Mathematical Foundation:**
The LOF algorithm calculates outlier scores based on Local Outlier Factor:
- **LOF ≈ 1**: Point has similar density to neighbors (normal point)
- **LOF >> 1**: Point has much lower density than neighbors (outlier)
- **Threshold**: Points with LOF > 1.5 are classified as outliers

**Two Operating Modes:**
- **Advanced mode** (default): LOF-based mathematical outlier detection with k=5 to k=20 adaptive parameter
- **Basic mode** (`--basic` flag): Simple velocity-based filtering (max 25 m/s speed threshold)

**Algorithm Properties:**
- No target distance fitting - purely geometric outlier detection
- Mathematically proven algorithm with density-based anomaly detection
- Parallel processing for k-nearest neighbor calculations
- Conservative outlier threshold (1.5) to avoid over-filtering

### Performance Optimizations

- Parallel k-nearest neighbor calculation using goroutines and chunk processing
- Efficient sorting algorithms for neighbor distance ranking
- Haversine distance calculation with early termination optimizations
- Minimal memory allocation with pre-sized slices

## Testing Strategy

Unit tests cover:
- Basic distance calculations (Haversine formula accuracy)
- LOF algorithm effectiveness with synthetic outliers
- Over-aggressive filtering prevention (<10% point removal on clean tracks)
- Basic velocity filter reasonableness for trail running speeds
- LOF outlier detection accuracy with known test cases

## File Structure

- `main.go`: Single-file CLI application with all functionality
- `main_test.go`: Unit tests for core algorithms
- `Makefile`: Build automation and cross-platform compilation
- GitHub Actions: CI/CD for automated testing and releases

## Development Notes

**Go Version**: Requires Go 1.24+ for modern language features (min/max functions)

**Dependencies**: Minimal - only Cobra for CLI and golang.org/x/sync for concurrent processing

**Cross-platform**: Builds for Windows, macOS (Intel/ARM), and Linux with optimized binaries (`-ldflags "-s -w"`)