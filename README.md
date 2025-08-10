# GPX Track Cleaner

Fix GPS errors in your running, hiking, or cycling tracks. Removes GPS spikes and phantom distance while keeping real data like stops at aid stations.

## What It Does

- Removes GPS errors that add fake distance to your tracks
- Keeps real stops (like aid station breaks)
- Works with any GPX file from Garmin, Strava, etc.
- No configuration needed - just works

## Download

1. Go to [Releases](https://github.com/timofeipermiakov/gpx-precision-cleaner/releases)
2. Download the file for your computer:
   - **Windows**: `gpx-cleaner-windows-amd64.exe`
   - **Mac (Intel)**: `gpx-cleaner-darwin-amd64`
   - **Mac (M1/M2/M3)**: `gpx-cleaner-darwin-arm64`
   - **Linux**: `gpx-cleaner-linux-amd64`

## How to Use

### Windows
1. Download `gpx-cleaner-windows-amd64.exe`
2. Put it in the same folder as your GPX file
3. Open Command Prompt in that folder
4. Type: `gpx-cleaner-windows-amd64.exe -i your-track.gpx`

### Mac
1. Download the Mac version
2. Open Terminal
3. Navigate to where you downloaded it
4. Type: `chmod +x gpx-cleaner-darwin-*` (makes it runnable)
5. Type: `./gpx-cleaner-darwin-* -i your-track.gpx`

### Linux
1. Download the Linux version
2. Open terminal
3. Type: `chmod +x gpx-cleaner-linux-amd64`
4. Type: `./gpx-cleaner-linux-amd64 -i your-track.gpx`

## Result

Your cleaned track will be saved as `your-track_cleaned.gpx` in the same folder.

## Before/After Examples

### Before Cleaning
![Before](docs/before.png)
*Track with GPS errors and phantom distance*

### After Cleaning
![After](docs/after.png)
*Clean track with errors removed*

## Common Problems It Fixes

- "Phantom distance" from GPS jumping around
- Spikes that make your track look like you teleported
- GPS errors in mountains or forests
- Tracks that show unrealistic speeds

## What It Preserves

- Real stops at aid stations
- Heart rate and power data
- Elevation information
- Timestamps

---

## For Developers

### Install with Go
```bash
go install github.com/timofeipermiakov/gpx-precision-cleaner@latest
```

### Build from Source
```bash
git clone https://github.com/timofeipermiakov/gpx-precision-cleaner.git
cd gpx-precision-cleaner
go build -o gpx-cleaner .
```

### Run Tests
```bash
make test
golangci-lint run
```