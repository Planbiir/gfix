# gfix - GPS track fixer

APP_NAME = gfix
BUILD_DIR = ./build

.PHONY: build test clean all

# Build the application
build:
	@echo "Building $(APP_NAME)..."
	@mkdir -p $(BUILD_DIR)
	go build -o $(BUILD_DIR)/$(APP_NAME) ./cmd/gfix

# Run tests
test:
	@echo "Running tests..."
	go test -v ./...

# Clean build artifacts
clean:
	@echo "Cleaning..."
	rm -rf $(BUILD_DIR)
	go clean

# Build for all platforms
all:
	@echo "Building for all platforms..."
	@mkdir -p $(BUILD_DIR)
	GOOS=linux GOARCH=amd64 go build -ldflags "-s -w" -o $(BUILD_DIR)/$(APP_NAME)-linux-amd64 ./cmd/gfix
	GOOS=darwin GOARCH=amd64 go build -ldflags "-s -w" -o $(BUILD_DIR)/$(APP_NAME)-darwin-amd64 ./cmd/gfix
	GOOS=darwin GOARCH=arm64 go build -ldflags "-s -w" -o $(BUILD_DIR)/$(APP_NAME)-darwin-arm64 ./cmd/gfix
	GOOS=windows GOARCH=amd64 go build -ldflags "-s -w" -o $(BUILD_DIR)/$(APP_NAME)-windows-amd64.exe ./cmd/gfix

# Help
help:
	@echo "Available targets:"
	@echo "  build    - Build the application"
	@echo "  test     - Run tests"
	@echo "  clean    - Clean build artifacts"
	@echo "  all      - Build for all platforms"
	@echo "  help     - Show this help message"