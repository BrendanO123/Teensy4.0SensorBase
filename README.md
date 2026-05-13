# Teensy4.0SensorBase

A comprehensive sensor data acquisition and processing system featuring Teensy 4.0 microcontroller firmware and a C++ post-processing pipeline for log file analysis.

## Project Structure

```
.
├── SensorBase/                 # Teensy 4.0 firmware (Arduino sketch)
│   └── SensorBase.ino         # Main sensor acquisition code
├── PostProcessing/            # Data processing pipeline
│   ├── DataScrapper.cpp       # Main processor executable
│   ├── QuatToEuler.h          # Quaternion to Euler angle converter
│   ├── makefile               # Build and process targets
│   ├── inputs/                # Place .bin files here for processing
│   └── outputs/               # Processed data organized by log number
└── README.md
```

## Quick Start

### Place Input Files

1. Add your `.bin` files to `PostProcessing/inputs/`
2. Run `make all` from the root directory
    - This compiles the data processor and processes all `.bin` files in `PostProcessing/inputs/`.

### Output Files

Processed data is organized in `PostProcessing/outputs/log_NAME/` directories:
- **rawData/** - Separated data streams:
  - `gps_data.csv` - GPS coordinates and altitude
  - `imu_accel_data.csv` - Accelerometer readings
  - `imu_gyro_data.csv` - Gyroscope readings
  - `imu_compass_data.csv` - Magnetometer readings
  - `imu_pressure_data.csv` - Barometric pressure data
  - `imu_rot_data.csv` - Raw rotation quaternions
- **processedData/** - Derived data:
  - `imu_rot_euler.csv` - Euler angles (roll, pitch, yaw)

## Makefile Commands

All commands can be run from the root directory. They forward to `PostProcessing/makefile`.

### `make all`
Compile the data processor and process all `.bin` files in the inputs folder. Creates output directories and processes only files that have changed since the last run.

### `make watch`
Monitor the inputs folder for new `.bin` files and automatically process them. Useful for real-time data ingestion. Press Ctrl+C to stop.

### `make status`
Display the current status of input and output files. Shows which `.bin` files are waiting to be processed and what outputs have been generated.

### `make clean`
Remove the compiled `data_scrapper.out` executable. Useful for forcing a fresh build.

### `make clean-markers`
Reset processing markers so all `.bin` files will be reprocessed on the next `make all`. Keeps all generated CSV files intact.

### `make clean-all`
Equivalent to running both `make clean` and `make clean-markers`. Removes the executable and processing markers but preserves all CSV outputs.

### `make distclean`
Complete cleanup. Removes the executable, markers, and ALL output folders. Use with caution—this deletes all processed data.

### `make help`
Display the help message showing all available targets and their descriptions.

## Usage Examples

### Process new data files
```bash
# Add your .bin file to inputs/
cp my_sensor_log.bin PostProcessing/inputs/

# Process it
make all
```

### Reset and reprocess
```bash
# Mark all for reprocessing but keep the data
make clean-markers
make all

# Or start completely fresh
make distclean
make all
```

## Building from Source

The PostProcessing pipeline is written in C++17 and requires a C++ compiler (e.g., g++). The makefile automatically compiles `DataScrapper.cpp` when needed.

## Dependencies

- C++17 compatible compiler (g++ recommended)
- Standard C++ library
- Make utility

## File Format

Input `.bin` files are expected to contain sensor data in a binary format that `DataScrapper.cpp` can parse. See the source code for detailed format specifications.
