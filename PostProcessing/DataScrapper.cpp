#include <iostream>
#include <string>
#include <fstream>
#include <queue>
#include <filesystem>
#include "QuatToEuler.h"

using namespace std;

enum DataHeaderType : unsigned char{
    GPS_DATA = 1,
    IMU_ROT_DATA = 2, 
    IMU_RAW_DATA = 4,
    IMU_ACCEL_DATA = 8,
    IMU_COMPASS_DATA = 16,
    IMU_GYRO_DATA = 32,
    IMU_PRESSURE_DATA = 64
};

struct GPSDump{
    uint32_t timestamp;
    uint8_t fixquality;
    float lat, lon, alt;
    char latDir, lonDir;
    float speed;
    float HDOP;
    int satellites;
    
    GPSDump(uint32_t ts, uint8_t fq, float la, float lo, float al, char ld, char lod, float s, float h, int sat)
        : timestamp(ts), fixquality(fq), lat(la), lon(lo), alt(al), latDir(ld), lonDir(lod), speed(s), HDOP(h), satellites(sat) {}
};
struct IMURotDump{
    uint32_t timestamp;
    double Q1, Q2, Q3; int16_t qAccuracy;
    double roll, pitch, yaw;
    
    IMURotDump(uint32_t ts, double q1, double q2, double q3, int16_t qa)
        : timestamp(ts), Q1(q1), Q2(q2), Q3(q3), qAccuracy(qa) {
        EulerAngles angles = quaternionToEuler(q1, q2, q3);
        roll = angles.roll;
        pitch = angles.pitch;
        yaw = angles.yaw;
    }
};
struct IMUAccelDump{
    uint32_t timestamp;
    int16_t ax, ay, az, aAccuracy;
    
    IMUAccelDump(uint32_t ts, int16_t aax, int16_t aay, int16_t aaz, int16_t aa)
        : timestamp(ts), ax(aax), ay(aay), az(aaz), aAccuracy(aa) {}
};
struct IMUCompassDump{
    uint32_t timestamp;
    int16_t cx, cy, cz, cAccuracy;
    
    IMUCompassDump(uint32_t ts, int16_t ccx, int16_t ccy, int16_t ccz, int16_t ca)
        : timestamp(ts), cx(ccx), cy(ccy), cz(ccz), cAccuracy(ca) {}
};
struct IMUGyroDump{
    uint32_t timestamp;
    int16_t gx, gy, gz, gAccuracy;
    
    IMUGyroDump(uint32_t ts, int16_t ggx, int16_t ggy, int16_t ggz, int16_t ga)
        : timestamp(ts), gx(ggx), gy(ggy), gz(ggz), gAccuracy(ga) {}
};
struct IMUPressureDump{
    uint32_t timestamp;
    int16_t pressure; int8_t temperature[3];
    
    IMUPressureDump(uint32_t ts, int16_t p, int8_t* t)
        : timestamp(ts), pressure(p) {
        copy(t, t + 3, temperature);
    }
};


struct DataDump{
    queue<GPSDump> GPSData;
    queue<IMURotDump> IMURotData;
    queue<IMUAccelDump> IMUAccelData;
    queue<IMUCompassDump> IMUCompassData;
    queue<IMUGyroDump> IMUGyroData;
    queue<IMUPressureDump> IMUPressureData;
};

inline uint8_t getByte(char * contents, int& offset){
    uint8_t output;
    copy(contents + offset, contents + offset + sizeof(uint8_t), (char*)&output);
    offset += sizeof(uint8_t);
    return output;
}
inline int16_t getShort(char * contents, int& offset){
    int16_t output;
    copy(contents + offset, contents + offset + sizeof(int16_t), (char*)&output);
    offset += sizeof(int16_t);
    return output;
}
inline int getInt(char * contents, int& offset){
    int output;
    copy(contents + offset, contents + offset + sizeof(int), (char*)&output);
    offset += sizeof(int);
    return output;
}
inline long getLong(char * contents, int& offset){
    long int output;
    copy(contents + offset, contents + offset + sizeof(long int), (char*)&output);
    offset += sizeof(long int);
    return output;
}
inline float getFloat(char * contents, int& offset){
    float output;
    copy(contents + offset, contents + offset + sizeof(float), (char*)&output);
    offset += sizeof(float);
    return output;
}
inline double getDouble(char * contents, int& offset){
    double output;
    copy(contents + offset, contents + offset + sizeof(double), (char*)&output);
    offset += sizeof(double);
    return output;
}

// ===== Unit Conversion Helper Functions =====

// Convert raw accelerometer value to m/s² (SI unit)
// fullScaleRange: 0=±2g, 1=±4g, 2=±8g, 3=±16g
inline float convertAccelToMS2(int16_t rawValue, int fullScaleRange) {
    float scaleFactors[] = {16.384, 8.192, 4.096, 2.048};
    if (fullScaleRange < 0 || fullScaleRange > 3) fullScaleRange = 1; // Default to ±4g
    float milliGs = ((float)rawValue) / scaleFactors[fullScaleRange];
    return milliGs * 0.00981; // Convert milli-g to m/s² (1g = 9.81 m/s²)
}

// Convert raw gyroscope value to rad/s (SI unit)
// fullScaleRange: 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
inline float convertGyroToRadPerS(int16_t rawValue, int fullScaleRange) {
    float scaleFactors[] = {131.0, 65.5, 32.8, 16.4};
    if (fullScaleRange < 0 || fullScaleRange > 3) fullScaleRange = 0; // Default to ±250°/s
    float degreesPerSecond = ((float)rawValue) / scaleFactors[fullScaleRange];
    return degreesPerSecond * (3.14159265359 / 180.0); // Convert °/s to rad/s
}

// Convert raw magnetometer value to Tesla (SI unit)
inline float convertMagToTesla(int16_t rawValue) {
    float microTeslas = ((float)rawValue) * 0.15;
    return microTeslas * 1e-6; // Convert µT to T (1 µT = 1e-6 T)
}

// Alternative: Convert raw magnetometer to microTeslas (more practical unit)
inline float convertMagToMicroTesla(int16_t rawValue) {
    return ((float)rawValue) * 0.15;
}

// Convert raw temperature value to Celsius
// Formula from ICM-20948 datasheet: °C = (raw_value / 333.87) + 21
inline float convertTempToC(int16_t rawValue) {
    return (((float)rawValue) / 333.87) + 21.0;
}

// Convert temperature from Celsius to Fahrenheit
inline float celsiusToFahrenheit(float celsius) {
    return (celsius * 9.0 / 5.0) + 32.0;
}
//TODO: pressure conversion 

// Convert GPS coordinates from DDMM.MMMM (degrees/minutes) format to decimal degrees
// Input: coordinate in DDMM.MMMM format, direction ('N', 'S', 'E', 'W')
// Output: decimal degrees (positive for N/E, negative for S/W)
inline float convertToDecimalDegrees(float coordinate, char direction) {
    int degrees = (int)(coordinate / 100.0);
    float minutes = coordinate - (degrees * 100.0);
    float decimalDegrees = degrees + (minutes / 60.0);
    
    // Apply sign based on direction
    if (direction == 'S' || direction == 'W') {
        decimalDegrees = -decimalDegrees;
    }
    
    return decimalDegrees;
}
constexpr const double Kn_toMs = 1852.0 / 3600.0;
inline float convertKnotsToMPS(float knots) {
    return knots * Kn_toMs;
}

DataDump* readFile(string filePath){
    ifstream file(filePath.c_str(), std::ios::binary | std::ios::ate | std::ios::in);
    if(file.is_open() && file.good()){
        streamsize size = file.tellg();
        file.seekg(0, std::ios::beg);
        char* contents = new char[size];
        file.read(contents, size);
        file.close();

        DataDump* data = new DataDump;
        int index = 0;
        while(index < size){
            char header = contents[index];
            index++; // Consume the header byte
            if(header & IMU_ROT_DATA){
                int timestamp = getInt(contents, index);
                double Q1 = getDouble(contents, index);
                double Q2 = getDouble(contents, index);
                double Q3 = getDouble(contents, index);
                int16_t Accuracy = getShort(contents, index);
                data->IMURotData.emplace(timestamp, Q1, Q2, Q3, Accuracy);
            }
            if(header & IMU_RAW_DATA){
                if(header & IMU_ACCEL_DATA){
                    int timestamp = getInt(contents, index);
                    int16_t ax = getShort(contents, index);
                    int16_t ay = getShort(contents, index);
                    int16_t az = getShort(contents, index);
                    int16_t aAccuracy = getShort(contents, index);
                    data->IMUAccelData.emplace(timestamp, ax, ay, az, aAccuracy);
                }
                if(header & IMU_COMPASS_DATA){
                    int timestamp = getInt(contents, index);
                    int16_t cx = getShort(contents, index);
                    int16_t cy = getShort(contents, index);
                    int16_t cz = getShort(contents, index);
                    int16_t cAccuracy = getShort(contents, index);
                    data->IMUCompassData.emplace(timestamp, cx, cy, cz, cAccuracy);
                }
                if(header & IMU_GYRO_DATA){
                    int timestamp = getInt(contents, index);
                    int16_t gx = getShort(contents, index);
                    int16_t gy = getShort(contents, index);
                    int16_t gz = getShort(contents, index);
                    int16_t gAccuracy = getShort(contents, index);
                    data->IMUGyroData.emplace(timestamp, gx, gy, gz, gAccuracy);
                }
                if(header & IMU_PRESSURE_DATA){
                    int timestamp = getInt(contents, index);
                    int16_t pressure = getShort(contents, index);
                    int8_t temperature[3];
                    copy(contents + index, contents + index + 3, (char*)temperature);
                    index += 3;
                    data->IMUPressureData.emplace(timestamp, pressure, temperature);
                }
            }
            if(header & GPS_DATA){
                int timestamp = getInt(contents, index);
                uint8_t fix = getByte(contents, index);
                float lat = getFloat(contents, index);
                float lon = getFloat(contents, index);
                float alt = getFloat(contents, index);
                uint8_t latDir = getByte(contents, index);
                uint8_t lonDir = getByte(contents, index);
                float speed = getFloat(contents, index);
                float HDOP = getFloat(contents, index);
                int satellites = getInt(contents, index);
                data->GPSData.emplace(timestamp, fix, lat, lon, alt, latDir, lonDir, speed, HDOP, satellites);
            }
        }
        return data;
    }
    else{file.close(); std :: cout << "FAILED TO OPEN FILE: " << filePath << std :: endl;}
    return nullptr;
}
bool makeCSVs(DataDump* data, string outputDir){
    // Ensure output directory exists
    filesystem::create_directories(outputDir);
    
    // Create subdirectories for raw and processed data
    string rawDataDir = outputDir + "/rawData";
    string processedDataDir = outputDir + "/processedData";
    filesystem::create_directories(rawDataDir);
    filesystem::create_directories(processedDataDir);
    
    // Construct full paths for output files
    string gpsPath = rawDataDir + "/gps_data.csv";
    string imuRotPath = rawDataDir + "/imu_rot_data.csv";
    string imuRotEulerPath = processedDataDir + "/imu_rot_euler.csv";
    string imuAccelPath = rawDataDir + "/imu_accel_data.csv";
    string imuCompassPath = rawDataDir + "/imu_compass_data.csv";
    string imuGyroPath = rawDataDir + "/imu_gyro_data.csv";
    string imuPressurePath = rawDataDir + "/imu_pressure_data.csv";
    string imuProcessedSIPath = processedDataDir + "/imu_processed_SI.csv"; // celsius, m/s², teslas, etc
    string imuProcessedReadableUnitsPath = processedDataDir + "/imu_processed_readable.csv"; // fahrenheit, micro-teslas, etc
    //TODO: add processed data outputs with converted SI units and more human readable units (fahrenheit, micro-teslas, etc)
    
    ofstream gpsFile(gpsPath);
    if(gpsFile.is_open()){
        gpsFile << "timestamp,fixquality,lat,lon,alt,latDir,lonDir,speed,HDOP,satellites\n";
        while(!data->GPSData.empty()){
            GPSDump dump = data->GPSData.front(); data->GPSData.pop();
            gpsFile << dump.timestamp << "," << (int)dump.fixquality << "," << dump.lat << "," << dump.lon << "," << dump.alt << "," << dump.latDir << "," << dump.lonDir << "," << dump.speed << "," << dump.HDOP << "," << dump.satellites << "\n";
        }
        gpsFile.close();
        cout << "Created: " << gpsPath << endl;
    }
    else{cout << "FAILED TO OPEN GPS CSV FILE: " << gpsPath << endl; return false;}

    ofstream imuRotFile(imuRotPath);
    ofstream imuRotEulerFile(imuRotEulerPath);
    
    if(imuRotFile.is_open() && imuRotEulerFile.is_open()){
        imuRotFile << "timestamp,Q1,Q2,Q3,qAccuracy\n";
        imuRotEulerFile << "timestamp,roll,pitch,yaw\n";
        
        while(!data->IMURotData.empty()){
            IMURotDump dump = data->IMURotData.front(); data->IMURotData.pop();
            imuRotFile << dump.timestamp << "," << dump.Q1 << "," << dump.Q2 << "," << dump.Q3 << "," << dump.qAccuracy << "\n";
            imuRotEulerFile << dump.timestamp << "," << dump.roll << "," << dump.pitch << "," << dump.yaw << "\n";
        }
        imuRotFile.close();
        imuRotEulerFile.close();
        cout << "Created: " << imuRotPath << endl;
        cout << "Created: " << imuRotEulerPath << endl;
    }
    else{
        if(imuRotFile.is_open()) imuRotFile.close();
        if(imuRotEulerFile.is_open()) imuRotEulerFile.close();
        cout << "FAILED TO OPEN IMU ROT CSV FILES" << endl;
        return false;
    }

    ofstream imuAccelFile(imuAccelPath);
    if(imuAccelFile.is_open()){
        imuAccelFile << "timestamp,ax,ay,az,aAccuracy\n";
        while(!data->IMUAccelData.empty()){
            IMUAccelDump dump = data->IMUAccelData.front(); data->IMUAccelData.pop();
            imuAccelFile << dump.timestamp << "," << dump.ax << "," << dump.ay << "," << dump.az << "," << dump.aAccuracy << "\n";
        }
        imuAccelFile.close();
        cout << "Created: " << imuAccelPath << endl;
    }
    else{cout << "FAILED TO OPEN IMU ACCEL CSV FILE: " << imuAccelPath << endl; return false;}

    ofstream imuCompassFile(imuCompassPath);
    if(imuCompassFile.is_open()){
        imuCompassFile << "timestamp,cx,cy,cz,cAccuracy\n";
        while(!data->IMUCompassData.empty()){
            IMUCompassDump dump = data->IMUCompassData.front(); data->IMUCompassData.pop();
            imuCompassFile << dump.timestamp << "," << dump.cx << "," << dump.cy << "," << dump.cz << "," << dump.cAccuracy << "\n";
        }
        imuCompassFile.close();
        cout << "Created: " << imuCompassPath << endl;
    }
    else{cout << "FAILED TO OPEN IMU COMPASS CSV FILE: " << imuCompassPath << endl; return false;}

    ofstream imuGyroFile(imuGyroPath);
    if(imuGyroFile.is_open()){
        imuGyroFile << "timestamp,gx,gy,gz,gAccuracy\n";
        while(!data->IMUGyroData.empty()){
            IMUGyroDump dump = data->IMUGyroData.front(); data->IMUGyroData.pop();
            imuGyroFile << dump.timestamp << "," << dump.gx << "," << dump.gy << "," << dump.gz << "," << dump.gAccuracy << "\n";
        }
        imuGyroFile.close();
        cout << "Created: " << imuGyroPath << endl;
    }
    else{cout << "FAILED TO OPEN IMU GYRO CSV FILE: " << imuGyroPath << endl; return false;}

    ofstream imuPressureFile(imuPressurePath);
    if(imuPressureFile.is_open()){
        imuPressureFile << "timestamp,pressure,temperature\n";
        while(!data->IMUPressureData.empty()){
            IMUPressureDump dump = data->IMUPressureData.front(); data->IMUPressureData.pop();
            imuPressureFile << dump.timestamp << "," << dump.pressure << "," << (int)dump.temperature[0] << "," << (int)dump.temperature[1] << "," << (int)dump.temperature[2] << "\n";
        }
        imuPressureFile.close();
        cout << "Created: " << imuPressurePath << endl;
    }
    else{cout << "FAILED TO OPEN IMU PRESSURE CSV FILE: " << imuPressurePath << endl; return false;}

    return true;
}

int main(int argc, char* argv[]){
    if(argc < 2){
        cerr << "Usage: " << argv[0] << " <input_file> [output_directory]" << endl;
        cerr << "Example: " << argv[0] << " inputs/sensor_data.bin outputs/sensor_data" << endl;
        return 1;
    }
    
    string inputFile = argv[1];
    filesystem::path inputPath(inputFile);
    string baseName = inputPath.stem().string();
    
    filesystem::path outputPath;
    if(argc >= 3){
        outputPath = argv[2];
    }
    else{
        outputPath = filesystem::path("outputs") / baseName;
    }
    
    cout << "Reading: " << inputFile << endl;
    auto data = readFile(inputFile);
    
    if(data == nullptr){
        cerr << "Failed to read input file: " << inputFile << endl;
        return 1;
    }
    
    cout << "Processing data into: " << outputPath.string() << endl;
    if(!makeCSVs(data, outputPath.string())){
        cerr << "Failed to create CSV files" << endl;
        return 1;
    }
    
    cout << "Successfully processed " << baseName << " - output in " << outputPath.string() << endl;
    
    delete data;
    return 0;
}