// Libraries
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GPS.h>
#include <SD.h>
#include "ICM_20948.h"
#include <string.h>
#include <sstream>
using namespace std;

/************** CONFIGURATION **************/

// ===== Select SD interface =====
//#define USE_SDIO   1

// ===== Serial Port Selection =====
#define GPS_SERIAL      Serial3
#define SERIAL_PORT     Serial

// ===== Pin Assignments =====
#define PIN_GPS_PPS     9    // PPS interrupt pin

#define PIN_IMU_INT     20    // IMU data-ready interrupt
#define PIN_IMU_FSYNC   22    // FSYNC output (optional sync pulse)

#define PIN_SD_CS       10

#define PIN_STATUS_LED  16

// ===== Sampling Rates =====
#define IMU_SAMPLE_HZ   100
#define LOG_BLOCK_SIZE  4096 

// ===== I2C Settings =====
#define IMU_WIRE_PORT Wire // Your desired Wire port.
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#define calcEulerAngles true
#define WAIT_FOR_GPS false

// Vars
Adafruit_GPS GPS(&GPS_SERIAL);
File logFile;
ICM_20948_I2C myICM;

string DataFileName;
char logBuffer[LOG_BLOCK_SIZE];
volatile uint32_t logIndex = 0;

volatile uint32_t GPSPingMicros = 0;
volatile uint32_t IMUPingMicros = 0;

bool firstPPS = true;
volatile bool GPSDataReady = false;
volatile bool imuDataReady = false;

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
};
struct IMUDump{
    uint32_t timestamp;

    double Q1, Q2, Q3; int16_t qAccuracy;
    int16_t ax, ay, az, aAccuracy;
    int16_t cx, cy, cz, cAccuracy;
    int16_t gx, gy, gz, gAccuracy;
    int16_t pressure; int8_t temperature[3];
};

struct dataBlock {
    unsigned char type = 0;
    GPSDump GPSData;
    IMUDump IMUData;
};

// PPS Interrupt
void gpsPPS_ISR() {
    if(GPSDataReady == true){return;}
    GPSPingMicros = micros();
    GPSDataReady = true;

    if(firstPPS){
        digitalWrite(PIN_IMU_FSYNC, HIGH);
        digitalWrite(PIN_IMU_FSYNC, LOW);
        firstPPS = false;
    }
}

// IMMU data ready callback
void imuINT_ISR() {
    if(imuDataReady == true){return;}
    IMUPingMicros = micros();
    imuDataReady = true;
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_STATUS_LED, OUTPUT);
    digitalWrite(PIN_STATUS_LED, HIGH);

    Serial.println(F("Initialization Started"));

    pinMode(PIN_GPS_PPS, INPUT);
    pinMode(PIN_IMU_INT, INPUT_PULLUP);
    pinMode(PIN_IMU_FSYNC, OUTPUT);
    
    digitalWrite(PIN_IMU_FSYNC, LOW);

    // 9600 NMEA is the default baud rate for Adafruit MTK GPS's
    GPS_SERIAL.begin(9600);
    GPS.begin(9600);  // Baud rate for UART GPS

    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);

    initIMU();

    if(WAIT_FOR_GPS){
        Serial.println(F("Searching for Fix"));
        while(GPS.fixquality == 0){
            if(GPS.newNMEAreceived()){GPS.parse(GPS.lastNMEA());}
            delay(10);
        }
        Serial.println(F("Fix good"));
    }

    initSD();
    
    attachInterrupt(digitalPinToInterrupt(PIN_GPS_PPS), gpsPPS_ISR, RISING); // this one is weird and goes on rising
    attachInterrupt(digitalPinToInterrupt(PIN_IMU_INT), imuINT_ISR, FALLING);

    firstPPS = true;
    Serial.println(F("Initialization Complete"));
    digitalWrite(PIN_STATUS_LED, LOW);

    while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
    SERIAL_PORT.read();
}

void initSD() {
    SPI.begin();
    if(!SD.begin(PIN_SD_CS)) {
        Serial.println(F("SD Card Init Failed"));
        exit(1);
    }
    
    Wire1.begin();
    int i = 0;
    while(true){
        std::stringstream ss;
        ss << "log_" << i << ".bin";
        DataFileName = ss.str();
        if(!SD.exists(DataFileName.c_str())){break;}
        i++;
    }
    logFile = SD.open(DataFileName.c_str(), FILE_WRITE);
    logFile.truncate();
    Serial.print(F("SD Card Started. Log File: "));
    Serial.println(DataFileName.c_str());
}

void initIMU(){
    Wire.begin();
    Wire.setClock(400000);  // Fast I2C

    bool initialized = false;
    while (!initialized)
    {
        myICM.begin(IMU_WIRE_PORT, AD0_VAL);
        SERIAL_PORT.print(F("Initialization of the sensor returned: "));
        SERIAL_PORT.println(myICM.statusString());
        if (myICM.status != ICM_20948_Stat_Ok)
        {
        SERIAL_PORT.println("Trying again...");
        delay(500);
        }
        else
        {
        initialized = true;
        }
    }

    bool success = true; // Use success to show if the DMP configuration was successful

    // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
    success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

    // DMP sensor options are defined in ICM_20948_DMP.h
    //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
    //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
    //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
    //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
    //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
    //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
    //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
    //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
    //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
    //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

    // Enable the DMP Game Rotation Vector sensor
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

    // Enable any additional sensors / features
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
    success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
    // DMP is capable of outputting multiple sensor data at different rates to FIFO.
    // Setting value can be calculated as follows:
    // Value = (DMP running rate / ODR ) - 1
    // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
    success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum

    // Enable the FIFO
    success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

    // Enable the DMP
    success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

    // Reset DMP
    success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

    // Reset FIFO
    success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

    // Check success
    if (!success)
    {
        SERIAL_PORT.println(F("Enable DMP failed!"));
        SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
        while (1)
        ;
    }

    // Now wake the sensor up
    myICM.sleep(false);
    myICM.lowPower(false);

    // Set Gyro and Accelerometer to a particular sample mode
    // options: ICM_20948_Sample_Mode_Continuous
    //          ICM_20948_Sample_Mode_Cycled
    /*myICM.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), ICM_20948_Sample_Mode_Cycled);
    SERIAL_PORT.print(F("setSampleMode returned: "));
    SERIAL_PORT.println(myICM.statusString());*/

    // Set full scale ranges for both acc and gyr
    ICM_20948_fss_t myFSS; // This uses a "Full Scale Settings" structure that can contain values for all configurable sensors

    myFSS.a = gpm4; // (ICM_20948_ACCEL_CONFIG_FS_SEL_e)
                    // gpm2
                    // gpm4
                    // gpm8
                    // gpm16

    myICM.setFullScale((ICM_20948_Internal_Acc), myFSS);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("setFullScale returned: "));
        SERIAL_PORT.println(myICM.statusString());
    }

    /*ICM_20948_smplrt_t mySmplrt;
    mySmplrt.g = 1125 / IMU_SAMPLE_HZ - 1;
    mySmplrt.a = 1125 / IMU_SAMPLE_HZ - 1;
    myICM.setSampleRate(ICM_20948_Internal_Gyr, mySmplrt);
    SERIAL_PORT.print(F("setSampleRate returned: "));
    SERIAL_PORT.println(myICM.statusString());*/

    // Set up Digital Low-Pass Filter configuration
    /*ICM_20948_dlpcfg_t myDLPcfg;    // Similar to FSS, this uses a configuration structure for the desired sensors
    myDLPcfg.a = acc_d473bw_n499bw; // (ICM_20948_ACCEL_CONFIG_DLPCFG_e)
                                    // acc_d246bw_n265bw      - means 3db bandwidth is 246 hz and nyquist bandwidth is 265 hz
                                    // acc_d111bw4_n136bw
                                    // acc_d50bw4_n68bw8
                                    // acc_d23bw9_n34bw4
                                    // acc_d11bw5_n17bw
                                    // acc_d5bw7_n8bw3        - means 3 db bandwidth is 5.7 hz and nyquist bandwidth is 8.3 hz
                                    // acc_d473bw_n499bw

    myDLPcfg.g = gyr_d361bw4_n376bw5; // (ICM_20948_GYRO_CONFIG_1_DLPCFG_e)
                                        // gyr_d196bw6_n229bw8
                                        // gyr_d151bw8_n187bw6
                                        // gyr_d119bw5_n154bw3
                                        // gyr_d51bw2_n73bw3
                                        // gyr_d23bw9_n35bw9
                                        // gyr_d11bw6_n17bw8
                                        // gyr_d5bw7_n8bw9
                                        // gyr_d361bw4_n376bw5

    myICM.setDLPFcfg((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myDLPcfg);
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("setDLPcfg returned: "));
        SERIAL_PORT.println(myICM.statusString());
    }

    // Choose whether or not to use DLPF
    // Here we're also showing another way to access the status values, and that it is OK to supply individual sensor masks to these functions
    ICM_20948_Status_e accDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Acc, true);
    ICM_20948_Status_e gyrDLPEnableStat = myICM.enableDLPF(ICM_20948_Internal_Gyr, true);
    SERIAL_PORT.print(F("Enable DLPF for Accelerometer returned: "));
    SERIAL_PORT.println(myICM.statusString(accDLPEnableStat));
    SERIAL_PORT.print(F("Enable DLPF for Gyroscope returned: "));
    SERIAL_PORT.println(myICM.statusString(gyrDLPEnableStat));*/

    myICM.startupMagnetometer();
    if (myICM.status != ICM_20948_Stat_Ok)
    {
        SERIAL_PORT.print(F("startupMagnetometer returned: "));
        SERIAL_PORT.println(myICM.statusString());
    }

    myICM.cfgIntActiveLow(true);  // Active low to be compatible with the breakout board's pullup resistor
    myICM.cfgIntOpenDrain(false); // Push-pull, though open-drain would also work thanks to the pull-up resistors on the breakout
    myICM.cfgIntLatch(false);      // Don't latch the interrupt until cleared
    SERIAL_PORT.print(F("cfgIntLatch returned: "));
    SERIAL_PORT.println(myICM.statusString());

    myICM.intEnableRawDataReady(true); // enable interrupts on raw data ready
    SERIAL_PORT.print(F("intEnableRawDataReady returned: "));
    SERIAL_PORT.println(myICM.statusString());
}

void readIMU(dataBlock& data) {
    icm_20948_DMP_data_t dataIn;
    myICM.readDMPdataFromFIFO(&dataIn);
    if(data.type & IMU_ROT_DATA){
        if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
        {
            if ((dataIn.header & DMP_header_bitmap_Quat9) > 0)
            {
                // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
                // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
                // The quaternion data is scaled by 2^30.

                // Scale to +/- 1
                double q1 = ((double)dataIn.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
                double q2 = ((double)dataIn.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
                double q3 = ((double)dataIn.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

                data.IMUData.Q1 = q1;
                data.IMUData.Q2 = q2;
                data.IMUData.Q3 = q3;
                data.IMUData.qAccuracy = dataIn.Quat9.Data.Accuracy;

                if(calcEulerAngles){

                    // The ICM 20948 chip has axes y-forward, x-right and Z-up - see Figure 12:
                    // Orientation of Axes of Sensitivity and Polarity of Rotation
                    // in DS-000189-ICM-20948-v1.6.pdf  These are the axes for gyro and accel and quat
                    //
                    // For conversion to roll, pitch and yaw for the equations below, the coordinate frame
                    // must be in aircraft reference frame.
                    //  
                    // We use the Tait Bryan angles (in terms of flight dynamics):
                    // ref: https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles
                    //
                    // Heading – ψ : rotation about the Z-axis (+/- 180 deg.)
                    // Pitch – θ : rotation about the new Y-axis (+/- 90 deg.)
                    // Bank – ϕ : rotation about the new X-axis  (+/- 180 deg.)
                    //
                    // where the X-axis points forward (pin 1 on chip), Y-axis to the right and Z-axis downward.
                    // In the conversion example above the rotation occurs in the order heading, pitch, bank. 
                    // To get the roll, pitch and yaw equations to work properly we need to exchange the axes

                    // Note when pitch approaches +/- 90 deg. the heading and bank become less meaningfull because the
                    // device is pointing up/down. (Gimbal lock)

                    double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

                    double qw = q0;
                    double qx = q2;
                    double qy = q1;
                    double qz = -q3;

                    // roll (x-axis rotation)
                    double t0 = +2.0 * (qw * qx + qy * qz);
                    double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
                    double roll = atan2(t0, t1) * 180.0 / PI;

                    // pitch (y-axis rotation)
                    double t2 = +2.0 * (qw * qy - qx * qz);
                    t2 = t2 > 1.0 ? 1.0 : t2;
                    t2 = t2 < -1.0 ? -1.0 : t2;
                    double pitch = asin(t2) * 180.0 / PI;

                    // yaw (z-axis rotation)
                    double t3 = +2.0 * (qw * qz + qx * qy);
                    double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
                    double yaw = atan2(t3, t4) * 180.0 / PI;

                    SERIAL_PORT.print(F("Roll:"));
                    SERIAL_PORT.print(roll, 1);
                    SERIAL_PORT.print(F(" Pitch:"));
                    SERIAL_PORT.print(pitch, 1);
                    SERIAL_PORT.print(F(" Yaw:"));
                    SERIAL_PORT.println(yaw, 1);
                }
            }
            else{
                data.type &= ~IMU_ROT_DATA;
            }
        }
    }
    if(data.type & IMU_RAW_DATA){
        if(dataIn.header & DMP_header_bitmap_Accel){
            data.IMUData.ax = dataIn.Raw_Accel.Data.X;
            data.IMUData.ay = dataIn.Raw_Accel.Data.Y;
            data.IMUData.az = dataIn.Raw_Accel.Data.Z;
            data.IMUData.aAccuracy = dataIn.Accel_Accuracy;
            data.type |= IMU_ACCEL_DATA;
        }
        if(dataIn.header & DMP_header_bitmap_Compass){
            data.IMUData.cx = dataIn.Compass.Data.X;
            data.IMUData.cy = dataIn.Compass.Data.Y;
            data.IMUData.cz = dataIn.Compass.Data.Z;
            data.IMUData.cAccuracy = dataIn.Compass_Accuracy;
            data.type |= IMU_COMPASS_DATA;
        }
        if(dataIn.header & DMP_header_bitmap_Gyro){
            data.IMUData.gx = dataIn.Raw_Gyro.Data.X;
            data.IMUData.gy = dataIn.Raw_Gyro.Data.Y;
            data.IMUData.gz = dataIn.Raw_Gyro.Data.Z;
            data.IMUData.gAccuracy = dataIn.Gyro_Accuracy;
            data.type |= IMU_GYRO_DATA;
        }
        if(dataIn.header & DMP_header_bitmap_Pressure){
            memcpy(&data.IMUData.pressure, dataIn.Pressure, 2);
            memcpy(&data.IMUData.temperature, dataIn.Pressure+2, 3);
            data.type |= IMU_PRESSURE_DATA;
        }
    }
}

inline void readGPS(dataBlock& data) {
    if (GPS.newNMEAreceived() && GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
        data.GPSData.fixquality = GPS.fixquality;
        data.GPSData.lat = GPS.latitude; data.GPSData.lon = GPS.longitude;
        data.GPSData.alt = GPS.altitude;
        data.GPSData.latDir = GPS.lat; data.GPSData.lonDir = GPS.lon;
        data.GPSData.speed = GPS.speed;
        data.GPSData.HDOP = GPS.HDOP;
    }
}

void logData(dataBlock& data) {

    if(!data.type){return;}
    if ((logIndex + 1) + (sizeof(data) + 1) >= LOG_BLOCK_SIZE) {
        logFile.write((uint8_t*)logBuffer, logIndex);
        logFile.flush();
        logIndex = 0;
        if(SERIAL_PORT.available()){
            logFile.close(); 
            Serial.println(F("Goodbye!"));
            exit(0);
        }
    }

    memcpy(&logBuffer[logIndex], &(data.type), 1); logIndex += 1;

    if(data.type & GPS_DATA){
        constexpr const size_t GPS_DATA_SIZE = sizeof(GPSDump);
        memcpy(&logBuffer[logIndex], (char*)&(data.GPSData), GPS_DATA_SIZE); logIndex += GPS_DATA_SIZE;
    }
    if(data.type & IMU_ROT_DATA){
        constexpr const size_t ROT_DATA_SIZE = sizeof(double) * 3 + sizeof(int16_t);
        memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.timestamp), sizeof(uint32_t)); logIndex += sizeof(uint32_t);
        memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.Q1), ROT_DATA_SIZE); logIndex += ROT_DATA_SIZE;
    }
    if(data.type & IMU_RAW_DATA){
        if(data.type & IMU_ACCEL_DATA){
            constexpr const size_t ACCEL_DATA_SIZE = sizeof(int16_t) * 3 + sizeof(int16_t);
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.timestamp), sizeof(uint32_t)); logIndex += sizeof(uint32_t);
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.ax), ACCEL_DATA_SIZE); logIndex += ACCEL_DATA_SIZE;
        }
        if(data.type & IMU_COMPASS_DATA){
            constexpr const size_t COMPASS_DATA_SIZE = sizeof(int16_t) * 3 + sizeof(int16_t);
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.timestamp), sizeof(uint32_t)); logIndex += sizeof(uint32_t);
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.cx), COMPASS_DATA_SIZE); logIndex += COMPASS_DATA_SIZE;
        }
        if(data.type & IMU_GYRO_DATA){
            constexpr const size_t GYRO_DATA_SIZE = sizeof(int16_t) * 3 + sizeof(int16_t);
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.timestamp), sizeof(uint32_t)); logIndex += sizeof(uint32_t);
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.gx), GYRO_DATA_SIZE); logIndex += GYRO_DATA_SIZE;
        }
        if(data.type & IMU_PRESSURE_DATA){
            constexpr const size_t PRESSURE_DATA_SIZE = sizeof(int16_t) + sizeof(int8_t) * 3;
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.timestamp), sizeof(uint32_t)); logIndex += sizeof(uint32_t);
            memcpy(&logBuffer[logIndex], (char*)&(data.IMUData.pressure), PRESSURE_DATA_SIZE); logIndex += PRESSURE_DATA_SIZE;
        }
    }
    logBuffer[logIndex] = 0; // do not advance the index, we want to overwrite the null terminator with the next data point
}


void loop() {
    dataBlock data;
    if(GPSDataReady){
        GPSDataReady = false;
        data.GPSData.timestamp = GPSPingMicros;
        data.type |= GPS_DATA;
        readGPS(data);
    }

    if (imuDataReady) {
        imuDataReady = false;
        data.IMUData.timestamp = IMUPingMicros;
        data.type |= IMU_ROT_DATA | IMU_RAW_DATA;
        readIMU(data);
    }

    logData(data);
}