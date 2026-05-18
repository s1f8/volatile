/*
    esp32 + mpu6050 dmp teapot
    drift-Reduced Quaternion Streaming
    compatible with Processing Teapot Visualization
    quaterninion damping while stationary -> if  stationary lock changes in quaterninon
    but the problem of yaw preserves as there is no magnetometer 
    better remove the testing code of mpu connection.
    using example code of mpu6050 lib of electronic cats for teapot 3d orientation
    using jrowbergs library
*/

#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
// dmp variables
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

// quaternions
Quaternion q;
Quaternion qFiltered;
bool firstRun = true;



// Gyro deadband
const int gyroThreshold = 3;
// Smoothing factor
const float alpha = 0.92;
bool stationary = false;

// teapot packet
uint8_t teapotPacket[14] = {
    '$', 0x02,
    0, 0,
    0, 0,
    0, 0,
    0, 0,
    0x00, 0x00,
    '\r', '\n'
};

//stream control
bool streamEnabled = false;

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println();
    Serial.println("=================================");
    Serial.println("ESP32 MPU6050 DMP TEAPOT");
    Serial.println("=================================");
    //i2c
    Wire.begin(21, 22);
    Wire.setClock(400000);
    delay(100);
    Serial.println("I2C STARTED");
    mpu.initialize();
    delay(100);

/*
    Serial.println("TESTING MPU...");

    if (!mpu.testConnection()) {

        Serial.println("MPU FAIL");

    }

    Serial.println("MPU OK");
*/
    
    //offset callibration
    // need mean ofset of multiple tests
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);

    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

   //dmp initialization
    Serial.println("DMP INIT...");
    devStatus = mpu.dmpInitialize();
    Serial.print("DMP STATUS: ");
    Serial.println(devStatus);
    if (devStatus != 0) {
        Serial.println("DMP FAILED");
        return;
    }
    // enabling dmp
    mpu.setDMPEnabled(true);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("DMP READY");
    Serial.println("Waiting for Processing...");
}

void loop() {

    if (Serial.available()) {

        char c = Serial.read();
        if (c == 'r') {
            streamEnabled = true;
        }
    }
    if (!streamEnabled)
        return;
    if (!dmpReady)
        return;
    //fifo conting and overflow
    fifoCount = mpu.getFIFOCount();
    if (fifoCount == 1024) {
        mpu.resetFIFO();
        return;
    }

    //packets h andlng
    if (fifoCount < packetSize)
        return;

    while (fifoCount >= packetSize) {
        mpu.getFIFOBytes(
            fifoBuffer,
            packetSize
        );

        fifoCount -= packetSize;
    }
    // quaternions #########################
    mpu.dmpGetQuaternion(
        &q,
        fifoBuffer
    );
    
    //gyro
    int16_t gx, gy, gz;
    mpu.getRotation(
        &gx,
        &gy,
        &gz
    );
  
    //stationary position - if no gaingnig inthe values then stationary and slows the drifting rate
    if (
        abs(gx) < gyroThreshold &&
        abs(gy) < gyroThreshold &&
        abs(gz) < gyroThreshold
    ) stationary = true;
    else stationary = false;
    if (firstRun) {
        qFiltered = q;
        firstRun = false;
    }

    //drift reduction
    if (stationary) {
        qFiltered.w =
            alpha * qFiltered.w +
            (1.0 - alpha) * q.w;

        qFiltered.x =
            alpha * qFiltered.x +
            (1.0 - alpha) * q.x;

        qFiltered.y =
            alpha * qFiltered.y +
            (1.0 - alpha) * q.y;

        qFiltered.z =
            alpha * qFiltered.z +
            (1.0 - alpha) * q.z;
    }
    else qFiltered = q;
   

    //quaternion normalization
    float norm =
        sqrt(
            qFiltered.w * qFiltered.w +
            qFiltered.x * qFiltered.x +
            qFiltered.y * qFiltered.y +
            qFiltered.z * qFiltered.z
        );

    qFiltered.w /= norm;
    qFiltered.x /= norm;
    qFiltered.y /= norm;
    qFiltered.z /= norm;

    // teapot requiured formate and sending in binary  pacckets for processing to process
    int16_t qw = qFiltered.w * 16384.0f;
    int16_t qx = qFiltered.x * 16384.0f;
    int16_t qy = qFiltered.y * 16384.0f;
    int16_t qz = qFiltered.z * 16384.0f;

    teapotPacket[2] = (qw >> 8) & 0xFF;
    teapotPacket[3] = qw & 0xFF;

    teapotPacket[4] = (qx >> 8) & 0xFF;
    teapotPacket[5] = qx & 0xFF;

    teapotPacket[6] = (qy >> 8) & 0xFF;
    teapotPacket[7] = qy & 0xFF;

    teapotPacket[8] = (qz >> 8) & 0xFF;
    teapotPacket[9] = qz & 0xFF;

    // Packet counter
    teapotPacket[11]++;

    // bin packets
    Serial.write(teapotPacket, 14);
}
