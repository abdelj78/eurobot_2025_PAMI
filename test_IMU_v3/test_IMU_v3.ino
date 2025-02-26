#include <Wire.h>
#include <MPU6050.h>
#include "Kalman.h"

MPU6050 mpu;
Kalman kalmanYaw;

float yaw = 0;
float gyroBiasZ = 0;
long lastTime;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);
    }
    
    // Gyro bias calibration
    calibrateGyro();
    
    lastTime = millis();
}

void loop() {
    long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;
    
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);
    
    float gyroZ = (gz / 131.0) - gyroBiasZ; // Convert to deg/s and subtract bias
    yaw += gyroZ * dt; // Integrate gyro data to get yaw angle
    
    // Apply complementary filter
    float alpha = 0.98;
    yaw = alpha * (yaw + gyroZ * dt) + (1 - alpha) * getAccelYaw();
    
    Serial.print("Yaw: ");
    Serial.println(yaw);
    delay(10);
}

void calibrateGyro() {
    const int numSamples = 1000;
    long sum = 0;
    
    for (int i = 0; i < numSamples; i++) {
        int16_t gx, gy, gz;
        mpu.getRotation(&gx, &gy, &gz);
        sum += gz;
        delay(3);
    }
    
    gyroBiasZ = sum / (float)numSamples / 131.0; // Average and convert to deg/s
}

float getAccelYaw() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    // Calculate yaw angle from accelerometer data
    float accelYaw = atan2(ay, ax) * 180 / PI;
    return accelYaw;
}

////I2C MPU DETECTION TEST
//#include <Wire.h>
//
//void setup() {
//    Serial.begin(115200);
//    Wire.begin();
//    Serial.println("Scanning for I2C devices...");
//
//    for (byte address = 1; address < 127; address++) {
//        Wire.beginTransmission(address);
//        if (Wire.endTransmission() == 0) {
//            Serial.print("Found I2C device at 0x");
//            Serial.println(address, HEX);
//        }
//    }
//}
//
//void loop() {}
//



////MPU CONNECTION TEST
//#include <Wire.h>
//#include <MPU6050.h>
//
//MPU6050 mpu;
//
//void setup() {
//    Serial.begin(115200);
//    Wire.begin();
//    
//    Serial.println("Initializing MPU6050...");
//    delay(1000);  // Wait for MPU6050 to power up
//    mpu.initialize();
//
//    if (mpu.testConnection()) {
//        Serial.println("MPU6050 connected!");
//    } else {
//        Serial.println("MPU6050 connection failed!");
//    }
//}
//
//void loop() {}
