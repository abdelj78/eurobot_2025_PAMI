#include "Wire.h"
#include "MPU6050.h"

MPU6050 mpu;

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    Serial.println("Testing device...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Place MPU6050 flat and don't move it for calibration...");
    delay(3000);
    
    Serial.println("Calibrating...");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    
    Serial.println("Calibration Done!");
    Serial.println("Use the following values in your code:");

    Serial.print("accel offsets: "); Serial.print(mpu.getXAccelOffset()); Serial.print("\t");
    Serial.print(mpu.getYAccelOffset()); Serial.print("\t");
    Serial.print(mpu.getZAccelOffset()); Serial.println();

    Serial.print("gyro offsets: "); Serial.print(mpu.getXGyroOffset()); Serial.print("\t");
    Serial.print(mpu.getYGyroOffset()); Serial.print("\t");
    Serial.print(mpu.getZGyroOffset()); Serial.println();
}

void loop() { }
