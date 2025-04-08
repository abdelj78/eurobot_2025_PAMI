#include "mpu.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// MPU6050 default I2C address is 0x68
MPU6050 mpu;

int const INTERRUPT_PIN = 23;  // Define the interruption #0 pin

// MPU6050 Control/Status Variables
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

// Orientation/Motion Variables
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

float currentYaw = 0; // Variable to store the current yaw angle
//float targetYaw = 0; // Variable to store the target yaw angle


// Packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// Interrupt detection routine
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void IRAM_ATTR DMPDataReady() {
  MPUInterrupt = true;
}

void mpuSetup() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  //Serial.begin(115200); //115200 is required for Teapot Demo output
  //while (!Serial);

  // Initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // Verify connection
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  // Initialize and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // Make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.CalibrateAccel(10);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(10);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   // Turning ON DMP
    mpu.setDMPEnabled(true);

    // Enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    // Set the DMP Ready flag so the main loop() function knows it is okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); // Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }
  discardInitialReadings(500);  // Discard the first 10 readings to stabilize the sensor
  //pinMode(2, OUTPUT);
}

void mpuLoop() {

}

void readIMU() {
    if (MPUInterrupt) { // Check if MPU interrupt has occurred
        MPUInterrupt = false; // Reset interrupt flag
        
        MPUIntStatus = mpu.getIntStatus();

        // Check for overflow
        if ((MPUIntStatus & 0x10) || mpu.getFIFOCount() >= 1024) {
            Serial.println("FIFO overflow!");
            mpu.resetFIFO();
            return;
        }

        if (mpu.getFIFOCount() >= packetSize) { // Ensure FIFO has enough data
            if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get new data from FIFO
                mpu.dmpGetQuaternion(&q, FIFOBuffer); // Get quaternion data
                mpu.dmpGetGravity(&gravity, &q); // Get gravity vector
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); // Get yaw, pitch, roll angles
                currentYaw = ypr[0] * 180 / M_PI; // Convert yaw to degrees
            }

        }
    }

}

void discardInitialReadings(int numReadings) {
    for (int i = 0; i < numReadings; i++) {
        if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
            mpu.dmpGetQuaternion(&q, FIFOBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        }
        delay(10);  // Small delay to let the sensor stabilize
    }
}