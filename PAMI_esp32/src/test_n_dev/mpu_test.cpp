// #include "mpu_test.h"
// #include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"

// // MPU6050 default I2C address is 0x68
// MPU6050 mpu;

// // Output format definition
// #define OUTPUT_READABLE_YAWPITCHROLL

// int const INTERRUPT_PIN = 23;  // Define the interruption #0 pin
// bool blinkState;

// // MPU6050 Control/Status Variables
// bool DMPReady = false;  // Set true if DMP init was successful
// uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
// uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
// uint8_t FIFOBuffer[64]; // FIFO storage buffer

// // Orientation/Motion Variables
// Quaternion q;           // [w, x, y, z]         Quaternion container
// VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
// VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            Gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// // Packet structure for InvenSense teapot demo
// uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

// // Interrupt detection routine
// volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
// void IRAM_ATTR DMPDataReady() {
//   MPUInterrupt = true;
// }

// void mpuSetup() {
//   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     Wire.begin();
//     Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
//   #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//     Fastwire::setup(400, true);
//   #endif
  
//   Serial.begin(115200); //115200 is required for Teapot Demo output
//   while (!Serial);

//   // Initialize device
//   Serial.println(F("Initializing I2C devices..."));
//   mpu.initialize();
//   pinMode(INTERRUPT_PIN, INPUT);

//   // Verify connection
//   Serial.println(F("Testing MPU6050 connection..."));
//   if(mpu.testConnection() == false){
//     Serial.println("MPU6050 connection failed");
//     while(true);
//   }
//   else {
//     Serial.println("MPU6050 connection successful");
//   }

//   // Wait for Serial input
//   Serial.println(F("\nSend any character to begin: "));
//   while (Serial.available() && Serial.read()); // Empty buffer
//   while (!Serial.available());                 // Wait for data
//   while (Serial.available() && Serial.read()); // Empty buffer again

//   // Initialize and configure the DMP
//   Serial.println(F("Initializing DMP..."));
//   devStatus = mpu.dmpInitialize();

//   // Supply your gyro offsets here, scaled for min sensitivity
//   // mpu.setXGyroOffset(144);
//   // mpu.setYGyroOffset(28);
//   // mpu.setZGyroOffset(61);
//   // mpu.setXAccelOffset(-272);
//   // mpu.setYAccelOffset(-2003);
//   // mpu.setZAccelOffset(580);

//   // Make sure it worked (returns 0 if so)
//   if (devStatus == 0) {
//     mpu.CalibrateAccel(10);  // Calibration Time: generate offsets and calibrate our MPU6050
//     mpu.CalibrateGyro(10);
//     Serial.println("These are the Active offsets: ");
//     mpu.PrintActiveOffsets();
//     Serial.println(F("Enabling DMP..."));   // Turning ON DMP
//     mpu.setDMPEnabled(true);

//     // Enable Arduino interrupt detection
//     Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//     Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//     Serial.println(F(")..."));
//     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
//     MPUIntStatus = mpu.getIntStatus();

//     // Set the DMP Ready flag so the main loop() function knows it is okay to use it
//     Serial.println(F("DMP ready! Waiting for first interrupt..."));
//     DMPReady = true;
//     packetSize = mpu.dmpGetFIFOPacketSize(); // Get expected DMP packet size for later comparison
//   } 
//   else {
//     Serial.print(F("DMP Initialization failed (code ")); // Print the error code
//     Serial.print(devStatus);
//     Serial.println(F(")"));
//     // 1 = initial memory load failed
//     // 2 = DMP configuration updates failed
//   }
//   pinMode(2, OUTPUT);
// }

// void mpuLoop() {
//   if (!DMPReady) return; // Stop the program if DMP programming fails.
  
//   // Check if MPU interrupt has occurred
//   if (MPUInterrupt) {
//     MPUInterrupt = false; // Reset interrupt flag

//     // Read a packet from FIFO
//     if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
//       #ifdef OUTPUT_READABLE_YAWPITCHROLL
//         // Update Euler angles in degrees
//         mpu.dmpGetQuaternion(&q, FIFOBuffer);
//         mpu.dmpGetGravity(&gravity, &q);
//         mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//       #endif

//       // Display the updated Euler angles
//       #ifdef OUTPUT_READABLE_YAWPITCHROLL
//         Serial.print("ypr\t");
//         Serial.print(ypr[0] * 180/M_PI);
//         Serial.print("\t");
//         Serial.print(ypr[1] * 180/M_PI);
//         Serial.print("\t");
//         Serial.println(ypr[2] * 180/M_PI);
//       #endif

//       // Blink LED to indicate activity
//       blinkState = !blinkState;
//       digitalWrite(2, blinkState);
//     }
//   }
// }