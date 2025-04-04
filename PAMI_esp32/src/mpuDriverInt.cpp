// //Code for the superstar PAMI, simple commands of translation and rotation for now with attempt
// //to integrate servo motor with ultrasound sensor, gyro (without interrupt) etc
// #include "mpuDriverInt.h"
// //#include "I2Cdev.h"
// #include "MPU6050_6Axis_MotionApps20.h"
// #include "Arduino.h"
// //#include <Servo.h> 


// //Servo settings
// // Declare the Servo pin 
// //const int servoPin = 9;
// // Create a servo object 
// //Servo myServo; 

// // int servoState = 0;

// //Ultrasound sensor settings
// //const int trigPin = 11;
// //const int echoPin = 10;
// //float duration, distance;
// //bool obstacle = false; 


// MPU6050 mpu;
// int const INTERRUPT_PIN = 5;  // Define the interruption #0 pin

// /*---MPU6050 Control/Status Variables---*/
// bool DMPReady = false;  // Set true if DMP init was successful
// uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
// uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
// uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
// uint8_t FIFOBuffer[64]; // FIFO storage buffer

// /*---Orientation/Motion Variables---*/ 
// Quaternion q;           // [w, x, y, z]         Quaternion container
// VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
// VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
// VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
// VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
// VectorFloat gravity;    // [x, y, z]            Gravity vector
// float euler[3];         // [psi, theta, phi]    Euler angle container
// float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

// float currentYaw = 0;
// float targetYaw = 0;

// // PID Constants
// float Kp = 1.0, Ki = 0.01, Kd = 0.1;
// float errorSum = 0, lastError = 0;
// unsigned long lastTime = 0;

// /* ROBOT STURCTURE
// front is rolling ball

// LEFT motor
// encoder1
// motorA

// RIGHT motor
// encoder2
// motorB
// */
// // Encoder 1
// const int encoder1PinA = 34;//3;//2; // Interrupt pin
// const int encoder1PinB = 35;//14;//15; // Regular digital pin

// // Encoder 2
// const int encoder2PinA = 32; // Interrupt pin
// const int encoder2PinB = 33; // Regular digital pin


// // Variable to keep track of the encoder position
// volatile long encoder1Position = 0;
// volatile long encoder2Position = 0;

// // Motor A
// int pwmA = 25;//5;//5;
// int in1A = 26;//2; //used to be 3 but changed to save pwm output
// int in2A = 27;//4;
// // Motor B
// int pwmB = 13;//6;
// int in1B = 14;//7;
// int in2B = 12;//8;

// // volatile float encoder1Dist = 0;
// // volatile float encoder2Dist = 0;

// // Interrupt detection routine
// volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
// void IRAM_ATTR DMPDataReady() {
//   MPUInterrupt = true;
// }

// void mpuDriverSetup() {
// //   pinMode(trigPin, OUTPUT);
// //   pinMode(echoPin, INPUT);
//   // put your setup code here, to run once:
//   // Set the encoder pins as inputs
//   pinMode(encoder1PinA, INPUT_PULLUP);
//   pinMode(encoder1PinB, INPUT);
//   pinMode(encoder2PinA, INPUT_PULLUP);
//   pinMode(encoder2PinB, INPUT);

//     // Set all the motor control pins to outputs
//   pinMode(pwmA, OUTPUT);
//   pinMode(pwmB, OUTPUT);
//   pinMode(in1A, OUTPUT);
//   pinMode(in2A, OUTPUT);
//   pinMode(in1B, OUTPUT);
//   pinMode(in2B, OUTPUT);

//   // Attach an interrupt to encoderPinA
//   attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1, CHANGE);
//   attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2, CHANGE);
  
//   // Initialize serial communication for debugging
//   Serial.begin(115200);

//  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//     Wire.begin();
//     Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
//   #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//     Fastwire::setup(400, true);
//   #endif

//   /*Initialize device*/
//   //Serial.println(F("Initializing I2C devices..."));
//   mpu.initialize();

//   /*Verify connection*/
//   //Serial.println(F("Testing MPU6050 connection..."));
//   if(mpu.testConnection() == false){
//     //Serial.println("MPU6050 connection failed");
//     while(true);
//   }
//   else {
//     //Serial.println("MPU6050 connection successful");
//   }

//   /* Initializate and configure the DMP*/
//   //Serial.println(F("Initializing DMP..."));
//   devStatus = mpu.dmpInitialize();

//   /* Making sure it worked (returns 0 if so) */ 
//   if (devStatus == 0) {
//     mpu.CalibrateAccel(10);  // Calibration Time: generate offsets and calibrate our MPU6050
//     mpu.CalibrateGyro(10);
//     //Serial.println("These are the Active offsets: ");
//     //mpu.PrintActiveOffsets();
//     //Serial.println(F("Enabling DMP..."));   //Turning ON DMP
//     mpu.setDMPEnabled(true);

//     /*Enable Arduino interrupt detection*/
//     Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//     Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//     Serial.println(F(")..."));
//     attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
//     MPUIntStatus = mpu.getIntStatus();

//     /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
//     //Serial.println(F("DMP ready! Waiting for first interrupt..."));
//     DMPReady = true;
//     //packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
//   } 
//   else {
//     Serial.print(F("DMP Initialization failed (code ")); //Print the error code
//     Serial.print(devStatus);
//     Serial.println(F(")"));
//     // 1 = initial memory load failed
//     // 2 = DMP configuration updates failed
//   }

//   discardInitialReadings(500);  // Discard the first 10 readings

//   // put your setup code here, to run once:
//   //myServo.attach(servoPin);  

//   //myServo.write(78);
//   //delay(1000);

//   // //SUPERSTAR BLUE
//   moveStraight(1.27876, 80);
//   turnByAngle(-90.0, 100);
//   moveStraight(0.15,80);

//   // //GROUPIE BLUE CLOSE
//   // moveStraight(0.35, 80);
//   // turnByAngle(-90, 100);
//   // moveStraight(0.3, 80);
//   // turnByAngle(90, 100);
//   // moveStraight(0.55, 80);  

//   // //GROUPIE BLUE MIDDLE
//   // moveStraight(0.25, 80);
//   // turnByAngle(-90, 100);
//   // moveStraight(0.3, 80);
//   // turnByAngle(90, 100);
//   // moveStraight(1.25, 80);  

//   // // //GROUPIE BLUE FURTHEST
// //   moveStraight(0.15, 80);
// //   turnByAngle(-90, 100);
// //   moveStraight(0.3, 80);
// //   turnByAngle(90, 100);
// //   moveStraight(1.7, 80);  
// //   turnByAngle(90, 100);
// //   moveStraight(0.15, 80);  
// }

// void mpuDriverLoop() {
// //  put your main code here, to run repeatedly:
// //   myServo.write(100);
// //   delay(500);
// //   myServo.write(60);
// //   delay(500);
// }

// void moveStraight(float distance, int baseSpeed) {
//   encoder1Position = 0;
//   encoder2Position = 0;
//   targetYaw = currentYaw; // Set target heading

//   while (true) {
//     //distanceCheck();

//     // while (obstacle == true) {//later can include the distanceCheck in the while loop and the function should return true or false it's better
//     //   stopMotors();
//     //   distanceCheck();
//     // }
//     if (MPUInterrupt) {
//       MPUInterrupt = false; // Reset interrupt flag
//       readIMU();
//     }

    

//     // Correct heading using PID
//     float yawError = normalizeAngle(targetYaw - currentYaw);
//     float correction = PIDControl(yawError);

//     // Adjust motor speeds
//     int leftSpeed = baseSpeed + correction;
//     int rightSpeed = baseSpeed - correction;
    
//     Serial.print("speed PWM: ");
//     Serial.print(leftSpeed);
//     Serial.print(" | ");
//     Serial.println(rightSpeed);

//     //WARNING: when changing forward and backward you also have to inverse the signs
//     //in the encoder reading ISR and also inverse the sign of the correction on baseSpeed
//     digitalWrite(in1A, LOW);
//     digitalWrite(in2A, HIGH);
//     analogWrite(pwmA, constrain(leftSpeed, 50, 150));

//     // Set Motor B forward
//     digitalWrite(in1B, LOW);
//     digitalWrite(in2B, HIGH);
//     analogWrite(pwmB, constrain(rightSpeed, 50, 150));

//     // Serial.print("traveled dist: ");
//     // Serial.println(traveledDistance);
//     float traveledDistance = (encoder1Position + encoder2Position) / (2.0 * 6777.0);

//     if (traveledDistance >= distance) {
//       stopMotors();
//       break;
//     }
//   }
// }

// void turnByAngle(float angle, int speed) {
  
//   readIMU();
//   //float angledifforg = normalize(currentYaw + angle);
//   float angleReal = normalizeAngle(currentYaw + angle); 


//   while (true) {
//     //distanceCheck();
//     // while (obstacle == true) {
//     //   stopMotors();
//     //   distanceCheck();
//     // }
//     if (MPUInterrupt) {
//         MPUInterrupt = false; // Reset interrupt flag
//         readIMU();
//       }
//     targetYaw = normalizeAngle(angleReal - currentYaw); //not really target yaw but angle difference shortest way +ive for clockwise
//     float yawError = targetYaw;
//     float correction = PIDControl(yawError);
//     Serial.print(correction);
//     float adjustedSpeed = constrain(50 + abs(correction), 50, speed);
//     Serial.print(" | ");
//     Serial.println(adjustedSpeed);

//     //rotate clockwise
//     if (yawError < 0) {
//       digitalWrite(in1A, HIGH);
//       digitalWrite(in2A, LOW);
//       analogWrite(pwmA, adjustedSpeed);
//       // Set Motor B forward
//       digitalWrite(in1B, LOW);
//       digitalWrite(in2B, HIGH);
//       analogWrite(pwmB, adjustedSpeed);
      
//     } else { //rotate anticlock
//       digitalWrite(in1A, LOW);
//       digitalWrite(in2A, HIGH);
//       analogWrite(pwmA, adjustedSpeed);
//       // Set Motor B forward
//       digitalWrite(in1B, HIGH);
//       digitalWrite(in2B, LOW);
//       analogWrite(pwmB, adjustedSpeed);
//     }

//     if (abs(yawError) < 1.0) { // Stop when error is small
//       stopMotors();
//       break;
//     }

//   }

// }

// void stopMotors() {
//   analogWrite(pwmA, 0);
//   digitalWrite(in1A, LOW);
//   digitalWrite(in2A, LOW);

//   analogWrite(pwmB, 0);
//   digitalWrite(in1B, LOW);
//   digitalWrite(in2B, LOW);
// }


// float PIDControl(float error) {
//   unsigned long now = millis();
//   float deltaTime = (now - lastTime) / 1000.0; // Convert to seconds

//   // PID calculations
//   errorSum += error * deltaTime;
//   float errorRate = (error - lastError) / deltaTime;

//   // PID output
//   float output = (Kp * error) + (Ki * errorSum) + (Kd * errorRate);

//   // Save last values
//   lastError = error;
//   lastTime = now;

//   return output;
// }

// void readIMU() {
//   // if (!DMPReady) return; // Stop the program if DMP programming fails.
    
//   // /* Read a packet from FIFO */
//   // if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
//   //   /* Display Euler angles in degrees */
//   //   mpu.dmpGetQuaternion(&q, FIFOBuffer);
//   //   mpu.dmpGetGravity(&gravity, &q);
//   //   mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//   //   // Serial.print("ypr\t");
//   //   // Serial.print(ypr[0] * 180/M_PI);
//   //   // Serial.print("\t");
//   //   // Serial.print(ypr[1] * 180/M_PI);
//   //   // Serial.print("\t");
//   //   // Serial.println(ypr[2] * 180/M_PI);

//   //   /* Blink LED to indicate activity */
//   //   // blinkState = !blinkState;
//   //   // digitalWrite(LED_BUILTIN, blinkState);
//   //   currentYaw = ypr[0] * 180 / M_PI;
//   // }

//   if (mpu.getFIFOCount() >= packetSize) {  // Ensure FIFO has enough data
//       if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { 
//           mpu.dmpGetQuaternion(&q, FIFOBuffer);
//           mpu.dmpGetGravity(&gravity, &q);
//           mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//           currentYaw = ypr[0] * 180 / M_PI;
//       }
//   }

// }

// float normalizeAngle(float angle) {
//   while (angle > 180) angle -= 360;
//   while (angle < -180) angle += 360;
//   return angle;
// }

// void discardInitialReadings(int numReadings) {
//     for (int i = 0; i < numReadings; i++) {
//         if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
//             mpu.dmpGetQuaternion(&q, FIFOBuffer);
//             mpu.dmpGetGravity(&gravity, &q);
//             mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//         }
//         delay(10);  // Small delay to let the sensor stabilize
//     }
// }

// // Interrupt service routine to handle the encoder pulses
// void handleEncoder1() {

//   if (digitalRead(encoder1PinA) == HIGH) {
//     if (digitalRead(encoder1PinB) == LOW) {
//       encoder1Position++;
//     } else { 
//       encoder1Position--;
//     }
    
//   } else {
//     if (digitalRead(encoder1PinB) == HIGH) {
//       encoder1Position++;
//     } else {
//       encoder1Position--;
//     }
//   }
// }

// // Interrupt service routine to handle the encoder pulses
// void handleEncoder2() {

//   if (digitalRead(encoder2PinA) == HIGH) {
//     if (digitalRead(encoder2PinB) == LOW) {
//       encoder2Position--;
//     } else { 
//       encoder2Position++;
//     }
    
//   } else {
//     if (digitalRead(encoder2PinB) == HIGH) {
//       encoder2Position--;
//     } else {
//       encoder2Position++;
//     }
//   }
// }

// // void forward() {
// //   // Set Motor A backward 
// //   digitalWrite(in1A, LOW);
// //   digitalWrite(in2A, HIGH);
// //   analogWrite(pwmA, MotorSpeed1);

// //   // Set Motor B forward
// //   digitalWrite(in1B, LOW);
// //   digitalWrite(in2B, HIGH);
// //   analogWrite(pwmB, MotorSpeed2);
// // }

// void stopMotor1() {
//   digitalWrite(in1A, LOW);
//   digitalWrite(in2A, LOW);
//   analogWrite(pwmA, 0);
// }

// void stopMotor2() {
//   digitalWrite(in1B, LOW);
//   digitalWrite(in2B, LOW);
//   analogWrite(pwmB, 0);
// }
