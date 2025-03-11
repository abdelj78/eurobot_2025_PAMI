
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;

int const INTERRUPT_PIN = 2;  // Define the interruption #0 pin
bool blinkState;

/*---MPU6050 Control/Status Variables---*/
bool DMPReady = false;  // Set true if DMP init was successful
uint8_t MPUIntStatus;   // Holds actual interrupt status byte from MPU
uint8_t devStatus;      // Return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // Expected DMP packet size (default is 42 bytes)
uint8_t FIFOBuffer[64]; // FIFO storage buffer

/*---Orientation/Motion Variables---*/ 
Quaternion q;           // [w, x, y, z]         Quaternion container
VectorInt16 aa;         // [x, y, z]            Accel sensor measurements
VectorInt16 gy;         // [x, y, z]            Gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            Gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            World-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            Gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   Yaw/Pitch/Roll container and gravity vector

/*-Packet structure for InvenSense teapot demo-*/ 
uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };

/*------Interrupt detection routine------*/
volatile bool MPUInterrupt = false;     // Indicates whether MPU6050 interrupt pin has gone high
void DMPDataReady() {
  MPUInterrupt = true;
}

const int startPin = 12;  // Best practice

float angleDes = 90;
float currentYaw = 0;

/* ROBOT STURCTURE
front is rolling ball

LEFT motor
encoder1
motorA

RIGHT motor
encoder2
motorB
*/


// Encoder 1
const int encoder1PinA = 2;//3;//2; // Interrupt pin
const int encoder1PinB = 15;//14;//15; // Regular digital pin
// Encoder 2
const int encoder2PinA = 3; // Interrupt pin
const int encoder2PinB = 14; // Regular digital pin
// Variable to keep track of the encoder position
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;
// Motor A
int pwmA = 9;//5;
int in1A = 7;//2; //used to be 3 but changed to save pwm output
int in2A = 4;//4;
// Motor B
int pwmB = 10;//6;
int in1B = 8;//7;
int in2B = 12;//8;
// Motor Speed Values - Start at zero
int MotorSpeed1 = 100;
int MotorSpeed2 = 100;

volatile float encoder1Dist = 0;
volatile float encoder2Dist = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  // Set all the motor control pins to outputs
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment on this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  
  Serial.begin(115200); //115200 is required for Teapot Demo output
  while (!Serial);

  /*Initialize device*/
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  /*Verify connection*/
  Serial.println(F("Testing MPU6050 connection..."));
  if(mpu.testConnection() == false){
    Serial.println("MPU6050 connection failed");
    while(true);
  }
  else {
    Serial.println("MPU6050 connection successful");
  }

  /* Initializate and configure the DMP*/
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  /* Making sure it worked (returns 0 if so) */ 
  if (devStatus == 0) {
    mpu.CalibrateAccel(10);  // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateGyro(10);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));   //Turning ON DMP
    mpu.setDMPEnabled(true);

    /*Enable Arduino interrupt detection*/
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), DMPDataReady, RISING);
    MPUIntStatus = mpu.getIntStatus();

    /* Set the DMP Ready flag so the main loop() function knows it is okay to use it */
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize(); //Get expected DMP packet size for later comparison
  } 
  else {
    Serial.print(F("DMP Initialization failed (code ")); //Print the error code
    Serial.print(devStatus);
    Serial.println(F(")"));
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
  }

  
  // Set start pin and wait until pressed
  pinMode(startPin, INPUT_PULLUP);  // Set the pin as an input

  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN,LOW);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN,LOW);

  Serial.println("Waiting for start signal...");
  while (digitalRead(startPin) == HIGH) {
    // Do nothing, just wait
  }
  Serial.println("Start signal received! end of setup");
  //float targetYaw = normalizeAngle(angleDes);

  delay(2000);


  //read just once for first time
  if (!DMPReady) return; // Stop the program if DMP programming fails.
  discardInitialReadings(1000);  // Discard the first 10 readings

  /* Read a packet from FIFO */
  while(!mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    //do nothing, just wait
  }
  /* Display Euler angles in degrees */
  mpu.dmpGetQuaternion(&q, FIFOBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  currentYaw = ypr[0] * 180 / M_PI;

  angleDes = normalizeAngle(currentYaw + 90);
  Serial.print("FIRST TIME: ");
  Serial.print("currentYaw: ");
  Serial.print(currentYaw);
  Serial.print("   angleDes: "); 
  Serial.println(angleDes);
}


void loop() {
  // put your main code here, to run repeatedly:

  readIMU();

  if (abs(normalizeAngle(angleDes-currentYaw)) > 2) {
    rotateClockwise();
  } else {
    Serial.print("REACHED TARGET: ");
    Serial.print("currentYaw: ");
    Serial.print(currentYaw);
    Serial.print("   angleDes: ");
    Serial.println(angleDes);
    stopMotors();
    delay(5000); // Wait for 5 seconds //better to use millis() to keep looking at rotation changes
    // Set new target yaw for the next 90° turn
    readIMU();
    angleDes = normalizeAngle(currentYaw + 90);
    Serial.print("NEW TARGET: ");
    Serial.print("currentYaw: "); 
    Serial.print(currentYaw);
    Serial.print("   angleDes: ");  
    Serial.println(angleDes);
  }

}

void readIMU() {
  if (!DMPReady) return; // Stop the program if DMP programming fails.
    
  /* Read a packet from FIFO */
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Get the Latest packet 
    /* Display Euler angles in degrees */
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    // Serial.print("ypr\t");
    // Serial.print(ypr[0] * 180/M_PI);
    // Serial.print("\t");
    // Serial.print(ypr[1] * 180/M_PI);
    // Serial.print("\t");
    // Serial.println(ypr[2] * 180/M_PI);

    /* Blink LED to indicate activity */
    blinkState = !blinkState;
    digitalWrite(LED_BUILTIN, blinkState);
    currentYaw = ypr[0] * 180 / M_PI;
  }
}


void rotateClockwise() {
  // Set Motor A backward 
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  //analogWrite(pwmA, MotorSpeed1);

  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  //analogWrite(pwmB, MotorSpeed2);

  float yawError = abs(normalizeAngle(angleDes - currentYaw));

  if (yawError > 20) {
    analogWrite(pwmA, 50);  // Full speed
    analogWrite(pwmB, 50);  // Full speed
  } else if (yawError > 10) {
    analogWrite(pwmA, 40);  // Full speed
    analogWrite(pwmB, 40);  // Full speed
  } else {
    analogWrite(pwmA, 30);  // Full speed
    analogWrite(pwmB, 30);  // Full speed
  }


}

void stopMotors() {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  analogWrite(pwmA, 0);

  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  analogWrite(pwmB, 0);
}

float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
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
