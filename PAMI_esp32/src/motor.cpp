#include "motor.h"
#include "Arduino.h"
#include "mpu.h"
#include "ultrasound.h"

/* ROBOT STURCTURE
front is rolling ball

LEFT motor
encoder1
motorA

RIGHT motor
encoder2
motorB
*/

// Encoder 1 left
const int encoderLPinA = 39; // Interrupt pin
const int encoderLPinB = 36; // Regular digital pin

// Encoder 2
const int encoderRPinA = 35; // Interrupt pin
const int encoderRPinB = 34; // Regular digital pin

// Variable to keep track of the encoder position
volatile long encoderLPosition = 0;
volatile long encoderRPosition = 0;


// Motor A AKA left motor
int pwmA = 19;
int in1A = 26;
int in2A = 18;
// Motor B AKA right motor
int pwmB = 4;
int in1B = 17;
int in2B = 16;

// PWM properties
const int pwmFreq = 5000; // PWM frequency in Hz
const int pwmResolution = 8; // PWM resolution in bits (1-16)
const int pwmChannelA = 0; // PWM channel for Motor A
const int pwmChannelB = 1; // PWM channel for Motor B

void IRAM_ATTR handleEncoderR();
void IRAM_ATTR handleEncoderL();

float targetYaw = 0; // Variable to store the target yaw angle

// PID Constants
float Kp = 1.0, Ki = 0.01, Kd = 0.1;
float errorSum = 0, lastError = 0;
unsigned long lastTime = 0;

void motorSetup() {
  //Serial.begin(115200);
  Serial.println("Motor setup started.");
  // Set all the motor control pins to outputs
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Configure PWM channels
  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcAttachPin(pwmA, pwmChannelA);

  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcAttachPin(pwmB, pwmChannelB);

  // Set the encoder pins as inputs
  pinMode(encoderLPinA, INPUT);
  pinMode(encoderLPinB, INPUT);
  pinMode(encoderRPinA, INPUT);
  pinMode(encoderRPinB, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderLPinA), handleEncoderL, CHANGE); // Attach interrupt
  attachInterrupt(digitalPinToInterrupt(encoderRPinA), handleEncoderR, CHANGE); // Attach interrupt


  // Initialize serial communication for debugging
  
  Serial.println("Motor setup complete.");
  delay(2000);
}

void motorLoop() {
  // Forward
  forward(80);
  delay(2000);
  stopMotors();
  delay(500);

  backward();
  delay(2000);
  stopMotors();
  delay(500);

  left();
  delay(2000);
  stopMotors();
  delay(500);

  right();
  delay(2000);
  stopMotors();
  delay(500);
}

void stopMotors() {
  ledcWrite(pwmChannelA, 0);
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);

  ledcWrite(pwmChannelB, 0);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  //Serial.println("Motors stopped.");
}

void forward(int speed) {
  // Set Motor A forward
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  ledcWrite(pwmChannelA, speed);

  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannelB, speed);
  //Serial.println("Motors moving forward.");
}

void backward() {
  // Set Motor A backward
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  ledcWrite(pwmChannelA, 180);

  // Set Motor B backward
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  ledcWrite(pwmChannelB, 180);
  //Serial.println("Motors moving backward.");
}

void left() {
  // Set Motor A backward
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  ledcWrite(pwmChannelA, 180);

  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannelB, 180);
  //Serial.println("Turning left.");
}

void right() {
  // Set Motor A forward
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  ledcWrite(pwmChannelA, 180);

  // Set Motor B backward
  digitalWrite(in1B, HIGH);
  digitalWrite(in2B, LOW);
  ledcWrite(pwmChannelB, 180);
  //Serial.println("Turning right.");
}

void moveStraight(float distance, int baseSpeed) {
  encoderLPosition = 0;
  encoderRPosition = 0;
  targetYaw = currentYaw; // Set target heading

  while (true) {
    distanceCheck();

    while (obstacle == true) {//later can include the distanceCheck in the while loop and the function should return true or false it's better
      stopMotors();
      distanceCheck();
    }

    readIMU();

    

    // Correct heading using PID
    float yawError = normalizeAngle(targetYaw - currentYaw);
    float correction = PIDControl(yawError);
    // Debugging output
    // Serial.print("currentYaw: ");
    // Serial.print(currentYaw);
    // Serial.print(" | targetYaw: ");
    // Serial.print(targetYaw);
    // Serial.print(" | yawError: ");
    // Serial.print(yawError);
    // Serial.print(" | correction: ");
    // Serial.println(correction);
    // Adjust motor speeds
    int leftSpeed = baseSpeed + correction;
    int rightSpeed = baseSpeed - correction;
    
    Serial.print("speed PWM: ");
    Serial.print(leftSpeed);
    Serial.print(" | ");
    Serial.println(rightSpeed);

    //WARNING: when changing forward and backward you also have to inverse the signs
    //in the encoder reading ISR and also inverse the sign of the correction on baseSpeed
    digitalWrite(in1A, HIGH);
    digitalWrite(in2A, LOW);
    analogWrite(pwmA, constrain(leftSpeed, 50, 150));
    //analogWrite(pwmA, 150);

    // Set Motor B forward
    digitalWrite(in1B, LOW);
    digitalWrite(in2B, HIGH);
    analogWrite(pwmB, constrain(rightSpeed, 50, 150));
    //analogWrite(pwmB, 150);
    //forward(80);

    // Serial.print("traveled dist: ");
    // Serial.println(traveledDistance);
    float traveledDistance = (encoderLPosition + encoderRPosition) / (2.0 * 6777.0);

    if (traveledDistance >= distance) {
      stopMotors();
      break;
    }
  }
}

void turnByAngle(float angle, int speed) {
  
  readIMU();
  //float angledifforg = normalize(currentYaw + angle);
  float angleReal = normalizeAngle(currentYaw + angle); 


  while (true) {
    distanceCheck();
    while (obstacle == true) {
      stopMotors();
      distanceCheck();
    }

    readIMU();
    targetYaw = normalizeAngle(angleReal - currentYaw); //not really target yaw but angle difference shortest way +ive for clockwise
    float yawError = targetYaw;
    float correction = PIDControl(yawError);
    Serial.print(correction);
    float adjustedSpeed = constrain(50 + abs(correction), 50, speed);
    Serial.print(" | ");
    Serial.println(adjustedSpeed);

    //rotate clockwise
    if (yawError < 0) {
      digitalWrite(in1A, LOW);
      digitalWrite(in2A, HIGH);
      analogWrite(pwmA, adjustedSpeed);
      // Set Motor B forward
      digitalWrite(in1B, LOW);
      digitalWrite(in2B, HIGH);
      analogWrite(pwmB, adjustedSpeed);
      
    } else { //rotate anticlock
      digitalWrite(in1A, HIGH);
      digitalWrite(in2A, LOW);
      analogWrite(pwmA, adjustedSpeed);
      // Set Motor B forward
      digitalWrite(in1B, HIGH);
      digitalWrite(in2B, LOW);
      analogWrite(pwmB, adjustedSpeed);
    }

    if (abs(yawError) < 1.0) { // Stop when error is small
      stopMotors();
      break;
    }

  }

}

void IRAM_ATTR handleEncoderL() {
  if (digitalRead(encoderLPinA) == HIGH) {
    if (digitalRead(encoderLPinB) == LOW) {
      encoderLPosition++;
    } else { 
      encoderLPosition--;
    }
    
  } else {
    if (digitalRead(encoderLPinB) == HIGH) {
      encoderLPosition++;
    } else {
      encoderLPosition--;
    }
  }
}

void IRAM_ATTR handleEncoderR() {

  if (digitalRead(encoderRPinA) == HIGH) {
    if (digitalRead(encoderRPinB) == LOW) {
      encoderRPosition--;
    } else { 
      encoderRPosition++;
    }
    
  } else {
    if (digitalRead(encoderRPinB) == HIGH) {
      encoderRPosition--;
    } else {
      encoderRPosition++;
    }
  }
}

float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

float PIDControl(float error) {
  unsigned long now = millis();
  float deltaTime = (now - lastTime) / 1000.0; // Convert to seconds

  // Prevent division by zero or extremely small deltaTime
  // for now use this method as it seems like the normal ones are about 0.01
  // but later need to improve this, maybe using interrupts to calculate pid at predefined intervals
  if (deltaTime <= 0) {
    deltaTime = 0.01; // Set a minimum deltaTime (10 ms)
  }

  // PID calculations
  errorSum += error * deltaTime; // large sum over time can cause intergral windup need to sort this later
  float errorRate = (error - lastError) / deltaTime;

  // PID output
  float output = (Kp * error) + (Ki * errorSum) + (Kd * errorRate);

      // Debugging output
      Serial.print("deltaTime: ");
      Serial.print(deltaTime);
      Serial.print("error: ");
      Serial.print(error);
      Serial.print(" | errorSum: ");
      Serial.print(errorSum);
      Serial.print(" | errorRate: ");
      Serial.print(errorRate);
      Serial.print(" | output: ");
      Serial.println(output);
      

  // Save last values
  lastError = error;
  lastTime = now;

  return output;
}