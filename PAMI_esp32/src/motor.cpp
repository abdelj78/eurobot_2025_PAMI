#include "motor.h"
#include "Arduino.h"

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

void motorSetup() {
    Serial.begin(115200);
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

  // Initialize serial communication for debugging
  
  Serial.println("Motor setup complete.");
}

void motorLoop() {
    // Forward
    forward();
    delay(2000);
    
    // Stop
    stopMotors();
    delay(2000);
}

void stopMotors() {
  ledcWrite(pwmChannelA, 0);
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);

  ledcWrite(pwmChannelB, 0);
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
    Serial.println("Motors stopped.");
}

void forward() {
  // Set Motor A forward
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  ledcWrite(pwmChannelA, 255);

  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  ledcWrite(pwmChannelB, 255);
    Serial.println("Motors moving forward.");
}