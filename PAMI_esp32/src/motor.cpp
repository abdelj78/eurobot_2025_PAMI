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

extern SemaphoreHandle_t positionMutex; 

// Encoder 1 left
const int encoderLPinA = 39; // Interrupt pin
const int encoderLPinB = 36; // Regular digital pin

// Encoder 2
const int encoderRPinA = 35; // Interrupt pin
const int encoderRPinB = 34; // Regular digital pin

// Variable to keep track of the encoder position
volatile long encoderLPosition = 0;
volatile long encoderRPosition = 0;

float tickPerMeter = 6425.5; // Number of ticks per meter for the encoders
//used to be 6777.0 for original robot will see for the second one (wood plate)

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
// float Kp = 1.0, Ki = 0.01, Kd = 0.1; //original values
//playing with these values
float Kp = 5.0, Ki = 0.01, Kd = 0.1; //new values


//PID variabales for different PID control straight and rotation 
// Define separate PID constants and variables
// For straight line motion
float Kp_straight = 5.0, Ki_straight = 0.01, Kd_straight = 0.1;
float errorSum_straight = 0, lastError_straight = 0;
unsigned long lastTime_straight = 0;

// For rotation
float Kp_turn = 0.5, Ki_turn = 0.1, Kd_turn = 0.05;
float errorSum_turn = 0, lastError_turn = 0;
unsigned long lastTime_turn = 0;

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
    
    // Serial.print("speed PWM: ");
    // Serial.print(leftSpeed);
    // Serial.print(" | ");
    // Serial.println(rightSpeed);

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
    float traveledDistance = (encoderLPosition + encoderRPosition) / (2.0 * tickPerMeter);

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
    //Serial.print(correction);
    float adjustedSpeed = constrain(50 + abs(correction), 50, speed);
    //Serial.print(" | ");
    //Serial.println(adjustedSpeed);

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


void moveStraight2(float distance, int baseSpeed) {
  encoderLPosition = 0;
  encoderRPosition = 0;

  float currentYawLocal = 0;
  if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
    currentYawLocal = currentYaw;
    xSemaphoreGive(positionMutex);
  }
  targetYaw = currentYawLocal; // Set target heading

  while (true) {
    distanceCheck2();

    while (obstacle == true) {//later can include the distanceCheck in the while loop and the function should return true or false it's better
      stopMotors();
      distanceCheck2();
    }

    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
      currentYawLocal = currentYaw;
      xSemaphoreGive(positionMutex);
    }

    

    // Correct heading using PID
    float yawError = normalizeAngle(targetYaw - currentYawLocal);
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
    
    // Serial.print("speed PWM: ");
    // Serial.print(leftSpeed);
    // Serial.print(" | ");
    // Serial.println(rightSpeed);

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
    float traveledDistance = (encoderLPosition + encoderRPosition) / (2.0 * tickPerMeter);

    if (traveledDistance >= distance) {
      stopMotors();
      break;
    }
  }
}

void turnByAngle2(float angle, int speed) {
  
  float currentYawLocal = 0;
  if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
    currentYawLocal = currentYaw;
    xSemaphoreGive(positionMutex);
  }
  //float angledifforg = normalize(currentYaw + angle);
  float angleReal = normalizeAngle(currentYawLocal + angle); 


  while (true) {
    distanceCheck2();
    while (obstacle == true) {
      stopMotors();
      distanceCheck2();
    }


    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
      currentYawLocal = currentYaw;
      xSemaphoreGive(positionMutex);
    }

    targetYaw = normalizeAngle(angleReal - currentYawLocal); //not really target yaw but angle difference shortest way +ive for clockwise
    float yawError = targetYaw;
    float correction = PIDControl(yawError);
    //Serial.print(correction);
    float adjustedSpeed = constrain(50 + abs(correction), 50, speed);
    //Serial.print(" | ");
    //Serial.println(adjustedSpeed);

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

    if (abs(yawError) < 1.5) { // Stop when error is small
      stopMotors();
      break;
    }

  }
  

}

/*
void turnAngle3(float angle, int maxSpeed) {
  resetTurnPID();  // Reset the turn PID controller

    // Get initial heading
    float currentYawLocal = 0;
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
        currentYawLocal = currentYaw;
        xSemaphoreGive(positionMutex);
    }
    
    // Calculate target angle
    float targetAngle = normalizeAngle(currentYawLocal + angle);
    
    // Debug info
    Serial.print("Starting turn from ");
    Serial.print(currentYawLocal);
    Serial.print(" to ");
    Serial.println(targetAngle);
    
    // Motion profile parameters
    const int minSpeed = 100;             // Minimum speed to overcome static friction
    const float accelTime = 0.2;         // seconds to reach max speed
    const float decelTime = 0.3;         // seconds to stop from max speed
    const float accelRate = (maxSpeed - minSpeed) / accelTime;  
    const float decelRate = (maxSpeed - minSpeed) / decelTime;  
    
    // Convert angle to "distance" for the profile
    float absTurnAngle = abs(angle);
    
    // Calculate angular "distances" for each phase (in degrees)
    float accelAngle = (0.5 * (maxSpeed + minSpeed) * accelTime) / 25.0; // Convert speed to degrees
    float decelAngle = (0.5 * (maxSpeed + minSpeed) * decelTime) / 25.0; // Convert speed to degrees
    
    // Check if turn is too small for full profile
    if (accelAngle + decelAngle > absTurnAngle) {
        // Scale back for shorter turns
        float ratio = absTurnAngle / (accelAngle + decelAngle);
        float peakSpeed = minSpeed + (maxSpeed - minSpeed) * ratio;
        
        // Recalculate acceleration and deceleration angles
        accelAngle = absTurnAngle * (peakSpeed - minSpeed) / (2 * peakSpeed - 2 * minSpeed);
        decelAngle = absTurnAngle - accelAngle;
        maxSpeed = peakSpeed;
    }
    
    // Calculate cruising angle
    float cruiseAngle = absTurnAngle - accelAngle - decelAngle;
    
    // Debug information
    Serial.print("Turn profile: ");
    Serial.print(accelAngle);
    Serial.print("° accel, ");
    Serial.print(cruiseAngle);
    Serial.print("° cruise, ");
    Serial.print(decelAngle);
    Serial.print("° decel, max speed ");
    Serial.println(maxSpeed);
    
    // Timing variables
    unsigned long startTime = millis();
    unsigned long lastDebugTime = startTime;
    
    while (true) {
        // Check for obstacles
        // distanceCheck2();
        // while (obstacle == true) {
        //     stopMotors();
        //     distanceCheck2();
        // } 
        //do we really need to check for obstacles while turning?
        
        // Get current heading
        if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
            currentYawLocal = currentYaw;
            xSemaphoreGive(positionMutex);
        }
        
        // Calculate error and progress
        float yawError = normalizeAngle(targetAngle - currentYawLocal);
        float turnProgress = absTurnAngle - abs(yawError);
        
        // Calculate base speed according to position in trapezoidal profile
        int baseSpeed;
        
        if (turnProgress < accelAngle) {
            // Acceleration phase - linear speed increase
            float progress = turnProgress / accelAngle;
            baseSpeed = minSpeed + progress * (maxSpeed - minSpeed);
            
        } else if (turnProgress < (accelAngle + cruiseAngle)) {
            // Cruise phase - constant speed
            baseSpeed = maxSpeed;
            
        } else if (turnProgress < absTurnAngle) {
            // Deceleration phase - linear speed decrease
            float remaining = absTurnAngle - turnProgress;
            float progress = remaining / decelAngle;
            baseSpeed = minSpeed + progress * (maxSpeed - minSpeed);
            
        } else {
            // We've completed the turn
            stopMotors();
            Serial.println("Target angle reached!");
            break;
        }
        
        // Apply PID for fine control

        float correction = PIDControlTurn(yawError);  // Use turn-specific PID

        float adjustedSpeed = constrain(baseSpeed + abs(correction), minSpeed, maxSpeed);
        
        // Turn based on error direction
        if (yawError < 0) {
            // Turn clockwise
            digitalWrite(in1A, LOW);
            digitalWrite(in2A, HIGH);
            analogWrite(pwmA, adjustedSpeed);
            digitalWrite(in1B, LOW);
            digitalWrite(in2B, HIGH);
            analogWrite(pwmB, adjustedSpeed);
            
        } else {
            // Turn counter-clockwise
            digitalWrite(in1A, HIGH);
            digitalWrite(in2A, LOW);
            analogWrite(pwmA, adjustedSpeed);
            digitalWrite(in1B, HIGH);
            digitalWrite(in2B, LOW);
            analogWrite(pwmB, adjustedSpeed);
        }
        
        // Debug output every 100ms
        unsigned long currentTime = millis();
        if (currentTime - lastDebugTime > 100) {
            Serial.print("Angle: ");
            Serial.print(currentYawLocal);
            Serial.print("/");
            Serial.print(targetAngle);
            Serial.print(" (");
            Serial.print(turnProgress);
            Serial.print("°/");
            Serial.print(absTurnAngle);
            Serial.print("°), Speed: ");
            Serial.println(baseSpeed);
            
            lastDebugTime = currentTime;
        }
        
        // Stop condition - more precise for final positioning
        if (abs(yawError) < 0.5) { 
            stopMotors();
            Serial.println("Target angle reached precisely!");
            break;
        }
        
        // Small delay for system stability
        delay(5);
    }
    
    Serial.print("Turn completed. Requested: ");
    Serial.print(angle);
    Serial.print("°, Final heading: ");
    Serial.println(currentYawLocal);
}
*/

void turnByAngle3(float angle, int maxSpeed) {
    // Reset PID controller
    resetTurnPID();
    
    // Get initial heading
    float currentYawLocal = 0;
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
        currentYawLocal = currentYaw;
        xSemaphoreGive(positionMutex);
    }
    
    // Calculate target angle
    float targetAngle = normalizeAngle(currentYawLocal + angle);
    
    // Debug info
    Serial.print("Starting turn from ");
    Serial.print(currentYawLocal);
    Serial.print(" to ");
    Serial.println(targetAngle);
    
    // Set minimum speed to overcome friction
    const int minSpeed = 60; 
    
    // Timing variables for debug output
    unsigned long lastDebugTime = millis();
    
    while (true) {
        // Get current heading
        if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
            currentYawLocal = currentYaw;
            xSemaphoreGive(positionMutex);
        }
        
        // Calculate error 
        float yawError = normalizeAngle(targetAngle - currentYawLocal);
        
        // Apply PID for control
        float correction = PIDControlTurn(yawError);
        
        // Calculate speed based on error magnitude and PID output
        // This creates a natural speed curve without a trapezoidal profile
        float adjustedSpeed = constrain(minSpeed + abs(correction), minSpeed, maxSpeed);
        
        // Apply motor control based on direction
        if (yawError < 0) {
            // Turn clockwise
            digitalWrite(in1A, LOW);
            digitalWrite(in2A, HIGH);
            analogWrite(pwmA, adjustedSpeed);
            digitalWrite(in1B, LOW);
            digitalWrite(in2B, HIGH);
            analogWrite(pwmB, adjustedSpeed);
        } else {
            // Turn counter-clockwise
            digitalWrite(in1A, HIGH);
            digitalWrite(in2A, LOW);
            analogWrite(pwmA, adjustedSpeed);
            digitalWrite(in1B, HIGH);
            digitalWrite(in2B, LOW);
            analogWrite(pwmB, adjustedSpeed);
        }
        
        // Debug output every 100ms
        if (millis() - lastDebugTime > 100) {
            Serial.print("Current: ");
            Serial.print(currentYawLocal);
            Serial.print(" Target: ");
            Serial.print(targetAngle);
            Serial.print(" Error: ");
            Serial.print(yawError);
            Serial.print(" Speed: ");
            Serial.println(adjustedSpeed);
            lastDebugTime = millis();
        }
        
        // Stop when close enough to target
        if (abs(yawError) < 0.5 ) {
            stopMotors();
            Serial.println("Target angle reached!");
            break;
        }
        
        delay(5); // Small stability delay
    }
}

void moveStraight3(float distance, int maxSpeed) {
    // Reset encoders
    encoderLPosition = 0;
    encoderRPosition = 0;
    resetStraightPID();  // Reset the straight-line PID controller
    
    // Get initial heading to maintain
    float currentYawLocal = 0;
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
        currentYawLocal = currentYaw;
        xSemaphoreGive(positionMutex);
    }
    float targetHeading = currentYawLocal; // Set target heading to maintain
    
    // Calculate the distance in encoder ticks
    int ticksToTravel = abs(distance) * tickPerMeter; // Convert meters to ticks
    
    // Direction based on sign of distance
    bool forward = distance > 0;
    
    // Motion profile parameters
    const int minSpeed = 100;           // Minimum speed to overcome static friction
    const float accelTime = 0.1;       // seconds to reach max speed
    const float decelTime = 0.1;       // seconds to stop from max speed
    const float accelRate = (maxSpeed - minSpeed) / accelTime;  // speed units/second
    const float decelRate = (maxSpeed - minSpeed) / decelTime;  // speed units/second
    
    // Calculate distances for each phase (in ticks)
    int accelDist = (0.5 * (maxSpeed + minSpeed) * accelTime) * (tickPerMeter / 60.0); // in ticks
    int decelDist = (0.5 * (maxSpeed + minSpeed) * decelTime) * (tickPerMeter / 60.0); // in ticks
    
    // Check if movement is too short for full profile
    if (accelDist + decelDist > ticksToTravel) {
        // Scale back for shorter moves - calculate peak speed reachable
        float ratio = (float)ticksToTravel / (accelDist + decelDist);
        float peakSpeed = minSpeed + (maxSpeed - minSpeed) * ratio;
        
        // Recalculate acceleration and deceleration distances
        accelDist = ticksToTravel * (peakSpeed - minSpeed) / (2 * peakSpeed - 2 * minSpeed);
        decelDist = ticksToTravel - accelDist;
        maxSpeed = peakSpeed;
    }
    
    // Calculate cruising distance
    int cruiseDist = ticksToTravel - accelDist - decelDist;
    
    // Debug information
    Serial.print("Movement profile: ");
    Serial.print(accelDist);
    Serial.print(" ticks accel, ");
    Serial.print(cruiseDist);
    Serial.print(" ticks cruise, ");
    Serial.print(decelDist);
    Serial.print(" ticks decel, max speed ");
    Serial.println(maxSpeed);
    
    // Timing variables
    unsigned long startTime = millis();
    unsigned long lastDebugTime = startTime;
    
    while (true) {
        // Check for obstacles
        // distanceCheck2();
        // // if (obstacle) {
        // //     stopMotors();
        // //     Serial.println("Obstacle detected! Stopping.");
        // //     //break;// unteresting for path replanning but not for now
        // // }
        //     while (obstacle == true) {//later can include the distanceCheck in the while loop and the function should return true or false it's better
        //     stopMotors();
        //     distanceCheck2();
        //   }

        obstacleDetect(); // Check for obstacles using the ultrasound sensor
        while(obstacle == true) { // If an obstacle is detected, stop and check again
            stopMotors();
            obstacleClear(); // Re-check for obstacle if cleared so basically verify longer
        }
        
        // Get current position in ticks
        int currentPosition = (abs(encoderLPosition) + abs(encoderRPosition)) / 2;
        
        // Get current heading
        if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
            currentYawLocal = currentYaw;
            xSemaphoreGive(positionMutex);
        }
        
        // Calculate heading error and correction
        float yawError = normalizeAngle(targetHeading - currentYawLocal);
        float correction = PIDControlStraight(yawError);  // Use straight-specific PID
        
        // Calculate base speed according to position in trapezoidal profile
        int baseSpeed;
        
        if (currentPosition < accelDist) {
            // Acceleration phase - linear speed increase
            float progress = (float)currentPosition / accelDist;
            baseSpeed = minSpeed + progress * (maxSpeed - minSpeed);
            
        } else if (currentPosition < accelDist + cruiseDist) {
            // Cruise phase - constant speed
            baseSpeed = maxSpeed;
            
        } else if (currentPosition < ticksToTravel) {
            // Deceleration phase - linear speed decrease
            float remaining = (float)(ticksToTravel - currentPosition);
            float progress = remaining / decelDist;
            baseSpeed = minSpeed + progress * (maxSpeed - minSpeed);
            
        } else {
            // We've reached or exceeded the target distance
            stopMotors();
            Serial.println("Target distance reached!");
            break;
        }
        
        // Apply heading correction to motor speeds
        int leftSpeed = baseSpeed + correction;
        int rightSpeed = baseSpeed - correction;
        
        // Apply speed to motors with correction

            // Set Motor A 
            digitalWrite(in1A, HIGH);
            digitalWrite(in2A, LOW);
            analogWrite(pwmA, constrain(leftSpeed, minSpeed, maxSpeed));
            
            // Set Motor B
            digitalWrite(in1B, LOW);
            digitalWrite(in2B, HIGH);
            analogWrite(pwmB, constrain(rightSpeed, minSpeed, maxSpeed));


        
        // Debug output every 200ms
        unsigned long currentTime = millis();
        if (currentTime - lastDebugTime > 200) {
            Serial.print("Position: ");
            Serial.print(currentPosition);
            Serial.print("/");
            Serial.print(ticksToTravel);
            Serial.print(", Speed: ");
            Serial.print(baseSpeed);
            Serial.print(", Yaw error: ");
            Serial.print(yawError);
            Serial.print(", L/R: ");
            Serial.print(leftSpeed);
            Serial.print("/");
            Serial.println(rightSpeed);
            
            lastDebugTime = currentTime;
        }
        
        // Small delay for system stability
        delay(5);
    }
    
    // Final position report
    int finalPosition = (abs(encoderLPosition) + abs(encoderRPosition)) / 2;
    float actualDistance = (float)finalPosition / tickPerMeter; // Convert ticks back to meters
    
    Serial.print("Move completed. Requested: ");
    Serial.print(distance);
    Serial.print("m, Actual: ");
    Serial.print(actualDistance);
    Serial.println("m");
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
      // Serial.print("deltaTime: ");
      // Serial.print(deltaTime);
      // Serial.print("error: ");
      // Serial.print(error);
      // Serial.print(" | errorSum: ");
      // Serial.print(errorSum);
      // Serial.print(" | errorRate: ");
      // Serial.print(errorRate);
      // Serial.print(" | output: ");
      // Serial.println(output);
      

  // Save last values
  lastError = error;
  lastTime = now;

  return output;
}



// PID control for straight line motion
float PIDControlStraight(float error) {
  unsigned long now = millis();
  float deltaTime = (now - lastTime_straight) / 1000.0; // Convert to seconds

  if (deltaTime <= 0) {
    deltaTime = 0.01; // Set a minimum deltaTime (10 ms)
  }

  // Anti-windup - limit integral term
  errorSum_straight += error * deltaTime;
  errorSum_straight = constrain(errorSum_straight, -10.0, 10.0);  // Prevent windup
  
  float errorRate = (error - lastError_straight) / deltaTime;

  // PID output
  float output = (Kp_straight * error) + (Ki_straight * errorSum_straight) + (Kd_straight * errorRate);

  // Save last values
  lastError_straight = error;
  lastTime_straight = now;

  return output;
}

// PID control for rotation
float PIDControlTurn(float error) {
  unsigned long now = millis();
  float deltaTime = (now - lastTime_turn) / 1000.0; // Convert to seconds

  if (deltaTime <= 0) {
    deltaTime = 0.01; // Set a minimum deltaTime (10 ms)
  }

  // Anti-windup - limit integral term
  errorSum_turn += error * deltaTime;
  errorSum_turn = constrain(errorSum_turn, -20.0, 20.0);  // Prevent windup
  
  float errorRate = (error - lastError_turn) / deltaTime;

  // PID output
  float output = (Kp_turn * error) + (Ki_turn * errorSum_turn) + (Kd_turn * errorRate);

  // Save last values
  lastError_turn = error;
  lastTime_turn = now;

  return output;
}

// Reset functions to use when starting new movements
void resetStraightPID() {
  errorSum_straight = 0;
  lastError_straight = 0;
  lastTime_straight = millis();
}

void resetTurnPID() {
  errorSum_turn = 0;
  lastError_turn = 0;
  lastTime_turn = millis();
}