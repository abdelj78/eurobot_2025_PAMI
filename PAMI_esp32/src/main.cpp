#include <Arduino.h>
#include "test_n_dev\mpu_test.h"
#include "motor.h"
#include "servo.h"
#include "mpu.h"
#include "ultrasound.h"

#define TEST_PIN 36 // GPIO pin for testing

void setup() {
    Serial.begin(115200);
    mpuSetup();
    ultrasoundSetup();

    // Serial.println("Setup started.");
    motorSetup();
    //pinMode(TEST_PIN, INPUT_PULLUP); // Set GPIO 19 as output for LED
    //servoSetup(); // Initialize servo control
    //moveForwardFor(0.5, 100); // Move forward for 0.5 seconds at speed 180
    
    // //SUPERSTAR BLUE
    // moveStraight(1.27876, 80);
    // turnByAngle(-90.0, 100);
    // moveStraight(0.15,80);
    moveStraight(1.26076, 120);
    turnByAngle(-90.0, 100);
    moveStraight(0.247,120);
  
    // //GROUPIE BLUE CLOSE
    // moveStraight(0.35, 80);
    // turnByAngle(-90, 100);
    // moveStraight(0.3, 80);
    // turnByAngle(90, 100);
    // moveStraight(0.55, 80);  

    // //GROUPIE BLUE MIDDLE
    // moveStraight(0.25, 80);
    // turnByAngle(-90, 100);
    // moveStraight(0.3, 80);
    // turnByAngle(90, 100);
    // moveStraight(1.25, 80);  

    // // //GROUPIE BLUE FURTHEST
    // moveStraight(0.15, 80);
    // turnByAngle(-90, 100);
    // moveStraight(0.3, 80);
    // turnByAngle(90, 100);
    // moveStraight(1.7, 80);  
    // turnByAngle(90, 100);
    // moveStraight(0.15, 80);  

}

void loop() {
    //mpuLoop();
    //motorLoop();
    
    //servoLoop(); // Control the servo
    //int state = digitalRead(TEST_PIN);
    //Serial.println(state);  // Print HIGH (1) or LOW (0)
    //delay(500);
}
