#include <Arduino.h>
#include "mpu_test.h"
#include "motor.h"
#define TEST_PIN 36 // GPIO pin for testing

void setup() {
    //mpuSetup();
    //Serial.begin(115200);
    // Serial.println("Setup started.");
    motorSetup();
    //pinMode(TEST_PIN, INPUT_PULLUP); // Set GPIO 19 as output for LED

}

void loop() {
    //mpuLoop();
    motorLoop();
    //int state = digitalRead(TEST_PIN);
    //Serial.println(state);  // Print HIGH (1) or LOW (0)
    //delay(500);
}
