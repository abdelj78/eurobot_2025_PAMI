#include "Arduino.h"


//Ultrasound sensor settings
const int trigPin = 25;
const int echoPin = 33;
float duration, distance;
bool obstacle = false; 


void ultrasoundSetup() {
    pinMode(trigPin, OUTPUT); // Set trigPin as output
    pinMode(echoPin, INPUT);  // Set echoPin as input
    Serial.println("Ultrasound setup complete.");
}

void ultrasoundLoop() {
    // This function can be used to continuously check the distance if needed
    // For now, it's empty as we are only using distanceCheck() in the main loop
}

void distanceCheck() {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    duration = pulseIn(echoPin, HIGH); //code blocks here until gets response (or times out, max 1 second)
                                       //can set different timout like this: duration = pulseIn(echoPin, HIGH, 30000);  // Timeout in 30ms 
    distance = (duration*.0343)/2;
    Serial.print("Distance: ");
    Serial.println(distance);
    //delay(100);
    if (distance < 10) {
      obstacle = true;
    } else {
      obstacle = false;
    }
  
  }

  