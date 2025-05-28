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

  duration = pulseIn(echoPin, HIGH, 30000); //code blocks here until gets response (or times out, max 1 second)
                                      //can set different timout like this: duration = pulseIn(echoPin, HIGH, 30000);  // Timeout in 30ms 
  distance = (duration*.0343)/2;
  // Serial.print("Distance: ");
  // Serial.println(distance);
  //delay(100);
  if (distance < 10 && distance > 0) { // Check if the distance is less than 10 cm and greater than 0 cm
    //here we add it larger than 0 to avoid false positives when the sensor is not detecting anything
    //occurs when obstacle is very far away or not plugged in
    obstacle = true;
  } else {
    obstacle = false;
  }
}


void distanceCheck2() {
  for (int i = 0; i < 4; i++) { // Take 5 to not have the robot slightly moving if misreads one time
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
  
    duration = pulseIn(echoPin, HIGH, 30000); //code blocks here until gets response (or times out, max 1 second)
                                       //can set different timout like this: duration = pulseIn(echoPin, HIGH, 30000);  // Timeout in 30ms 
    distance = (duration*.0343)/2;
    // Serial.print("Distance: ");
    // Serial.println(distance);
    //delay(100);
    if (distance < 10 && distance > 0) { // Check if the distance is less than 10 cm and greater than 0 cm
      //here we add it larger than 0 to avoid false positives when the sensor is not detecting anything
      //occurs when obstacle is very far away or not plugged in
      obstacle = true;
      break;
    } else {
      obstacle = false;
    }
  }
}


////Alternative: Non-Blocking Method (Avoid pulseIn())
////
////Instead of using pulseIn(), you can use interrupts for better performance:
//
//volatile long startTime, endTime;
//volatile bool received = false;
//
//void echoISR() {
//  if (digitalRead(echoPin) == HIGH) {
//    startTime = micros();
//  } else {
//    endTime = micros();
//    received = true;
//  }
//}
//
//void ultrasoundSetup() {
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
//  Serial.begin(9600);
//}
//
//void loop() {
//  digitalWrite(trigPin, LOW);
//  delayMicroseconds(2);
//  digitalWrite(trigPin, HIGH);
//  delayMicroseconds(10);
//  digitalWrite(trigPin, LOW);
//
//  if (received) {
//    long duration = endTime - startTime;
//    float distance = (duration * 0.0343) / 2;
//    Serial.print("Distance: ");
//    Serial.println(distance);
//    received = false;
//  }
//
//  delay(100);
//}

  