/*
 * HC-SR04 example sketch
 *
 * https://create.arduino.cc/projecthub/Isaac100/getting-started-with-the-hc-sr04-ultrasonic-sensor-036380
 *
 * by Isaac100
 */

const int trigPin = 25; // 9;
const int echoPin = 33; //10;

unsigned long start_time = 0;
unsigned long end_time = 0;
unsigned long time_spent = 0;

float duration, distance;

void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  Serial.begin(9600);
}

void loop() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  start_time = micros();
  duration = pulseIn(echoPin, HIGH); //code blocks here until gets response (or times out, max 1 second)
                                     //can set different timout like this: duration = pulseIn(echoPin, HIGH, 30000);  // Timeout in 30ms 
  end_time = micros();
  time_spent = end_time - start_time;
  Serial.print("time spent = ");
  Serial.println(time_spent);

  distance = (duration*.0343)/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(100);
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
//void setup() {
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
//
////✅ Pros of this method:
////
////    Does not block the loop, so other parts of the robot continue running.
////    Uses hardware interrupts, making it more efficient.



////BETTER METHOD
////✅ Improved Non-Blocking Ultrasonic Sensor Code
//
//#include <Arduino.h>
//
//const int trigPin = 9;
//const int echoPin = 10;
//
//volatile long startTime = 0;
//volatile long endTime = 0;
//volatile bool received = false;
//
//void echoISR() {
//  if (digitalRead(echoPin) == HIGH) {
//    startTime = micros();  // Start counting when echo pin goes HIGH
//  } else {
//    endTime = micros();    // Stop counting when echo pin goes LOW
//    received = true;       // Signal that a measurement was received
//  }
//}
//
//void setup() {
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE); // Attach interrupt to echoPin
//  Serial.begin(9600);
//}
//
//void loop() {
//  static unsigned long lastTriggerTime = 0;
//  const unsigned long triggerInterval = 60;  // Minimum recommended interval between measurements (in ms)
//
//  if (millis() - lastTriggerTime >= triggerInterval) {
//    lastTriggerTime = millis();  // Update last trigger time
//
//    // Send a 10µs pulse to trigger the sensor
//    digitalWrite(trigPin, LOW);
//    delayMicroseconds(2);
//    digitalWrite(trigPin, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(trigPin, LOW);
//  }
//
//  if (received) {  
//    noInterrupts();  // Temporarily disable interrupts to read shared variables safely
//    long duration = endTime - startTime;
//    received = false;  // Reset flag
//    interrupts();  // Re-enable interrupts
//
//    // Convert duration to distance in cm
//    float distance = (duration * 0.0343) / 2;
//    Serial.print("Distance: ");
//    Serial.println(distance);
//  }
//}



////best top level 
//const int trigPin = 9;
//const int echoPin = 10;
//
//volatile long startTime = 0;
//volatile long endTime = 0;
//volatile bool received = false;
//float distance = 0;
//
//void echoISR() {
//  if (digitalRead(echoPin) == HIGH) {
//    startTime = micros();  // Start timing when echo pin goes HIGH
//  } else {
//    endTime = micros();    // Stop timing when echo pin goes LOW
//    received = true;       // Flag to indicate a new measurement
//  }
//}
//
//void setup() {
//  pinMode(trigPin, OUTPUT);
//  pinMode(echoPin, INPUT);
//  attachInterrupt(digitalPinToInterrupt(echoPin), echoISR, CHANGE);
//  Serial.begin(115200);
//}
//
//void loop() {
//  static unsigned long lastTriggerTime = 0;
//  const unsigned long triggerInterval = 50;  // Trigger every 50ms (20Hz)
//
//  // Trigger the sensor at a fixed rate
//  if (millis() - lastTriggerTime >= triggerInterval) {
//    lastTriggerTime = millis();
//
//    // Send a 10µs pulse to start measurement
//    digitalWrite(trigPin, LOW);
//    delayMicroseconds(2);
//    digitalWrite(trigPin, HIGH);
//    delayMicroseconds(10);
//    digitalWrite(trigPin, LOW);
//  }
//
//  // Process the received measurement
//  if (received) {
//    noInterrupts();  // Prevent interrupt conflicts
//    long duration = endTime - startTime;
//    received = false;
//    interrupts();  // Re-enable interrupts
//
//    distance = (duration * 0.0343) / 2;  // Convert time to distance in cm
//
//    Serial.print("Distance: ");
//    Serial.print(distance);
//    Serial.println(" cm");
//  }
//}
