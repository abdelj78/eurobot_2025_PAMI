#define TRIG_PIN 4  // GPIO5 for Trigger
#define ECHO_PIN 18 // GPIO18 for Echo

void setup() {
  Serial.begin(115200);  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // Send a 10µs pulse to the TRIG pin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Measure the duration of the ECHO pulse
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Convert time (µs) to distance (cm)
  float distance = duration * 0.0343 / 2;  

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(500);  // Wait 500ms before next measurement
}