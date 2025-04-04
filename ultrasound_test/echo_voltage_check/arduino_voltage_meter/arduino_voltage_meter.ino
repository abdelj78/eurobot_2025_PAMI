int Ain = 14;
float value = 0;
float voltage = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  value = analogRead(Ain);
  voltage = value * 5 / 1023;
  Serial.println(voltage);
}
