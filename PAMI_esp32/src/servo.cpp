// #include <Arduino.h>

// // Servo control using PWM on ESP32
// const int servoPin = 32;  // Choose a PWM-capable GPIO (e.g., 18)

// const int minPulseWidth = 500;   // in microseconds (for 0 degrees)
// const int maxPulseWidth = 2400;  // in microseconds (for 180 degrees)
// const int frequency = 50;        // Servo signal frequency (50Hz)
// const int pwmChannel = 0;        // LEDC channel
// const int resolution = 16;       // 16-bit resolution

// void setServoAngle(int angle) {
//   // Clamp angle
//   angle = constrain(angle, 0, 180);

//   // Map angle to duty cycle (based on pulse width)
//   int us = map(angle, 0, 180, minPulseWidth, maxPulseWidth);
//   int duty = (int)((us / 20000.0) * 65535); // 20ms period for 50Hz

//   ledcWrite(pwmChannel, duty);
// }

// void servoSetup() {
//   ledcSetup(pwmChannel, frequency, resolution);
//   ledcAttachPin(servoPin, pwmChannel);
// }

// void servoLoop() {
//   // Sweep from 0° to 180°
//   for (int angle = 0; angle <= 180; angle += 1) {
//     setServoAngle(angle);
//     delay(15);  // 15ms per step = smooth motion
//   }

//   // Sweep back from 180° to 0°
//   for (int angle = 180; angle >= 0; angle -= 1) {
//     setServoAngle(angle);
//     delay(15);
//   }
// }


#include <Arduino.h>
#include <ESP32Servo.h>

Servo myServo;

int servoPin = 32; // GPIO pin for the servo

void servoSetup() {
  myServo.setPeriodHertz(50);             // Standard 50Hz servo
  myServo.attach(servoPin, 500, 2400);          // pin, min & max pulse widths in µs
}

void servoLoop() {
  while (true) {
    // Sweep 0 to 180
    for (int pos = 45; pos <= 100; pos++) {
      myServo.write(pos);
      delay(10);  // small delay for smooth motion
    }

    // Sweep 180 back to 0
    for (int pos = 100; pos >= 45; pos--) {
      myServo.write(pos);
      delay(10);
    }
  }


    // myServo.write(0); // Set servo to 0 degrees
    // delay(1000); // Wait for 1 second
    // myServo.write(180); // Set servo to 90 degrees  
    // delay(1000); // Wait for 1 second
}
