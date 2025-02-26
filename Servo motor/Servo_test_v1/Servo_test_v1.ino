// Code to test servo motor for PAMI


// Include the Servo library 
#include <Servo.h> 

// Declare the Servo pin 
int servoPin = 11;
 
// Create a servo object 
Servo myServo; 

void setup() { 
   // We need to attach the servo to the used pin number 
   myServo.attach(servoPin); 
}

void loop(){ 
   // Make servo go to 0 degrees 
   myServo.write(0); 
   delay(1000); 
   // Make servo go to 90 degrees 
   myServo.write(90); 
   delay(1000); 
   // Make servo go to 180 degrees 
   myServo.write(180); 
   delay(1000); 
}
