// Code to set the servo motor for PAMI at the right angle (front) on the osoyoo robot kit

// Include the Servo library 
#include <Servo.h> 

const int startPin = 12;  // Best practice

// Declare the Servo pin 
const int servoPin = 11;
 
// Create a servo object 
Servo myServo; 

void setup() { 
  // We need to attach the servo to the used pin number 
  //myServo.detach(); //used to disable servo and move if freely by hand
  myServo.attach(servoPin);  
  myServo.write(0);
  delay(3000);
  myServo.write(180);
  delay(1000);

  Serial.begin(9600);  // Start serial communication (optional)
  
  // Set start pin and wait until pressed
  pinMode(startPin, INPUT_PULLUP);  // Set the pin as an input
  Serial.println("Waiting for start signal...");
  while (digitalRead(startPin) == HIGH) {
    // Do nothing, just wait
  }
  
  Serial.println("Start signal received! Running loop...");

}

void loop(){ 
  
  // Make servo go to 90 degrees 
  myServo.write(78); //78 degrees seems to be looking straight in front for the robot
  delay(1000);    

}
