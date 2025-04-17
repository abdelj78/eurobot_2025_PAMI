//This code tries to integrate functional sub-systems of the PAMI
//US sensor and motor driver with normal DC motors for now
// Include the Servo library 
#include <Servo.h> 


//Servo settings
// Declare the Servo pin 
const int servoPin = 10;
// Create a servo object 
Servo myServo; 

int servoState = 0;

//Ultrasound sensor settings
const int trigPin = 6;
const int echoPin = 13;
float duration, distance;
bool obstacle = false; 


const int startPin = 12;  // Best practice


// Motor A
int pwmA = 3;
int in1A = 2;
int in2A = 4;

// Motor B
int pwmB = 11;
int in1B = 7;
int in2B = 8;

// Motor Speed Values - Start at zero
int MotorSpeed1 = 100;
int MotorSpeed2 = 100;

void setup() {
  Serial.begin(9600);  // Start serial communication (optional)

  // Set all the motor control pins to outputs
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Set ultrasound sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  // put your setup code here, to run once:
  myServo.attach(servoPin);  

  myServo.write(0);
  delay(3000);
  myServo.write(180);
  delay(1000);

  // Set start pin and wait until pressed
  pinMode(startPin, INPUT_PULLUP);  // Set the pin as an input
  Serial.println("Waiting for start signal...");
  while (digitalRead(startPin) == HIGH) {
    // Do nothing, just wait
  }
  
  Serial.println("Start signal received! setting motor...");

  // Make servo go to 90 degrees 
  myServo.write(78); //78 degrees seems to be looking straight in front for the robot
  delay(1000);    


  // Set Motor A forward 
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  
  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  
  // Set the motor speeds
  analogWrite(pwmA, MotorSpeed1);
  analogWrite(pwmB, MotorSpeed2);
  Serial.println("end of setup");
  


}


void loop() {
  servoState++;
  if (servoState == 0){
    myServo.write(48);
    delay(100);
  }else if (servoState == 1){
    myServo.write(78);
    delay(100);
  }else if (servoState == 2){
    myServo.write(108);
    delay(100);
  }else if (servoState == 3){
    myServo.write(78);
    delay(100);
    servoState = -1;
  }

  
  // put your main code here, to run repeatedly:
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
  delay(100);

  if(distance < 30){ //les than x cm then rotate 
    
  // Set Motor A backward 
  digitalWrite(in1A, HIGH);
  digitalWrite(in2A, LOW);
  
  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  
  delay(500);  

  // Set Motor A forward 
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  
  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);

  }

}
