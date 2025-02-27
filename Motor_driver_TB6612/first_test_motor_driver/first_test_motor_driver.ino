/*
  TB6612FNG H-Bridge Demo
  TB6612-Demo.ino
  Demonstrates use of TB6612FNG H-Bridge Motor Controller
  Drives two DC Motors

  DroneBot Workshop 2019
  https://dronebotworkshop.com
*/

// Motor A

int pwmA = 5;
int in1A = 2; //used to be 3 but changed to save pwm output
int in2A = 4;


// Motor B

int pwmB = 6;
int in1B = 7;
int in2B = 8;

// Speed control potentiometers

//int SpeedControl1 = A0;  
//int SpeedControl2 = A1;


// Motor Speed Values - Start at zero

int MotorSpeed1 = 100;
int MotorSpeed2 = 100;

void setup()

{

  // Set all the motor control pins to outputs

  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Set Motor A forward

  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);

  // Set Motor B forward

  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);

    // Set the motor speeds
  
  analogWrite(pwmA, MotorSpeed1);
  analogWrite(pwmB, MotorSpeed2);
   
}

void loop() {
  

  
}
