/* ROBOT STURCTURE
front is rolling ball

LEFT motor
encoder1
motorA

RIGHT motor
encoder2
motorB
*/


// Encoder 1
const int encoder1PinA = 2;//3;//2; // Interrupt pin
const int encoder1PinB = 15;//14;//15; // Regular digital pin

// Encoder 2
const int encoder2PinA = 3; // Interrupt pin
const int encoder2PinB = 14; // Regular digital pin


// Variable to keep track of the encoder position
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;

// Variable to store the last state of encoderPinB
volatile int lastEncoderPinBState = 0;

// Motor A
int pwmA = 9;//5;
int in1A = 7;//2; //used to be 3 but changed to save pwm output
int in2A = 4;//4;
// Motor B
int pwmB = 10;//6;
int in1B = 8;//7;
int in2B = 12;//8;
// Motor Speed Values - Start at zero
int MotorSpeed1 = 100;
int MotorSpeed2 = 100;

volatile float encoder1Dist = 0;
volatile float encoder2Dist = 0;

void setup() {
  // Set the encoder pins as inputs
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT);

    // Set all the motor control pins to outputs
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(in1A, OUTPUT);
  pinMode(in2A, OUTPUT);
  pinMode(in1B, OUTPUT);
  pinMode(in2B, OUTPUT);

  // Attach an interrupt to encoderPinA
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2, CHANGE);

  // Initialize serial communication for debugging
  Serial.begin(15200);
  forward();
}

void loop() {

  encoder1Dist = encoder1Position / 6777.0;
  encoder2Dist = encoder2Position / 6777.0; 

  // Serial.print("Distance: ");
  // Serial.print(encoder1Dist);
  // Serial.print(" - ");
  // Serial.println(encoder2Dist);


  if(encoder1Dist > 0.1) {
    stopMotor1();
  }

  if(encoder2Dist > 0.1) {
    stopMotor2();
  }



  // Print the encoder position
  // Serial.print("Encoder position 1 - 2 : ");
  // Serial.print(encoder1Position);
  // Serial.print(" - ");
  // Serial.println(encoder2Position);

  // Add a small delay to avoid flooding the serial output
  //delay(100);
}

void forward() {
  // Set Motor A backward 
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, HIGH);
  analogWrite(pwmA, MotorSpeed1);

  // Set Motor B forward
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, HIGH);
  analogWrite(pwmB, MotorSpeed2);
}

void stopMotor1() {
  digitalWrite(in1A, LOW);
  digitalWrite(in2A, LOW);
  analogWrite(pwmA, 0);
}

void stopMotor2() {
  digitalWrite(in1B, LOW);
  digitalWrite(in2B, LOW);
  analogWrite(pwmB, 0);
}

// Interrupt service routine to handle the encoder pulses
void handleEncoder1() {

  if (digitalRead(encoder1PinA) == HIGH) {
    if (digitalRead(encoder1PinB) == LOW) {
      encoder1Position++;
    } else { 
      encoder1Position--;
    }
    
  } else {
    if (digitalRead(encoder1PinB) == HIGH) {
      encoder1Position++;
    } else {
      encoder1Position--;
    }
  }
}

// Interrupt service routine to handle the encoder pulses
void handleEncoder2() {

  if (digitalRead(encoder2PinA) == HIGH) {
    if (digitalRead(encoder2PinB) == LOW) {
      encoder2Position--;
    } else { 
      encoder2Position++;
    }
    
  } else {
    if (digitalRead(encoder2PinB) == HIGH) {
      encoder2Position--;
    } else {
      encoder2Position++;
    }
  }
}