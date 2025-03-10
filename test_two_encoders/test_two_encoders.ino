// Encoder 1
const int encoder1PinA = 2;//3;//2; // Interrupt pin
const int encoder1PinB = 15;//14;//15; // Regular digital pin

// Encoder 2
const int encoder2PinA = 3; // Interrupt pin
const int encoder2PinB = 14; // Regular digital pin

// Variable to keep track of the encoder position
volatile long encoder1Position = 0;
volatile long encoder2Position = 0;

void setup() {
  // Set the encoder pins as inputs
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT);

  // Attach an interrupt to encoderPinA
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder2, CHANGE);

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Print the encoder position
  Serial.print("Encoder Position: ");
  Serial.print(encoder1Position);
  Serial.print(" | ");
  Serial.println(encoder2Position);

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
