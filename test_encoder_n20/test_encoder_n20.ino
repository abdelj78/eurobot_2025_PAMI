// Encoder 1
const int encoder1PinA = 39;//3;//2; // Interrupt pin
const int encoder1PinB = 36;//14;//15; // Regular digital pin

// Encoder 2
// const int encoder2PinA = 3; // Interrupt pin
// const int encoder2PinB = 14; // Regular digital pin


// Variable to keep track of the encoder position
//volatile long encoderPosition = 0;
volatile long encoderPosition = 0;

// Variable to store the last state of encoderPinB
volatile int lastEncoderPinBState = 0;

void setup() {
  // Set the encoder pins as inputs
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT);
  // pinMode(encoder2PinA, INPUT_PULLUP);
  // pinMode(encoder2PinB, INPUT);

  // Attach an interrupt to encoderPinA
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), handleEncoder, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(encoder2PinA), handleEncoder, CHANGE);

  // Initialize serial communication for debugging
  Serial.begin(9600);
}

void loop() {
  // Print the encoder position
  Serial.print("Encoder Position: ");
  Serial.println(encoderPosition);

  // Add a small delay to avoid flooding the serial output
  delay(100);
}

// Interrupt service routine to handle the encoder pulses
void handleEncoder() {
  // Read the state of encoderPinB
  //int encoderPinBState = digitalRead(encoder1PinB);

  // // Determine the direction of rotation
  // if (encoderPinBState != lastEncoderPinBState) { // To check if the encoder has moved and avoid double counting, not sure if necessary
  //   if (encoderPinBState == HIGH) {
  //     encoderPosition++;
  //   } else {
  //     encoderPosition--;
  //   }
  // }
  if (digitalRead(encoder1PinA) == HIGH) {
    if (digitalRead(encoder1PinB) == LOW) {
      encoderPosition++;
    } else { 
      encoderPosition--;
    }
    
  } else {
    if (digitalRead(encoder1PinB) == HIGH) {
      encoderPosition++;
    } else {
      encoderPosition--;
    }
  }
  //Serial.println("in rising");

  // Update the last state of encoderPinB
  //lastEncoderPinBState = encoderPinBState;
}