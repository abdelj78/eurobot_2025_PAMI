// this is for all top pcb components except servo motor
#include "general_pins.h"
#include "Arduino.h"


int led_red = 2; // GPIO pin for red LED
int led_yellow = 5;
int led_blue = 12;
int uswitch = 27; // GPIO pin for micro-switch
int clr_conf = 14; // GPIO pin for press switch colour confirmation
int clr_select = 13; // GPIO pin for switch colour selection

bool blue_flag = false; // Flag to indicate if blue is selected

void generalPinsSetup() {
    // Set all inputs
    pinMode(uswitch, INPUT_PULLUP);
    pinMode(clr_conf, INPUT_PULLUP);
    pinMode(clr_select, INPUT_PULLUP);

    // Set all outputs  
    pinMode(led_red, OUTPUT);
    pinMode(led_yellow, OUTPUT);
    pinMode(led_blue, OUTPUT);

    digitalWrite(led_red, HIGH); // Turn off red LED

    while (digitalRead(clr_conf) == HIGH) {
        if (digitalRead(clr_select) == LOW) {
            blue_flag = true; // Set blue_flag to true if blue is selected
            digitalWrite(led_yellow, LOW); 
            digitalWrite(led_blue, HIGH); // Turn on blue LED
    
        } else {
            digitalWrite(led_yellow, HIGH); // Turn on yellow LED
            digitalWrite(led_blue, LOW); // Turn off yellow LED
            blue_flag = false; // Set blue_flag to false if yellow is selected
        }
    }

    if (blue_flag) {
        digitalWrite(led_blue, LOW); // Turn off blue LED
        delay(200);
        digitalWrite(led_blue, HIGH); // Turn on yellow LED
        delay(200);
        digitalWrite(led_blue, LOW); // Turn off blue LED
        delay(200);
        digitalWrite(led_blue, HIGH); // Turn on yellow LED
        
    } else {
        digitalWrite(led_yellow, LOW); // Turn off blue LED
        delay(200);
        digitalWrite(led_yellow, HIGH); // Turn on yellow LED
        delay(200);
        digitalWrite(led_yellow, LOW); // Turn off blue LED
        delay(200);
        digitalWrite(led_yellow, HIGH); // Turn on yellow LED
    }


}

void waitUswitchRelease() {
    while(digitalRead(uswitch) == HIGH) { // while the micro-switch is not released from key 
        // do nothing
    }
}