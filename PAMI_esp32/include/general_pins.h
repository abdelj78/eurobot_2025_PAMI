// this is for all top pcb components except servo motor
#ifndef GENERAL_PINS_H
#define GENERAL_PINS_H

#include "Arduino.h"

extern bool blue_flag; // Flag to indicate if blue is selected

// Function declarations
void generalPinsSetup();
void waitUswitchRelease();
#endif // MPU_DRIVER_INT_H