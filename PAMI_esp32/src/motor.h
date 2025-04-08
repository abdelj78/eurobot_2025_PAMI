#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"

// Function declarations
void motorSetup();
void motorLoop();
void stopMotors() ;
void forward(int speed) ;
void backward() ;
void left() ;
void right() ;
void moveStraight(float distance, int baseSpeed) ;
void turnByAngle(float angle, int speed) ;
float normalizeAngle(float angle);
float PIDControl(float error);

#endif // MPU_DRIVER_INT_H