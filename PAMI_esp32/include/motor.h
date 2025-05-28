#ifndef MOTOR_H
#define MOTOR_H

#include "Arduino.h"
extern volatile long encoderLPosition;
extern volatile long encoderRPosition;

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
void moveStraight2(float distance, int baseSpeed) ;
void turnByAngle2(float angle, int speed) ;
void moveStraight3(float distance, int baseSpeed) ;
void turnByAngle3(float angle, int speed) ;

float normalizeAngle(float angle);
float PIDControl(float error);
float PIDControlTurn(float error);
float PIDControlStraight(float error);
void resetStraightPID();
void resetTurnPID();


#endif // MPU_DRIVER_INT_H