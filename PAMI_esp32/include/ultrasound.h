#ifndef ULTRASOUND_H
#define ULTRASOUND_H

extern bool obstacle;

// Function prototypes
void ultrasoundSetup();
void ultrasoundLoop();
void distanceCheck();
void distanceCheck2();

void obstacleDetect ();
void obstacleClear() ;

#endif // MPU_TEST_H