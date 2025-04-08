#ifndef MPU_H
#define MPU_H

extern float currentYaw;

// Function prototypes
void mpuSetup();
void mpuLoop();
void readIMU();
void discardInitialReadings(int numReadings);

#endif // MPU_TEST_H