#ifndef MPU_H
#define MPU_H

extern volatile float currentYaw;
extern volatile bool MPUInterrupt;  
extern float ypr[3];  // [yaw, pitch, roll]
extern volatile float yawOffset;  // Declare the offset variable
// Function prototypes
void mpuSetup();
void mpuLoop();
void readIMU();
void readIMU2();
void discardInitialReadings(int numReadings);

#endif // MPU_TEST_H