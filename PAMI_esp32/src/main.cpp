#include <Arduino.h>
#include "motor.h"
#include "servo.h"
#include "mpu.h"
#include "ultrasound.h"
#include "general_pins.h"
#include "navigation.h"


int ROBOT_NB = 1; // 1= Super

// Define waypoints for blue side
// Waypoint blueWaypoints[] = {
//     {1.26076, 0.0, 0.0, true},    // Move forward, then turn left
//     {1.26076, -0.247, -90.0, true},     // Move to final position
//     {0.0, 0.0, 0.0, true} // Optional waypoint for testing
// };

////////SUPERSTAR//////
//BLUE
// define initial position and orientation
float initialXb = 1.932; // Initial X position in meters
float initialYb = 2.9482; // Initial Y position in meters
float initialOrientationb = -90.0; // Initial orientation in degrees
Waypoint blueWaypoints[] = {
    {1.932, 1.800, -90, true},    // Move forward, then turn left
    {1.619025, 1.800, -180.0, true},     // Move to final position
};
//YELLOW
// define initial position and orientation
float initialXy = 1.932; // Initial X position in meters
float initialYy = 0.0518; // Initial Y position in meters
float initialOrientationy = 90.0; // Initial orientation in degrees
Waypoint yellowWaypoints[] = {   // Move forward, then turn left
    {1.932, 1.200, 90, true}, 
    {1.619025 , 1.200, 180, true }    // Move to final position
};



// /////////GROUPIE CLOSE//////
// // BLUE
// // define initial position and orientation
// float initialXb= 1.822;
// float initialYb = 2.9482; // Initial Y position in meters
// float initialOrientationb = -90.0; // Initial orientation in degrees
// Waypoint blueWaypoints[] = {
//     {1.822, 2.900, -90, true},    // Move forward, then turn left
//     {1.450, 2.000, -90, true},     // Move to final position
//     {1.450, 1.900, -180.0, true} // Optional waypoint for testing
// };

// //YELLOW
// float initialXy= 1.822;
// float initialYy = 0.0518; // Initial Y position in meters
// float initialOrientationy = 90.0; // Initial orientation in degrees
// Waypoint yellowWaypoints[] = {
//     {1.822, 0.100, 90, true},    // Move forward, then turn left
//     {1.450, 1.000, 90, true},     // Move to final position
//     {1.450, 1.150, 180.0, true} // Optional waypoint for testing
// };


// /////////GROUPIE MIDDLE//////
// // BLUE
// // define initial position and orientation
// float initialXb= 1.712;
// float initialYb = 2.9482; // Initial Y position in meters
// float initialOrientationb = -90.0; // Initial orientation in degrees
// Waypoint blueWaypoints[] = {
//     {1.712, 2.750, -90, true},    // Move forward, then turn left
//     {1.450, 2.000, -90, true},     // Move to final position
//     {1.450, 1.500, -180.0, true} // Optional waypoint for testing
// };

// //YELLOW
// float initialXy= 1.712;
// float initialYy = 0.0518; // Initial Y position in meters
// float initialOrientationy = 90.0; // Initial orientation in degrees
// Waypoint yellowWaypoints[] = {
//     {1.712, 0.250, 90, true},    // Move forward, then turn left
//     {1.450, 1.000, 90, true},     // Move to final position
//     {1.450, 1.500, 180.0, true} // Optional waypoint for testing
// };

// /////////GROUPIE FURTHEST//////
// // BLUE
// // define initial position and orientation
// float initialXb= 1.602;
// float initialYb = 2.9482; // Initial Y position in meters
// float initialOrientationb = -90.0; // Initial orientation in degrees
// Waypoint blueWaypoints[] = {
//     {1.602, 2.750, -90, true},    // Move forward, then turn left
//     {1.450, 2.000, -90, true},     // Move to final position
//     {1.450, 1.150, -180.0, true} // Optional waypoint for testing
// };

// //YELLOW
// float initialXy= 1.602;
// float initialYy = 0.0518; // Initial Y position in meters
// float initialOrientationy = 90.0; // Initial orientation in degrees
// Waypoint yellowWaypoints[] = {
//     {1.602, 0.250, 90, true},    // Move forward, then turn left
//     {1.450, 1.000, 90, true},     // Move to final position
//     {1.450, 1.900, 180.0, true} // Optional waypoint for testing
// };





// Waypoint blueWaypoints[] = {
//     {0.50, 0.0, -90.0, true},    // Move forward, then turn left
//     {0.50, -0.50, -180.0, true},     // Move to final position
//     {0.0, -0.50, 90.0, true}, // Optional waypoint for testing
//     {0.0, 0.0, 0.0, true}, // Optional waypoint for testing
//     {0.50, 0.0, -90.0, true},    // Move forward, then turn left
//     {0.50, -0.50, -180.0, true},     // Move to final position
//     {0.0, -0.50, 90.0, true}, // Optional waypoint for testing
//     {0.0, 0.0, 0.0, true} // Optional waypoint for testing
// };

// // Define waypoints for red side
// Waypoint redWaypoints[] = {
//     {1.26076, 0.0, 90.0, true},     // Move forward, then turn right
//     {1.26076, 0.247, 90.0, true}    // Move to final position
// };




// VARIABLES FOR TESTING
// Shared variables
SemaphoreHandle_t positionMutex;
// volatile float currentYaw = 0;
// volatile float robotX = 0, robotY = 0;

void sensorTask(void *parameter);
void navigationTask(void *parameter);

void setup() {
    //PUT MAIN SETUP CODE HERE 
    Serial.begin(115200);
    
    mpuSetup();
    ultrasoundSetup();
    motorSetup();
    servoSetup();

        //PUT TESTING SETUP CODE HERE


    // Create the mutex at the beginning of setup
    positionMutex = xSemaphoreCreateMutex();
    if (positionMutex == NULL) {
        Serial.println("Error creating mutex!");
        while(1); // Stop execution if mutex creation fails
    }
  
    xTaskCreatePinnedToCore(
        sensorTask,     // Function
        "SensorTask",   // Name
        10000,          // Stack size
        NULL,           // Parameters
        1,              // Priority
        NULL,           // Task handle
        0               // Core 0
    );

    xTaskCreatePinnedToCore(
        navigationTask, // Function  
        "NavTask",      // Name
        10000,          // Stack size
        NULL,           // Parameters
        1,              // Priority
        NULL,           // Task handle
        1               // Core 1
    );




    // Wait for start signal

    


}

void sensorTask(void *parameter) {
  while(true) {
    // Read sensors
    if (MPUInterrupt) {
      // Process MPU data...
      readIMU2();
      // Update shared variables safely
    //   xSemaphoreTake(positionMutex, portMAX_DELAY);
    //   currentYaw = ypr[0] * 180 / M_PI;
    //   xSemaphoreGive(positionMutex);
        xSemaphoreTake(positionMutex, portMAX_DELAY);
        currentYaw = normalizeAngle((ypr[0] * 180 / M_PI)+yawOffset);
        xSemaphoreGive(positionMutex);
    }
    // Serial.print("core 0: ");
    // Serial.print("Yaw: ");
    // Serial.println(currentYaw);
    vTaskDelay(1); // Small delay
  }
}

void navigationTask(void *parameter) {
//   while(true) {
//     // Get current position safely
//     float yaw, x, y;
//     xSemaphoreTake(positionMutex, portMAX_DELAY);
//     yaw = currentYaw;
//     x = robotX;
//     y = robotY;
//     xSemaphoreGive(positionMutex);
//     // Use the values for navigation
//     // ...
//     Serial.print("core 1: ");
//     Serial.print("Yaw: ");
//     Serial.println(yaw);
    
//     vTaskDelay(10);
//   }

    generalPinsSetup();
    // Initialize navigation

    waitUswitchRelease();
    Serial.println("Micro-switch released, delay start.");
    delay(1000);
    Serial.println("end of delay 85 seconds");
    
    
    // Choose path based on selected side
    if (blue_flag) {
        Serial.println("Following blue path");
        //testPositionTracking();
        //initial position superstar
        //initial position groupie close
        //initial position groupie middle
        //initial position groupie furthest
        initNavigation(initialXb, initialYb, initialOrientationb);
        followPath(blueWaypoints, sizeof(blueWaypoints)/sizeof(Waypoint));
    } else {
        Serial.println("Following red path");
        initNavigation(initialXy, initialYy, initialOrientationy);
        followPath(yellowWaypoints, sizeof(yellowWaypoints)/sizeof(Waypoint));
    }
        servoLoop(); // Control the servo

}


void loop() {
    //PUT MAIN LOOP CODE HERE
    // Main work is done in setup for the competition
    // You can add status monitoring or manual control here

    //PUT TESTING LOOP CODE HERE
}




/*
// POST WAYBPOINT CODE
// working code for simple movement
#include <Arduino.h>
#include "test_n_dev\mpu_test.h"
#include "motor.h"
#include "servo.h"
#include "mpu.h"
#include "ultrasound.h"
#include "general_pins.h"

//#define TEST_PIN 36 // GPIO pin for testing

void setup() {
    //servoSetup();
    //PAMI CODE
    
    Serial.begin(115200);

    mpuSetup();
    ultrasoundSetup();
    motorSetup();
    servoSetup(); // Initialize servo control

    generalPinsSetup(); // Initialize general pins
    waitUswitchRelease(); // Wait for the micro-switch to be released
    delay(1000); // Delay for 1 second to ensure the switch is stable
    //blue_flag = false; // Initialize blue_flag to false

    if (blue_flag) { // if blue selected
    // latest pami version size
        moveStraight(1.26076, 120);
        turnByAngle(-90.0, 100);
        moveStraight(0.247,120);
        turnByAngle(-101.08, 100);
        moveStraight(1.2847,120);
        turnByAngle(-168.92, 100);
    } else {
        moveStraight(1.26076, 120);
        turnByAngle(90.0, 100);
        moveStraight(0.247,120);  
    }



    // Serial.println("Setup started.");

    //pinMode(TEST_PIN, INPUT_PULLUP); // Set GPIO 19 as output for LED

    //moveForwardFor(0.5, 100); // Move forward for 0.5 seconds at speed 180
    
    // //SUPERSTAR BLUE
    // moveStraight(1.27876, 80);
    // turnByAngle(-90.0, 100);
    // moveStraight(0.15,80);


    // //GROUPIE BLUE CLOSE
    // moveStraight(0.35, 80);
    // turnByAngle(-90, 100);
    // moveStraight(0.3, 80);
    // turnByAngle(90, 100);
    // moveStraight(0.55, 80);  

    // //GROUPIE BLUE MIDDLE
    // moveStraight(0.25, 80);
    // turnByAngle(-90, 100);
    // moveStraight(0.3, 80);
    // turnByAngle(90, 100);
    // moveStraight(1.25, 80);  

    // // //GROUPIE BLUE FURTHEST
    // moveStraight(0.15, 80);
    // turnByAngle(-90, 100);
    // moveStraight(0.3, 80);
    // turnByAngle(90, 100);
    // moveStraight(1.7, 80);  
    // turnByAngle(90, 100);
    // moveStraight(0.15, 80);  

    servoLoop(); // Control the servo
    

}

void loop() {
    //mpuLoop();
    //motorLoop();
    
    //servoLoop(); // Control the servo
    //int state = digitalRead(TEST_PIN);
    //Serial.println(state);  // Print HIGH (1) or LOW (0)
    //delay(500);
}
*/