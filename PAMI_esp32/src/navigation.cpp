#include <Arduino.h>
#include "motor.h"
#include "mpu.h"
#include "ultrasound.h"
#include <math.h>
#include "navigation.h"

extern SemaphoreHandle_t positionMutex;  // Declare that this is defined elsewhere

// Global position tracking
volatile float robotX = 0.0;
volatile float robotY = 0.0;
float robotOrientation = 0.0;

float yaw;  

// Constants for navigation
const float POSITION_TOLERANCE = 0.005;  // 5cm position tolerance
const float ANGLE_TOLERANCE = 2.0;      // 2 degrees angle tolerance
const float OBSTACLE_DISTANCE = 0.25;   // 25cm obstacle detection distance

// struct Waypoint {
//     float x;           // X coordinate (meters)
//     float y;           // Y coordinate (meters)
//     float orientation; // Target orientation at waypoint (degrees)
//     bool isRequired;   // If true, this waypoint must be reached (even after detour)
// };

// void initNavigation() {
//     // Reset position tracking
//     robotX = 0.0;
//     robotY = 0.0;
//     if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
//         robotOrientation = currentYaw;
//         xSemaphoreGive(positionMutex);
//     }
//     //robotOrientation = currentYaw;
// }

void initNavigation(float startX, float startY, float startOrientation) {
    // Set initial position from parameters
    robotX = startX;
    robotY = startY;
    robotOrientation = startOrientation;

    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
        // We're setting an offset rather than directly changing currentYaw
        yawOffset = startOrientation - currentYaw;
        xSemaphoreGive(positionMutex);
    }
    // If specific orientation is provided, use it
    // if (startOrientation > -360.0) {
    //     robotOrientation = startOrientation;
    // } else {
    //     // Otherwise, use the current IMU reading
    //     if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
    //         robotOrientation = currentYaw;
    //         xSemaphoreGive(positionMutex);
    //     }
    // }
    
    Serial.print("Navigation initialized at (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.print(robotY);
    Serial.print("), Orientation: ");
    Serial.print(robotOrientation);
    Serial.println("°");
}

// Add this function to your navigation.cpp file
float minValue(float a, float b) {
    return (a < b) ? a : b;
}


void updatePosition(float movementHeading, float distance) {
    // movementHeading should be in degrees (-180 to +180)
    
    // Convert heading to radians for trig functions
    float headingRad = movementHeading * PI / 180.0;
    
    // Update position using your coordinate system (X=North, Y=East)
    robotX += distance * cos(headingRad);
    robotY += distance * sin(headingRad);
    
    Serial.print("Position updated: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.print(robotY);
    Serial.print("), Distance: ");
    Serial.print(distance);
    Serial.print("m, Heading: ");
    Serial.println(movementHeading);
}

// Calculate distance between two points
float distanceBetween(float x1, float y1, float x2, float y2) {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

// Calculate angle to target point
float angleToTarget(float x1, float y1, float x2, float y2) {
    return atan2(y2 - y1, x2 - x1) * 180.0 / PI;
}

// bool detectObstacle(float maxDistance) {
//     return getDistance() < maxDistance;
// }

// void navigateToWaypoint(Waypoint waypoint) {
//     bool waypointReached = false;
    
//     while (!waypointReached) {
//         // Update current position
//         readIMU();
//         updatePosition();
        
//         // Calculate distance and angle to waypoint
//         float distToWaypoint = distanceBetween(robotX, robotY, waypoint.x, waypoint.y);
//         float angleToWaypoint = angleToTarget(robotX, robotY, waypoint.x, waypoint.y);
        
//         // Check if we've reached the waypoint
//         if (distToWaypoint < POSITION_TOLERANCE) {
//             stopMotors();
            
//             // If the waypoint has an orientation requirement, turn to that orientation
//             if (waypoint.orientation >= -360) {
//                 turnByAngle(normalizeAngle(waypoint.orientation - robotOrientation), 100);
//             }
            
//             waypointReached = true;
//             continue;
//         }
        
//         // Check for obstacles
//         if (obstacle) {
//             stopMotors();
//             Serial.println("Obstacle detected! Planning detour...");
            
//             // Simple obstacle avoidance - turn 45 degrees away and move a bit
//             float avoidanceAngle = 45.0;
            
//             // Determine which way to turn based on ultrasonic readings
//             // This is a simple approach - you might want something more sophisticated
//             turnByAngle(avoidanceAngle, 100);
//             Serial.println("Turning to avoid obstacle...");
//             moveStraight(0.3, 100);  // Move 30cm to get around obstacle
//             Serial.println("Moving forward after avoidance...");
            
//             continue;  // Re-evaluate after avoidance maneuver
//         }
        
//         // Turn towards waypoint
//         float turnAngle = normalizeAngle(angleToWaypoint - robotOrientation);
//         if (abs(turnAngle) > ANGLE_TOLERANCE) {
//             turnByAngle(turnAngle, 100);
//         }
        
//         // Move towards waypoint
//         //float moveDistance = minValue(distToWaypoint, 0.05);  // Move in 20cm increments max
//         //moveStraight(moveDistance, 120);
//         moveStraight(distToWaypoint, 120);  // Move towards the waypoint
//     }
    
//     Serial.print("Reached waypoint (");
//     Serial.print(waypoint.x);
//     Serial.print(", ");
//     Serial.print(waypoint.y);
//     Serial.println(")");
// }

void navigateToWaypoint(Waypoint waypoint) {
    Serial.print("Navigating to waypoint (");
    Serial.print(waypoint.x);
    Serial.print(", ");
    Serial.print(waypoint.y);
    Serial.print("), Target orientation: ");
    Serial.println(waypoint.orientation);

    // Update and synchronize orientation, could use 
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
        robotOrientation = currentYaw;
        xSemaphoreGive(positionMutex);
    }
    
    // Debug current position and orientation
    Serial.print("Current position: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.print(robotY);
    Serial.print("), Orientation: ");
    Serial.println(robotOrientation);

    // Calculate distance and angle to waypoint
    float distToWaypoint = distanceBetween(robotX, robotY, waypoint.x, waypoint.y);
    float angleToWaypoint = angleToTarget(robotX, robotY, waypoint.x, waypoint.y);
    
    // First, turn toward the waypoint
    float turnAngle = normalizeAngle(angleToWaypoint - robotOrientation);
    Serial.print("Angle to waypoint: ");
    Serial.print(angleToWaypoint);
    Serial.print(", Current orientation: ");
    Serial.print(robotOrientation);
    Serial.print(", Turn angle: ");
    Serial.println(turnAngle);
    
    // Execute the turn
    turnByAngle3(turnAngle, 40);
        // After turning, get the current heading - this is what we'll use for position update
    float movementHeading;
    if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
        movementHeading = currentYaw;  // This is the direction we'll move in
        robotOrientation = currentYaw;  // Update stored orientation
        xSemaphoreGive(positionMutex);
    }
    
    delay(200);  // Short pause after turning
    
  // Reset encoders before movement
    encoderLPosition = 0;
    encoderRPosition = 0;

    // Then move to the waypoint
    Serial.print("Moving distance: ");
    Serial.println(distToWaypoint);
    //JUST FOR SUPERSTAR KEEP THIS SPEED LOW
    moveStraight3(distToWaypoint, 255); // pre-homologation was 255 but seem to much as stop too close to objects and misses some obstacles
      // Calculate distance traveled
    float distanceTraveled = (encoderLPosition + encoderRPosition) / (2.0 * 6777.0);
        // Update position using the heading we were moving in and distance traveled
    updatePosition(movementHeading, distanceTraveled);
    
    delay(200);  // Short pause after moving
    volatile long encoderLPositionLocal = encoderLPosition;
    volatile long encoderRPositionLocal = encoderRPosition;

    float finalTurnAngle = 0.0;
    
    // After reaching the waypoint, adjust to the final required orientation if specified
    if (waypoint.orientation >= -360) {
        // Update current orientation after movement
        if (xSemaphoreTake(positionMutex, portMAX_DELAY) == pdTRUE) {
            robotOrientation = currentYaw;  // Update orientation
            xSemaphoreGive(positionMutex);
        }
        
        finalTurnAngle = normalizeAngle(waypoint.orientation - robotOrientation);
        
        Serial.print("Final orientation adjustment: Current=");
        Serial.print(robotOrientation);
        Serial.print(", Target=");
        Serial.print(waypoint.orientation);
        Serial.print(", Turn=");
        Serial.println(finalTurnAngle);
        
        // Execute the final orientation adjustment
        turnByAngle3(finalTurnAngle, 80);
    }
    
    // Small pause after reaching waypoint
    delay(200);
    
    Serial.print("Reached waypoint (");
    Serial.print(waypoint.x);
    Serial.print(", ");
    Serial.print(waypoint.y);
    Serial.println(")");
    //updatePosition(finalTurnAngle, encoderLPositionLocal, encoderRPositionLocal);  // Update position after reaching the waypoint
}

void followPath(Waypoint* waypoints, int numWaypoints) {
    for (int i = 0; i < numWaypoints; i++) {
        Serial.print("Navigating to waypoint ");
        Serial.print(i+1);
        Serial.print(" of ");
        Serial.println(numWaypoints);
        
        navigateToWaypoint(waypoints[i]);
        
        // Optional delay between waypoints
        delay(10);
    }
    
    Serial.println("Path complete!");
}


void testPositionTracking() {
    // Reset position
    robotX = 0.0;
    robotY = 0.0;
    
    Serial.println("Testing position tracking with different headings:");
    
    // Test North (0°)
    updatePosition(0.0, 1.0);
    Serial.print("After 1m North: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.println(robotY);
    
    // Test East (90°)
    updatePosition(90.0, 1.0);
    Serial.print("After 1m East: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.println(robotY);
    
    // Test South (180° or -180°)
    updatePosition(180.0, 1.0);
    Serial.print("After 1m South: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.println(robotY);
    
    // Test West (-90°)
    updatePosition(-90.0, 1.0);
    Serial.print("After 1m West: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.println(robotY);
    
    // Should end up back at origin
    Serial.print("Final position: (");
    Serial.print(robotX);
    Serial.print(", ");
    Serial.println(robotY);
}