#ifndef NAVIGATION_H
#define NAVIGATION_H

#include <Arduino.h>

struct Waypoint {
    float x;           // X coordinate (meters)
    float y;           // Y coordinate (meters)
    float orientation; // Target orientation at waypoint (degrees)
    bool isRequired;   // If true, this waypoint must be reached (even after detour)
};

// Global position tracking
extern volatile float robotX;
extern volatile float robotY;
//extern float robotOrientation;

// Navigation functions
//void initNavigation();
void initNavigation(float startX = 0.0, float startY = 0.0, float startOrientation = -400.0);
void updatePosition(float movementHeading, float distance);
void navigateToWaypoint(Waypoint waypoint);
void followPath(Waypoint* waypoints, int numWaypoints);
void testPositionTracking();
//bool detectObstacle(float maxDistance);
Waypoint* calculateDetour(Waypoint current, Waypoint target);

#endif // NAVIGATION_H