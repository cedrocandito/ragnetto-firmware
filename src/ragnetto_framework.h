#ifndef RAGNETTO_FRAMEWORK_H
#define RAGNETTO_FRAMEWORK_H

#include <Arduino.h>
#include "ragnetto_hardware.h"
#include "logging.h"
#include "ragnetto.h"

class Point3d
{
    public:
    float x;
    float y;
    float z;

    Point3d();
    Point3d(const float newx, const float newy, const float newz);
    Point3d(const Point3d &p);
    void subtract(const Point3d &p);
    void add(const Point3d &p);
    float dotProduct(const Point3d &p);
    float norm();
};

class Point2d
{
    public:
    float x;
    float y;

    Point2d(const float newx, const float newy);
    Point2d(const Point3d &p3d);
    Point2d();
    Point2d(const Point2d &p);
    void subtract(const Point2d p);
    void add(const Point2d p);
    float dotProduct(const Point2d p);
    float norm();
};

class Leg
{
    public:
    uint8_t leg_id;
    Point3d attachmentPosition;
    float attachmentAngle; // range is from -PI to +PI (the same atan2() uses)
    bool invertServo;      // true for legs that have joints 2 and 3 mounted in the opposite direction

    // setup leg position and angle
    void setup(const uint8_t id, bool invert);
    
    // move servos to indicated position (relative to leg attachment point)
    void moveTo(const Point3d &point);
};

class Ragnetto
{
    public:
    Leg legs[NUM_LEGS];

    // constructor
    Ragnetto();
};

/* Calculate the angles to be applied to the joints in order to move the leg to
a point in space. Coordinates are relative to attachment point of the leg, angles are absolute.
Returns true if there is a solution, false if not. */
bool pointIn3dSpaceToJointAngles(const Point3d &p, const Leg &leg, float result_angles[]);

//???????????? joint 2: gambe 0-2 clockwise giù, 3-5 clockwise su

// ??????????? invertire offset per gambe 3-5!!!!

#endif