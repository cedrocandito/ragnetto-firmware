#include <Arduino.h>
#include "ragnetto_framework.h"
#include "ragnetto_hardware.h"

/*
 * Array of legs; each legs contains 3 servo ids (from upper to lower leg).
 * Servo id 0-15 are for PWM controller 0, 16-31 are for PWM controller 1.
 * Legs are numbered zero-based from front left, counterclockwise
 * (so front right is #5).
 */
const uint8_t legs[NUM_LEGS][SERVOS_PER_LEG] =
{
    {20, 21, 22},
    {23, 24, 25},
    {26, 27, 28},
    {4, 5, 6},
    {7, 8, 9},
    {10, 11, 12}
};


class Point3d
{
    public:
        float x;
        float y;
        float z;

    Point3d()
    {
        x = 0.0;
        y = 0.0;
        z = 0.0;
    }

    Point3d(const float newx, const float newy, const float newz)
    {
        x = newx;
        y = newy;
        z = newz;
    }

    void subtract(const Point3d &p)
    {
        x = x - p.x;
        y = y - p.y;
        z = z - p.z;
    }

    void add(const Point3d &p)
    {
        x = x + p.x;
        y = y + p.y;
        z = z + p.z;
    }

    float dotProduct(const Point3d &p)
    {
        return sqrt(x*p.x + y*p.y + z*p.z);
    }

    float norm()
    {
        return sqrt(x*x + y*y + z*z);
    }
};

class Point2d
{
    public:
        float x;
        float y;

    Point2d(const float newx, const float newy)
    {
        x = newx;
        y = newy;
    }

    Point2d(const Point3d *p3d)
    {
        x = p3d->x;
        y = p3d->y;
    }

    Point2d()
    {
        x = 0.0;
        y = 0.0;
    }

    void subtract(const Point2d p)
    {
        x = x - p.x;
        y = y - p.y;
    }

    void add(const Point2d p)
    {
        x = x + p.x;
        y = y + p.y;
    }

    float dotProduct(const Point2d p)
    {
        return sqrt(x*p.x + y*p.y);
    }

    float norm()
    {
        return sqrt(x*x + y*y);
    }
};

class Leg
{
    Point3d attachmentPosition;
    float attachmentAngle;

    public:

    // setup leg position and angle
    void setup(const uint8_t leg_id)
    {
        attachmentAngle = (leg_id - 2) * PI / 3.0;
        attachmentPosition.x = cos(attachmentAngle) * LEG_ATTACHMENT_RADIUS;
        attachmentPosition.y = sin(attachmentAngle) * LEG_ATTACHMENT_RADIUS;
        attachmentPosition.z = 0.0;
    };

    // move servos to indicated position ????????? tempo!!!
    void moveTo(const Point3d &point, const int duration)
    {
        //TODO: fare??????????
    };
};

class Ragnetto
{
    private:
    Leg legs[NUM_LEGS];

    public:
    // constructor
    Ragnetto()
    {
        for (uint8_t i=0; i<NUM_LEGS; i++)
        {
            legs[i].setup(i);
        }
    }
};

class MovementSegment
{
    Point3d startPositions[NUM_LEGS];
    Point3d endPositions[NUM_LEGS];
    long startTime;
    long endTime;
};

class Bevaviour
{
    virtual void work();
};

float horizontalAngleBetweenTwoPoints(const Point3d *p0, const Point3d *p1, const Point3d *p2)
{
    Point2d p0b(p0);
    Point2d p1b(p1);
    Point2d p2b(p2);
    p1b.subtract(p0b);
    p2b.subtract(p0b);
    return acos(p1b.dotProduct(p2b)/p1b.norm()/p2b.norm());
}

bool pointIn3dSpaceToJointAngles(Point3d p, float l1, float l2, float resultAngles[])
{
    float cosb = (p.x*p.x+p.y*p.y-l1*l1-l2*l2)/(2*l1*l2);
    if (abs(cosb) > 1.0)
        return false;

    // 2 solutions for square root
    float sinb1 = sqrt(1-cosb*cosb);
    float sinb2 = -sinb1;

    // note: the check on sinb1 covers sinb2 too
    if (abs(sinb1) > 1.0)
        return false;

    float b1 = atan2(sinb1,cosb);
    float b2 = atan2(sinb2,cosb);

    float a1 = atan2(p.y,p.x) - atan2(l2 * sinb1, l1 + l2 * cosb);
    float a2 = atan2(p.y,p.x) - atan2(l2 * sinb2, l1 + l2 * cosb);

    
};