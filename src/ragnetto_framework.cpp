#include <Arduino.h>
#include "ragnetto_framework.h"
#include "ragnetto_hardware.h"
#include "ragnetto_config.h"
#include "logging.h"


/*
 * Array of legs; each legs contains 3 servo ids (from upper to lower leg).
 * Servo id 0-15 are for PWM controller 0, 16-31 are for PWM controller 1.
 * Legs are numbered zero-based from front left, counterclockwise
 * (so front right is #5).
 */
static const uint8_t legs[NUM_LEGS][SERVOS_PER_LEG] =
    {
        {20, 21, 22},
        {23, 24, 25},
        {26, 27, 28},
        {4, 5, 6},
        {7, 8, 9},
        {10, 11, 12}};

/* Normalize an angle to the range 0 - 2*PI; optimized for angles just out of the range */
float normalize_0_to_2pi(float a)
{
    while (a < 0.0)
    {
        a += TWO_PI;
    }

    while (a > TWO_PI)
    {
        a -= TWO_PI;
    }
    return a;
}

/* Normalize an angle to the range -PI - +PI; optimized for angles just out of the range */
float normalize_minus_pi_plus_pi(float a)
{
    while (a < -PI)
    {
        a += TWO_PI;
    }

    while (a > PI)
    {
        a -= TWO_PI;
    }
    return a;
}



Point3d::Point3d()
{
    x = 0.0;
    y = 0.0;
    z = 0.0;
}

Point3d::Point3d(const float newx, const float newy, const float newz)
{
    x = newx;
    y = newy;
    z = newz;
}

Point3d::Point3d(const Point3d &p)
{
    x = p.x;
    y = p.y;
    z = p.z;
}

Point2d::Point2d(const float newx, const float newy)
{
    x = newx;
    y = newy;
}

Point2d::Point2d(const Point3d &p3d)
{
    x = p3d.x;
    y = p3d.y;
}

Point2d::Point2d()
{
    x = 0.0;
    y = 0.0;
}

Point2d::Point2d(const Point2d &p)
{
    x = p.x;
    y = p.y;
}


// setup leg position and angle
void Leg::setup(const uint8_t id, bool invert)
{
    leg_id = id;
    attachmentAngle = normalize_minus_pi_plus_pi(((leg_id + 2) % 6) * PI / 3.0);
    attachmentPosition.x = cos(attachmentAngle) * LEG_ATTACHMENT_RADIUS;
    attachmentPosition.y = sin(attachmentAngle) * LEG_ATTACHMENT_RADIUS;
    attachmentPosition.z = 0.0;
    invertServo = invert;
}

// move servos to indicated position (relative to leg attachment point)
void Leg::moveTo(const Point3d &point)
{
    LOGS("[");
    LOGN(point.x);
    LOGS(", ");
    LOGN(point.y);
    LOGS(", ");
    LOGN(point.z);
    LOGS("]: ");
 
    float angles[3];
    bool ok = pointIn3dSpaceToJointAngles(point, *this, angles);
    if (ok)
    {
        LOGS("=> (");
        LOGN(angles[0]);
        LOGS(", ");
        LOGN(angles[1]);
        LOGS(", ");
        LOGN(angles[2]);
        LOGS(")");
    
        set_servo_position(legs[leg_id][0],angles[0]);
        set_servo_position(legs[leg_id][1],angles[1]+(invertServo ? JOINT2_OFFSET:-JOINT2_OFFSET));
        set_servo_position(legs[leg_id][2],angles[2]+(invertServo ? JOINT3_OFFSET:-JOINT3_OFFSET));
    }
    else
    {
        LOGS("Fuori portata");
    }

    LOGLN();
}

Ragnetto::Ragnetto()
{
    for (uint8_t i = 0; i < NUM_LEGS; i++)
    {
        legs[i].setup(i, i>=3);
    }
}

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

/* Calculate the angles to be applied to the joints in order to move the leg to
a point in space. Coordinates are relative to attachment point of the leg, angles are absolute.
Returns true if there is a solution, false if not. */
bool pointIn3dSpaceToJointAngles(const Point3d &p, const Leg &leg, float result_angles[])
{
    /* working on top-view projection, calculate the horizontal angle between
    the target point and the leg attachment point */
    float h_angle = atan2f(p.y, p.x);

    /* subtracting the previous angle from the attachment angle of the leg we can
    calculate the servo rotation needed for the first joint */
    float joint1_angle = normalize_minus_pi_plus_pi(h_angle - leg.attachmentAngle);

    /* if the foot would point toward the inside of the body, use an angle which
    is 180 degrees from the previously calculated one (the servo cannot move
    toward the inside) */
    if (abs(joint1_angle) > HALF_PI)
    {
        h_angle = normalize_minus_pi_plus_pi(h_angle + PI);
        // recalculate the joint angle
        joint1_angle = normalize_minus_pi_plus_pi(h_angle - leg.attachmentAngle);
    }

    /* check if the servo rotation is within limits */
    if ((joint1_angle < JOINT1_MIN_ANGLE) || (joint1_angle > JOINT1_MAX_ANGLE))
        return false;

    result_angles[0] = joint1_angle;

    float cosa = cos(h_angle);
    float sina = sin(h_angle);

    /* calculate the attachment point of joint 2 */
    Point2d joint2attachment(cosa * LEG_SEGMENT_1_LENGTH, sina * LEG_SEGMENT_1_LENGTH);

    /* now let's image a vertical plane along the line between the destination
    point and joint 2.; p.x in this plane corresponds to the distance between p and
    joint 2 attachment position (0, 0) and y corresponds to real-world p.z */

    float l = sqrt(p.x * p.x + p.y * p.y);
    // correction for target points beyond the leg attachment point (toward the body) 
    if ((signbit(sina) != signbit(p.y)) && (signbit(cosa)!=signbit(p.x)))
        l=-l;
    Point2d pp(l - LEG_SEGMENT_1_LENGTH, p.z);

    /* inverse kinematics for joint 2 and 3 */
    const float cosb = (pp.x * pp.x + pp.y * pp.y - LEG_SEGMENT_2_LENGTH * LEG_SEGMENT_2_LENGTH - LEG_SEGMENT_3_LENGTH * LEG_SEGMENT_3_LENGTH) / (2 * LEG_SEGMENT_2_LENGTH * LEG_SEGMENT_3_LENGTH);

    /* check if the point is out of reach (no solution) */
    if (abs(cosb) > 1.0)
        return false;
    
    // 2 solutions for square root
    const float sinb1 = sqrt(1 - cosb * cosb);
    const float sinb2 = -sinb1;

    // note: the check on sinb1 covers sinb2 too
    if (abs(sinb1) > 1.0)
        return false;

    float v_ang = atan2(pp.y, pp.x);

    // solution 2 is more likely to happen, so calculate that one first
    float c = atan2(sinb2, cosb);
    float b = v_ang - atan2(LEG_SEGMENT_3_LENGTH * sinb2, LEG_SEGMENT_2_LENGTH + LEG_SEGMENT_3_LENGTH * cosb);
    if ((c >= JOINT3_MIN_ANGLE) && (c <= JOINT3_MAX_ANGLE) && (b >= JOINT2_MIN_ANGLE) && (b <= JOINT2_MAX_ANGLE))
    {
        // solution 2 is ok
        result_angles[1] = b;
        result_angles[2] = c;
    }
    else
    {
        float c = atan2(sinb1, cosb);
        float b = v_ang - atan2(LEG_SEGMENT_3_LENGTH * sinb1, LEG_SEGMENT_2_LENGTH + LEG_SEGMENT_3_LENGTH * cosb);
        if ((c >= JOINT3_MIN_ANGLE) && (c <= JOINT3_MAX_ANGLE) && (b >= JOINT2_MIN_ANGLE) && (b <= JOINT2_MAX_ANGLE))
        {
            // solution 1 is ok
            result_angles[1] = b;
            result_angles[2] = c;
        }
        else
        {
            // no solution is good
            return false;
        }
    }

    if (leg.invertServo)
    {
        result_angles[1] = -result_angles[1];
        result_angles[2] = -result_angles[2];
    }

    return true;
}
