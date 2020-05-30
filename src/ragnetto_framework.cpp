#include <Arduino.h>
#include <inttypes.h>
#include "ragnetto_framework.h"
#include "ragnetto_hardware.h"
#include "ragnetto_config.h"
#include "logging.h"
#include "serial.h"

/*
 * Array of legs; each legs contains 3 servo ids (from upper to lower leg).
 * Servo id 0-15 are for PWM controller 0, 16-31 are for PWM controller 1.
 * Legs are numbered zero-based from front left, counterclockwise
 * (so front right is #5).
 */
static const uint8_t servos_by_leg[NUM_LEGS][SERVOS_PER_LEG] =
    {
        {16, 17, 18},
        {19, 20, 21},
        {22, 23, 24},
        {0, 1, 2},
        {3, 4, 5},
        {6, 7, 8}};

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

// setup leg position and angle for the leg
void Leg::setup(const uint8_t id, bool invert)
{
    leg_id = id;

    // calculate attachment position and angle
    attachmentAngle = normalize_minus_pi_plus_pi(((leg_id + 2) % 6) * PI / 3.0);
    attachmentAngleCos = cos(attachmentAngle);
    attachmentAngleSin = sin(attachmentAngle);
    attachmentPosition.x = attachmentAngleCos * LEG_ATTACHMENT_RADIUS;
    attachmentPosition.y = attachmentAngleSin * LEG_ATTACHMENT_RADIUS;
    attachmentPosition.z = 0.0;
   
    // calculate base foot position
    baseFootPosition.x = attachmentAngleCos * BASE_FOOT_R;
    baseFootPosition.y = attachmentAngleSin * BASE_FOOT_R;
    baseFootPosition.z = BASE_FOOT_Z + configuration.height_offset;

    // calculate rotation x and y offset for 1-degree rotation
    float foot_r = sqrt(baseFootPosition.x * baseFootPosition.x + baseFootPosition.y * baseFootPosition.y) + LEG_ATTACHMENT_RADIUS;
    rotation_x_per_degree = foot_r * cos(attachmentAngle + HALF_PI);
    rotation_y_per_degree = foot_r * sin(attachmentAngle + HALF_PI);

    // initialize other fields
    invertServo = invert;
}

// move servos to indicated position (relative to leg attachment point)
void Leg::moveTo(const Point3d &point)
{
    /*
    LOGS("Leg ");
    LOGN(leg_id);
    LOGS(" [");
    LOGN(point.x);
    LOGS(", ");
    LOGN(point.y);
    LOGS(", ");
    LOGN(point.z);
    LOGS("mm]: ");
    */

    float angles[3];
    bool ok = pointIn3dSpaceToJointAngles(point, *this, angles);
    if (ok)
    {
        /*
        LOGS("=> (");
        LOGN(angles[0]);
        LOGS(", ");
        LOGN(angles[1]);
        LOGS(", ");
        LOGN(angles[2]);
        LOGS(" rad)");
        LOGLN();
        */

        currentPosition = Point3d(point);
        set_servo_position(servos_by_leg[leg_id][0], angles[0], configuration.servo_trim[leg_id][0]);
        set_servo_position(servos_by_leg[leg_id][1], angles[1] + (invertServo ? JOINT2_OFFSET : -JOINT2_OFFSET), configuration.servo_trim[leg_id][1]);
        set_servo_position(servos_by_leg[leg_id][2], angles[2] + (invertServo ? JOINT3_OFFSET : -JOINT3_OFFSET), configuration.servo_trim[leg_id][2]);
    }
    else
    {
        LOGS("Leg ");
        LOGN(leg_id);
        LOGS(" [");
        LOGN(point.x);
        LOGS(", ");
        LOGN(point.y);
        LOGS(", ");
        LOGN(point.z);
        LOGS("mm]: ");
        LOGSLN("out of reach");
    }
    
}

bool Joystick::idle()
{
    return abs(x)<JOYSTICK_NULL_ZONE && abs(y)<JOYSTICK_NULL_ZONE && abs(r)<JOYSTICK_NULL_ZONE;
}

Ragnetto::Ragnetto()
{
    for (uint8_t legnum = 0; legnum < NUM_LEGS; legnum++)
    {
        // initialize leg
        legs[legnum].setup(legnum, legnum >= 3);
    }

    // setup initial mode
    mode = MODE_STANCE;
}

void Ragnetto::process_input()
{
    char *command = serial_receive_command();
    if (command != nullptr)
    {
        // ignore empty commands
        if (strlen(command) < 1)
            return;

        serial_print(*command);
        switch (*command)
        {
        case COMMAND_JOYSTICK:
            if (sscanf(command, "J%" SCNi8 ";%" SCNi8 ";%" SCNi8, &joystick.y, &joystick.x, &joystick.r) == 3)
                serial_println(OUTPUT_OK);
            else
                serial_println(OUTPUT_ERROR);
            break;

        case COMMAND_SET_HEIGHT:
            if (sscanf(command, "H%" SCNi8, &configuration.height_offset) == 1)
                serial_println(OUTPUT_OK);
            else
                serial_println(OUTPUT_ERROR);
            break;
        
        case COMMAND_SET_MAX_PHASE_DURATION:
            if (sscanf(command, "P%" SCNu16, &configuration.phase_duration) == 1)
                serial_println(OUTPUT_OK);
            else
                serial_println(OUTPUT_ERROR);
            break;
        
        case COMMAND_SET_LIFT_HEIGHT:
            if (sscanf(command, "L%" SCNu8, &configuration.leg_lift_height) == 1)
                serial_println(OUTPUT_OK);
            else
                serial_println(OUTPUT_ERROR);
            break;

        case COMMAND_SET_TRIM:
            uint8_t leg_num;
            uint8_t joint_num;
            int8_t trim;
            if (sscanf(command, "T%" SCNu8 ";%" SCNi8 ";%" SCNi8, &leg_num, &joint_num, &trim) == 3
                && leg_num>=0 && leg_num<NUM_LEGS && joint_num>=0 && joint_num<3)
            {
                configuration.servo_trim[leg_num][joint_num] = trim;
                serial_println(OUTPUT_OK);
            }
            else
            {
                serial_println(OUTPUT_ERROR);
            }
            break;

        case COMMAND_READ_CONFIGURATION:
            configuration.read();
            break;

        case COMMAND_WRITE_CONFIGURATION:
            configuration.write();
            break;

        case COMMAND_SHOW_CONFIGURATION:
            for (uint8_t leg = 0; leg < NUM_LEGS; leg++)
            {
                for (uint8_t joint = 0; joint < SERVOS_PER_LEG; joint++)
                {
                    serial_print(configuration.servo_trim[leg][joint]);
                    serial_print(';');
                }
            }
            serial_print(configuration.height_offset);
            serial_print(';');
            serial_print(configuration.leg_lift_height);
            serial_print(';');
            serial_print(configuration.leg_drop_deceleration);
            serial_print(';');
            serial_print(configuration.phase_duration);
            serial_println();
            break;

        case COMMAND_SET_MODE:
            if (sscanf(command, "M%" SCNi8, &mode) == 1)
                serial_println(OUTPUT_OK);
            else
                serial_println(OUTPUT_ERROR);
            break;

        default:
            serial_println(F("Unknown command"));
            break;
        }
    }
}

void Ragnetto::run()
{
    process_input();
    switch(mode)
    {
        case MODE_CALIBRATION:
            for (uint8_t legnum=0; legnum<NUM_LEGS; legnum++)
            {
                for(uint8_t joint=0; joint<SERVOS_PER_LEG; joint++)
                {
                    set_servo_position(servos_by_leg[legnum][joint], 0, configuration.servo_trim[legnum][joint]);
                }
            }
            break;
        
        case MODE_JOYSTICK:
            runJoystickMode();
            break;

        case MODE_STANCE:
            for (uint8_t legnum=0; legnum<NUM_LEGS; legnum++)
            {
                Point3d footPosition = Point3d(legs[legnum].baseFootPosition);
                footPosition.x -= configuration.height_offset;
                legs[legnum].moveTo(footPosition);
            }
            break;
    }
}



void Ragnetto::runJoystickMode()
{
    unsigned long now = millis();
    if (coordinatedMovement.stillRunning(now))
    {
        // same phase
        Point3d feetPositions[NUM_LEGS];
        coordinatedMovement.interpolatePositions(now, feetPositions);
        for (uint8_t legnum=0; legnum<NUM_LEGS; legnum++)
        {
            legs[legnum].moveTo(feetPositions[legnum]);
        }
    }
    else
    {
        // new phase
        walking_phase = 1 - walking_phase;
        LOGVLN("Phase",walking_phase);

        if (joystick.idle())
        {
            // joystick in null zone: lower all the legs to stance position
            for (int legnum=0; legnum<NUM_LEGS; legnum++)
            {
                Point3d footPosition = Point3d(legs[legnum].baseFootPosition);
                footPosition.x -= configuration.height_offset;
                coordinatedMovement.legMovements[legnum].setLinearMovement(legs[legnum].currentPosition, footPosition);
            }
            coordinatedMovement.start(now, configuration.phase_duration);
        }
        else
        {
            // joystick outside null zone

            float half_forward_mm_per_cycle = (float)joystick.y * (float)configuration.phase_duration / 1000.0 / 2.0;
            float half_right_mm_per_cycle = (float)joystick.x * (float)configuration.phase_duration / 1000.0 / 2.0;

            // down (pushing) legs
            for (int legnum=walking_phase; legnum<NUM_LEGS; legnum+=2)
            {
                Leg *leg = &legs[legnum];

                Point3d footPosition = Point3d();
                footPosition.x = leg->baseFootPosition.x - half_forward_mm_per_cycle - leg->rotation_x_per_degree * DEG_TO_RAD * joystick.r / 2.0;
                footPosition.y = leg->baseFootPosition.y - half_right_mm_per_cycle - leg->rotation_y_per_degree * DEG_TO_RAD * joystick.r / 2.0;
                footPosition.z = leg->baseFootPosition.z - configuration.height_offset;

                coordinatedMovement.legMovements[legnum].setLinearMovement(legs[legnum].currentPosition, footPosition);
            }

            // up legs
            for (int legnum=1-walking_phase; legnum<NUM_LEGS; legnum+=2)
            {
                Leg *leg = &legs[legnum];
                Point3d footPosition = Point3d();
                footPosition.x = leg->baseFootPosition.x + half_forward_mm_per_cycle + leg->rotation_x_per_degree * DEG_TO_RAD * joystick.r / 2.0;
                footPosition.y = leg->baseFootPosition.y + half_right_mm_per_cycle + leg->rotation_y_per_degree * DEG_TO_RAD * joystick.r / 2.0;
                footPosition.z = leg->baseFootPosition.z - configuration.height_offset;
                
                coordinatedMovement.legMovements[legnum].setQuadraticMovement(legs[legnum].currentPosition,
                        footPosition, leg->baseFootPosition.z - configuration.height_offset + configuration.leg_lift_height);
            }

            coordinatedMovement.start(now, configuration.phase_duration);
        }
    }
}



void LegMovement::interpolatePosition(float progress, Point3d &destination)
{
    switch(type)
    {
        case MOVEMENT_TYPE_LINEAR:
            destination.x = (endPoint.x - startPoint.x) * progress + startPoint.x;
            destination.y = (endPoint.y - startPoint.y) * progress + startPoint.y;
            destination.z = (endPoint.z - startPoint.z) * progress + startPoint.z;
            break;
        
        case MOVEMENT_TYPE_QUADRATIC:
            {   // (explicit block to silence a warning)
                destination.x = (endPoint.x - startPoint.x) * progress + startPoint.x;
                destination.y = (endPoint.y - startPoint.y) * progress + startPoint.y;
                /* ????????????????
                float decelerated_progress = configuration.leg_drop_deceleration * progress * progress * progress
                    - (2*configuration.leg_drop_deceleration + 1) * progress * progress + (configuration.leg_drop_deceleration + 2) * progress;
                destination.z = a * decelerated_progress * decelerated_progress + b * decelerated_progress + c;
                */
                destination.z = a * progress * progress + b * progress + c;
                break;
            }
        
        default:
            serial_senderror("Invalid movement type");
    }
}

void LegMovement::setLinearMovement(Point3d &new_startpoint, Point3d &new_endpoint)
{
    type = MOVEMENT_TYPE_LINEAR;
    startPoint = new_startpoint;
    endPoint = new_endpoint;
}

void LegMovement::setQuadraticMovement(Point3d &new_startpoint, Point3d &new_endpoint, float new_intermediate_z)
{
    type = MOVEMENT_TYPE_QUADRATIC;
    startPoint = new_startpoint;
    endPoint = new_endpoint;
    intermediate_z = new_intermediate_z;

    // coefficients for the quadratic curve passing through starting z, intermediate z and end z.
    // (painfully calculated on paper :-) )
    a = 2*startPoint.z + 2*endPoint.z - 4*intermediate_z;
    b = -3*startPoint.z - endPoint.z + 4*intermediate_z;
    c = startPoint.z;
}

void CoordinatedMovement::start(long durationMillis)
{
    start(millis());
}

void CoordinatedMovement::start(unsigned long newStartMillis, long durationMillis)
{
    startMillis = newStartMillis;
    endMillis = startMillis + durationMillis;
}

void CoordinatedMovement::interpolatePositions(unsigned long millis, Point3d destinationPoints[NUM_LEGS])
{
    float progress;
    if ((endMillis == startMillis) || (millis >= endMillis))
        progress = 1.0;
    else
        progress = (float)(millis - startMillis) / (float)(endMillis - startMillis);

    for (int i = 0; i < NUM_LEGS; i++)
    {
        legMovements[i].interpolatePosition(progress, destinationPoints[i]);
    }
}

bool CoordinatedMovement::stillRunning(unsigned long millis)
{
    return millis < endMillis;
}

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
    {
        LOGVLN("?j1",joint1_angle);
        return false;
    }

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
    if ((signbit(sina) != signbit(p.y)) && (signbit(cosa) != signbit(p.x)))
        l = -l;
    Point2d pp(l - LEG_SEGMENT_1_LENGTH, p.z);

    /* inverse kinematics for joint 2 and 3 */
    const float cosb = (pp.x * pp.x + pp.y * pp.y - LEG_SEGMENT_2_LENGTH * LEG_SEGMENT_2_LENGTH - LEG_SEGMENT_3_LENGTH * LEG_SEGMENT_3_LENGTH) / (2 * LEG_SEGMENT_2_LENGTH * LEG_SEGMENT_3_LENGTH);

    /* check if the point is out of reach (no solution) */
    if (abs(cosb) > 1.0)
    {
        LOGVLN("?cos b",cosb);
        return false;
    }

    // 2 solutions for square root
    const float sinb1 = sqrt(1 - cosb * cosb);
    const float sinb2 = -sinb1;

    // note: the check on sinb1 covers sinb2 too
    if (abs(sinb1) > 1.0)
    {
        LOGVLN("?sin b1",sinb1);
        return false;
    }

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
        float c1 = c, b1=b; //?????????????
        c = atan2(sinb1, cosb);
        b = v_ang - atan2(LEG_SEGMENT_3_LENGTH * sinb1, LEG_SEGMENT_2_LENGTH + LEG_SEGMENT_3_LENGTH * cosb);
        if ((c >= JOINT3_MIN_ANGLE) && (c <= JOINT3_MAX_ANGLE) && (b >= JOINT2_MIN_ANGLE) && (b <= JOINT2_MAX_ANGLE))
        {
            // solution 1 is ok
            result_angles[1] = b;
            result_angles[2] = c;
        }
        else
        {
            // no solution is good
            LOGVLN("?b1",b1);
            LOGVLN("?b2",b);
            LOGVLN("?b min",JOINT2_MIN_ANGLE);
            LOGVLN("?b max",JOINT2_MAX_ANGLE);
            LOGVLN("?c1",c1);
            LOGVLN("?c2",c);
            LOGVLN("?c min",JOINT3_MIN_ANGLE);
            LOGVLN("?c max",JOINT3_MAX_ANGLE);
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
