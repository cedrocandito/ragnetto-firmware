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

/* Character between every field of the output of command "C" */
static const char CONFIG_SEPARATOR = ';';

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

bool Point3d::operator ==(const Point3d &o)
{
    return (abs(x-o.x) < 0.1) && (abs(y-o.y) < 0.1) && (abs(z-o.z) < 0.1);
}
bool Point3d::operator !=(const Point3d &o)
{
    return !(*this==o);
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
bool Leg::moveTo(const Point3d &point)
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
    
    return ok;
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
    char *command = ragnetto_serial.receive_command();
    char *params = command+1;

    if (command != nullptr)
    {
        // ignore empty commands
        if (strlen(command) < 1)
            return;

        ragnetto_serial.print(*command);
        switch (*command)
        {
            // TODO: scanning via custom function (find field end + atoi) instead of sscanf

        case COMMAND_JOYSTICK:
            if (sscanf(params, "%" SCNi16 ";%" SCNi16 ";%" SCNi16, &joystick.y, &joystick.x, &joystick.r) == 3)
                ragnetto_serial.println(OUTPUT_OK);
            else
                ragnetto_serial.println(OUTPUT_ERROR);
            break;

        case COMMAND_SET_HEIGHT:
            if (sscanf(params, "%" SCNi8, &configuration.height_offset) == 1)
                ragnetto_serial.println(OUTPUT_OK);
            else
                ragnetto_serial.println(OUTPUT_ERROR);
            break;
        
        case COMMAND_SET_MAX_PHASE_DURATION:
            if (sscanf(params, "%" SCNu16, &configuration.phase_duration) == 1)
                ragnetto_serial.println(OUTPUT_OK);
            else
                ragnetto_serial.println(OUTPUT_ERROR);
            break;
        
        case COMMAND_SET_LIFT_HEIGHT:
            if (sscanf(params, "%" SCNu8, &configuration.leg_lift_height) == 1)
                ragnetto_serial.println(OUTPUT_OK);
            else
                ragnetto_serial.println(OUTPUT_ERROR);
            break;

        case COMMAND_SET_TRIM:
            uint8_t leg_num;
            uint8_t joint_num;
            int8_t trim;
            if (sscanf(params, "%" SCNu8 ";%" SCNi8 ";%" SCNi8, &leg_num, &joint_num, &trim) == 3
                && leg_num>=0 && leg_num<NUM_LEGS && joint_num>=0 && joint_num<3)
            {
                configuration.servo_trim[leg_num][joint_num] = trim;
                ragnetto_serial.println(OUTPUT_OK);
            }
            else
            {
                ragnetto_serial.println(OUTPUT_ERROR);
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
                    ragnetto_serial.print(configuration.servo_trim[leg][joint]);
                    ragnetto_serial.print(CONFIG_SEPARATOR);
                }
            }
            ragnetto_serial.print(configuration.height_offset);
            ragnetto_serial.print(CONFIG_SEPARATOR);
            ragnetto_serial.print(configuration.leg_lift_height);
            ragnetto_serial.print(CONFIG_SEPARATOR);
            ragnetto_serial.print(configuration.phase_duration);
            ragnetto_serial.print(CONFIG_SEPARATOR);
            ragnetto_serial.print(configuration.leg_lift_duration_percent);
            ragnetto_serial.print(CONFIG_SEPARATOR);
            ragnetto_serial.print(configuration.leg_drop_duration_percent);
            ragnetto_serial.println();
            break;

        case COMMAND_SET_MODE:
            if (sscanf(params, "%" SCNi8, &mode) == 1)
                ragnetto_serial.println(OUTPUT_OK);
            else
                ragnetto_serial.println(OUTPUT_ERROR);
            break;
        
        case COMMAND_SET_LIFT_DROP_TICK:
            if (sscanf(params, "%" SCNu8 ";%" SCNu8, &configuration.leg_lift_duration_percent, &configuration.leg_drop_duration_percent) == 2)
                ragnetto_serial.println(OUTPUT_OK);
            else
                ragnetto_serial.println(OUTPUT_ERROR);
            break;

        default:
            ragnetto_serial.println(F("Unknown command"));
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
            runJoystickMode(millis());
            break;

        case MODE_STANCE:
            for (uint8_t legnum=0; legnum<NUM_LEGS; legnum++)
            {
                Point3d footPosition = Point3d(legs[legnum].baseFootPosition);
                footPosition.z -= configuration.height_offset;
                legs[legnum].moveTo(footPosition);
            }
            break;
    }
}



bool Ragnetto::runJoystickMode(unsigned long now)
{
    if (!coordinatedMovement.stillRunning(now))
    {
        /* ??????
        LOGN(coordinatedMovement.samples);
        LOGS("/");
        LOGNLN(configuration.phase_duration);
        */
        
        // new phase
        walking_phase = 1 - walking_phase;

        if (joystick.idle())
        {
            // joystick inside null zone: lower all the legs to stance position
            // (unless they are alla already in stance position)
            bool at_leat_one_leg_not_at_rest = false;
            for (int legnum=0; legnum<NUM_LEGS; legnum++)
            {
                // target foot position
                Point3d footPosition = Point3d(legs[legnum].baseFootPosition);
                footPosition.z -= configuration.height_offset;
                if (footPosition != legs[legnum].currentPosition)
                {
                    at_leat_one_leg_not_at_rest = true;
                    if ((legnum & 1) == walking_phase)
                    {
                        coordinatedMovement.legMovements[legnum].setLinearMovement(legs[legnum].currentPosition, footPosition);
                    }
                    else
                    {
                        coordinatedMovement.legMovements[legnum].setUpSlideDownMovement(legs[legnum].currentPosition,
                            footPosition, footPosition.z + configuration.leg_lift_height,
                            configuration.leg_lift_duration_percent / 100.0,
                            1.0 - configuration.leg_drop_duration_percent / 100.0);
                    }
                }
            }
            
            if (at_leat_one_leg_not_at_rest)
                coordinatedMovement.start(now, configuration.phase_duration);
        }
        else
        {
            // joystick outside null zone: program movement for next phase
            setupNextPhase(now);
        }
    }

    // interpolate legs positions and activate servos
    if (coordinatedMovement.stillRunning(now))
    {
        Point3d feetPositions[NUM_LEGS];
        coordinatedMovement.interpolatePositions(now, feetPositions);

        bool ok = true;

        for (uint8_t legnum=0; legnum<NUM_LEGS; legnum++)
        {
            if (!legs[legnum].moveTo(feetPositions[legnum]))
                ok = false;
        }

        return ok;
    }
    else
    {
        return true;
    }
}

/* Prepare the next coordinated movement */
void Ragnetto::setupNextPhase(unsigned long now)
{
    float half_forward_mm_per_phase = (float)joystick.y * (float)configuration.phase_duration / 1000.0 / 2.0;
    float half_right_mm_per_phase = (float)joystick.x * (float)configuration.phase_duration / 1000.0 / 2.0;

    float distance_mm_per_phase = 2.0 * sqrtf(half_forward_mm_per_phase * half_forward_mm_per_phase
        + half_right_mm_per_phase * half_right_mm_per_phase);
    
    uint16_t phaseDuration = configuration.phase_duration;
    /* if the distance per phase is above the maximum (where out-of-reach errors begin to appear)
    stay at that maximum and instead lower the phase duration to compensate and reache the requested speed */
    if (distance_mm_per_phase > MAX_PHASE_DISTANCE)
    {
        float normalization_factor = MAX_PHASE_DISTANCE / distance_mm_per_phase;
        phaseDuration = (uint16_t) (configuration.phase_duration * normalization_factor);
        // re-normalize distances to stay at maximum
        half_forward_mm_per_phase *= normalization_factor;
        half_right_mm_per_phase *= normalization_factor;
    }

    // down (pushing) legs
    for (int legnum=walking_phase; legnum<NUM_LEGS; legnum+=2)
    {
        Leg *leg = &legs[legnum];

        Point3d footPosition = Point3d();
        footPosition.x = leg->baseFootPosition.x - half_right_mm_per_phase - leg->rotation_x_per_degree * DEG_TO_RAD * joystick.r / 2.0;
        footPosition.y = leg->baseFootPosition.y - half_forward_mm_per_phase - leg->rotation_y_per_degree * DEG_TO_RAD * joystick.r / 2.0;
        footPosition.z = leg->baseFootPosition.z - configuration.height_offset;

        coordinatedMovement.legMovements[legnum].setLinearMovement(legs[legnum].currentPosition, footPosition);
    }

    // up legs
    for (int legnum=1-walking_phase; legnum<NUM_LEGS; legnum+=2)
    {
        Leg *leg = &legs[legnum];
        Point3d footPosition = Point3d();
        footPosition.x = leg->baseFootPosition.x + half_right_mm_per_phase + leg->rotation_x_per_degree * DEG_TO_RAD * joystick.r / 2.0;
        footPosition.y = leg->baseFootPosition.y + half_forward_mm_per_phase + leg->rotation_y_per_degree * DEG_TO_RAD * joystick.r / 2.0;
        footPosition.z = leg->baseFootPosition.z - configuration.height_offset;
        
        coordinatedMovement.legMovements[legnum].setUpSlideDownMovement(legs[legnum].currentPosition,
                footPosition, leg->baseFootPosition.z - configuration.height_offset + configuration.leg_lift_height,
                configuration.leg_lift_duration_percent / 100.0,
                1.0 - configuration.leg_drop_duration_percent / 100.0);
    }

    coordinatedMovement.start(now, phaseDuration);
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
        
        case MOVEMENT_TYPE_UP_SLIDE_DOWN:
        {
            destination.x = (endPoint.x - startPoint.x) * progress + startPoint.x;
            destination.y = (endPoint.y - startPoint.y) * progress + startPoint.y;

            if (progress < linear_lift_tick)
            {
                // 0 to threshold_in: linear from start z to intermediate z
                destination.z = progress * (intermediate_z - startPoint.z) / linear_lift_tick + startPoint.z;
            }
            else if (progress <= linear_drop_tick)
            {
                // threshold_in to threshold_out: stay at intermediate z
                destination.z = intermediate_z;
            }
            else
            {
                // threshold_out to 1.0: linear from intermediate z to end z
                float m = (intermediate_z - endPoint.z) / (linear_drop_tick - 1.0);
                destination.z = progress * m + endPoint.z - m;
            }

            break;
        }
        
        default:
            ragnetto_serial.send_error(F("Invalid movement type"));
    }
}

void LegMovement::setLinearMovement(Point3d &new_startpoint, Point3d &new_endpoint)
{
    type = MOVEMENT_TYPE_LINEAR;
    startPoint = new_startpoint;
    endPoint = new_endpoint;
}

void LegMovement::setUpSlideDownMovement(Point3d &new_startpoint, Point3d &new_endpoint, float new_intermediate_z, float new_linear_lift_tick, float new_linear_drop_tick)
{
    type = MOVEMENT_TYPE_UP_SLIDE_DOWN;
    startPoint = new_startpoint;
    endPoint = new_endpoint;
    intermediate_z = new_intermediate_z;
    linear_lift_tick = new_linear_lift_tick;
    linear_drop_tick = new_linear_drop_tick;
}

void CoordinatedMovement::start(long durationMillis)
{
    start(millis());
}

void CoordinatedMovement::start(unsigned long newStartMillis, long durationMillis)
{
    startMillis = newStartMillis;
    endMillis = startMillis + durationMillis;
    samples = 0;
    running = true;
}

void CoordinatedMovement::interpolatePositions(unsigned long millis, Point3d destinationPoints[NUM_LEGS])
{
    float progress;
    if ((millis >= endMillis) || (endMillis == startMillis))
    {
        // movement terminated
        progress = 1.0;
        running = false;
    }
    else
    {
        // movement still running
        progress = (float)(millis - startMillis) / (float)(endMillis - startMillis);
    }

    for (int i = 0; i < NUM_LEGS; i++)
    {
        legMovements[i].interpolatePosition(progress, destinationPoints[i]);
    }

    samples++;
}

bool CoordinatedMovement::stillRunning(unsigned long millis)
{
    return running;
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
        return false;
    }

    // 2 solutions for square root
    const float sinb1 = sqrt(1 - cosb * cosb);
    const float sinb2 = -sinb1;

    // note: the check on sinb1 covers sinb2 too
    if (abs(sinb1) > 1.0)
    {
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
