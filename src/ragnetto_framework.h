#ifndef RAGNETTO_FRAMEWORK_H
#define RAGNETTO_FRAMEWORK_H

#include <Arduino.h>
#include "ragnetto_hardware.h"
#include "logging.h"
#include "ragnetto.h"

// working modes
#define MODE_CALIBRATION -1
#define MODE_CALIBRATION2 -2
#define MODE_STANCE 0
#define MODE_JOYSTICK 1

// movement types
#define MOVEMENT_TYPE_LINEAR 0
#define MOVEMENT_TYPE_UP_SLIDE_DOWN 1

// serial commands
#define COMMAND_JOYSTICK 'J'
#define COMMAND_JOYSTICK_PERCENT 'K'
#define COMMAND_SET_TRIM 'T'
#define COMMAND_SET_HEIGHT 'H'
#define COMMAND_SET_LIFT_HEIGHT 'L'
#define COMMAND_SET_MAX_PHASE_DURATION 'P'
#define COMMAND_READ_CONFIGURATION 'R'
#define COMMAND_WRITE_CONFIGURATION 'W'
#define COMMAND_SHOW_CONFIGURATION 'C'
#define COMMAND_SET_MODE 'M'
#define COMMAND_SET_LIFT_DROP_TICK 'Q'

// output content
#define COMMAND_ERROR F("CMD_ERROR")

// separator characted for commands and responses
#define SEPARATOR_CHAR  ';'


// a point in 2d space
class Point3d
{
    public:
    float x;
    float y;
    float z;

    Point3d();
    Point3d(const float newx, const float newy, const float newz);
    Point3d(const Point3d &p);

    bool operator ==(const Point3d &o);
    bool operator !=(const Point3d &o);
};

// a point in 3d space
class Point2d
{
    public:
    float x;
    float y;

    Point2d();
    Point2d(const float newx, const float newy);
    Point2d(const Point3d &p3d);
    Point2d(const Point2d &p);
};

// single leg
class Leg
{
    public:
    uint8_t leg_id;
    Point3d attachmentPosition;
    Point3d baseFootPosition;
    float attachmentAngle; // range is from -PI to +PI (the same atan2() uses)
    float attachmentAngleCos;   // pre-calculated cos(attachmentAngle)
    float attachmentAngleSin;   // pre-calculated sin(attachmentAngle)
    float rotation_x_per_degree;    // pre-calculated x offset to rotate the robot 1 degree
    float rotation_y_per_degree;    // pre-calculated y offset to rotate the robot 1 degree
    bool invertServo;      // true for legs that have joints 2 and 3 mounted in the opposite direction
    Point3d currentPosition;    // last moveTo() position

    // setup leg position and angle
    void setup(const uint8_t id, bool invert);
    
    // move servos to indicated position (relative to leg attachment point); returns true if
    // the destination point is reachable.
    bool moveTo(const Point3d &point);
};

// a single move for a leg (abstract)
class LegMovement
{
    public:
    uint8_t type;
    Point3d startPoint;
    Point3d endPoint;
    float intermediate_z;
    float linear_lift_tick;
    float linear_drop_tick;

    void setLinearMovement(Point3d &new_startpoint, Point3d &new_endpoint);
    void setQuadraticMovement(Point3d &new_startpoint, Point3d &new_endpoint, float new_intermediate_z);
    void setUpSlideDownMovement(Point3d &new_startpoint, Point3d &new_endpoint, float new_intermediate_z, float new_linear_lift_tick, float new_linear_drop_tick);
    /* Interpolate middle points during the move storing che current point into destination.
    "progress" is 0.0 at segment start and 1.0 at segment end. */
    void interpolatePosition(float progress, Point3d &destination);
};

/* movements fot all the six legs. */
class CoordinatedMovement
{
    public:
    unsigned long startMillis;
    unsigned long endMillis;
    LegMovement legMovements[NUM_LEGS];
    bool running;

    void start(long durationMillis);
    void start(unsigned long newStartMillis, long durationMillis);
    void interpolatePositions(unsigned long millis, Point3d destinationPoints[NUM_LEGS]);
    bool stillRunning(unsigned long millis);
};

/* virtual joystick for controlling the motion of the robot */
class Joystick
{
    public:
    /* left/right speed in mm/sec (positive = right) */
    int16_t x;
    /* forward/backward in mm/sec (positive is forward) */
    int16_t y; 
    /* rotation in degrees/sec (positive is ccw) */
    int16_t r;

    bool idle();
};

/* top level class for the robot */
class Ragnetto
{
    public:
    int8_t mode;
    Leg legs[NUM_LEGS];
    Joystick joystick;
    CoordinatedMovement coordinatedMovement;
    /* during phase 0 legs 0, 2, 4 are down and legs 1, 3, 5 are up; during
    phase 1 the opposite is true */
    uint8_t walking_phase = 0;

    // constructor
    Ragnetto();

    // Method to put in mail loop: read commands and move the robot
    void run();

    // the following methods should be private, but making them public is useful
    // for reusing the code in a pc for simulation purposes

    // Read characters from serial and act upon them
    void process_input();
    // Delegated by run() when in joystick walk mode; returns false if one or more legs are moved out of range
    bool runJoystickMode(unsigned long now);
    // Prepare the next coordinated movement
    void setupNextPhase(unsigned long now);

    private:
    void commandError(char * commandText);
};

// ============ functions ===============

/* Copis the characters in "string" in the "destination_buffer" until the end of the string
or a separator character is found. "string" is updated with the next position after the separator
(it will point to '\0' if there are no more characters to read). */
void scanForNextSymbol(char * & string, char * destination_buffer, const uint8_t buffer_size);

/* Scans "string" until '\0' or the separator character are found. Convert the scanned characters
to int and store into "number" (passed by reference).
"string" is updated with the next position after the separator (it will
point to '\0' if there are no more characters to read). Returns true if at least one
character was read (but the functions, which uses atoi(), just returns 0 if the string is not
a valid number). */
bool scanForNextNumericField(char * & string, int &number);

/* Scans "string" for "how_many_numbers" int's separated by SEPARATOR_CHAR; they will be stored
in the pre-allocated "numbers" array. If all the fields were read (but not necessarily
valid numbers) returns true. */
bool scanForNumericFields(char * string, int *numbers, const uint8_t how_many_numbers);

/* Calculate the angles to be applied to the joints in order to move the leg to
a point in space. Coordinates are relative to attachment point of the leg, angles are absolute.
Returns true if there is a solution, false if not. */
bool pointIn3dSpaceToJointAngles(const Point3d &p, const Leg &leg, float result_angles[]);

#endif