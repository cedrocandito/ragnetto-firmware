#ifndef RAGNETTO_H
#define RAGNETTO_H

// Basic application constants

#define NUM_LEGS 6
#define SERVOS_PER_LEG 3

// radius from center of body to leg attachment axis (mm)
#define LEG_ATTACHMENT_RADIUS 77.0

// distance between the first axis (horizontal movement) and the next (mm)
#define LEG_SEGMENT_1_LENGTH 29.28

// distance between second axis and the last (third) one (mm)
#define LEG_SEGMENT_2_LENGTH 70.0

// distance between the last axis and the end of the foot (mm)
#define LEG_SEGMENT_3_LENGTH 70.0

// minimum and maximum angle for the first joint (radians)
#define JOINT1_MAX_ANGLE (DEG_TO_RAD * 40)
#define JOINT1_MIN_ANGLE (-JOINT1_MAX_ANGLE)

// minimum and maximum angle for the second joint (radians)
#define JOINT2_MAX_ANGLE HALF_PI
#define JOINT2_MIN_ANGLE -HALF_PI
// offset to be subtracted from the joint 2 angle to get the servo angle (radians)
#define JOINT2_OFFSET 0

// minimum and maximum angle for the third joint (radians)
#define JOINT3_MAX_ANGLE 0
#define JOINT3_MIN_ANGLE (-HALF_PI -DEG_TO_RAD * 30)
// offset to be subtracted from the joint 3 angle to get the servo angle (radians)
#define JOINT3_OFFSET -HALF_PI

// z-position of the foot relative to the attachment point of the leg (mm)
#define BASE_FOOT_Z -LEG_SEGMENT_3_LENGTH
// position of the foot along the horizontal plane, relative to the attachment point of the leg (mm)
#define BASE_FOOT_R (LEG_SEGMENT_1_LENGTH + LEG_SEGMENT_2_LENGTH)

// when all values of the joystick are below this threshold the robot
// will be stopped (all legs down).
#define JOYSTICK_NULL_ZONE 2

// maximum distance to move a leg in a single phase (in mm)
#define MAX_PHASE_DISTANCE 40.0

// maximum angle to rotate in a single phase (in degrees)
#define MAX_PHASE_ROTATION 30.0

// minimum phase duration
#define MIN_PHASE_DURATION 200

// max speed in mm/sec (calculated)
#define MAX_SPEED (MAX_PHASE_DISTANCE * 1000.0 / MIN_PHASE_DURATION)

// max rotation speed in deg/sec (calculated)
#define MAX_ROTATION_SPEED (MAX_PHASE_ROTATION * 1000.0 / MIN_PHASE_DURATION)

// if defined, commands will be accepted from software serial;
// if not defined they will be accepted from serial (usb) port
#define BLUETOOTH_SERIAL

// size of the buffer, including the end of string character.
#define COMMAND_BUFFER_SIZE 32

// transmit and receive pins for software serial
#define SOFTWARESERIAL_TX_PIN 3
#define SOFTWARESERIAL_RX_PIN 2

// baudrate for serial port (usb)
#define HARDWARE_SERIAL_BAUDRATE 9600

// baudrate for software serial (bluetooth)
#define SOFTWARE_SERIAL_BAUDRATE 9600

// enable verbose info? (comment to disable)
#define LOGGING_ENABLED

// minimum time (in milliseconds) between error lines sent via serial
// (if not limited they will clutter the port and cause delays)
#define MIN_TIME_BETWEEN_ERRORS 100

#endif