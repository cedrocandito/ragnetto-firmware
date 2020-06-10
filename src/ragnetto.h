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

// minimum phase duration
#define MIN_PHASE_DURATION 200

#define MAX_SPEED (MAX_PHASE_DISTANCE * 1000.0 / MIN_PHASE_DURATION)

#endif