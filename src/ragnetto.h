#ifndef RAGNETTO_H
#define RAGNETTO_H

// Basic application constants

#define NUM_LEGS 6
#define SERVOS_PER_LEG 3

// radius from center of body to leg attachment axis (mm)
#define LEG_ATTACHMENT_RADIUS 77.0

// distance between the first axis (horizontal movement) and the next (mm)
#define LEG_SEGMENT_1_LENGTH 27.0

// distance between second axis and the last (third) one (mm)
#define LEG_SEGMENT_2_LENGTH 70.0

// distance between the last axis and the end of the foot (mm)
#define LEG_SEGMENT_3_LENGTH 70.0

// minimum and maximum angle for the first joint (radians)
#define JOINT1_MAX_ANGLE ((TWO_PI * 360.0) / 35.0)
#define JOINT1_MIN_ANGLE (-JOINT1_MAX_ANGLE)

// minimum and maximum angle for the second joint (radians)
#define JOINT2_MAX_ANGLE HALF_PI
#define JOINT2_MIN_ANGLE -HALF_PI
// offset to be subtracted from the joint 2 angle to get the servo angle (radians)
#define JOINT2_OFFSET 0

// minimum and maximum angle for the third joint (radians)
#define JOINT3_MAX_ANGLE HALF_PI
#define JOINT3_MIN_ANGLE (-TWO_PI * 30.0 / 360.0)
// offset to be subtracted from the joint 2 angle to get the servo angle (radians)
#define JOINT3_OFFSET -HALF_PI


#endif