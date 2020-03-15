#ifndef ragnetto_framework_h
#define ragnetto_framework_h

#include <Arduino.h>
#include "ragnetto_hardware.h"

typedef struct
{
    float x;
    float y;
    float z;
}  Point;

const Point LEG_POSITION[NUM_LEGS] = {};
const float LEG_DIRECTION[NUM_LEGS] = {};

#endif