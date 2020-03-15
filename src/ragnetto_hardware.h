
#ifndef ragnetto_hardware_h
#define ragnetto_hardware_h

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


#define NUM_LEGS 6
#define SERVOS_PER_LEG 3

#define NUM_PWM_CONTROLLERS 2
#define CHANNELS_PER_PWM_CONTROLLER 16

#define PWM_BASE_ADDR 0x40

#define FREQ 50
#define CENTER_MICROS 1500
/* experiments suggest 2560 micros for 90 degrees counterclockwise
 * and 490 micros for 90 degress clockwise. */
#define PULSE_MICROS_OFFSET_90_DEG 1000

#define TERMINAL_BAUD 9600

#define CONFIG_ID_SIZE 9+1
#define CONFIG_ID "RAGNETTO"
#define CONFIG_BASE_ADDR 0

struct Configuration
{
    char version[CONFIG_ID_SIZE];
    uint16_t config_size;
    int8_t servo_trim[NUM_LEGS * SERVOS_PER_LEG];
};

/* Initialize all the hardware. */
void setup_hardware();

/* Reset the configuration to default values. */
void reset_default_configuration();



#endif