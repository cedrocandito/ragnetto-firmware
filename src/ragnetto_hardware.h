
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

#define FREQ 50.0
#define PULSE_MICROS_CENTER 1500.0
/* experiments suggest 2560 micros for 90 degrees counterclockwise
 * and 490 micros for 90 degress clockwise. */
#define PULSE_MICROS_OFFSET_90_DEG 1000.0

/* multiplier to convert from microseconds to PWM controller units */
#define MICROSEC_TO_PWM_CONTROLLER_UNITS(m) ((m) * 4096.0 * FREQ / 1000000.0)

/* multiplier to convert from radians to PWM controller units */
#define RADIANS_TO_PWM_CONTROLLER_UNITS(a) (MICROSEC_TO_PWM_CONTROLLER_UNITS((a) * PULSE_MICROS_OFFSET_90_DEG / HALF_PI + PULSE_MICROS_CENTER))

/* angle corresponding to one PWM controller unit */
#define TRIM_ANGLE_UNIT (HALF_PI / MICROSEC_TO_PWM_CONTROLLER_UNITS(1) / PULSE_MICROS_OFFSET_90_DEG)


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

/* Set the position (angle) of one servo, using servo_id. Trim values are applied. */
void set_servo_position(uint8_t servo_id, float angle);


#endif