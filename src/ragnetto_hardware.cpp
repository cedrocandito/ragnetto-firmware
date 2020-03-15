#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include "ragnetto_hardware.h"

void read_eeprom_block(uint16_t position, char *buffer, uint16_t size);
bool compare_eeprom_block(uint16_t position, char *buffer, uint16_t size);

/*
 * Array of legs; each legs contains 3 servo ids (from upper to lower leg).
 * Servo id 0-15 are for PWM controller 0, 16-31 are for PWM controller 1.
 * Legs are numbered from top left, counterclockwise (so upper right is #5).
 */
const uint8_t legs[NUM_LEGS][SERVOS_PER_LEG] =
    {
        {20, 21, 22},
        {23, 24, 25},
        {26, 27, 28},
        {4, 5, 6},
        {7, 8, 9},
        {10, 11, 12}};

struct Configuration configuration;

/* PWM controllers. */
Adafruit_PWMServoDriver pwm[NUM_PWM_CONTROLLERS];

/* Map pulse microseconds to pwm controller units (0-4096). */
uint16_t microsec_to_pwm_controller_units(uint16_t microsec)
{
    return 4096L * microsec * FREQ / 1000000;
}

/* Map radians to pwm controller units (assuming 0 radians = 1500 microseconds
 * pulse and a right angle = approx. 500 microseconds). Trim values are applied. */
uint16_t angle_to_pwm_controller_units(float angle)
{
    uint8_t microsec_trim = 0; // TODO ??? configurazione trim
    return microsec_to_pwm_controller_units(angle * HALF_PI / PULSE_MICROS_OFFSET_90_DEG + microsec_trim);
}

/** Initialize PWM controllers. */
void setup_pwm_controllers()
{
    for (uint8_t i = 0; i < NUM_PWM_CONTROLLERS; i++)
    {
        pwm[i] = Adafruit_PWMServoDriver(PWM_BASE_ADDR + i);
        pwm[i].begin();
        pwm[i].setPWMFreq(FREQ);
    }
}

/* Set the position (angle) of one servo, using servo_id. Trim values are applied. */
void set_servo_position(uint8_t servo_id, float angle)
{
    uint8_t controller_id = servo_id / CHANNELS_PER_PWM_CONTROLLER;
    uint8_t channel = servo_id % CHANNELS_PER_PWM_CONTROLLER;

    pwm[controller_id].setPWM(channel, 0, angle_to_pwm_controller_units(angle));
}

/* Overwrite the configuration with values from EEPROM. Do not overwrite values
 * not defined in EEPROM (for example in case of an older, shorter configuration) */
void read_configuration()
{
    reset_default_configuration();
    if (compare_eeprom_block(CONFIG_BASE_ADDR, configuration.version, CONFIG_ID_SIZE))
    {
        // configuration is valid, read it
        read_eeprom_block(CONFIG_BASE_ADDR, (char*)&configuration, sizeof(configuration));
    }
    else
    {
        // configuration is not valid, keep default
    }
}

/* Read a block of bytes from EEPROM into a buffer. */
void read_eeprom_block(uint16_t position, char *buffer, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        *buffer++ = EEPROM.read(position + i);
    }
}

/* Compare a block of bytes from EEPROM to the content of a buffer.
return true if they are the same. */
bool compare_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        if (*buffer++ != EEPROM.read(position + i))
            return false;
    }
    return true;
}

/* Reset the configuration to default values. */
void reset_default_configuration()
{
    strcpy(configuration.version, CONFIG_ID);
    configuration.config_size = sizeof(Configuration);
    for (uint8_t leg = 0; leg < NUM_LEGS; leg++)
    {
        for (uint8_t servo = 0; servo < SERVOS_PER_LEG; servo++)
        {
            configuration.servo_trim[leg][servo] = 0;
        }
    }
}
