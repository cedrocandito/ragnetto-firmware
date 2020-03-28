#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include "ragnetto_hardware.h"
#include "logging.h"

// local functions (not public)
void read_eeprom_block(uint16_t position, char *buffer, uint16_t size);
void read_configuration();
bool compare_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size);
void setup_pwm_controllers();
void setup_console();

struct Configuration configuration;

/* PWM controllers. */
Adafruit_PWMServoDriver pwm[NUM_PWM_CONTROLLERS];


/* Map radians to pwm controller units (assuming 0 radians = 1500 microseconds
 * pulse and a right angle = approx. 500 microseconds). */
uint16_t angle_to_pwm_controller_units(float angle)
{
    return RADIANS_TO_PWM_CONTROLLER_UNITS(angle);
}

/* Initialize all the hardware */
void setup_hardware()
{
    setup_console();
    read_configuration();
    setup_pwm_controllers();
    LOGSLN("Hardware setup complete.");
}

/* Initialize console serial port. */
void setup_console()
{
    Serial.begin(TERMINAL_BAUD);
    unsigned long t1 = micros();
    while(!Serial)
    {
        delay(2);
    }
    unsigned long t2 = micros();
    LOGS("Had to wait ");
    LOGN(t2-t1);
    LOGSLN(" microseconds for the console serial port to become active.");
}

/* Initialize PWM controllers. */
void setup_pwm_controllers()
{
    for (uint8_t i = 0; i < NUM_PWM_CONTROLLERS; i++)
    {
        LOGS("Initializing PWM controller at address ");
        LOGNLN(PWM_BASE_ADDR + i);
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

    //??????????????????
    LOGLN();
    LOGS("Channel ");
    LOGN(controller_id);
    LOGS(":");
    LOGN(channel);
    LOGS(" set to ");
    LOGN(angle);
    LOGS(" rad,  ");
    LOGN(angle * PULSE_MICROS_OFFSET_90_DEG / HALF_PI + PULSE_MICROS_CENTER);
    LOGS(" micros,  ");
    LOGN(angle_to_pwm_controller_units(angle));
    LOGSLN(" units");

    pwm[controller_id].setPWM(channel, 0, angle_to_pwm_controller_units(angle));
}

/* Overwrite the configuration with values from EEPROM. Do not overwrite values
 * not defined in EEPROM (for example in case of an older, shorter configuration) */
void read_configuration()
{
    reset_default_configuration();
    if (compare_eeprom_block(CONFIG_BASE_ADDR, (uint8_t*)configuration.version, CONFIG_ID_SIZE))
    {
        // configuration is valid, read it
        LOGSLN("Configuration in EEPROM is valid");
        read_eeprom_block(CONFIG_BASE_ADDR, (char*)&configuration, sizeof(configuration));
    }
    else
    {
        // configuration is not valid, keep default
        LOGSLN("Configuration in EEPROM is not valid. Default values will be used.");
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
    for (uint8_t servo = 0; servo < NUM_LEGS * SERVOS_PER_LEG; servo++)
    {
        configuration.servo_trim[servo] = 0;
    }
}
