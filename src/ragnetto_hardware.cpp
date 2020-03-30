#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ragnetto_hardware.h"
#include "ragnetto_config.h"
#include "logging.h"

// local functions (not public)
void read_eeprom_block(uint16_t position, char *buffer, uint16_t size);
void read_configuration();
bool compare_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size);
void setup_pwm_controllers();
void setup_console();

static Configuration configuration;

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
    /*
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
    */

    pwm[controller_id].setPWM(channel, 0, angle_to_pwm_controller_units(angle));
}
