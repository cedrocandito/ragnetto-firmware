#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ragnetto_hardware.h"
#include "logging.h"

// local functions (not public)
void setup_pwm_controllers();


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
    setup_pwm_controllers();
    LOG_LINE("Hardware setup complete.");
}

/* Initialize PWM controllers. */
void setup_pwm_controllers()
{
    for (uint8_t i = 0; i < NUM_PWM_CONTROLLERS; i++)
    {
        ragnetto_serial.start_line(LINE_TYPE_INFO);
        ragnetto_serial.print(F("Initializing PWM controller at address "));
        ragnetto_serial.print(PWM_BASE_ADDR + i);
        ragnetto_serial.end_line();
        pwm[i] = Adafruit_PWMServoDriver(PWM_BASE_ADDR + i);
        pwm[i].begin();
        pwm[i].setPWMFreq(FREQ);
    }
}

/* Set the position (angle) of one servo, using servo_id. Trim values are applied
to the final calculated 0-4096 value. */
void set_servo_position(uint8_t servo_id, float angle, int8_t trim)
{
    uint8_t controller_id = servo_id / CHANNELS_PER_PWM_CONTROLLER;
    uint8_t channel = servo_id % CHANNELS_PER_PWM_CONTROLLER;
    
    pwm[controller_id].setPWM(channel, 0, angle_to_pwm_controller_units(angle) + trim);
}
