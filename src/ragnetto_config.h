#ifndef RAGNETTO_CONFIG_H
#define RAGNETTO_CONFIG_H

#include <arduino.h>
#include "logging.h"
#include "ragnetto.h"

#define CONFIG_ID_SIZE 9+1
#define CONFIG_ID "RAGNETTO"
#define CONFIG_BASE_ADDR 0

#define DEFAULT_TRIM 0
#define DEFAULT_HEIGHT_OFFSET 0
#define DETAULT_LEG_LIFT_HEIGHT 50
#define DEFAULT_LEG_DROP_DECELERATION 0.1
#define DEFAULT_MAX_PHASE_DURATION 500

class Configuration
{
    private:
    char version[CONFIG_ID_SIZE];
    uint16_t config_size;

    public:
    int8_t servo_trim[NUM_LEGS][SERVOS_PER_LEG];
    int8_t height_offset;
    int8_t leg_lift_height;
    float leg_drop_deceleration;
    uint16_t max_phase_duration;
    

    Configuration();
    /* Reset in-memory configuration to default values. */
    void restore_default();
    /* Read EEPROM to memory. */
    void read();
    /* Write memory configuration to EEPROM. */
    void write();

    private:
    void read_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size);
    void write_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size);
    bool compare_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size);
};

extern Configuration configuration;

#endif