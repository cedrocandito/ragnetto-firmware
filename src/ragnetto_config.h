#ifndef RAGNETTO_CONFIG_H
#define RAGNETTO_CONFIG_H

#include <arduino.h>
#include "logging.h"
#include "ragnetto.h"

#define CONFIG_ID_SIZE 9+1
#define CONFIG_ID "RAGNETTO"
#define CONFIG_BASE_ADDR 0

#define DEFAULT_TRIM 0
#define DEFAULT_HEIGHT_OFFSET 10
#define DETAULT_LEG_LIFT_HEIGHT 25
#define DEFAULT_PHASE_DURATION 400
#define DEFAULT_LEG_LIFT_DURATION_PERCENT 20
#define DEFAULT_LEG_DROP_DURATION_PERCENT 20

class Configuration
{
    private:
    char version[CONFIG_ID_SIZE];
    uint16_t config_size;

    public:
    int8_t servo_trim[NUM_LEGS][SERVOS_PER_LEG];
    int8_t height_offset;
    uint8_t leg_lift_height;
    uint16_t phase_duration;
    uint8_t leg_lift_duration_percent;
    uint8_t leg_drop_duration_percent;
    
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