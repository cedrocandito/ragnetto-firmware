#ifndef ragnetto_config_h
#define ragnetto_config_h

#include <arduino.h>
#include "ragnetto_hardware.h"
#include "logging.h"

#define CONFIG_ID_SIZE 9+1
#define CONFIG_ID "RAGNETTO"
#define CONFIG_BASE_ADDR 0

class Configuration
{
    private:
    char version[CONFIG_ID_SIZE];
    uint16_t config_size;

    public:
    int8_t servo_trim[NUM_LEGS][SERVOS_PER_LEG];

    /* Reset in-memory configuration to default values. */
    Configuration();
    void restore_default();
    void read();
    void write();

    private:
    void read_eeprom_block(uint16_t position, char *buffer, uint16_t size);
    bool compare_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size);
};



#endif