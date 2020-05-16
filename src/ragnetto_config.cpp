#include <arduino.h>
#include <EEPROM.h>
#include "ragnetto_config.h"
#include "logging.h"

// configuration instance */
Configuration configuration;


/* Constructor */
Configuration::Configuration()
{
    read();
}

/* Overwrite the configuration with values from EEPROM. Do not overwrite values
 * not defined in EEPROM (for example in case of an older, shorter configuration).
 * If EEPROM configuration is not valid the default values will be set in memory. */
void Configuration::read()
{
    restore_default();
    if (compare_eeprom_block(CONFIG_BASE_ADDR, (uint8_t*)version, CONFIG_ID_SIZE))
    {
        // configuration is valid, read it
        LOGSLN("Configuration in EEPROM is valid");
        read_eeprom_block(CONFIG_BASE_ADDR, (char*)this, sizeof(Configuration));
    }
    else
    {
        // configuration is not valid, keep default
        LOGSLN("Configuration in EEPROM is not valid. Default values will be used.");
    }
}

/* Write in-memory configuration to EEPROM. */
void Configuration::write()
{
    LOGSLN("Configuration::write not yet implemented!!");
}

/* Read a block of bytes from EEPROM into a buffer. */
void Configuration::read_eeprom_block(uint16_t position, char *buffer, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        *buffer++ = EEPROM.read(position + i);
    }
}

/* Compare a block of bytes from EEPROM to the content of a buffer.
return true if they are the same. */
bool Configuration::compare_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size)
{
    for (uint16_t i = 0; i < size; i++)
    {
        if (*buffer++ != EEPROM.read(position + i))
            return false;
    }
    return true;
}

/* Reset in-memory configuration to default values. */
void Configuration::restore_default()
{
    strcpy(version, CONFIG_ID);
    config_size = sizeof(Configuration);
    for (uint8_t leg = 0; leg < NUM_LEGS; leg++)
    {
        for (uint8_t servo = 0; servo < SERVOS_PER_LEG; servo++)
        {
            servo_trim[leg][servo] = 0;
        }
    }
    height_offset = 0;
}
