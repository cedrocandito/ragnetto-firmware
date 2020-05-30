#include <arduino.h>
#include <EEPROM.h>
#include "ragnetto_config.h"
#include "logging.h"

// configuration instance */
Configuration configuration;


/* Constructor */
Configuration::Configuration()
{
    restore_default();
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
        // read stored config length
        uint16_t size;
        read_eeprom_block(sizeof(version), (uint8_t*)&size, sizeof(config_size));
        // read configuration (it may be shorter than the one we have in memory)
        read_eeprom_block(CONFIG_BASE_ADDR, (uint8_t*)this, size);
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
    LOGSLN("Writing configuration to EEPROM");
    config_size = sizeof(Configuration);
    write_eeprom_block(CONFIG_BASE_ADDR, (uint8_t*)this, sizeof(Configuration));
    LOGSLN("Configuration written");
}

/* Read a block of bytes from EEPROM into a buffer. */
void Configuration::read_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size)
{
    for (uint16_t i = position; i < position + size; i++)
    {
        *buffer++ = EEPROM.read(i);
    }
}

/* Write a block of bytes to EEPROM from a buffer. */
void Configuration::write_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size)
{
    uint8_t previous, to_be_written;
    for (uint16_t i = position; i < position + size; i++)
    {
        previous = EEPROM.read(i);
        to_be_written = *buffer++;
        if (to_be_written != previous)
        {
            EEPROM.write(i, to_be_written);
        }
    }
}

/* Compare a block of bytes from EEPROM to the content of a buffer.
return true if they are the same. */
bool Configuration::compare_eeprom_block(uint16_t position, uint8_t *buffer, uint16_t size)
{
    for (uint16_t i = position; i < position + size; i++)
    {
        if (*buffer++ != EEPROM.read(i))
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
            servo_trim[leg][servo] = DEFAULT_TRIM;
        }
    }
    height_offset = DEFAULT_HEIGHT_OFFSET;
    leg_lift_height = DETAULT_LEG_LIFT_HEIGHT;
    leg_drop_deceleration = DEFAULT_LEG_DROP_DECELERATION;
    phase_duration = DEFAULT_PHASE_DURATION;
}
