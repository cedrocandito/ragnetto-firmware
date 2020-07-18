#ifndef CHECKSUM_H
#define CHECKSUM_H

#include <Arduino.h>
#include "logging.h"
#include "ragnetto.h"

// checksum character for commands and responses
#define CHECKSUM_CHAR  '#'


/* Calculates a 16 bit checksum on a string buffer. */
uint16_t calculateChecksum(char * buffer, uint8_t size);

/* If the buffer ends with "#HHHH#" (where x is any character), the "HHHH" string is extracted
and parsed as a hex unsigned 16-bit integer. The a checksum is calculated on the
previous bytes of the buffer.
If the calculated checksum matches the extracted 16 bit integer, the first "#" is
replaced with a NUL and a true is returned. If not (checksums don't match)
a false is returned and the buffer is left untouched.
If the buffer doesn't contain a checksum the function returns true if allowNoChecksum
is true or false if allowNoChecksum is false. */ 
bool validateChecksum(char * buffer, bool allowNoChecksum);

/* Parses a single character into a 4 bit value. If the character
is not a valid hex character returns -1. */
int8_t parseHexHalfByte(char c);

/* Parses a 4 character string into a 16 bit unsigned int.
If the buffer doesn't contain a valid 4-character hex number the
function will return 0. */
uint16_t parseHex16bit(char *buffer);

/* Formats a single hex digit. */
char formatHexHalfByte(uint8_t halfbyte);

/* Formats a checksum as a 4 digit hex string into buffer (which
must be at least 5 bytes long). */
void formatChecksum(uint16_t checksum, char *buffer);

#endif