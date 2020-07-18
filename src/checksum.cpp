
#include "checksum.h"

/* If the buffer ends with "#HHHH#" (where x is any character), the "HHHH" string is extracted
and parsed as a hex unsigned 16-bit integer. The a checksum is calculated on the
previous bytes of the buffer.
If the calculated checksum matches the extracted 16 bit integer, the first "#" is
replaced with a NUL and a true is returned. If not (checksums don't match)
a false is returned and the buffer is left untouched.
If the buffer doesn't contain a checksum the function returns true if allowNoChecksum
is true or false if allowNoChecksum is false. */ 
bool validateChecksum(char * buffer, bool allowNoChecksum)
{
    uint8_t l = strlen(buffer);
    char * checksumBlockPosition = buffer + l - 6;
    if (l >=6 && *checksumBlockPosition == CHECKSUM_CHAR && *(checksumBlockPosition+5) == CHECKSUM_CHAR)
    {
        // read checksum field
        uint16_t found_checksum = parseHex16bit(checksumBlockPosition+1);
        uint16_t calculated_checksum = calculateChecksum(buffer, l - 6);
        if (calculated_checksum == found_checksum)
        {
            *checksumBlockPosition = '\0';
            return true;
        }
        else
        {
            return false;
        }
    }
    else
    {
        return allowNoChecksum;
    }
}

/* Parses a single character into a 4 bit value. If the character
is not a valid hex character returns -1. */
int8_t parseHexHalfByte(char c)
{
    if (c>='0' && c<='9')
        return c-'0';
    else if (c>='A' && c<='F')
        return c-'A'+10;
    else if (c>='a' && c<='f')
        return c-'a'+10;
    else
        return -1;
}

/* Parses a 4 character string into a 16 bit unsigned int.
If the buffer doesn't contain a valid 4-character hex number the
function will return 0. */
uint16_t parseHex16bit(char *buffer)
{
    uint16_t number = 0;
    for (uint8_t i=0; i<4; i++)
    {
        char c = *buffer++;
        int16_t halfbyte = parseHexHalfByte(c);
        if (halfbyte == -1)
            return 0;
        number |= (halfbyte << ((3-i)*4));
    }
    return number;
}

/* Calculates a 16 bit checksum on a string buffer, up to the indicated
number of characters. */
uint16_t calculateChecksum(char * buffer, uint8_t size)
{
    uint16_t checksum = 0;
    unsigned char b;
    while (((b = *buffer++) != '\0') && (size-- > 0))
    {
        checksum += b;
    }
    return checksum;
}

/* Formats a single hex digit. */
char formatHexHalfByte(uint8_t halfbyte)
{
	if (halfbyte<=9)
		return '0' + halfbyte;
	else
		return 'A' + halfbyte - 10;
}

/* Formats a checksum as a 4 digit hex string into buffer (which
must be at least 5 bytes long). */
void formatChecksum(uint16_t checksum, char *buffer)
{
	for (uint8_t i=0; i<4; i++)
	{
		*buffer++ = formatHexHalfByte((checksum >> ((3-i)*4)) & 0x0f);
	}
	*buffer++ = '\0';
}

