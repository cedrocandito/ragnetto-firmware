#include <Arduino.h>
#include "logging.h"
#include "ragnetto.h"
#include "checksum.h"

#ifdef BLUETOOTH_SERIAL
    #include <AltSoftSerial.h>
    AltSoftSerial ss;
#else
    #include "serial.h"
#endif


int RagnettoSerial::read()
{
    #ifdef BLUETOOTH_SERIAL
    return ss.read();
    #else
    return Serial.read();
    #endif
}

int RagnettoSerial::availableForWrite()
{
    #ifdef BLUETOOTH_SERIAL
    return ss.availableForWrite();
    #else
    return Serial.availableForWrite();
    #endif
}

int RagnettoSerial::available()
{
    #ifdef BLUETOOTH_SERIAL
    return ss.available();
    #else
    return Serial.available();
    #endif
}

void RagnettoSerial::flush()
{
    #ifdef BLUETOOTH_SERIAL
    ss.flush();
    #else
    Serial.flush();
    #endif
}

size_t internal_write(uint8_t x)
{
    #ifdef BLUETOOTH_SERIAL
    return ss.write(x);
    #else
    return Serial.write(x);
    #endif
}

size_t RagnettoSerial::write(uint8_t x)
{
    switch(x)
    {
        case '\r':
            // if a previous CR was stored, flush it
            // (a double '\r' is very unlikely, anyway)
            if (previous_was_cr)
                internal_write('\r');
            // store the CR, don't write it to the stream
            previous_was_cr = true;
            return 1;

        case '\n':
            // write a checksum to the stream
            flush_checksum();
            // flush the CR buffer if one was stored
            if (previous_was_cr)
            {
                internal_write('\r');
                previous_was_cr = false;
            }
            // write the LF
            return internal_write(x);

        default:
            // flush the CR buffer if one was stored
            if (previous_was_cr)
            {
                internal_write('\r');
                previous_was_cr = false;
            }
            // update the checksum
            checksum += x;
            // write the requested character
            return internal_write(x);
    }
}

/* Flush the current checksum to the underlying stream and reset the checkum accumulator. */
void RagnettoSerial::flush_checksum()
{
    internal_write(CHECKSUM_CHAR);
    char hexchecksum[5];
    formatChecksum(checksum,hexchecksum);
    for (uint8_t i = 0; i<4; i++)
    {
        internal_write(hexchecksum[i]);
    }
    internal_write(CHECKSUM_CHAR);
    // reset the checksum
    checksum = 0;
}

RagnettoSerial::operator bool()
{
    #ifdef BLUETOOTH_SERIAL
    return true;
    #else
    return Serial;
    #endif
}

int RagnettoSerial::peek()
{
    #ifdef BLUETOOTH_SERIAL
    return ss.peek();
    #else
    return Serial.peek();
    #endif
}

void RagnettoSerial::start_line(char type)
{
    print(type);
    print(LINE_TYPE_SEPARATOR_CHARACTER);
}

void RagnettoSerial::end_line()
{
    println();
}

void RagnettoSerial::send_line(char type, const __FlashStringHelper * message)
{
    start_line(type);
    println(message);
}

void RagnettoSerial::send_info(const __FlashStringHelper * message)
{

    send_line(LINE_TYPE_INFO, message);
}

bool RagnettoSerial::send_error(const __FlashStringHelper * message, char * extrainfo)
{
    if (millis() >= timestamp_next_error_can_be_sent)
    {
        start_line(LINE_TYPE_ERROR);
        print(message);
        if (extrainfo != nullptr)
        {
            print(F(": "));
            print(extrainfo);
        }
        end_line();

        timestamp_next_error_can_be_sent = millis() + MIN_TIME_BETWEEN_ERRORS;
        return true;
    }
    else
    {
        return false;
    }
}

bool RagnettoSerial::send_error(const __FlashStringHelper * message)
{
    return send_error(message, nullptr);
}

void RagnettoSerial::send_debug(const __FlashStringHelper * message)
{
    send_line(LINE_TYPE_DEBUG, message);
}

/* Initialize serial port. */
void RagnettoSerial::begin()
{
    #ifdef BLUETOOTH_SERIAL
    ss.begin(SOFTWARE_SERIAL_BAUDRATE);
    #else
    Serial.begin(HARDWARE_SERIAL_BAUDRATE);
    while(!Serial)
    {
        delay(2);
    }
    #endif
}

char * RagnettoSerial::receive_command()
{
    if (*this)
    {
        while (available() > 0)
        {
            char c = read();

            // completely skip CR characters
            if (c == '\r')
            {
                continue;
            }

            // end of line? end with NUL and return the buffer
            if (c == '\n')
            {
                command_buffer[buffer_index] = '\0';
                buffer_index = 0;
                return command_buffer;
            }

            // other character? add it to the buffer if thers is room for it
            if (buffer_index < COMMAND_BUFFER_SIZE-1)
            {
                command_buffer[buffer_index++] = c;
            }
        }
    }

    return nullptr;
}

RagnettoSerial ragnetto_serial;