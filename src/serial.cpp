#include <Arduino.h>
#include "serial.h"
#include "logging.h"


#ifdef USE_SOFTWARE_SERIAL
SoftwareSerial ss(SOFTWARESERIAL_RX_PIN, SOFTWARESERIAL_TX_PIN);
#endif


/* Start serial communication on serial port (always) and on software
serial (if defined). */
void RagnettoSerial::begin()
{
    setup_console();
    #ifdef USE_SOFTWARE_SERIAL
    ss.begin(HARDWARE_SERIAL_BAUDRATE);
    #endif
}

void RagnettoSerial::end()
{
    Serial.end();
    #ifdef USE_SOFTWARE_SERIAL
    ss.end();
    #endif
}

int RagnettoSerial::read()
{
    #ifdef USE_SOFTWARE_SERIAL
    return ss.read();
    #else
    return Serial.read();
    #endif
}

int RagnettoSerial::availableForWrite()
{
    #ifdef USE_SOFTWARE_SERIAL
    return ss.availableForWrite();
    #else
    return Serial.availableForWrite();
    #endif
}

int RagnettoSerial::available()
{
    #ifdef USE_SOFTWARE_SERIAL
    return ss.available();
    #else
    return Serial.available();
    #endif
}

void RagnettoSerial::flush()
{
    #ifdef USE_SOFTWARE_SERIAL
    ss.flush();
    #else
    Serial.flush();
    #endif
}

size_t RagnettoSerial::write(uint8_t x)
{
    #ifdef USE_SOFTWARE_SERIAL
    return ss.write(x);
    #else
    return Serial.write(x);
    #endif  
}

size_t RagnettoSerial::write(const uint8_t* pointer, size_t size)
{
    #ifdef USE_SOFTWARE_SERIAL
    return ss.write(pointer, size);
    #else
    return Serial.write(pointer, size);
    #endif
}

RagnettoSerial::operator bool()
{
    #ifdef USE_SOFTWARE_SERIAL
    return ss;
    #else
    return Serial;
    #endif
}

int RagnettoSerial::peek()
{
    #ifdef USE_SOFTWARE_SERIAL
    return ss.peek();
    #else
    return Serial.peek();
    #endif
}
void RagnettoSerial::send_error(const __FlashStringHelper *description)
{
    print('E');
    println(description);
}

void RagnettoSerial::send_error(const char *description)
{
    print('E');
    println(description);
}

/* Initialize console serial port. */
void RagnettoSerial::setup_console()
{
    Serial.begin(HARDWARE_SERIAL_BAUDRATE);
    #ifdef LOGGING_ENABLED
    unsigned long t1 = micros();
    #endif
    while(!Serial)
    {
        delay(2);
    }
    #ifdef LOGGING_ENABLED
    unsigned long t2 = micros();
    LOGS("Had to wait ");
    LOGN(t2-t1);
    LOGSLN(" microseconds for the console serial port to become active.");
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