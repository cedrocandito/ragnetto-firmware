#include <Arduino.h>
#include "serial.h"
#include "logging.h"

char command_buffer[COMMAND_BUFFER_SIZE];
uint8_t buffer_index = 0;

#ifdef USE_SOFTWARE_SERIAL
SoftwareSerial ss(SOFTWARESERIAL_RX_PIN, SOFTWARESERIAL_TX_PIN);
#endif

/* Initialize console serial port. */
void setup_console()
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

/* Start serial communication on serial port (always) and on software
serial (if defined). */
void serial_begin()
{
    setup_console();
    #ifdef USE_SOFTWARE_SERIAL
    ss.begin(HARDWARE_SERIAL_BAUDRATE);
    #endif
}

/*
Receive available characters from the serial line and puts them in a buffer.
When a LF cratacter is received the buffer is returned (excluding the LF).
This function will not block. If there are not enough characters immediately
available to complete che line it returns nullptr.
If the buffer becomes full the remain characters (until the LF) are discarded.
CR characters are skipped.
*/
char *serial_receive_command()
{
    if (SER)
    {
        while (SER.available() > 0)
        {
            char c = SER.read();

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
