#include <Arduino.h>

// size of the buffer, including the end of string character.
#define COMMAND_BUFFER_SIZE 32

char command_buffer[COMMAND_BUFFER_SIZE];
uint8_t buffer_index = 0;

/*
Receive available characters from the serial line and puts them in a buffer.
When a LF cratacter is received the buffer is returned (excluding the LF).
This function will not block. If there are not enough characters immediately
available to complete che line it returns nullptr.
If the buffer becomes full the remain characters (until the LF) are discarded.
CR characters are skipped.
*/
char *receiveCommand()
{
    if (Serial)
    {
        while (Serial.available() > 0)
        {
            char c = Serial.read();

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
                command_buffer[buffer_index++] = Serial.read();
            }
        }
    }

    return nullptr;
}