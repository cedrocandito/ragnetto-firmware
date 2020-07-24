#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>
#include "ragnetto.h"

// First chraracter for an error message
#define LINE_TYPE_ERROR 'E'
// First chraracter for a debug message
#define LINE_TYPE_DEBUG 'D'
// First chraracter for an info message
#define LINE_TYPE_INFO 'I'
// First chraracter for configuration data
#define LINE_TYPE_CONFIG 'C'

// Separator between line type and content
#define LINE_TYPE_SEPARATOR_CHARACTER ';'


#ifdef BLUETOOTH_SERIAL
    #include <SoftwareSerial.h>
    extern SoftwareSerial ss;
#endif


/*
Wrapper for Serial (USB) and SoftwareSerial (Bluetooth module).
Contains methods for reading commands and sending responses and errors.
*/
class RagnettoSerial : public Stream
{
public:
    RagnettoSerial();
	virtual int read();
	virtual int availableForWrite();
    virtual int available();
	virtual void flush();
	virtual size_t write(uint8_t);
	virtual int peek();
	using Print::write;
    operator bool();
    /*
    Receive available characters from the serial line and puts them in a buffer.
    When a LF character is received the buffer is returned (excluding the LF).
    This function will not block. If there are not enough characters immediately
    available to complete che line it returns nullptr.
    If the buffer becomes full the remain characters (until the LF) are discarded.
    CR characters are skipped.
    */
    char *receive_command();
    void start_line(char type);
    void end_line();
    void send_line(char type, const char * message);
    void send_line(char type, const __FlashStringHelper * message);
    void send_debug(const __FlashStringHelper * message);
    void send_info(const __FlashStringHelper * message);
    bool send_error(const __FlashStringHelper * message, char * extrainfo);
    bool send_error(const __FlashStringHelper * message);

private:
    char command_buffer[COMMAND_BUFFER_SIZE];
    uint8_t buffer_index = 0;
    unsigned long timestamp_next_error_can_be_sent = 0;
    uint16_t checksum = 0;
    // true when the previous char was a CR
    char previous_was_cr;
    void flush_checksum();
};

extern RagnettoSerial ragnetto_serial;

#endif
