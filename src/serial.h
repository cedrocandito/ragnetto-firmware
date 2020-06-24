#ifndef SERIAL_H
#define SERIAL_H

#include <Arduino.h>

/*
if defined, commands will be accepted from software serial;
if not defined they will be accepted from serial (usb) port
*/
//#define USE_SOFTWARE_SERIAL

// size of the buffer, including the end of string character.
#define COMMAND_BUFFER_SIZE 32

// transmit and receive pins for software serial
#define SOFTWARESERIAL_TX_PIN 3
#define SOFTWARESERIAL_RX_PIN 2

// baudrate for serial port (usb)
#define HARDWARE_SERIAL_BAUDRATE 9600

// baudrate for software serial
#define SOFTWARE_SERIAL_BAUDRATE 9600


#ifdef USE_SOFTWARE_SERIAL
    #include <SoftwareSerial.h>
    extern SoftwareSerial ss;
#endif



class RagnettoSerial : public Stream
{
public:
	void begin();
	void end();
	virtual int read();
	virtual int availableForWrite();
    virtual int available();
	virtual void flush();
	virtual size_t write(uint8_t);
	virtual size_t write(const uint8_t*, size_t);
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
    void send_error_legacy(const __FlashStringHelper *);
    void send_error_legacy(const char *);


private:
    char command_buffer[COMMAND_BUFFER_SIZE];
    uint8_t buffer_index = 0;
    void setup_console();
};

extern RagnettoSerial ragnetto_serial;

#endif
