#ifndef SERIAL_H
#define SERIAL_H

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
    #define SER ss
#else
    #define SER Serial
#endif

#define serial_print(x) SER.print(x)
#define serial_println(x) SER.println(x)


/*
Receive available characters from the serial line and puts them in a buffer.
When a LF cratacter is received the buffer is returned (excluding the LF).
This function will not block. If there are not enough characters immediately
available to complete che line it returns nullptr.
If the buffer becomes full the remain characters (until the LF) are discarded.
CR characters are skipped.
*/
char *serial_receive_command();

/* Start serial communication. */
void serial_begin();


#endif
