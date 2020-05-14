#ifndef SERIAL_H
#define SERIAL_H

/*
Receive available characters from the serial line and puts them in a buffer.
When a LF cratacter is received the buffer is returned (excluding the LF).
This function will not block. If there are not enough characters immediately
available to complete che line it returns nullptr.
If the buffer becomes full the remain characters (until the LF) are discarded.
CR characters are skipped.
*/
char *receiveCommand();

#endif
