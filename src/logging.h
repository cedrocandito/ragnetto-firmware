#ifndef LOGGING_H
#define LOGGING_H

#include "ragnetto.h"
#include "serial.h"


#ifdef LOGGING_ENABLED
    // start log line
    #define LOG_START ragnetto_serial.start_line(LINE_TYPE_DEBUG)
    // end log line (EOL)
    #define LOG_END ragnetto_serial.end_line()

    // log string (needs LOG_START and LOG_END)
    #define LOGS(x) ragnetto_serial.print(F(x))
    // log non-string (needs LOG_START and LOG_END)
    #define LOGN(x) ragnetto_serial.print(x)
    // log label and value (needs LOG_START and LOG_END)
    #define LOGV(x,y) { ragnetto_serial.print(F(x)); ragnetto_serial.print(F("=")); ragnetto_serial.print(y); }

    // log entire line with type and separator
    #define LOG_LINE(x) ragnetto_serial.send_debug(F(x))
    // log label and value with type and separator
    #define LOG_VALUE(x,y) { LOG_START; ragnetto_serial.print(F(x)); ragnetto_serial.print(F("=")); ragnetto_serial.print(y); LOG_END; }
    
#else
    #define LOG_START
    #define LOG_END
    #define LOGS(x)
    #define LOGN(x)
    #define LOGV(x,y)
    #define LOG_LINE(x)
    #define LOG_VALUE(x,y)
#endif

#endif