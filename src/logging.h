#ifndef LOGGING_H
#define LOGGING_H

#define LOGGING_ENABLED

#ifdef LOGGING_ENABLED
    // log string
    #define LOGS(x) Serial.print(F(x))
    // log string with EOL
    #define LOGSLN(x) Serial.println(F(x))
    // log non-string
    #define LOGN(x) Serial.print(x)
    // log non-string with EOL
    #define LOGNLN(x) Serial.println(x)
    // log label and value
    #define LOGV(x,y) { Serial.print(F(x)); Serial.print(F("=")); Serial.print(y); }
    // log label and value with EOL
    #define LOGVLN(x,y) { Serial.print(F(x)); Serial.print(F("=")); Serial.println(y); }
    // just a line feed
    #define LOGLN() Serial.println()
#else
    #define LOGS(x)
    #define LOGSLN(x)
    #define LOGN(x)
    #define LOGNLN(x)
    #define LOGV(x,y)
    #define LOGVLN(x,y)
    #define LOGLN()
#endif

#endif