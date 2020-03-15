#ifndef logging_h
#define logging_h

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
#else
    #define LOGS(x)
    #define LOGSLN(x)
    #define LOGN(x)
    #define LOGNLN(x)
#endif

#endif