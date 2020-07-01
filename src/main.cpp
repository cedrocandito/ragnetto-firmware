#include <Arduino.h>
#include "ragnetto_framework.h"
#include "ragnetto_config.h"
#include "logging.h"
#include "serial.h"

Ragnetto ragnetto;

void setup()
{
  #ifndef BLUETOOTH_SERIAL 
  /* For some reason the serial port must be initialized immediately or else it will not work. */
  Serial.begin(HARDWARE_SERIAL_BAUDRATE);
  delay(300);
  #endif

  setup_hardware();
  configuration.read();
  ragnetto_serial.send_info(F("Ragnetto is ready."));
}

void loop()
{
  ragnetto.run();
}
