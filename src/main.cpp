#include <Arduino.h>
#include "ragnetto_framework.h"
#include "ragnetto_config.h"
#include "logging.h"
#include "serial.h"

Ragnetto ragnetto;

void setup()
{
  ragnetto_serial.begin();
  setup_hardware();
  configuration.read();
}

void loop()
{
  ragnetto.run();
}
