#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "ragnetto_framework.h"
#include "ragnetto_hardware.h"


void setup()
{
  Serial.begin(TERMINAL_BAUD);
  setup_hardware();
}

void loop()
{
  // put your main code here, to run repeatedly:
}