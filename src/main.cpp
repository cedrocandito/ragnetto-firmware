#include <Arduino.h>
#include "ragnetto_framework.h"
#include "logging.h"

Ragnetto ragnetto;

void setup()
{
  Serial.begin(TERMINAL_BAUD);
  setup_hardware();
}


#define STEP 5
Point3d p(-50.0, 0.0, -90.0);
void loop()
{
  if (Serial && Serial.available())
  {
    int c = Serial.read();
    switch(c)
    {
      case '+': p.z+=STEP;
        break;
      case '-': p.z-=STEP;
        break;
      case 'a': p.x-=STEP;
        break;
      case 'd': p.x+=STEP;
        break;
      case 'w': p.y+=STEP;
        break;
      case 's': p.y-=STEP;
        break;
    }
    ragnetto.legs[1].moveTo(p);

  }
}