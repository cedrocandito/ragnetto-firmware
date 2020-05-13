#include <Arduino.h>
#include "ragnetto_framework.h"
#include "logging.h"

Ragnetto ragnetto;

void setup()
{
  Serial.begin(TERMINAL_BAUD);
  setup_hardware();
}


void loop()
{
  float a = (millis() % (int)(PI*1000)) * 2.0 / 1000.0 * 2.0;
  float x = cos(a) * 30.0;
  float y = sin(a) * 30.0;
  float z = -85.0;
  for (uint8_t i = 0; i<NUM_LEGS; i++)
  {
    Leg *l = &ragnetto.legs[i];
    Point3d p(
      x + cos(l->attachmentAngle)*(LEG_SEGMENT_1_LENGTH + LEG_SEGMENT_2_LENGTH/3+30),
      y+sin(l->attachmentAngle)*(LEG_SEGMENT_1_LENGTH + LEG_SEGMENT_2_LENGTH/3+30),
      z);
    l->moveTo(p);
  }
  delay(10);
}

/*

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
*/