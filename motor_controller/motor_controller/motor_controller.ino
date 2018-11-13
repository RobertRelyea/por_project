// Software Serial Sample
// Copyright (c) 2012 Dimension Engineering LLC
// See license.txt for license details.

#include <SoftwareSerial.h>
#include <SabertoothSimplified.h>

SoftwareSerial SWSerial(12, 11); // RX on no pin (unused), TX on pin 11 (to S1).
SabertoothSimplified ST(SWSerial); // Use SWSerial as the serial port.

void setup()
{
  SWSerial.begin(9600);
  Serial.begin(9600);
  Serial2.begin(9600); 
}

void loop()
{
  int powe;
  
  // Ramp from -127 to 127 (full reverse to full forward), waiting 20 ms (1/50th of a second) per value.
  for (powe = -127; powe <= 127; powe ++)
  {
    ST.motor(1, powe);
    delay(20);
  }
  
  // Now go back the way we came.
  for (powe = 127; powe >= -127; powe --)
  {
    ST.motor(1, powe);
    delay(20);
  }
}

