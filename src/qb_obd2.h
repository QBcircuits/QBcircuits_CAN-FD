/***************************************************
  This is our library for the QBcircuits Arduino Pro mini/micro CAN FD shield
  ----> https://www.qbcircuits.com

  Check out the link above for our tutorials, wiring diagrams
  QBcircuits invests time and effort providing this open source code,
  please support QBcircuits and open-source hardware by purchasing
  products from QBcircuits - GoLong!

  Written by Brother Q for QBcircuits
  All text above must be included in any redistribution
 ****************************************************/

#ifndef QB_OBD2_H
#define QB_OBD2_H

#if ARDUINO>=100
#include <Arduino.h> // Arduino 1.0
#else
#include <Wprogram.h> // Arduino 0022
#endif

void obd2s1PidDecrypt(uint8_t *ptrBuf_u8);

#endif
