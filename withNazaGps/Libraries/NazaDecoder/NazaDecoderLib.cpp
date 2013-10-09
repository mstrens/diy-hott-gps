/*
  DJI Naza (v1, v1 Lite, V2) data decoder library
  (c) Pawelsky 20131009
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#include "Arduino.h"
#include "NazaDecoderLib.h"

NazaDecoderLib NazaDecoder;

NazaDecoderLib::NazaDecoderLib()
{
  seq = 0;
  cnt = 0;
  msgId = 0;
  msgLen = 0;
  startPwmReader();
}

int32_t NazaDecoderLib::decodeLong(uint8_t idx, uint8_t mask)
{
  union { uint32_t l; uint8_t b[4]; } val;
  for(int i = 0; i < 4; i++) val.b[i] = gpsPayload[idx + i] ^ mask;
  return val.l;
}

int16_t NazaDecoderLib::decodeShort(uint8_t idx, uint8_t mask)
{
  union { uint16_t s; uint8_t b[2]; } val;
  for(int i = 0; i < 2; i++) val.b[i] = gpsPayload[idx + i] ^ mask;
  return val.s;
}

double NazaDecoderLib::getLat() { return gpsData.lat; }
double NazaDecoderLib::getLon() { return gpsData.lon; }
double NazaDecoderLib::getAlt() { return gpsData.alt; }
double NazaDecoderLib::getSpeed() { return gpsData.spd; }
uint8_t NazaDecoderLib::getFixType() { return gpsData.fix; }
uint8_t NazaDecoderLib::getNumSat() { return gpsData.sat; }
double NazaDecoderLib::getHeading() { return gpsData.heading; }

uint8_t NazaDecoderLib::decode(int input)
{ 
  if((seq == 0) && (input == 0x55)) { seq++; }                                           // header (part 1 - 0x55)
  else if((seq == 1) && (input == 0xAA)) { seq++; }                                      // header (part 2 - 0xAA) 
  else if((seq == 2) && ((input == 0x10) || (input == 0x20))) { seq++; msgId = input; }  // message id
  else if(seq == 3) { seq++; msgLen = input; cnt = 0; }                                  // message payload lenght
  else if((seq == 4) && (cnt < msgLen)) { gpsPayload[cnt++] = input; }                   // store payload in buffer
  else if((seq == 4) || (seq == 5)) { seq++; }                                           // skip 2 checksum fields
  else seq = 0;

  if(seq == 6) // all data in buffer
  {
    seq = 0;
    // Decode GPS data
    if(msgId == NAZA_MESSAGE_GPS)
    {
      uint8_t mask = gpsPayload[55];
      gpsData.lon  = (double)decodeLong(4, mask) / 10000000;
      gpsData.lat  = (double)decodeLong(8, mask) / 10000000;
      gpsData.alt  = (double)decodeLong(12, mask) / 1000;
      double nVel  = (double)decodeLong(28, mask) / 100; 
      double eVel  = (double)decodeLong(32, mask) / 100;
      gpsData.spd  = sqrt(nVel * nVel + eVel * eVel) * 1.943884; // convert from m/s to knots
      gpsData.sat  = gpsPayload[48];
      gpsData.fix  = gpsPayload[50];
    }
    // Decode compass data (not tilt compensated)
    else if (msgId == NAZA_MESSAGE_COMPASS)
    {
      uint8_t mask = gpsPayload[5];
      int16_t x = decodeShort(0, mask); x ^= 0x0100;
      int16_t y = decodeShort(2, mask); y ^= 0x0100;
      gpsData.heading = -atan2(y, x) * 180.0 / M_PI;
      if(gpsData.heading < 0) gpsData.heading += 360.0; 
    }
    return msgId;
  }
  else
  {
    return NAZA_MESSAGE_NONE;
  }
}

#define PIN_MASK     0b00001100

void NazaDecoderLib::startPwmReader()
{
  pinMode(2, INPUT_PULLUP); // Pitch (Arduino D2 <-> Naza F2)
  pinMode(3, INPUT_PULLUP); // Roll  (Arduino D3 <-> Naza F1)
  cli();
  PCICR = 1 << PCIE2;
  PCMSK2 = PIN_MASK;
  sei();
}

void NazaDecoderLib::pwmInterruptHandler()
{
  uint8_t bit;
  uint8_t curr;
  uint8_t mask;
  uint32_t currentTime;
  uint32_t time;

  // get the pin states for the indicated port.
  curr = PIND & PIN_MASK;
  mask = curr ^ pcIntLast;
  pcIntLast = curr;

  currentTime = micros();

  // mask is pcint pins that have changed.
  for (uint8_t i = 0; i < 2; i++) {
    bit = 0b00000100 << i;
    if (bit & mask) {
      // for each pin changed, record time of change
      if (bit & pcIntLast) {
        time = currentTime - pwmData[i].fallTime;
        pwmData[i].riseTime = currentTime;
        if ((time >= 10000) && (time <= 26000))
          pwmData[i].edge = 1;
        else
          pwmData[i].edge = 0; // invalid rising edge detected
      }
      else {
        time = currentTime - pwmData[i].riseTime;
        pwmData[i].fallTime = currentTime;
        if ((time >= 800) && (time <= 2200) && (pwmData[i].edge == 1)) {
          pwmData[i].lastGoodWidth = time;
          pwmData[i].edge = 0;
        }
      }
    }
  }
}

ISR(PCINT2_vect)
{
  NazaDecoder.pwmInterruptHandler();
}

int8_t NazaDecoderLib::pwm2Deg(uint32_t pulseWidth)
{
  return (pulseWidth == 0) ? 0 : map(constrain(pulseWidth, 1000, 2000), 1000, 2000, -90, 90);
}

int8_t  NazaDecoderLib::getPitch()
{
  return pwm2Deg(pwmData[0].lastGoodWidth);
}

int8_t  NazaDecoderLib::getRoll()
{
  return pwm2Deg(pwmData[1].lastGoodWidth);
}
