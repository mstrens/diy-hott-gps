/*
  DJI Naza (v1, v1 Lite, V2) data decoder library
  (c) Pawelsky 20131009
  Not for commercial use

  Refer to naza_decoder_wiring.jpg diagram for proper connection
*/

#ifndef NazaDecoderLib_h
#define NazaDecoderLib_h

#include "Arduino.h"

#define NAZA_MESSAGE_NONE    0x00
#define NAZA_MESSAGE_GPS     0x10
#define NAZA_MESSAGE_COMPASS 0x20

class NazaDecoderLib
{
  private:
    int gpsPayload[58];
    int seq;
    int cnt;
    int msgId;
    int msgLen;

    typedef struct
    {
      double  lon;     // longitude in degree decimal
      double  lat;     // latitude in degree decimal
      double  alt;     // altitude in m
      double  spd;     // speed in knots
      uint8_t fix; 
      uint8_t sat;
      double  heading; // heading in degrees
    } tGpsData;  
    tGpsData gpsData;

    typedef struct
    {
      int8_t   edge;
      uint32_t riseTime;
      uint32_t fallTime;
      uint32_t lastGoodWidth;
    } tPwmData;
    tPwmData pwmData[2];

    volatile uint8_t pcIntLast;
    
    int32_t decodeLong(uint8_t idx, uint8_t mask);
    int16_t decodeShort(uint8_t idx, uint8_t mask);
    void    startPwmReader();
    int8_t  pwm2Deg(uint32_t pulseWidth);

  public:
    NazaDecoderLib();
    void   pwmInterruptHandler();

    uint8_t decode(int input);
    double getLat();
    double getLon();
    double getAlt();
    double getSpeed();
    uint8_t getFixType();
    uint8_t getNumSat();
    double getHeading();
    int8_t  getPitch();
    int8_t  getRoll();
    bool   motorsArmed();
};

extern NazaDecoderLib NazaDecoder;

#endif

