/*
 * GPS.c
 *
 *  Created on: 2019. 1. 23.
 *      Author: WANG
 */

//#include "GPS.h"
#include "Board.h"
gps_t GPS;

void USART2_TX(unsigned char data){while(!(USART2->SR&0x40)); USART2->DR=data;}
void USART2_TX_str(char *str){while(*str){USART2_TX(*str++);}}

void gps_Init(void)
{

}

#define FRAME_GGA  1
#define FRAME_RMC  2

bool GPS_newFrame(uint8_t c) {
  uint32_t time = 0;
  float timef = 0;
  uint8_t frameOK = 0;
  static uint8_t param = 0, offset = 0, parity = 0;
  static char string[15];
  static uint8_t checksum_param, frame = 0;

  if (c == '$') {
    param = 0; offset = 0; parity = 0;
  } else if (c == ',' || c == '*') {
    string[offset] = 0;
    if (param == 0) { //frame identification
      frame = 0;
      if (string[0] == 'G' && string[1] == 'N' && string[2] == 'G' && string[3] == 'G' && string[4] == 'A') frame = FRAME_GGA;
      if (string[0] == 'G' && string[1] == 'N' && string[2] == 'R' && string[3] == 'M' && string[4] == 'C') frame = FRAME_RMC;
    } else if (frame == FRAME_GGA) {
      if      (param == 1){
        timef = atof(string);
        //time = grab_fields(string_test,0);
        time = (uint32_t) timef;
        GPS.hour = (time / 10000)+TD;
        GPS.minute = (time % 10000) / 100;
        GPS.seconds = (time % 100);
        GPS.milliseconds = fmod(timef, 1.0) * 1000;
      }
      else if (param == 2)                     {GPS.latitudeDegrees = GPS_coord_to_degrees(string);}
      else if (param == 3 && string[0] == 'S') GPS.latitudeDegrees = -GPS.latitudeDegrees;
      else if (param == 4)                     {GPS.longitudeDegrees = GPS_coord_to_degrees(string);}
      else if (param == 5 && string[0] == 'W') GPS.longitudeDegrees = -GPS.longitudeDegrees;
      else if (param == 6)                     {f.GPS_FIX = (string[0]  > '0');}
      else if (param == 7)                     {GPS.satellites = grab_fields(string,0);}
      else if (param == 9)                     {GPS.altitude = grab_fields(string,0);}  // altitude in meters added by Mis
    } else if (frame == FRAME_RMC) {
      if      (param == 7)                     {GPS.speed = ((uint32_t)grab_fields(string,1)*5144L)/1000L;}  //gps speed in cm/s will be used for navigation
      else if (param == 8)                     {GPS.GPS_ground_course = grab_fields(string,1); }                 //ground course deg*10
    }
    param++; offset = 0;
    if (c == '*') checksum_param=1;
    else parity ^= c;
  } else if (c == '\r' || c == '\n') {
    if (checksum_param) { //parity checksum
      uint8_t checksum = hex_c(string[0]);
      checksum <<= 4;
      checksum += hex_c(string[1]);
      if (checksum == parity) frameOK = 1;
    }
    checksum_param=0;
  } else {
     if (offset < 15) string[offset++] = c;
     if (!checksum_param) parity ^= c;
  }
  return frameOK && (frame==FRAME_GGA);
}

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
    if (c < '0')
      return 0;
    if (c <= '9')
      return c - '0';
    if (c < 'A')
       return 0;
    if (c <= 'F')
       return (c - 'A')+10;
    // if (c > 'F')
    return 0;
}

#define DIGIT_TO_VAL(_x)        (_x - '0')
uint32_t GPS_coord_to_degrees(char* s) {
  char *p, *q;
  uint8_t deg = 0, min = 0;
  unsigned int frac_min = 0;
  uint8_t i;

  // scan for decimal point or end of field
  for (p = s; isdigit(*p); p++) ;
  q = s;

  // convert degrees
  while ((p - q) > 2) {
    if (deg)
      deg *= 10;
    deg += DIGIT_TO_VAL(*q++);
  }
  // convert minutes
  while (p > q) {
    if (min)
      min *= 10;
    min += DIGIT_TO_VAL(*q++);
  }
  // convert fractional minutes
  // expect up to four digits, result is in
  // ten-thousandths of a minute
  if (*p == '.') {
    q = p + 1;
    for (i = 0; i < 4; i++) {
      frac_min *= 10;
      if (isdigit(*q))
        frac_min += *q++ - '0';
    }
  }
  return deg * 10000000UL + (min * 1000000UL + frac_min*100UL) / 6;
}

// helper functions
uint16_t grab_fields(char* src, uint8_t mult) {  // convert string to uint16
  uint8_t i;
  uint16_t tmp = 0;

  for(i=0; src[i]!=0; i++) {
    if(src[i] == '.') {
      i++;
      if(mult==0){
        break;
      }else{
        src[i+mult] = 0;
      }
    }
    tmp *= 10;
    if(src[i] >='0' && src[i] <='9') tmp += src[i]-'0';
  }
  return tmp;
}

uint8_t hex_c(uint8_t n) {    // convert '0'..'9','A'..'F' to 0..15
  n -= '0';
  if(n>9)  n -= 7;
  n &= 0x0F;
  return n;
}
