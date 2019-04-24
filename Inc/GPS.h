/*
 * GPS.h
 *
 *  Created on: 2019. 1. 23.
 *      Author: WANG
 */

#ifndef GPS_H_
#define GPS_H_

#define TD 9 //KOREA 시차 +9시간

typedef struct gps_t {
  char GPS[120];

  uint8_t hour;
  uint8_t minute;
  uint8_t seconds;
  uint8_t year;
  uint8_t month;
  uint8_t day;

  uint16_t milliseconds;

  float latitude;
  float longitude;

  uint32_t latitude_fixed;
  uint32_t longitude_fixed;

  float latitudeDegrees;
  float longitudeDegrees;

  float geoidheight;
  float altitude;

  float speed;
  float angle;
  float magvariation;
  float HDOP;

  char lat;
  char lon;
  char mag;

  bool fix;

  uint8_t fixquality;
  uint8_t satellites;

  uint8_t lineidx;
  uint8_t recvdflag;

  uint32_t error;

} gps_t;

void USART2_TX(unsigned char data);

void USART2_TX_str(char *str);

void gps_Init(void);

bool gps_parse(void);

uint8_t parseHex(char c);

#endif /* GPS_H_ */
