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

  int32_t latitudeDegrees;
  int32_t longitudeDegrees;

  uint16_t altitude;

  uint16_t speed;

  uint8_t GPS_update;
  uint8_t GPS_Frame;
  uint16_t GPS_ground_course;                       //                   - unit: degree*10

  uint8_t fixquality;
  uint8_t satellites;

  uint32_t error;

} gps_t;

void USART2_TX(unsigned char data);

void USART2_TX_str(char *str);

void gps_Init(void);

//Function prototypes for GPS frame parsing
bool GPS_newFrame(uint8_t c);
uint32_t GPS_coord_to_degrees(char* s);
uint16_t grab_fields(char* src, uint8_t mult);
uint8_t hex_c(uint8_t n);

uint8_t parseHex(char c);

#endif /* GPS_H_ */
