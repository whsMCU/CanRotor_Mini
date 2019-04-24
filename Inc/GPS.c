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

bool gps_parse(void)
{
 while(QueueAvailable(&Q_buffer[0]) > 0 && GPS.recvdflag == 0){
   volatile char c = read_Q(&Q_buffer[UART1]);
   if(c == '$'){
     GPS.lineidx = 0;
     for(int i = 0; i < 120; i++){
       GPS.GPS[i] = '\0';
     }
   }
   GPS.GPS[GPS.lineidx]=c;

   if(c == '\n'){
     GPS.lineidx = 0;
     GPS.recvdflag = 1;
   }
   GPS.lineidx++;
 }
 if(GPS.recvdflag==1){
   GPS.recvdflag = 0;

   // first look if we even have one
   if (GPS.GPS[strlen(GPS.GPS)-5] == '*'){
     uint16_t sum = parseHex(GPS.GPS[strlen(GPS.GPS)-4]) * 16;
     sum += parseHex(GPS.GPS[strlen(GPS.GPS)-3]);

     // check checksum
     for (uint8_t i=1; i < (strlen(GPS.GPS)-5); i++) {
       sum ^= GPS.GPS[i];
     }
     if (sum != 0) {
       // bad checksum :(
       GPS.error++;
       return false;
     }
   }else return false;

   uint32_t degree;
   long minutes;
   char degreebuff[10];

      if (strstr(GPS.GPS, "$GNGGA")){
        RGB_B_TOGGLE;
//        sprintf(Buf, "%s", GPS.GPS);
//        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
       char *p = GPS.GPS;
       // get time
       p = strchr(p, ',')+1;
       float timef = atof(p);
       uint32_t time = timef;
       GPS.hour = (time / 10000)+TD;
       GPS.minute = (time % 10000) / 100;
       GPS.seconds = (time % 100);
       GPS.milliseconds = fmod(timef, 1.0) * 1000;

       // parse out latitude
       p = strchr(p, ',')+1;
       if (',' != *p){

         strncpy(degreebuff, p, 2);
         p += 2;
         degreebuff[2] = '\0';
         degree = atol(degreebuff) * 10000000;
         strncpy(degreebuff, p, 2); // minutes
         p += 3; // skip decimal point
         strncpy(degreebuff + 2, p, 4);
         degreebuff[6] = '\0';
         minutes = 50 * atol(degreebuff) / 3;
         GPS.latitude_fixed = degree + minutes;
         GPS.latitude = degree / 100000 + minutes * 0.000006F;
         GPS.latitudeDegrees = (GPS.latitude-100*(int)(GPS.latitude/100))/60.0;
         GPS.latitudeDegrees += (int)(GPS.latitude/100);
        }
        p = strchr(p, ',')+1;
        if (',' != *p){
         if (p[0] == 'S') GPS.latitudeDegrees *= -1.0;
         if (p[0] == 'N') GPS.lat = 'N';
         else if (p[0] == 'S') GPS.lat = 'S';
         else if (p[0] == ',') GPS.lat = 0;
         else return 0;
        }

        // parse out longitude
        p = strchr(p, ',')+1;
        if (',' != *p){
          strncpy(degreebuff, p, 3);
          p += 3;
          degreebuff[3] = '\0';
          degree = atol(degreebuff) * 10000000;
          strncpy(degreebuff, p, 2); // minutes
          p += 3; // skip decimal point
          strncpy(degreebuff + 2, p, 4);
          degreebuff[6] = '\0';
          minutes = 50 * atol(degreebuff) / 3;
          GPS.longitude_fixed = degree + minutes;
          GPS.longitude = degree / 100000 + minutes * 0.000006F;
          GPS.longitudeDegrees = (GPS.longitude-100*(int)(GPS.longitude/100))/60.0;
          GPS.longitudeDegrees += (int)(GPS.longitude/100);
         }

          p = strchr(p, ',')+1;
          if (',' != *p){
            if (p[0] == 'W') GPS.longitudeDegrees *= -1.0;
            if (p[0] == 'W') GPS.lon = 'W';
            else if (p[0] == 'E') GPS.lon = 'E';
            else if (p[0] == ',') GPS.lon = 0;
            else return FALSE;
           }

           p = strchr(p, ',')+1;
           if (',' != *p){
             GPS.fixquality = atoi(p);
           }

           p = strchr(p, ',')+1;
           if (',' != *p){
             GPS.satellites = atoi(p);
           }

           p = strchr(p, ',')+1;
           if (',' != *p){
            GPS.HDOP = atof(p);
           }

           p = strchr(p, ',')+1;
           if (',' != *p){
            GPS.altitude = atof(p);
           }

           p = strchr(p, ',')+1;
           p = strchr(p, ',')+1;
           if (',' != *p){
             GPS.geoidheight = atof(p);
           }
            return TRUE;
      }
      if (strstr(GPS.GPS, "$GNRMC")) {
//        sprintf(Buf, "%s", GPS.GPS);
//        HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
       // found RMC
        char *p = GPS.GPS;

        // get time
        p = strchr(p, ',')+1;
        float timef = atof(p);
        uint32_t time = timef;
        GPS.hour = (time / 10000) + TD;
        GPS.minute = (time % 10000) / 100;
        GPS.seconds = (time % 100);

        GPS.milliseconds = fmod(timef, 1.0) * 1000;

        p = strchr(p, ',')+1;
        // Serial.println(p);
        if (p[0] == 'A')
          GPS.fix = true;
        else if (p[0] == 'V')
          GPS.fix = false;
        else
          return false;

        // parse out latitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          strncpy(degreebuff, p, 2);
          p += 2;
          degreebuff[2] = '\0';
          long degree = atol(degreebuff) * 10000000;
          strncpy(degreebuff, p, 2); // minutes
          p += 3; // skip decimal point
          strncpy(degreebuff + 2, p, 4);
          degreebuff[6] = '\0';
          long minutes = 50 * atol(degreebuff) / 3;
          GPS.latitude_fixed = degree + minutes;
          GPS.latitude = degree / 100000 + minutes * 0.000006F;
          GPS.latitudeDegrees = (GPS.latitude-100*(int)(GPS.latitude/100))/60.0;
          GPS.latitudeDegrees += (int)(GPS.latitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          if (p[0] == 'S') GPS.latitudeDegrees *= -1.0;
          if (p[0] == 'N') GPS.lat = 'N';
          else if (p[0] == 'S') GPS.lat = 'S';
          else if (p[0] == ',') GPS.lat = 0;
          else return false;
        }

        // parse out longitude
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          strncpy(degreebuff, p, 3);
          p += 3;
          degreebuff[3] = '\0';
          degree = atol(degreebuff) * 10000000;
          strncpy(degreebuff, p, 2); // minutes
          p += 3; // skip decimal point
          strncpy(degreebuff + 2, p, 4);
          degreebuff[6] = '\0';
          minutes = 50 * atol(degreebuff) / 3;
          GPS.longitude_fixed = degree + minutes;
          GPS.longitude = degree / 100000 + minutes * 0.000006F;
          GPS.longitudeDegrees = (GPS.longitude-100*(int)(GPS.longitude/100))/60.0;
          GPS.longitudeDegrees += (int)(GPS.longitude/100);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          if (p[0] == 'W') GPS.longitudeDegrees *= -1.0;
          if (p[0] == 'W') GPS.lon = 'W';
          else if (p[0] == 'E') GPS.lon = 'E';
          else if (p[0] == ',') GPS.lon = 0;
          else return false;
        }
        // speed
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          GPS.speed = atof(p);
        }

        // angle
        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          GPS.angle = atof(p);
        }

        p = strchr(p, ',')+1;
        if (',' != *p)
        {
          uint32_t fulldate = atof(p);
          GPS.day = fulldate / 10000;
          GPS.month = (fulldate % 10000) / 100;
          GPS.year = (fulldate % 100);
        }
        // we dont parse the remaining, yet!
        return true;
      } return false;
    }
 return true;
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
