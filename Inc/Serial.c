//#include "Serial.h"
#include "Board.h"
char Buf[128];

static volatile uint8_t serialHeadTX[UART_MAX_CH],serialTailTX[UART_MAX_CH];
static uint8_t serialBufferTX[TX_BUFFER_SIZE][UART_MAX_CH];
static uint8_t serialBufTx_0[TX_BUFFER_SIZE];
static uint8_t serialBufTx_1[TX_BUFFER_SIZE];
static volatile uint8_t serialHead_0, serialHead_1;
static uint8_t CURRENTPORT=0;

volatile unsigned char command=0;
volatile unsigned char m = 0;
int MSP_TRIM[3]={0, 0, 0};

uint8_t Debug_TC=0;

uint8_t telemetry_loop_counter = 0;
uint16_t time=0, time1=0, aftertime=0;

////////////////////////////////////////////

uint8_t rx1_buffer[1];
uint8_t rx2_buffer[1];

//////////// MSP //////////////////
#define INBUF_SIZE 128
typedef struct mspPortState_t {
//    serialPort_t *port;
    uint8_t checksum;
    uint8_t indRX;
    uint8_t inBuf[INBUF_SIZE];
    uint8_t cmdMSP;
    uint8_t offset;
    uint8_t dataSize;
    serialState_t c_state;
} mspPortState_t;

static mspPortState_t ports[2];
static mspPortState_t *currentPortState = &ports[0];

///////////////////////////////////////////////////////

int fputc(int ch, FILE *f) // for printf
{
   uint8_t tmp[1]={ch};
   HAL_UART_Transmit(&huart1, tmp, 1, 1);
	 //HAL_UART_Transmit_DMA(&huart1, tmp, 1);
   return(ch);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) //current USART
		{
			write_Q(&Q_buffer[UART1], rx1_buffer[0]);
			//TX2_CHR(rx1_buffer[0]);
		}
		
	if(huart->Instance == USART2) //current USART
		{
			write_Q(&Q_buffer[UART2], rx2_buffer[0]);
			//printf("c %d",rx2_buffer[0]);
			//HAL_UART_Transmit_IT(&huart1, (uint8_t*)rx2_buffer, 100);
			//TX_CHR(rx2_buffer[0]);
		}
}

void TX_CHR(char ch){
	while(!(USART1->SR & 0x80));
  USART1->DR = ch;

}
void TX2_CHR(char ch){
  while(!(USART2->SR & 0x80));
  USART2->DR = ch;
}

///////////////////////////////////////////////////
void serialize8(uint8_t a)
{
  SerialSerialize(CURRENTPORT,a);
  //TX2_CHR(a);
  currentPortState->checksum ^= (a & 0xFF);
}

void serialize16(int16_t a)
{
    serialize8((a   ) & 0xFF);
    serialize8((a>>8) & 0xFF);
}

void serialize32(uint32_t a)
{
    serialize8((a    ) & 0xFF);
    serialize8((a>> 8) & 0xFF);
    serialize8((a>>16) & 0xFF);
    serialize8((a>>24) & 0xFF);
}

uint8_t read8(void)
{
    return currentPortState->inBuf[currentPortState->indRX++] & 0xff;
}

uint16_t read16(void)
{
    uint16_t t = read8();
    t += (uint16_t)read8() << 8;
    return t;
}

uint32_t read32(void)
{
    uint32_t t = read16();
    t += (uint32_t)read16() << 16;
    return t;
}

void headSerial(uint8_t err, uint8_t s, uint8_t cmdMSP)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    currentPortState->checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(cmdMSP);
}

void headSerialSend(uint8_t s, uint8_t cmdMSP)
{
    headSerial(0, s, cmdMSP);
}

void headSerialResponse(uint8_t err, uint8_t s)
{
    serialize8('$');
    serialize8('M');
    serialize8(err ? '!' : '>');
    currentPortState->checksum = 0;               // start calculating a new checksum
    serialize8(s);
    serialize8(currentPortState->cmdMSP);
}

void headSerialReply(uint8_t s)
{
    headSerialResponse(0, s);
}

void headSerialError(uint8_t s)
{
    headSerialResponse(1, s);
}

void tailSerialReply(void)
{
  SerialSerialize(CURRENTPORT,currentPortState->checksum);
  UartSendData(CURRENTPORT);
  //serialize8(currentPortState->checksum);
}

void s_struct_partial(uint8_t *cb,uint8_t siz) {
  while(siz--) serialize8(*cb++);
}

void s_struct(uint8_t *cb,uint8_t siz) {
  headSerialReply(siz);  //530
  s_struct_partial(cb,siz); //870
  tailSerialReply(); //170
}
///////////////////////////////////////////////////

void PrintData(uint8_t command)
{
  Debug_TC++;
  if(Debug_TC >= 12){ //12
    Debug_TC = 0;
    LED1_TOGGLE;  //GREEN
#ifdef SSD1306
    //clearDisplay();
    OLed_printf(0, 0, "CanRotor_Mini");
    OLed_printf(0, 16, "ROLL : %2.1f 도", imu.AHRS[ROLL]);
    OLed_printf(0, 32, "PITCH: %2.1f 도", imu.AHRS[PITCH]);
    OLed_printf(0, 48, "YAW  : %2.1f 도", imu.AHRS[YAW]);
    display();

//    for (int i=0; i<3; i++){
//    	imu.AHRS_DP[i] = map(imu.AHRS[i], -90, 90, 1, 60);
//    	fillRect(32*i, 64-imu.AHRS_DP[i], 10, imu.AHRS_DP[i], WHITE);
//    }
//
//    fillRect(64-5 + (imu.AHRS_DP[1]-imu.AHRS_DP[2])/1, 4, 10, 4, WHITE);
//    display();
#endif

	switch(command)
	{

	case 0:
		sprintf(Buf, "[1]9250 [3]Radio [4]Motor [5]Angle [6]PID [9]IMU [p]Kp [i]Ki [d]Kd [q,w,e] [z,x,c] \r\n ");
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
		//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;


	case 1:
//	     sprintf(Buf, " acc (%4.2f), (%4.2f), (%4.2f) / gyro (%4.2f), (%4.2f), (%4.2f) / mag (%3.f), (%3.f), (%3.f) / AHRS:(%4.f)(%4.f)(%4.f), (%4.2f) \r\n",
//	                    imu.accRaw[ROLL], imu.accRaw[PITCH], imu.accRaw[YAW], imu.gyroRaw[ROLL], imu.gyroRaw[PITCH], imu.gyroRaw[YAW], imu.magRaw[ROLL], imu.magRaw[PITCH], imu.magRaw[YAW], imu.AHRS[ROLL], imu.AHRS[PITCH], imu.gyroYaw, imu.AHRS[YAW]);
	      sprintf(Buf, " %d, %d, %d, %d, %d \r\n",
	                      f.ARMED, f.HEADFREE_MODE, f.ANGLE_MODE, f.HORIZON_MODE, f.ACRO_MODE);
//	  sprintf(Buf, " %.0f,%.0f,%.0f,%.0f.",imu.AHRS[ROLL], imu.AHRS[PITCH], imu.AHRS[YAW],imu.actual_compass_heading);
	     HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
	     break;

	case 2:
		sprintf(Buf, " magBias_x: (%3.2f), magBias_y: (%3.2f), magBias_z: (%3.2f)\r\n",
		        imu.accRaw[ROLL], imu.accRaw[PITCH], imu.accRaw[YAW]);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));

	    break;

	case 3:
	 // sprintf(Buf, "latDeg : %f, lat : %c, lonDeg : %f, lon : %c, fixQ: %d, satel : %d, altitude : %.2fM, geohi : %.2fM \n",
	 //         GPS.latitudeDegrees, GPS.lat, GPS.longitudeDegrees, GPS.lon, GPS.fixquality, GPS.satellites, GPS.altitude, GPS.geoidheight);
	  sprintf(Buf, "Y : %2d, M : %2d, D : %2d, H: %2d, min : %2d, sec : %2d, mil : %3d, speed : %.2f, update : %d\n",
	          GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds, GPS.speed, GPS.GPS_update);
    //sprintf(Buf, "latDeg : %f, lonDeg : %f \r\n", GPS.latitudeDegrees, GPS.longitudeDegrees);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
		break;

	case 4:
		sprintf(Buf, " %d %d %d %d\r\n", motor[0], motor[1], motor[2], motor[3]);
		HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 5:
//		sprintf(Buf, "motor:(%4.d)(%4.d)(%4.d)(%4.d), AHRS:(%4.f)(%4.f)(%4.f), RC:(%4.d)(%4.d)(%4.d)(%4.d)(%4.d)(%4.d), VBAT: (%4.1f), ARMED: (%d), Tuning : (%d), Headfree: (%d), %d \r\n",
//	  motor[0], motor[1], motor[2], motor[3], imu.AHRS[ROLL], imu.AHRS[PITCH], imu.gyroYaw, RC.rcCommand[ROLL], RC.rcCommand[PITCH], RC.rcCommand[YAW], RC.rcCommand[THROTTLE], RC.rcCommand[GEAR], RC.rcCommand[AUX1], BAT.VBAT, f.ARMED, f.Tuning_MODE, f.HEADFREE_MODE, test.VBAT_Compensat1);
	  sprintf(Buf, "AHRS:(%4.f)(%4.f)(%4.f), ARMED: (%d), Headfree: (%d), cycleTime : %d, %d, %d, error : %d, %d, %3.1f, %3.1f, %d\r\n",
	    imu.AHRS[ROLL], imu.AHRS[PITCH], imu.gyroRaw[YAW], f.ARMED, f.HEADFREE_MODE, cycleTime, cycleTimeMin, cycleTimeMax, Error.error, overrun_count, imu.actual_compass_heading, imu.AHRS[YAW], alt.EstAlt);
//		sprintf(Buf, "RC:(%4.d)(%4.d)(%4.d)(%4.d)(%4.d)(%4.d)\r\n",
//	   RC.rcCommand[ROLL], RC.rcCommand[PITCH], RC.rcCommand[YAW], RC.rcCommand[THROTTLE], RC.rcCommand[GEAR], RC.rcCommand[AUX1]);
//    sprintf(Buf, "Mag:(%5.f)(%5.f)(%5.f), AHRS:(%4.f)(%4.f)(%4.f), RC:(%4.d)(%4.d)(%4.d)(%4.d), (%4.d) (%4.2f), ARMED: (%2.1d), MS5611 : %.2f Pa , %.2f cm\r\n",
//            imu.magRaw[ROLL], imu.magRaw[PITCH], imu.magRaw[YAW], imu.AHRS[ROLL], imu.AHRS[PITCH], imu.AHRS[YAW], RC.rcCommand[ROLL], RC.rcCommand[PITCH], RC.rcCommand[YAW], RC.rcCommand[THROTTLE], BAT.VBAT_Sense, BAT.VBAT, f.ARMED, ms5611.actual_pressure, ms5611.GroundAltitude);
    //sprintf(Buf,"Hour: %d, minute : %d, second : %d, milliseconds : %d\n", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
	//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 6:
    sprintf(Buf,"R[P]: %2.2f, P[P]: %2.2f, R[I]: %2.2f, P[I]: %2.2f, R[D]: %2.2f, P[D]: %2.2f, Y[P]: %2.2f, Y[I]: %2.2f, Y[D]: %2.2f, ARMED: (%d), Tuning : (%d)\r\n",
            pid.kp[ROLL], pid.kp[PITCH], pid.ki[ROLL], pid.ki[PITCH], pid.kd[ROLL], pid.kd[PITCH], pid.kp[YAW], pid.ki[YAW], pid.kd[YAW], f.ARMED, f.Tuning_MODE);
    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
		break;
	case 7:
		  sprintf(Buf, " state: %d, data: %d \n ", hdma_usart1_rx.State, rx1_buffer[0]);
		  HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
		  //HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 8:
		sprintf(Buf, "%f %f %f\r\n",pid.output2[ROLL], pid.output2[PITCH], pid.output2[YAW]);
		HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
		//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);

		break;
	case 9:
		sprintf(Buf, "Roll:(%.2f), Pitch:(%.2f), Yaw:(%.2f), rx_buffer:(%d)\r\n",AHRS.Roll, AHRS.Pitch, AHRS.Yaw, rx1_buffer[0]);
	     HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
	     //HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;

	case 10:
		sprintf(Buf, "Data : %d, %d, %d, %d, %d, %d, %d, %d \r\n ", loopTime, ms5611.realTemperature, (uint32_t)ms5611.realPressure, baroPressureSum, ms5611.BaroAlt, (int16_t)alt.EstAlt, f.ARMED, (int16_t)imu.actual_compass_heading);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));

		break;

     case 11:
			sprintf(Buf, "\r\n [KP]: %.2f, %.2f, %.2f \r\n ", pid.kp[0], pid.kp[1], pid.kp[2]);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
			//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;

	case 12:
			sprintf(Buf, "\r\n [KI]: %.2f, %.2f, %.2f\r\n", pid.ki[0], pid.ki[1], pid.ki[2]);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
			//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;

	case 13:
			sprintf(Buf, "\r\n [KD]: %.2f, %.2f, %.2f\r\n", pid.kd[0], pid.kd[1], pid.kd[2]);
			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
			//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 14:
		sprintf(Buf,"R/P/Y: %f %f %f\r\n",AHRS.Roll, AHRS.Pitch, AHRS.Yaw);
	     HAL_UART_Transmit_DMA(&huart1, (uint8_t*)Buf, strlen(Buf));
		break;
	 }
  }
}

void SerialCom(void) {
	uint8_t c;
  uint32_t timeMax; // limit max time in this function in case of GPS
  timeMax = micros();
	for(int i = 0; i < 2; i++){
    currentPortState = &ports[i];
    CURRENTPORT = i;
    while(QueueAvailable(&Q_buffer[i]) > 0){
	  c = read_Q(&Q_buffer[i]);
    if (currentPortState->c_state == IDLE) {
      currentPortState->c_state = (c=='$') ? HEADER_START : IDLE;
    } else if (currentPortState->c_state == HEADER_START) {
      currentPortState->c_state = (c=='M') ? HEADER_M : IDLE;
    } else if (currentPortState->c_state == HEADER_M) {
      currentPortState->c_state = (c=='<') ? HEADER_ARROW : IDLE;
    } else if (currentPortState->c_state == HEADER_ARROW) {
      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        currentPortState->c_state = IDLE;
        continue;
      }
        currentPortState->dataSize = c;
        currentPortState->offset = 0;
        currentPortState->indRX = 0;
        currentPortState->checksum = 0;
        currentPortState->checksum ^= c;
        currentPortState->c_state = HEADER_SIZE;
    } else if (currentPortState->c_state == HEADER_SIZE) {
      currentPortState->cmdMSP = c;
      currentPortState->checksum ^= c;
      currentPortState->c_state = HEADER_CMD;
    } else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset < currentPortState->dataSize) {
      currentPortState->checksum ^= c;
      currentPortState->inBuf[currentPortState->offset++] = c;
    } else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset >= currentPortState->dataSize) {
      if (currentPortState->checksum == c) {
				evaluateCommand();
      }// else{
//        sprintf(Buf, "invalid checksum for command : %d, expected checksum: %d, got checksum : %d, dataSize : %d\r\n ", currentPortState->cmdMSP, currentPortState->checksum, c, currentPortState->dataSize);
//        HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf),1000);
//        for (i=0; i<currentPortState->dataSize; i++) {
//          if (i!=0) {
//            sprintf(Buf, " ");
//            HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf),1000);
//          }
//          sprintf(Buf, "%d", currentPortState->inBuf[i]);
//          HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf),1000);
//        }
//        sprintf(Buf, "\r\n\r\n");
//        HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf),1000);
//      }
      currentPortState->c_state = IDLE;
    }
#ifdef GPS_Recive
    if(i == 0){
      static uint32_t GPS_last_frame_seen; //Last gps frame seen at this time, used to detect stalled gps communication
      if (GPS_newFrame(c)){
        //We had a valid GPS data frame, so signal task scheduler to switch to compute
        if (GPS.GPS_update == 1) GPS.GPS_update = 0; else GPS.GPS_update = 1; //Blink GPS update
        GPS_last_frame_seen = timeMax;
        GPS.GPS_Frame = 1;
      }
      // Check for stalled GPS, if no frames seen for 1.2sec then consider it LOST
      if ((timeMax - GPS_last_frame_seen) > 1200000) {
        //No update since 1200ms clear fix...
        f.GPS_FIX = 0;
        GPS.satellites = 0;
      }
    }
    if (micros()-timeMax>250) return;  // Limit the maximum execution time of serial decoding to avoid time spike
#endif
   }
  }
 }

 void evaluateCommand(void) {
	 uint8_t i=0;
	 uint32_t tmp=0;
	 switch(currentPortState->cmdMSP){
		 case MSP_ARM:
			 mwArm();
//		 sprintf(Buf, "LOCK : %d, %d, %d, %d, %d, ARMD : %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4], f.ARMED);
//    		//HAL_UART_Transmit_IT(&huart2, (uint8_t*)Buf, strlen(Buf));
//		 HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
			 break;
		 
		 case MSP_DISARM:
			 mwDisarm();
//				sprintf(Buf, "UNLOCK : %d, %d, %d, %d, %d, ARMD : %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4], f.ARMED);
//    		//HAL_UART_Transmit_IT(&huart2, (uint8_t*)Buf, strlen(Buf));
//		 HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
			 break;
		 
		 case MSP_RC_RAW:
				for(i=0; i < 5; i++){
					RC_Raw.rcCommand[i]  = read8();
				}
    		RC.rcCommand[ROLL]     = map(RC_Raw.rcCommand[ROLL], 0, 250, -20, 20)+ MSP_TRIM[ROLL]; //0~250 left:0, right:250
		    RC.rcCommand[PITCH]    = map(RC_Raw.rcCommand[PITCH], 0, 250, -20, 20)+ MSP_TRIM[PITCH]; //0~250 rear:0, fornt:250
		    RC.rcCommand[YAW]      = map(RC_Raw.rcCommand[YAW], 0, 250, -100, 100); //0~250 left:0, right:250
	      RC.rcCommand[THROTTLE] = map(RC_Raw.rcCommand[THROTTLE], 0, 250, 0, 1800);//0~250
	      RC.rcCommand[AUX1] 	   =  RC_Raw.rcCommand[GEAR];
			 break;
				
		 case MSP_RC:
		 {  struct {
		    uint16_t roll, pitch, yaw, throttle, gear, aux1;
		    } rc;
		    rc.roll     = RC.rcCommand[ROLL];
		    rc.pitch    = RC.rcCommand[PITCH];
		    rc.yaw      = RC.rcCommand[YAW];
		    rc.throttle = RC.rcCommand[THROTTLE];
        rc.aux1     = RC.rcCommand[AUX1];
        rc.gear     = RC.rcCommand[GEAR];
		   s_struct((uint8_t*)&rc, 12);
			 break;
		 }

	    case MSP_STATUS:
	    	{ struct {
          uint32_t ArmedTime;
	        uint32_t cycleTime;
	        uint8_t error, flag;
	    	} st;
	    	    st.ArmedTime    = armedTime;
	      	  st.cycleTime    = loopTime;
	      	  st.error        = Error.error;
	      	  if(f.ARMED) tmp |= 1<<BOXARM;
            if(f.HEADFREE_MODE) tmp |= 1<<BOXHEADFREE;
	      	  st.flag         = tmp;
	      	  s_struct((uint8_t*)&st,10);
	      	  break;
	    	}

	    case MSP_ATTITUDE:
	      s_struct((uint8_t*)&att,8);
	      break;

	    case MSP_ALTITUDE:
      { struct {
        int16_t alt;
      } tmp;
	      tmp.alt = (int16_t) alt.EstAlt;
	      s_struct((uint8_t*)&tmp,2);
	      break;
      }

	    case MSP_MISC:
      { struct {
        uint16_t roll, pitch, yaw, throttle, gear, aux1; //12
        uint32_t ArmedTime; //16
        uint32_t cycleTime; //20
        uint8_t error, flag; //22
        int16_t angle[2];    //26
        int16_t heading;     //28
        int16_t mag_heading; //30
        int16_t alt;  //38
        int16_t VBAT;//32
        int16_t Temp; //34
        int16_t acc[3]; //44
        int16_t gyro[3]; //50
        int16_t mag[3]; //56
        uint8_t a,b; //58
        int32_t c,d; //66
        int16_t motor[4];//74
      } tele;
      tele.roll     = RC.rcCommand[ROLL];
      tele.pitch    = RC.rcCommand[PITCH];
      tele.yaw      = RC.rcCommand[YAW];
      tele.throttle = RC.rcCommand[THROTTLE];
      tele.aux1     = RC.rcCommand[AUX1];
      tele.gear     = RC.rcCommand[GEAR];
      tele.ArmedTime    = armedTime;
      tele.cycleTime    = loopTime;
      tele.error        = Error.error;
      if(f.ARMED) tmp |= 1<<BOXARM;
      if(f.HEADFREE_MODE) tmp |= 1<<BOXHEADFREE;
      if(f.ANGLE_MODE) tmp |= 1<<BOXANGLE_MODE;
      if(f.HORIZON_MODE) tmp |= 1<<BOXHORIZON_MODE;
      if(f.ACRO_MODE) tmp |= 1<<BOXACRO_MODE;
      if(f.CALIBRATE_ACC) tmp |= 1<<BOXCALIBRATE_ACC;
      if(f.CALIBRATE_MAG) tmp |= 1<<BOXCALIBRATE_MAG;
      tele.flag         = tmp;
      tele.angle[ROLL] = (int16_t) imu.AHRS[ROLL] * 10;
      tele.angle[PITCH] = (int16_t) imu.AHRS[PITCH] * 10;
      tele.heading = (int16_t) imu.AHRS[YAW];
      tele.mag_heading = (int16_t) imu.actual_compass_heading;
      tele.alt = (int16_t) alt.EstAlt;
      tele.VBAT = (int16_t) BAT.VBAT;
      tele.Temp = (int16_t) imu.Temp*10;
      for(uint8_t axis=0; axis<3;axis++){
        tele.acc[axis]  = (int16_t) imu.accSmooth[axis];//map(imu.accADC[axis], -32768, 32768, -1000, 1000);
        tele.gyro[axis] = (int16_t) imu.gyroRaw[axis];
        tele.mag[axis]  = (int16_t) imu.magRaw[axis];
      }
      tele.a     = GPS.fixquality;
      tele.b     = GPS.satellites;
      tele.c     = GPS.latitudeDegrees;
      tele.d     = GPS.longitudeDegrees;
      tele.motor[0] = motor[0];
      tele.motor[1] = motor[1];
      tele.motor[2] = motor[2];
      tele.motor[3] = motor[3];
      s_struct((uint8_t*)&tele,74);
      break;
     }

	    case MSP_RAW_IMU:
        { struct {
          int16_t acc[3];
          int16_t gyro[3];
          int16_t mag[3];
        } mpu;
        for(uint8_t axis=0; axis<3;axis++){
          mpu.acc[axis]  = (int16_t) map(imu.accADC[axis], -32768, 32768, -1000, 1000);
          mpu.gyro[axis] = (int16_t) imu.gyroRaw[axis];
          mpu.mag[axis]  = (int16_t) imu.magRaw[axis];
        }
	      s_struct((uint8_t*)&mpu,18);
	      break;
       }

	    case MSP_RAW_GPS:
	      { struct {
	        uint8_t a,b;
	        int32_t c,d;
//	        int16_t e;
//	        uint16_t f,g;
	      } msp_raw_gps;
	      msp_raw_gps.a     = GPS.fixquality;
	      msp_raw_gps.b     = GPS.satellites;
	      msp_raw_gps.c     = GPS.latitudeDegrees;
	      msp_raw_gps.d     = GPS.longitudeDegrees;
//	      msp_raw_gps.e     = GPS_altitude;
//	      msp_raw_gps.f     = GPS_speed;
//	      msp_raw_gps.g     = GPS_ground_course;
	      s_struct((uint8_t*)&msp_raw_gps,10);
	      break;
	     }

	    case MSP_MOTOR:
	      s_struct((uint8_t*)&motor,8);
	      break;

		 case MSP_PID:
     { struct {
        uint16_t ROLL[3];
        uint16_t outer_ROLL[2];
        uint16_t inner_ROLL[3];
        uint16_t ROLL_rate[3];

        uint16_t PITCH[3];
        uint16_t outer_PITCH[2];
        uint16_t inner_PITCH[3];
        uint16_t PITCH_rate[3];

        uint16_t YAW[3];
        uint16_t outer_YAW[2];
        uint16_t inner_YAW[3];
        uint16_t YAW_rate[3];
      } pid_t;


          pid_t.ROLL[0]  = (int16_t) (pid.kp[ROLL]  * 10);
          pid_t.ROLL[1]  = (int16_t) (pid.ki[ROLL]  * 10);
          pid_t.ROLL[2]  = (int16_t) (pid.kd[ROLL]  * 10);
          pid_t.PITCH[0] = (int16_t) (pid.kp[PITCH] * 10);
          pid_t.PITCH[1] = (int16_t) (pid.ki[PITCH] * 10);
          pid_t.PITCH[2] = (int16_t) (pid.kd[PITCH] * 10);
          pid_t.YAW[0]   = (int16_t) (pid.kp[YAW]   * 10);
          pid_t.YAW[1]   = (int16_t) (pid.ki[YAW]   * 10);
          pid_t.YAW[2]   = (int16_t) (pid.kd[YAW]   * 10);

          pid_t.outer_ROLL[0] = (int16_t) (pid.kp1[ROLL] * 10);
          pid_t.outer_ROLL[1] = (int16_t) (pid.ki1[ROLL] * 10);
          pid_t.outer_PITCH[0] = (int16_t) (pid.kp1[PITCH] * 10);
          pid_t.outer_PITCH[1] = (int16_t) (pid.ki1[PITCH] * 10);
          pid_t.outer_YAW[0] = (int16_t) (pid.kp1[YAW] * 10);
          pid_t.outer_YAW[1] = (int16_t) (pid.ki1[YAW] * 10);

          pid_t.inner_ROLL[0] = (int16_t) (pid.kp2[ROLL] * 10);
          pid_t.inner_ROLL[1] = (int16_t) (pid.ki2[ROLL] * 10);
          pid_t.inner_ROLL[2] = (int16_t) (pid.kd2[ROLL] * 10);
          pid_t.inner_PITCH[0] = (int16_t) (pid.kp2[PITCH] * 10);
          pid_t.inner_PITCH[1] = (int16_t) (pid.ki2[PITCH] * 10);
          pid_t.inner_PITCH[2] = (int16_t) (pid.kd2[PITCH] * 10);
          pid_t.inner_YAW[0] = (int16_t) (pid.kp2[YAW] * 10);
          pid_t.inner_YAW[1] = (int16_t) (pid.ki2[YAW] * 10);
          pid_t.inner_YAW[2] = (int16_t) (pid.kd2[YAW] * 10);

          pid_t.ROLL_rate[0]  = (int16_t) (pid.kp_rate[ROLL]  * 10);
          pid_t.ROLL_rate[1]  = (int16_t) (pid.ki_rate[ROLL]  * 10);
          pid_t.ROLL_rate[2]  = (int16_t) (pid.kd_rate[ROLL]  * 10);
          pid_t.PITCH_rate[0] = (int16_t) (pid.kp_rate[PITCH] * 10);
          pid_t.PITCH_rate[1] = (int16_t) (pid.ki_rate[PITCH] * 10);
          pid_t.PITCH_rate[2] = (int16_t) (pid.kd_rate[PITCH] * 10);
          pid_t.YAW_rate[0]   = (int16_t) (pid.kp_rate[YAW]   * 10);
          pid_t.YAW_rate[1]   = (int16_t) (pid.ki_rate[YAW]   * 10);
          pid_t.YAW_rate[2]   = (int16_t) (pid.kd_rate[YAW]   * 10);

        s_struct((uint8_t*)&pid_t,66);

       break;			
     }

	    case MSP_ANALOG:
	     { struct {
	        uint16_t VBAT;
	        uint16_t Temp;
	      } analog;

	      analog.VBAT = BAT.VBAT;
	      analog.Temp = (imu.Temp*10);

	      s_struct((uint8_t*)&analog,4);
	      break;
	     }

		 case MSP_SET_PID:
			 	for(i=0; i < 3; i++){
				 pid.kp[i] = (float) read16();
				 pid.kp[i] /= 10;
				 pid.ki[i] = (float) read16();
				 pid.ki[i] /= 10;
				 pid.kd[i] = (float) read16();
				 pid.kd[i] /= 10;

				 pid.kp1[i] = (float) read16();
	       pid.kp1[i] /= 10;
         pid.ki1[i] = (float) read16();
         pid.ki1[i] /= 10;
         pid.kp2[i] = (float) read16();
         pid.kp2[i] /= 10;
         pid.ki2[i] = (float) read16();
         pid.ki2[i] /= 10;
         pid.kd2[i] = (float) read16();
         pid.kd2[i] /= 10;

         pid.kp_rate[i] = (float) read16();
         pid.kp_rate[i] /= 10;
         pid.ki_rate[i] = (float) read16();
         pid.ki_rate[i] /= 10;
         pid.kd_rate[i] = (float) read16();
         pid.kd_rate[i] /= 10;
				}
       break;

     case MSP_RESET:
       Error.error = 0;
       RGB_R_OFF;
       cycleTimeMax = 0;
       cycleTimeMin = 65535;
       f.mag_reset = 1;
        break;

		 case MSP_ACC_CALIBRATION:
			 if(!f.ARMED){
			   calibratingA=512;
			   f.CALIBRATE_ACC = 1;
			 }
		break;

    case MSP_MAG_CALIBRATION:
      if(!f.ARMED){
        f.CALIBRATE_MAG=!f.CALIBRATE_MAG;
      }
    break;

		 	case MSP_TRIM_UP:
				MSP_TRIM[PITCH] += 1;
				sprintf(Buf, "MSP_TRIM_UP : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
				 
			case MSP_TRIM_DOWN:
				MSP_TRIM[PITCH] -= 1;
				sprintf(Buf, "MSP_TRIM_DOWN : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
						 
			case MSP_TRIM_LEFT:
				MSP_TRIM[ROLL] -= 1;
				sprintf(Buf, "MSP_TRIM_LEFT : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;
								 
			case MSP_TRIM_RIGHT:
				MSP_TRIM[ROLL] += 1;
				sprintf(Buf, "MSP_TRIM_RIGHT : %d, %d, %d, %d, %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4]);
    		HAL_UART_Transmit_IT(&huart1, (uint8_t*)Buf, strlen(Buf));
			 break;

	    case TELEMERY_PID_SAVE:
	      RGB_B_TOGGLE;
	      for(int i = 0; i < 3; i++){
	        writeFloat( 0+(4*i), pid.kp[i]);
	        writeFloat(12+(4*i), pid.ki[i]);
	        writeFloat(24+(4*i), pid.kd[i]);
	      }
	      writeFloat(36, magBias[0]);
	      writeFloat(40, magBias[1]);
	      writeFloat(44, magBias[2]);
	      writeFloat(48, magScale[0]);
	      writeFloat(52, magScale[1]);
	      writeFloat(56, magScale[2]);
	     break;

		 default:
		   //headSerialError();
		   //tailSerialReply();
		   break;
	 }

 }

void SerialSerialize(uint8_t port,uint8_t a) {
  uint8_t t = serialHeadTX[port];
  if (++t >= TX_BUFFER_SIZE) t = 0;
  serialBufferTX[t][port] = a;
  serialHeadTX[port] = t;
}

void UartSendData(uint8_t port) {
  uint8_t t = serialTailTX[port];
  switch(port){
    case 0:
      while (serialHeadTX[port] != t) {
        if (++t >= TX_BUFFER_SIZE) t = 0;
        serialBufTx_0[serialHead_0++] = serialBufferTX[t][port];
      }
      serialTailTX[port] = t;
      HAL_UART_Transmit_IT(&huart1, serialBufTx_0, serialHead_0);
      serialHead_0 = 0;
      break;

    case 1:
      while (serialHeadTX[port] != t) {
        if (++t >= TX_BUFFER_SIZE) t = 0;
        serialBufTx_1[serialHead_1++] = serialBufferTX[t][port];
      }
      serialTailTX[port] = t;
      HAL_UART_Transmit_IT(&huart2, serialBufTx_1, serialHead_1);
      serialHead_1 = 0;
      break;
  }
}
