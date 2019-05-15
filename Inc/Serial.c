//#include "Serial.h"
#include "Board.h"
char Buf[128];

volatile unsigned char command=0;
volatile unsigned char m = 0;
int MSP_TRIM[3]={0, 0, 0};

uint8_t Debug_TC=0;

////////////////////////////////////////////

volatile uint8_t rx1_buffer[16];
volatile uint8_t rx2_buffer[16];

//////////// MSP //////////////////
#define INBUF_SIZE 128
typedef  struct mspPortState_t {
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
   HAL_UART_Transmit(&huart2, tmp, 1, 1);
	 //HAL_UART_Transmit_DMA(&huart2, tmp, 1);
   return(ch);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1) //current USART
		{
			write_Q(&Q_buffer[UART1], rx1_buffer[0]);
			//TX_CHR(rx1_buffer[0]);
		}
		
	if(huart->Instance == USART2) //current USART
		{
			write_Q(&Q_buffer[UART2], rx2_buffer[0]);
		}
}

void TX_CHR(char ch){
	while(!(USART1->SR & 0x80));
	USART1->DR = ch;
}

///////////////////////////////////////////////////
void serialize8(uint8_t a)
{
    TX_CHR(a);
    currentPortState->checksum ^= a;
}

void serialize32(uint32_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
    serialize8((a >> 16) & 0xFF);
    serialize8((a >> 24) & 0xFF);
}

void serialize16(int16_t a)
{
    serialize8(a & 0xFF);
    serialize8((a >> 8) & 0xFF);
}

uint8_t read8(void)
{
    return currentPortState->inBuf[currentPortState->indRX++] & 0xff;
}
//currentPortState->inBuf[0];
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
    serialize8(currentPortState->checksum);
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
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
		//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;


	case 1:
	     sprintf(Buf, " acc (%6.2f), (%6.2f), (%6.2f) / gyro (%6.2f), (%6.2f), (%6.2f) / mag (%6.f), (%6.f), (%6.f)\r\n",
	                    imu.accRaw[ROLL], imu.accRaw[PITCH], imu.accRaw[YAW], imu.gyroRaw[ROLL], imu.gyroRaw[PITCH], imu.gyroRaw[YAW], imu.magRaw[ROLL], imu.magRaw[PITCH], imu.magRaw[YAW]);
	     HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	     break;

	case 2:
		sprintf(Buf, " gyroBias_x: (%3.2f), gyroBias_y: (%3.2f), gyroBias_z: (%3.2f)\r\n",
                 	imu.gyro_cal[ROLL], imu.gyro_cal[PITCH], imu.gyro_cal[YAW]);
			HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

	    break;

	case 3:
	  //sprintf(Buf, "latDeg : %f, lat : %c, lonDeg : %f, lon : %c, fixQ: %d, satel : %d, altitude : %.2fM, geohi : %.2fM \r\n",
	  //        GPS.latitudeDegrees, GPS.lat, GPS.longitudeDegrees, GPS.lon, GPS.fixquality, GPS.satellites, GPS.altitude, GPS.geoidheight);
	  sprintf(Buf, "Y : %2d, M : %2d, D : %2d, H: %2d, min : %2d, sec : %2d, mil : %3d, speed : %.2f, angle : %.2f, Error : %d\n",
	          GPS.year, GPS.month, GPS.day, GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds, GPS.speed, GPS.angle, GPS.error);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
		break;

	case 4:
		sprintf(Buf, " %d %d %d %d\r\n", motor[0], motor[1], motor[2], motor[3]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 5:
		sprintf(Buf, "motor:(%4.d)(%4.d)(%4.d)(%4.d), AHRS:(%4.f)(%4.f)(%4.f), RC:(%4.d)(%4.d)(%4.d)(%4.d)(%4.d)(%4.d), VBAT: (%4.1f), ARMED: (%d), Tuning : (%d) \r\n",
	  motor[0], motor[1], motor[2], motor[3], imu.AHRS[ROLL], imu.AHRS[PITCH], imu.gyroYaw, RC.rcCommand[ROLL], RC.rcCommand[PITCH], RC.rcCommand[YAW], RC.rcCommand[THROTTLE], RC.rcCommand[GEAR], RC.rcCommand[AUX1], BAT.VBAT, f.ARMED, f.Tuning_MODE);
//		sprintf(Buf, "RC:(%4.d)(%4.d)(%4.d)(%4.d)(%4.d)(%4.d)\r\n",
//	   RC.rcCommand[ROLL], RC.rcCommand[PITCH], RC.rcCommand[YAW], RC.rcCommand[THROTTLE], RC.rcCommand[GEAR], RC.rcCommand[AUX1]);
//    sprintf(Buf, "Mag:(%5.f)(%5.f)(%5.f), AHRS:(%4.f)(%4.f)(%4.f), RC:(%4.d)(%4.d)(%4.d)(%4.d), (%4.d) (%4.2f), ARMED: (%2.1d), MS5611 : %.2f Pa , %.2f cm\r\n",
//            imu.magRaw[ROLL], imu.magRaw[PITCH], imu.magRaw[YAW], imu.AHRS[ROLL], imu.AHRS[PITCH], imu.AHRS[YAW], RC.rcCommand[ROLL], RC.rcCommand[PITCH], RC.rcCommand[YAW], RC.rcCommand[THROTTLE], BAT.VBAT_Sense, BAT.VBAT, f.ARMED, ms5611.actual_pressure, ms5611.GroundAltitude);
    //sprintf(Buf,"Hour: %d, minute : %d, second : %d, milliseconds : %d\n", GPS.hour, GPS.minute, GPS.seconds, GPS.milliseconds);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
	//HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 6:
    sprintf(Buf,"R[P]: %2.2f, P[P]: %2.2f, R[I]: %2.2f, P[I]: %2.2f, R[D]: %2.2f, P[D]: %2.2f, Y[P]: %2.2f, Y[I]: %2.2f, Y[D]: %2.2f, ARMED: (%d), Tuning : (%d)\r\n",
            pid.kp[ROLL], pid.kp[PITCH], pid.ki[ROLL], pid.ki[PITCH], pid.kd[ROLL], pid.kd[PITCH], pid.kp[YAW], pid.ki[YAW], pid.kd[YAW], f.ARMED, f.Tuning_MODE);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
		break;
	case 7:
		  sprintf(Buf, " state: %d, data: %d \n ", hdma_usart1_rx.State, rx1_buffer[0]);
		  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
		  //HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 8:
		sprintf(Buf, "%f %f %f\r\n",pid.output2[ROLL], pid.output2[PITCH], pid.output2[YAW]);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
		//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);

		break;
	case 9:
		sprintf(Buf, "Roll:(%.2f), Pitch:(%.2f), Yaw:(%.2f), rx_buffer:(%d)\r\n",AHRSIMU.Roll, AHRSIMU.Pitch, AHRSIMU.Yaw, rx1_buffer[0]);
	     HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
	     //HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;

	case 10:
		sprintf(Buf, "Data : %d, %d, %d, %d, %d, %d \r\n ", l_t, ms5611.realTemperature, (uint32_t)ms5611.realPressure, baroPressureSum, ms5611.BaroAlt, alt.EstAlt);
		HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));

		break;

     case 11:
			sprintf(Buf, "\r\n [KP]: %.2f, %.2f, %.2f \r\n ", pid.kp[0], pid.kp[1], pid.kp[2]);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
			//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;

	case 12:
			sprintf(Buf, "\r\n [KI]: %.2f, %.2f, %.2f\r\n", pid.ki[0], pid.ki[1], pid.ki[2]);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
			//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;

	case 13:
			sprintf(Buf, "\r\n [KD]: %.2f, %.2f, %.2f\r\n", pid.kd[0], pid.kd[1], pid.kd[2]);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
			//HAL_UART_Transmit(&huart1, (uint8_t*)Buf, strlen(Buf), 1000);
		break;
	case 14:
		sprintf(Buf,"R/P/Y: %f %f %f\r\n",AHRSIMU.Roll, AHRSIMU.Pitch, AHRSIMU.Yaw);
	     HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));
		break;
	 }
  }
}

void SerialCom(void) {
		volatile uint8_t c;
		for(int i = 0; i < 2; i++){
    currentPortState = &ports[i];
//			printf("port: %d, Q_a : %2.1d, %2.1d \r\n",currentPortState->cmdMSP, Q_buffer[0].head, Q_buffer[0].tail);
//	    sprintf(Buf,"port: %d, Q_a : %2.1d, %2.1d \r\n",currentPortState->cmdMSP, Q_buffer[0].head, Q_buffer[0].tail);
//	    HAL_UART_Transmit_DMA(&huart2, (uint8_t*)Buf, strlen(Buf));

    while(QueueAvailable(&Q_buffer[i]) > 0){
	  c = read_Q(&Q_buffer[i]);
    if (currentPortState->c_state == IDLE) {
      currentPortState->c_state = (c=='$') ? HEADER_START : IDLE;
    }
    else if (currentPortState->c_state == HEADER_START) {
      currentPortState->c_state = (c=='M') ? HEADER_M : IDLE;
    }
    else if (currentPortState->c_state == HEADER_M) {
      currentPortState->c_state = (c=='<') ? HEADER_ARROW : IDLE;

    }
    else if (currentPortState->c_state == HEADER_ARROW) {

      if (c > INBUF_SIZE) {  // now we are expecting the payload size
        currentPortState->c_state = IDLE;
        continue;
      }
        currentPortState->dataSize = c;
        currentPortState->offset = 0;
        currentPortState->checksum = 0;
				currentPortState->indRX = 0;
        currentPortState->checksum ^= c;
        currentPortState->c_state = HEADER_SIZE;
    }
    else if (currentPortState->c_state == HEADER_SIZE) {
      currentPortState->cmdMSP = c;
      currentPortState->checksum ^= c;
      currentPortState->c_state = HEADER_CMD;
    }
    else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset < currentPortState->dataSize) {
      currentPortState->checksum ^= c;
      currentPortState->inBuf[currentPortState->offset++] = c;
    }
    else if (currentPortState->c_state == HEADER_CMD && currentPortState->offset >= currentPortState->dataSize) {

      if (currentPortState->checksum == c) {
				evaluateCommand();
      }

      currentPortState->c_state = IDLE;
    }
   }
  }
 }

 void evaluateCommand(void) {
	 uint32_t i=0;

	 switch(currentPortState->cmdMSP){
		 case MSP_ARM:
			 mwArm();
		 sprintf(Buf, "LOCK : %d, %d, %d, %d, %d, ARMD : %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4], f.ARMED);
    		//HAL_UART_Transmit_IT(&huart2, (uint8_t*)Buf, strlen(Buf));
		 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
			 break;
		 
		 case MSP_DISARM:
			 mwDisarm();
				sprintf(Buf, "UNLOCK : %d, %d, %d, %d, %d, ARMD : %d\r\n ", currentPortState->inBuf[0], currentPortState->inBuf[1], currentPortState->inBuf[2], currentPortState->inBuf[3], currentPortState->inBuf[4], f.ARMED);
    		//HAL_UART_Transmit_IT(&huart2, (uint8_t*)Buf, strlen(Buf));
		 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
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
			 	headSerialReply(10);
			  for (i = 0; i < 5; i++)
                serialize16(RC.rcCommand[i]);
				tailSerialReply();
			 break;
				
			case MSP_RAW_IMU:
				headSerialReply(12);
			  for (i = 0; i < 3; i++)
				  serialize32(angle[i]);
				tailSerialReply();
			 break;

		 case MSP_PID:
			 	headSerialReply(36);
			  for (i = 0; i < 3; i++){
				serialize32(pid.kp[i]*10);
				serialize32(pid.ki[i]*10);
				serialize32(pid.kd[i]*10);
				}
				tailSerialReply();
       break;			
				
		 case MSP_SET_PID:
			 	for(i=0; i < 3; i++){
				 pid.kp[i] = read32();
				 pid.kp[i]/=10;
				 pid.ki[i] = read32();
				 pid.ki[i]/=10;
				 pid.kd[i] = read32();
				 pid.kd[i]/=10;
				}
        //headSerialReply(0);
       break;

		 case MSP_ACC_CALIBRATION:
			 if(!f.ARMED) calibratingA=512;

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
		 
		 case MSP_RAW_GPS:
//          GPSValues result = p.evalGPS(inBuf);
//          logger.logGPS(result);
			 break;
	 }

 }

void SendTelemetry(void){
  static int x = 0;
  x++;
  if (x >= 1000){
    x = 0;
    RGB_B_TOGGLE;
    headSerialSend(3, MSP_PID);
    serialize8(10);
    serialize8(20);
    serialize8(110);
    tailSerialReply();
  }
}
