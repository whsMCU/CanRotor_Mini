//#include "Radio.h"
#include "Board.h"
flags_t f;
rc RC;
rc RC_Raw;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == TIM2){
	        if(Ch1_PIN){  // Timer2 Ch1 pin(PA0) is High
	            TIM2->CCR1 = 0;
	            RC.capture_rise[0] = TIM2->CCR1; // read capture data
	            Ch1_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer2 Ch1 pin(PA0) is Low
	            RC.capture_fall[0] = TIM2->CCR1; // read capture data
	            RC.rcADC[0] = RC.capture_fall[0] - RC.capture_rise[0];
	            Ch1_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && htim->Instance == TIM2){
	        if(Ch2_PIN){  // Timer2 Ch2 pin(PA1) is High
	            TIM2->CCR2 = 0;
	        	RC.capture_rise[1] = TIM2->CCR2; // read capture data
	            Ch2_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer2 Ch2 pin(PA1) is Low
	        	RC.capture_fall[1] = TIM2->CCR2; // read capture data
	        	RC.rcADC[1] = RC.capture_fall[1] - RC.capture_rise[1];
	            Ch2_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1 && htim->Instance == TIM3){
	        if(Ch3_PIN){  // Timer3 Ch1 pin(PA6) is High
	            TIM3->CCR1 = 0;
	        	RC.capture_rise[2] = TIM3->CCR1; // read capture data
	            Ch3_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer3 Ch1 pin(PA6) is Low
	        	RC.capture_fall[2] = TIM3->CCR1; // read capture data
	        	RC.rcADC[2] = RC.capture_fall[2] - RC.capture_rise[2];
	            Ch3_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2 && htim->Instance == TIM3){
	        if(Ch4_PIN){  // Timer3 Ch2 pin(PA7) is High
	            TIM3->CCR2 = 0;
	        	RC.capture_rise[3] = TIM3->CCR2; // read capture data
	            Ch4_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer3 Ch2 pin(PA7) is Low
	        	RC.capture_fall[3] = TIM3->CCR2; // read capture data
	            RC.rcADC[3] = RC.capture_fall[3] - RC.capture_rise[3];
	            Ch4_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 && htim->Instance == TIM3){
	        if(Ch5_PIN){  // Timer3 Ch3 pin(PB0) is High
	            TIM3->CCR3 = 0;
	        	RC.capture_rise[4] = TIM3->CCR3; // read capture data
	            Ch5_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer3 Ch3 pin(PB0) is Low
	        	RC.capture_fall[4] = TIM3->CCR3; // read capture data
	            RC.rcADC[4] = RC.capture_fall[4] - RC.capture_rise[4];
	            Ch5_POL_RISING;   // to rising edge
	        }
	    }

	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 && htim->Instance == TIM3){
	        if(Ch6_PIN){  // Timer3 Ch4 pin(PB1) is High
	            TIM3->CCR4 = 0;
	        	RC.capture_rise[5] = TIM3->CCR4; // read capture data
	            Ch6_POL_FALLING;  // to falling edge
	        }
	        else{   // Timer3 Ch4 pin(PB1) is Low
	        	RC.capture_fall[5] = TIM3->CCR4; // read capture data
	            RC.rcADC[5] = RC.capture_fall[5] - RC.capture_rise[5];
	            Ch6_POL_RISING;   // to rising edge
	        }
	    }

//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 && htim->Instance == TIM3){
//	        if(Ch7_PIN){  // Timer4 Ch3 pin(PB8) is High
//	            TIM3->CCR3 = 0;
//	        	RC.capture_rise[6] = TIM3->CCR3; // read capture data
//	            Ch7_POL_FALLING;  // to falling edge
//	        }
//	        else{   // Timer4 Ch3 pin(PB8) is Low
//	        	RC.capture_fall[6] = TIM3->CCR3; // read capture data
//	            RC.rcADC[6] = RC.capture_fall[6] - RC.capture_rise[6];
//	            Ch7_POL_RISING;   // to rising edge
//	        }
//	    }

//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4 && htim->Instance == TIM3){
//	        if(Ch8PIN){  // Timer4 Ch4 pin(PB9) is High
//	        	RC.capture_rise[7] = TIM3->CCR4; // read capture data
//	            Ch8_POL_FALLING;  // to falling edge
//	        }
//	        else{   // Timer4 Ch4 pin(PB9) is Low
//	        	RC.capture_fall[7] = TIM3->CCR4; // read capture data
//	            RC.rcADC[7] = RC.capture_fall[7] - RC.capture_rise[7];
//	            Ch8_POL_RISING;   // to rising edge
//	        }
//	    }
}

void mwArm(void)
{
	if(!f.ARMED){
		f.ARMED = 1;
		f.Tuning_MODE = 0;
		ms5611.ground_pressure = ms5611.avg_realPressure;
	}
}
void mwDisarm(void)
{
  if(!f.ARMED){
    f.Tuning_MODE = (f.Tuning_MODE+1) % 3;
  }
	if(f.ARMED){
		f.ARMED = 0;
	}
}

void RC_Init(void)
{
	int count = 0;
	Flight_Status = 0;
	do{
    Error.error = 3;
		LED0_OFF;
		count ++;
		if(count == 125){
			LED1_TOGGLE; // RED RC_Init
			count = 0;
		}
	}while((RC.rcADC[ROLL] < 990) || (RC.rcADC[PITCH] < 990)|| (RC.rcADC[YAW] < 990)|| (RC.rcADC[THROTTLE] < 990));
	 Flight_Status = 1;
	 Error.error = 0;
}

void computeRC(void)
{
	static uint8_t rcDelayCommand;      // this indicates the number of time (multiple of RC measurement at 50Hz) the sticks must be maintained to run or switch off motors
	static uint8_t rcSticks;            // this hold sticks position for command combos
    uint8_t stTmp = 0;
    int i;
	// ------------------ STICKS COMMAND HANDLER --------------------
  // checking sticks positions
     for (i = 0; i < 4; i++) {
      stTmp >>= 2;
      if (RC.rcADC[i] > 1150)
          stTmp |= 0x80;  // check for MIN
      if (RC.rcADC[i] < 1850)
          stTmp |= 0x40;  // check for MAX
      }
		  if (stTmp == rcSticks) {
      if (rcDelayCommand < 250)
          rcDelayCommand++;
      } else
          rcDelayCommand = 0;
		 rcSticks = stTmp;
	
		 if (rcDelayCommand == 50) {
			 
			 if(f.ARMED == 0 && (rcSticks == THR_LO + YAW_LO + PIT_CE + ROL_CE))
			 {
			   RGB_B_TOGGLE;
			 }else if(f.ARMED == 0 && (rcSticks == THR_LO + YAW_HI + PIT_CE + ROL_CE))
			 {
			   RGB_G_TOGGLE;
			 }else if(f.ARMED == 0 && (rcSticks == THR_LO + YAW_HI + PIT_HI + ROL_HI))
       {
			   mwArm();
       }else if(rcSticks == THR_LO + YAW_LO + PIT_HI + ROL_LO)
       {
         RGB_R_TOGGLE;
         mwDisarm();
       }
				 if (i) {
               // writeEEPROM(1, true);
               rcDelayCommand = 0; // allow autorepetition
         }
        }

 		RC.rcCommand[ROLL]     = map(zofs(RC.rcADC[ROLL], 1500, 10), 1100, 1900, -20, 20)+ MSP_TRIM[ROLL]; //0~250 left:0, right:250
		RC.rcCommand[PITCH]    = map(zofs(RC.rcADC[PITCH], 1500, 10), 1100, 1900, -20, 20)+ MSP_TRIM[PITCH]; //0~250 rear:0, fornt:250
		RC.rcCommand[YAW]      = map(zofs(RC.rcADC[YAW], 1500, 10), 1100, 1900, -100, 100); //0~250 left:0, right:250
	  RC.rcCommand[THROTTLE] = map(zofs(RC.rcADC[THROTTLE], 1100, 10), 1100, 1900, 0, 1800);//0~250
	  RC.rcCommand[GEAR] 	   = RC.rcADC[GEAR];
	  RC.rcCommand[AUX1] 	   = RC.rcADC[AUX1];

//  if (f.HEADFREE_MODE) {
//        float radDiff = (heading - headFreeModeHold) * M_PI / 180.0f;
//        float cosDiff = cosf(radDiff);
//        float sinDiff = sinf(radDiff);
//        int16_t rcCommand_PITCH = rcCommand[PITCH] * cosDiff + rcCommand[ROLL] * sinDiff;
//        rcCommand[ROLL] = rcCommand[ROLL] * cosDiff - rcCommand[PITCH] * sinDiff;
//        rcCommand[PITCH] = rcCommand_PITCH;
//    }

}
