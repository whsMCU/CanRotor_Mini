//#include "PWM.h"
#include "Board.h"
void PwmWriteMotor(void)
{
  TIM4->CCR1 = motor[0];  // Actual : REAR_L
  TIM4->CCR2 = motor[1];  // Actual : FRONT_R
  TIM4->CCR3 = motor[2];  // Actual : FRONT_L
  TIM4->CCR4 = motor[3];  // Actual : REAR_R
}
