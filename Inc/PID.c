//#include "PID.h"
#include "Board.h"
pidc_t pid;
float dt_recip;


// Configure PID
#ifdef MOTOR_DC
  #define OUT_MAX      800      // Out PID maximum 250
  #define OUT_MAX_Y    800      // Out PID maximum 250
  #define I_MAX        400      // Out I_term maximum 300v
  #define I_MAX_Y      400      // Out I_term maximum 300
  #define DIR        1        // Direct PID Direction 1
#endif
#ifdef MOTOR_ESC
  #define OUT_MAX      400      // Out PID maximum 250
  #define OUT_MAX_Y    400      // Out PID maximum 250
  #define I_MAX        200      // Out I_term maximum 300v
  #define I_MAX_Y      200      // Out I_term maximum 300
  #define DIR        1        // Direct PID Direction 1
#endif
/* AHRS/IMU structure */

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PIDControlInit(pidc_t *pid)
{
	pid->ts = 0.004f;
	pid->kp[ROLL]  = 15.0f;
	pid->kp[PITCH] = pid->kp[ROLL];
	pid->kp[YAW]   = 10.0f;  // angle Mode = 8

	pid->ki[ROLL]  = 5.0f;
	pid->ki[PITCH] = pid->ki[ROLL];
	pid->ki[YAW]   = 5.0f;  // angle Mode = 5

	pid->kd[ROLL]  = 4.0f;
	pid->kd[PITCH] = pid->kd[ROLL];
	pid->kd[YAW]   = 3.0f;  // angle Mode = 6

	///////////////////////////////////////
  pid->kp_rate[ROLL]  = 2.0f;
  pid->kp_rate[PITCH] = pid->kp_rate[ROLL];
  pid->kp_rate[YAW]   = 2.0f;

  pid->ki_rate[ROLL]  = 0.0f;
  pid->ki_rate[PITCH] = pid->ki_rate[ROLL];
  pid->ki_rate[YAW]   = 0.0f;

  pid->kd_rate[ROLL]  = 0.0f;
  pid->kd_rate[PITCH] = pid->kd_rate[ROLL];
  pid->kd_rate[YAW]   = 0.0f;
///////////////////////////////////	
	pid->kp1[ROLL] = 1.0f;
	pid->kp1[PITCH] = 1.0f;
	pid->kp1[YAW] = 1.0f;

	pid->ki1[ROLL] = 2.0f;
	pid->ki1[PITCH] = 2.0f;
	pid->ki1[YAW] = 2.0f;
/////////////////////////////////
	pid->kp2[ROLL] = 5.0f;
	pid->kp2[PITCH] = 5.0f;
	pid->kp2[YAW] = 5.0f;

  pid->ki2[ROLL] = 2.0f;
	pid->ki2[PITCH] = 2.0f;
	pid->ki2[YAW] = 2.0f;

	pid->kd2[ROLL] = 3.0f;
	pid->kd2[PITCH] = 3.0f;
	pid->kd2[YAW] = 3.0f;
////////////////////////////////////////

  pid->i1_limit[ROLL] = 5.0f;
	pid->i1_limit[PITCH] = 5.0f;
	pid->i1_limit[YAW] = 10.0f;

  pid->i2_limit[ROLL] = 500.0f;
	pid->i2_limit[PITCH] = 500.0f;
	pid->i2_limit[YAW] = 500.0f;

	pid->Iterm[ROLL] = 0.0f ;
	pid->Iterm[PITCH] = 0.0f;
	pid->Iterm[YAW] = 0.0f;

	pid->Iterm1[ROLL] = 0.0f ;
	pid->Iterm1[PITCH] = 0.0f;
	pid->Iterm1[YAW] = 0.0f;

	pid->Iterm2[ROLL] = 0.0f ;
	pid->Iterm2[PITCH] = 0.0f;
	pid->Iterm2[YAW] = 0.0f;

	pid->dInput[ROLL] = 0.0f ;
	pid->dInput[PITCH] = 0.0f;
	pid->dInput[YAW] = 0.0f;

	pid->error[ROLL] = 0.0f ;
	pid->error[PITCH] = 0.0f;
	pid->error[YAW] = 0.0f;

	pid->pre_error[ROLL] = 0.0f ;
	pid->pre_error[PITCH] = 0.0f;
	pid->pre_error[YAW] = 0.0f;

	pid->pre_deriv[ROLL] = 0.0f ;
	pid->pre_deriv[PITCH] = 0.0f;
	pid->pre_deriv[YAW] = 0.0f;

	pid->lastInput[ROLL] = 0.0f ;
	pid->lastInput[PITCH] = 0.0f;
	pid->lastInput[YAW] = 0.0f;

	pid->output1[ROLL] = 0.0f ;
	pid->output1[PITCH] = 0.0f;
	pid->output1[YAW] = 0.0f;

	pid->output2[ROLL] = 0.0f ;
	pid->output2[PITCH] = 0.0f;
	pid->output2[YAW] = 0.0f;
}

int16_t  magHold,headFreeModeHold; // [-180;+180]

void Control(void)
{
	dt_recip = 1/pid.ts;
	if(!f.ARMED){
	  headFreeModeHold = imu.actual_compass_heading;
	}

#if defined(HEADFREE)
  if(f.HEADFREE_MODE) { //to optimize
    float radDiff = (imu.actual_compass_heading - headFreeModeHold) * 0.0174533f; // where PI/180 ~= 0.0174533
    float cosDiff = cosf(radDiff);
    float sinDiff = sinf(radDiff);
    int16_t rcCommand_PITCH = RC.rcCommand[PITCH]*cosDiff + RC.rcCommand[ROLL]*sinDiff;
    RC.rcCommand[ROLL] =  RC.rcCommand[ROLL]*cosDiff - RC.rcCommand[PITCH]*sinDiff;
    RC.rcCommand[PITCH] = rcCommand_PITCH;
  }
#endif

    if(f.ANGLE_MODE){
      pid.error[ROLL] = RC.rcCommand[ROLL] - imu.AHRS[ROLL];
      pid.Iterm[ROLL] += pid.ki[ROLL] * pid.error[ROLL] * pid.ts;
      if(pid.Iterm[ROLL] > I_MAX) pid.Iterm[ROLL] = I_MAX;
      else if(pid.Iterm[ROLL] < -I_MAX) pid.Iterm[ROLL] = -I_MAX;
      pid.dInput[ROLL] = (imu.AHRS[ROLL] - pid.lastInput[ROLL])  / pid.ts;

      /*Compute PID Output*/
      pid.output2[ROLL] = pid.kp[ROLL] * pid.error[ROLL] + pid.Iterm[ROLL] - pid.kd[ROLL] * pid.dInput[ROLL];

      if(pid.output2[ROLL] > OUT_MAX) pid.output2[ROLL] = OUT_MAX;
      else if(pid.output2[ROLL] < -OUT_MAX) pid.output2[ROLL] = -OUT_MAX;

      /*Remember some variables for next time*/
      pid.lastInput[ROLL] = imu.AHRS[ROLL];

  /////////////////////////////////////////////////////////////////////////////////////////////////

      /*Compute all the working error variables*/
      pid.error[PITCH] = RC.rcCommand[PITCH] - imu.AHRS[PITCH];
      pid.Iterm[PITCH] += pid.ki[PITCH] * pid.error[PITCH] * pid.ts;
      if(pid.Iterm[PITCH] > I_MAX) pid.Iterm[PITCH] = I_MAX;
      else if(pid.Iterm[PITCH] < -I_MAX) pid.Iterm[PITCH] = -I_MAX;
      pid.dInput[PITCH] = (imu.AHRS[PITCH] - pid.lastInput[PITCH]) / pid.ts;

      /*Compute PID Output*/
      pid.output2[PITCH] = pid.kp[PITCH] * pid.error[PITCH] + pid.Iterm[PITCH] - pid.kd[PITCH] * pid.dInput[PITCH];

      if(pid.output2[PITCH] > OUT_MAX) pid.output2[PITCH] = OUT_MAX;
      else if(pid.output2[PITCH] < -OUT_MAX) pid.output2[PITCH] = -OUT_MAX;

      /*Remember some variables for next time*/
      pid.lastInput[PITCH] = imu.AHRS[PITCH];

  //////////////////////////////////////////////////////////////////////////////////////////////////

      /*Compute all the working error variables*/
      pid.error[YAW] = RC.rcCommand[YAW] - imu.gyroRaw[YAW];
      pid.Iterm[YAW] += pid.ki[YAW] * pid.error[YAW] * pid.ts;
      if(pid.Iterm[YAW] > I_MAX) pid.Iterm[YAW] = I_MAX;
      else if(pid.Iterm[YAW] < -I_MAX) pid.Iterm[YAW] = -I_MAX;
      pid.dInput[YAW] = (imu.gyroRaw[YAW] - pid.lastInput[YAW]) / pid.ts;

      /*Compute PID Output*/
      pid.output2[YAW] = pid.kp[YAW] * pid.error[YAW] + pid.Iterm[YAW] - pid.kd[YAW] * pid.dInput[YAW];

      if(pid.output2[YAW] > OUT_MAX) pid.output2[YAW] = OUT_MAX;
      else if(pid.output2[YAW] < -OUT_MAX) pid.output2[YAW] = -OUT_MAX;

      /*Remember some variables for next time*/
      pid.lastInput[YAW] = imu.gyroRaw[YAW];

    }else if(f.HORIZON_MODE){
      int axis;
      float error, deriv;
        //axis pid
        for(axis = 0; axis < 3; axis++){
        error = RC.rcCommand[axis] - imu.AHRS[axis];
        pid.Iterm1[axis] += error * pid.ts;
        if(pid.Iterm1[axis] > pid.i1_limit[axis]) pid.Iterm1[axis] = pid.i1_limit[axis];
        else if(pid.Iterm1[axis] < -pid.i1_limit[axis]) pid.Iterm1[axis] = -pid.i1_limit[axis];
        pid.output1[axis] = pid.kp1[axis]*error + pid.ki1[axis]*pid.Iterm1[axis];

        error = pid.output1[axis] - imu.gyroRaw[axis];
        pid.Iterm2[axis] += error * pid.ts;
        if(pid.Iterm2[axis] > pid.i2_limit[axis]) pid.Iterm2[axis] = pid.i2_limit[axis];
        else if(pid.Iterm2[axis] < -pid.i2_limit[axis]) pid.Iterm2[axis] = -pid.i2_limit[axis];
        deriv = (error - pid.pre_error[axis])*dt_recip;
        pid.pre_error[axis] = error;
        deriv = pid.pre_deriv[axis] + (deriv -pid.pre_deriv[axis]) * D_FILTER_COFF;
        pid.pre_deriv[axis] = deriv;
        pid.output2[axis] = pid.kp2[axis]*error + pid.ki2[axis]*pid.Iterm2[axis] + pid.kd2[axis]*deriv;

        if(pid.output2[axis] > OUT_MAX) pid.output2[axis] = OUT_MAX;
        if(pid.output2[axis] < -OUT_MAX) pid.output2[axis] = -OUT_MAX;
        }
  }else if(f.ACRO_MODE){
        pid.error[ROLL] = RC.rcCommand[ROLL] - imu.gyroRaw[ROLL];
        pid.Iterm[ROLL] += pid.ki_rate[ROLL] * pid.error[ROLL] * pid.ts;
        if(pid.Iterm[ROLL] > I_MAX) pid.Iterm[ROLL] = I_MAX;
        else if(pid.Iterm[ROLL] < -I_MAX) pid.Iterm[ROLL] = -I_MAX;
        pid.dInput[ROLL] = (imu.gyroRaw[ROLL] - pid.lastInput[ROLL])  / pid.ts;

        /*Compute PID Output*/
        pid.output2[ROLL] = pid.kp_rate[ROLL] * pid.error[ROLL] + pid.Iterm[ROLL] - pid.kd_rate[ROLL] * pid.dInput[ROLL];

        if(pid.output2[ROLL] > OUT_MAX) pid.output2[ROLL] = OUT_MAX;
        else if(pid.output2[ROLL] < -OUT_MAX) pid.output2[ROLL] = -OUT_MAX;

        /*Remember some variables for next time*/
        pid.lastInput[ROLL] = imu.gyroRaw[ROLL];

    /////////////////////////////////////////////////////////////////////////////////////////////////

        /*Compute all the working error variables*/
        pid.error[PITCH] = RC.rcCommand[PITCH] - imu.gyroRaw[PITCH];
        pid.Iterm[PITCH] += pid.ki_rate[PITCH] * pid.error[PITCH] * pid.ts;
        if(pid.Iterm[PITCH] > I_MAX) pid.Iterm[PITCH] = I_MAX;
        else if(pid.Iterm[PITCH] < -I_MAX) pid.Iterm[PITCH] = -I_MAX;
        pid.dInput[PITCH] = (imu.gyroRaw[PITCH] - pid.lastInput[PITCH]) / pid.ts;

        /*Compute PID Output*/
        pid.output2[PITCH] = pid.kp_rate[PITCH] * pid.error[PITCH] + pid.Iterm[PITCH] - pid.kd_rate[PITCH] * pid.dInput[PITCH];

        if(pid.output2[PITCH] > OUT_MAX) pid.output2[PITCH] = OUT_MAX;
        else if(pid.output2[PITCH] < -OUT_MAX) pid.output2[PITCH] = -OUT_MAX;

        /*Remember some variables for next time*/
        pid.lastInput[PITCH] = imu.gyroRaw[PITCH];

    //////////////////////////////////////////////////////////////////////////////////////////////////

        /*Compute all the working error variables*/
        pid.error[YAW] = RC.rcCommand[YAW] - imu.gyroRaw[YAW];
        pid.Iterm[YAW] += pid.ki_rate[YAW] * pid.error[YAW] * pid.ts;
        if(pid.Iterm[YAW] > I_MAX) pid.Iterm[YAW] = I_MAX;
        else if(pid.Iterm[YAW] < -I_MAX) pid.Iterm[YAW] = -I_MAX;
        pid.dInput[YAW] = (imu.gyroRaw[YAW] - pid.lastInput[YAW]) / pid.ts;

        /*Compute PID Output*/
        pid.output2[YAW] = pid.kp_rate[YAW] * pid.error[YAW] + pid.Iterm[YAW] - pid.kd_rate[YAW] * pid.dInput[YAW];

        if(pid.output2[YAW] > OUT_MAX) pid.output2[YAW] = OUT_MAX;
        else if(pid.output2[YAW] < -OUT_MAX) pid.output2[YAW] = -OUT_MAX;

        /*Remember some variables for next time*/
        pid.lastInput[YAW] = imu.gyroRaw[YAW];//imu.Yaw
    }
#ifdef Recive_PID_CHANGE
	  if(f.Tuning_MODE == 1){
	    RGB_G_TOGGLE;
	    f.Write_MODE = 1;
	    if(RC.rcCommand[ROLL]  >  10) pid.kp[ROLL] += (float)RC.rcCommand[ROLL]  * 0.0005;
	    if(RC.rcCommand[ROLL]  < -10) pid.kp[ROLL] += (float)RC.rcCommand[ROLL]  * 0.0005;
	    if(RC.rcCommand[PITCH] >  10) pid.ki[ROLL] += (float)RC.rcCommand[PITCH] * 0.0005;
	    if(RC.rcCommand[PITCH] < -10) pid.ki[ROLL] += (float)RC.rcCommand[PITCH] * 0.0005;
	    if(RC.rcCommand[YAW]   >  10) pid.kd[ROLL] += (float)RC.rcCommand[YAW]   * 0.00005;
	    if(RC.rcCommand[YAW]   < -10) pid.kd[ROLL] += (float)RC.rcCommand[YAW]   * 0.00005;

	    pid.kp[PITCH] = pid.kp[ROLL];
	    pid.ki[PITCH] = pid.ki[ROLL];
	    pid.kd[PITCH] = pid.kd[ROLL];

	  }else if(f.Tuning_MODE == 2){
	    RGB_G_OFF;
	    RGB_R_TOGGLE;
      if(RC.rcCommand[ROLL]  >  10) pid.kp[YAW] += (float)RC.rcCommand[ROLL]  * 0.0005;
      if(RC.rcCommand[ROLL]  < -10) pid.kp[YAW] += (float)RC.rcCommand[ROLL]  * 0.0005;
      if(RC.rcCommand[PITCH] >  10) pid.ki[YAW] += (float)RC.rcCommand[PITCH] * 0.0005;
      if(RC.rcCommand[PITCH] < -10) pid.ki[YAW] += (float)RC.rcCommand[PITCH] * 0.0005;
      if(RC.rcCommand[YAW]   >  10) pid.kd[YAW] += (float)RC.rcCommand[YAW]   * 0.00005;
      if(RC.rcCommand[YAW]   < -10) pid.kd[YAW] += (float)RC.rcCommand[YAW]   * 0.00005;
    }
	  if(f.Tuning_MODE == 0 && f.Write_MODE == 1){
	    f.Write_MODE = 0;
      RGB_G_OFF;
	    RGB_R_OFF;
	    pid.kp[ROLL] = ROUND(pid.kp[ROLL], 1);
	    pid.ki[ROLL] = ROUND(pid.ki[ROLL], 1);
	    pid.kd[ROLL] = ROUND(pid.kd[ROLL], 1);
      pid.kp[PITCH] = ROUND(pid.kp[PITCH], 1);
      pid.ki[PITCH] = ROUND(pid.ki[PITCH], 1);
      pid.kd[PITCH] = ROUND(pid.kd[PITCH], 1);
	    pid.kp[YAW] = ROUND(pid.kp[YAW], 1);
	    pid.ki[YAW] = ROUND(pid.ki[YAW], 1);
	    pid.kd[YAW] = ROUND(pid.kd[YAW], 1);

      for(int i = 0; i < 3; i++){
        writeFloat( 0+(4*i), pid.kp[i]);
        writeFloat(12+(4*i), pid.ki[i]);
        writeFloat(24+(4*i), pid.kd[i]);
      }

	    for(int i = 0; i < 10; i++) {
	      HAL_Delay(25);
	      //BEEP_ON;
	      RGB_B_ON;
	      HAL_Delay(25);
	      //BEEP_OFF;
	      RGB_B_OFF;
	    }
	  }
#endif
}

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}
