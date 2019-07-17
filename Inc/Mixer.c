//#include "Mixer.h"
#include "Board.h"
int16_t motor[4];

// Custom mixer data per motor
typedef struct motorMixer_t {
    float THROTTLE;
    float ROLL;
    float PITCH;
    float YAW;
} motorMixer_t;

// Custom mixer configuration
typedef struct mixer_t {
    uint8_t numberMotor;
    uint8_t useServo;
    const motorMixer_t *motor;
} mixer_t;

static motorMixer_t currentMixer[4];

static const motorMixer_t mixerQuadP[] = {
	{ 1.0f,  0.0f, -1.0f,  1.0f },          // REAR (CW)   1
  { 1.0f, -1.0f,  0.0f, -1.0f },        // RIGHT (CCW)  2
  { 1.0f,  1.0f,  0.0f, -1.0f },        // LEFT  (CCW)  3
	{ 1.0f,  0.0f,  1.0f,  1.0f },          // FRONT  (CW)   4
};

static const motorMixer_t mixerQuadX[] = {
    { 1.0f,  1.0f,  1.0f, -1.0f },          // (CW)  0  //REAR_L
    { 1.0f, -1.0f, -1.0f, -1.0f },          // (CW)  1  //FRONT_R
    { 1.0f,  1.0f, -1.0f,  1.0f },          // (CCW) 2  //FRONT_L
    { 1.0f, -1.0f,  1.0f,  1.0f },          // (CCW) 3  //REAR_R
}; // THR,  ROLL,   PITCH,   YAW

const mixer_t mixers[] = {
    { 4, 0, mixerQuadP },          // MULTITYPE_QUADP
    { 4, 0, mixerQuadX },          // MULTITYPE_QUADX
};

void mixerInit(void)
{
	int i;
    for (i = 0; i < 4; i++)
	  {
			#ifdef QUAD_X
      currentMixer[i] = mixers[QuadX].motor[i];   //0 = QuadP, 1 = QuadX
			#endif
			#ifdef QUAD_P
      currentMixer[i] = mixers[QuadP].motor[i];   //0 = QuadP, 1 = QuadX
			#endif
		}
}

test_t test;

void mixTable(void)
{
	uint8_t i = 0;
			if (RC.rcCommand[THROTTLE] > 1800) RC.rcCommand[THROTTLE] = 1800;                                   //We need some room to keep full control at full throttle.
			for (i = 0; i < 4; i++){
				motor[i] = (RC.rcCommand[THROTTLE] * currentMixer[i].THROTTLE) + (pid.output2[ROLL] * currentMixer[i].ROLL) + (pid.output2[PITCH] * currentMixer[i].PITCH) + ((1 * pid.output2[YAW]) * currentMixer[i].YAW);
				
  #ifdef VOLTAGEDROP_COMPENSATION
    #if (VBATNOMINAL == 42) // 1S
      #define GOV_R_NUM 12
			static int8_t g[] = { 0,4,10,17,25,34,44,55,67,80,94,109,126 };
    #elif (VBATNOMINAL == 126) // 3S
      #define GOV_R_NUM 36
			static int8_t g[] = { 0,3,5,8,11,14,17,19,22,25,28,31,34,38,41,44,47,51,54,58,61,65,68,72,76,79,83,87,91,95,99,104,108,112,117,121,126 };
    #endif
		uint8_t v = constrain( VBATNOMINAL - constrain(BAT.VBAT, VBATLEVEL_CRIT, VBATNOMINAL), 0, GOV_R_NUM);
		for (i = 0; i < 4; i++) {
      #ifdef MOTOR_DC
        motor[i] += ( ( (int32_t)(motor[i]) * (int32_t)g[v] ) )/ 500;
      #elif MOTOR_ESC
        motor[i] += ( ( (int32_t)(motor[i]-1000) * (int32_t)g[v] ) )/ 500;
      #endif
		}
  #endif
#ifdef MOTOR_DC
	  if(motor[i] <    0) motor[i] = 0;
	  if(motor[i] > 2000) motor[i] = 2000;
#endif
#ifdef MOTOR_ESC
  if(motor[i] < 1000) motor[i] = 1000;
  if(motor[i] > 2000) motor[i] = 2000;
#endif
#ifdef MOTOR_DC
  if(RC.rcCommand[THROTTLE] < 200 || f.ARMED == 0){
    motor[i] = 0;
#endif
#ifdef MOTOR_ESC
  if(RC.rcCommand[THROTTLE] < 1100 || f.ARMED == 0){
    motor[i] = 1000;
#endif
		pid.output1[i] = 0;
		pid.output2[i] = 0;
		pid.Iterm[i] = 0;
		pid.Iterm1[i] = 0;
		pid.Iterm2[i] = 0;
	}
  }
}
