#include <Board.h>

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

void mixTable(void)
{
	uint8_t i = 0;
			if (RC.rcCommand[THROTTLE] > 1800) RC.rcCommand[THROTTLE] = 1800;                                   //We need some room to keep full control at full throttle.
			for (i = 0; i < 4; i++){
				motor[i] = (RC.rcCommand[THROTTLE] * currentMixer[i].THROTTLE) + (pid.output2[ROLL] * currentMixer[i].ROLL) + (pid.output2[PITCH] * currentMixer[i].PITCH) + ((1 * pid.output2[YAW]) * currentMixer[i].YAW);
				
				if(motor[i]<0) motor[i] = 0;
				if(motor[i] > 2000) motor[i] = 2000;
	
				if(RC.rcCommand[THROTTLE] < 200 || f.ARMED == 0)
				{
					motor[i] = 0;
					pid.output1[i] = 0;
					pid.output2[i] = 0;
					pid.Iterm[i] = 0;
					pid.Iterm1[i] = 0;
					pid.Iterm2[i] = 0;
				}
			}
}
