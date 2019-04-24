#ifndef TYPEDEF_H_
#define TYPEDEF_H_

enum param {
  ROLL = 0,
  PITCH,
  YAW,
  THROTTLE,
  GEAR,
  AUX1,
  AUX2
};

typedef struct flags_t {
    uint8_t OK_TO_ARM;
	uint8_t ARMED;
	//////////////////////
    uint8_t Tuning_MODE;
    uint8_t Write_MODE;

    ///////////////////////
    uint8_t ACC_CALIBRATED;
    uint8_t ANGLE_MODE;
    uint8_t HORIZON_MODE;
    uint8_t MAG_MODE;
    uint8_t BARO_MODE;
	  uint8_t User_MODE;
    uint8_t GPS_HOME_MODE;
    uint8_t GPS_HOLD_MODE;
    uint8_t HEADFREE_MODE;
    uint8_t PASSTHRU_MODE;
    uint8_t GPS_FIX;
    uint8_t GPS_FIX_HOME;
    uint8_t SMALL_ANGLE;
    uint8_t CALIBRATE_MAG;
    uint8_t VARIO_MODE;
    uint8_t FIXED_WING;                     // set when in flying_wing or airplane mode. currently used by althold selection code
    uint8_t MOTORS_STOPPED;
    uint8_t FW_FAILSAFE_RTH_ENABLE;
    uint8_t CLIMBOUT_FW;
    uint8_t CRUISE_MODE;
} flags_t;

typedef struct bat_t {
    uint32_t VBAT_Sense;
    float VBAT;
} bat_t;

typedef struct eeror_t {
  uint8_t error;
  uint8_t error_counter;
  uint8_t error_led;
  uint32_t error_timer;
} eeror_t;

#define CALIBRATING_GYRO_CYCLES             1000
#define CALIBRATING_ACC_CYCLES              400
#define CALIBRATING_BARO_CYCLES             200

#endif
