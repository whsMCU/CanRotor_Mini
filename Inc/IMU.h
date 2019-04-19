#ifndef IMU_H_
#define IMU_H_

#define AHRSIMU_PI              3.141592653f            /*!< PI definition */
#define AHRSIMU_RAD2DEG(x)      ((x) * 57.2957795f)     /*!< Radians to degrees converter */
#define AHRSIMU_DEG2RAD(x)      ((x) * 0.0174532925f)   /*!< Radians to degrees converter */

typedef struct _TM_AHRSIMU_t {
    float Roll;             /*!< Roll angle value. This parameter is in units of degrees */
    float Pitch;            /*!< Pitch angle value. This parameter is in units of degrees */
    float Yaw;              /*!< Yaw angle value. This parameter is in units of degrees */
    float Inclination;      /*!< Inclination in units of degrees */

    float _beta;
    float _q0, _q1, _q2, _q3;
    float _sampleRate;
} TM_AHRSIMU_t;

/**
 * @defgroup TM_AHRSIMU_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * \brief  Initializes ARHS or IMU algorithm for motion calculations
 * \param  *AHRSIMU: Pointer to \ref TM_AHRSIMU_t empty structure
 * \param  sampleRate: Sample rate frequency for updating data
 * \param  beta: Gain for calculations and speed to stabilize. A value less than 0.2 is a good value but it mostly depends on applicaiton.
 * \param  inclination: Magnetic inclination for position on earth in units of degrees. This value depends on GPS coordinates on earth.
 * \retval None
 */
void TM_AHRSIMU_Init(TM_AHRSIMU_t* AHRSIMU, float sampleRate, float beta, float inclination);

/**
 * \brief  Updates AHRS algorithm with new values and calculates roll, pitch and yaw angles.
 * \param  *AHRSIMU: Pointer to \ref TM_AHRSIMU_t empty structure
 * \param  gx, gy, gz: Gyro data for X, Y and Z axis respectively. These parameters must be in units of rad/s. Use \ref AHRSIMU_DEG2RAD if your gyro outputs data in deg/s.
 * \param  ax, ay, az: Accelerometer data for X, Y and Z axis respectively. These parameters must be in units of gees.
 * \param  mx, my, mz: Magnetic data for X, Y and Z axis respectively. These parameters must be in units of uT (micro Tesla).
 *            When all parameters are zero (magnetic sensor not used), \ref TM_AHRSIMU_UpdateIMU is automatically called.
 * \retval None
 */
void TM_AHRSIMU_UpdateAHRS(TM_AHRSIMU_t* AHRSIMU, float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**
 * \brief  Updates IMU algorithm with new values and calculates roll, pitch and yaw angles.
 * \param  *AHRSIMU: Pointer to \ref TM_AHRSIMU_t empty structure
 * \param  gx, gy, gz: Gyro data for X, Y and Z axis respectively. These parameters must be in units of rad/s. Use \ref AHRSIMU_DEG2RAD if your gyro outputs data in deg/s.
 * \param  ax, ay, az: Accelerometer data for X, Y and Z axis respectively. These parameters must be in units of gees.
 * \retval None
 */
void TM_AHRSIMU_UpdateIMU(TM_AHRSIMU_t* AHRSIMU, float gx, float gy, float gz, float ax, float ay, float az);

/**
 * \brief  Sets new gain value for processing
 * \param  *AHRSIMU: Pointer to \ref TM_AHRSIMU_t empty structure
 * \param  beta: New beta value
 * \retval None
 */
#define TM_AHRSIMU_SetBeta(AHRSIMU, beta)       ((AHRSIMU)->_beta = (beta))

void computeIMU(void);

#endif

