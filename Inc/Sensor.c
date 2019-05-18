//#include "Sensor.h"
#include "Board.h"
uint16_t calibratingA = 0;  // the calibration is done in the main loop. Calibrating decreases at each cycle down to 0, then we enter in a normal mode.
uint16_t calibratingB = 200;      // baro calibration = get new ground pressure value
uint16_t calibratingG = 0;

uint8_t rawADC[12];
float magCalibration[3] = {0, 0, 0}, magBias[3] = {0, 0, 0};
float SelfTest[6] = {0, 0, 0};
float gyroBias[3] = {0, 0, 0}, accBias[3] = {0, 0, 0};

uint32_t baroPressureSum = 0;

float magBias[3], magScale[3];
float pre_yaw = 0;
int count = 16;

imu_t imu;
LPS_t lps;
ms5611_t ms5611;


#define ACC_ORIENTATION(X, Y, Z)  {imu.accADC[ROLL]  = X; imu.accADC[PITCH]  = Y; imu.accADC[YAW]  = Z;}
#define GYRO_ORIENTATION(X, Y, Z) {imu.gyroADC[ROLL] = X; imu.gyroADC[PITCH] = Y; imu.gyroADC[YAW] = Z;}
#define MAG_ORIENTATION(X, Y, Z)  {imu.magADC[ROLL]  = X; imu.magADC[PITCH]  = Y; imu.magADC[YAW]  = Z;}

// Specify sensor full scale
uint8_t Gscale = GFS_250DPS;
uint8_t Ascale = AFS_2G;
// Choose either 14-bit or 16-bit magnetometer resolution
uint8_t Mscale = MFS_16BITS;

// 2 for 8 Hz, 6 for 100 Hz continuous magnetometer data read
uint8_t Mmode = M_100HZ;

void getMres()
{
  switch (Mscale)
  {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
      mRes = 10.0f * 4912.0f / 8190.0f; // Proper scale to return milliGauss
      break;
    case MFS_16BITS:
      mRes = 10.0f * 4912.0f / 32760.0f; // Proper scale to return milliGauss
      break;
  }
}

void getGres()
{
  switch (Gscale)
  {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case GFS_250DPS:
      gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      gRes = 2000.0f / 32768.0f;
      break;
  }
}

void getAres()
{
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that
    // 2-bit value:
    case AFS_2G:
      aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      aRes = 16.0f / 32768.0f;
      break;
  }
}

void MPU9250_Init(void)
{
  uint8_t whoami = 0;
    I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_WHO_AM_I, 1, rawADC, 1);
    whoami = rawADC[0];
    sprintf(Buf, "I AM 0x%x\r\n", whoami);
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
    sprintf(Buf, "I SHOULD BE 0x71\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
  if(whoami == 0x71)
  {
		sprintf(Buf, "MPU-9250 Init Start (9-DOF 16-bit mortion sensor)\r\nMPU-9250 SelfTest Start\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

	  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH,MPU9250_RA_PWR_MGMT_1, 7, 8, 0x80); //resetMPU9250
	  MPU9250SelfTest(SelfTest);
		sprintf(Buf, "Self Test Finish\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability

		sprintf(Buf, " x-axis self test: acceleration trim within : %f percent of factory value\r\n",SelfTest[0]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, " y-axis self test: acceleration trim within : %f percent of factory value\r\n",SelfTest[1]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, " z-axis self test: acceleration trim within : %f percent of factory value\r\n",SelfTest[2]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, " x-axis self test: gyration trim within : %f percent of factory value\r\n",SelfTest[3]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, " y-axis self test: gyration trim within : %f percent of factory value\r\n",SelfTest[4]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, " z-axis self test: gyration trim within : %f percent of factory value\r\n",SelfTest[5]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability

		sprintf(Buf, "MPU-9250 Calibration Start\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  calibrateMPU9250(gyroBias, accBias);
		sprintf(Buf, "Calibration Finish\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, "   x       y      z\r\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, " %.2f   %.2f   %.2f mg\r\n", 1000*accBias[0], 1000*accBias[1], 1000*accBias[2]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability
		sprintf(Buf, " %.2f   %.2f   %.2f o/s\r\n", gyroBias[0], gyroBias[1], gyroBias[2]);
		HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	  HAL_Delay(100); // for stability

	// ACC Gyro_Init
	  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x00); // Clear sleep mode bit (6), enable all sensors
    HAL_Delay(100); // for stability
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x01);// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001;
    HAL_Delay(200); // for stability
	  //I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_PWR_MGMT_2, 5, 6, 0x00);
	  //HAL_Delay(100);
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_CONFIG, 7, 8, MPU9250_DLPF_BW_42); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
	  HAL_Delay(100);
    //I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, 0x1B, 1, 2, 0x03);
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_SMPLRT_DIV, 7, 8, 0x04);  //0x00 SMPLRT_DIV    -- SMPLRT_DIV = 0  Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	  HAL_Delay(100);

	  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 1, rawADC, 1);
	  uint8_t c = rawADC[0];
	  // c = c & ~0xE0; // Clear self-test bits [7:5]
	  c = c & ~0x02; // Clear Fchoice bits [1:0]
	  c = c & ~0x18; // Clear AFS bits [4:3]
	  c = c | Gscale << 3; // Set full scale range for the gyro
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, rawADC[0] & ~0xE0); // Clear self-test bits [7:5]
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, rawADC[0] & ~0x18); // Clear AFS bits [4:3]
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, rawADC[0] | (MPU9250_GYRO_FS_250<<3)); //GYRO_CONFIG   -- FS_SEL = 2: Full scale set to 1000 deg/sec
    sprintf(Buf, " GYRO : %d \r\n", c);
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, c);
	  HAL_Delay(100);

    I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 1, rawADC, 1);
    c = rawADC[0];
    // c = c & ~0xE0; // Clear self-test bits [7:5]
    c = c & ~0x18;  // Clear AFS bits [4:3]
    c = c | Ascale << 3; // Set full scale range for the accelerometer
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, rawADC[0] & ~0xE0);// Clear self-test bits [7:5]
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, rawADC[0] & ~0x18); // Clear AFS bits [4:3]
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, rawADC[0] | (MPU9250_ACCEL_FS_2<<3)); //ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
    sprintf(Buf, " ACEL : %d \r\n", c);
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, c);
	  HAL_Delay(100);

    I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG2, 1, rawADC, 1);
    c = rawADC[0];
    c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG2, 7, 8, rawADC[0] & ~0x0F);// Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
//    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG2, 7, 8, rawADC[0] | 0x03);// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
    sprintf(Buf, " ACEL2 : %d \r\n", c);
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG2, 7, 8, c);
		HAL_Delay(100);
//	  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, 0x1A, 2, 3, 0x03); //0x03
//		HAL_Delay(100);
    //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
    //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

   // I2C_BitWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_INT_PIN_CFG, MPU9250_INTCFG_I2C_BYPASS_EN_BIT, ENABLE);  // enable I2C bypass for AUX I2C
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_INT_PIN_CFG, 7, 8, 0x22);
    I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_INT_ENABLE, 7, 8, 0x01);

    //MPU9150_I2C_BitWrite(MPU9150_Address, MPU6050_RA_INT_PIN_CFG, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
    //MPU9150_I2C_BitWrite(MPU9150_Address, MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DATA_RDY_BIT, ENABLE);
	HAL_Delay(100); // for stability

	//Compass_Init
  sprintf(Buf, "AK8963 Init Start!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
  I2C_ByteRead(MPU9250_RA_MAG_ADDRESS, WHO_AM_I_AK8963, 1, rawADC, 1);
  whoami = rawADC[0];
  sprintf(Buf, "I AM 0x%x\r\n", whoami);
  HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
  sprintf(Buf, "I SHOULD BE 0x48\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

	sprintf(Buf, "AK8963 Init (Compass sensor)\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	HAL_Delay(100); // for stability
	initAK8963(magCalibration);
	HAL_Delay(100); // for stability

  getAres();
  getGres();
  getMres();

	sprintf(Buf, "ASAX : %.2f\r\n", magCalibration[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	HAL_Delay(100); // for stability
	sprintf(Buf, "ASAY : %.2f\r\n", magCalibration[1]);
	HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	HAL_Delay(100); // for stability
	sprintf(Buf, "ASAZ : %.2f\r\n", magCalibration[2]);
	HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	HAL_Delay(100); // for stability

#ifdef MAG_cal
  magcalMPU9250(magBias, magScale);
#endif

  }else{
    sprintf(Buf, "Could not connect to MPU9250: %#x\r\n", whoami);
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
    sprintf(Buf, "Communication failed, abort!\r\n");
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

    while (Error.error !=0)
    {
      Error.error = 1;
      error_signal();
      HAL_Delay(4);
    }
  }
}

void Calibrate_gyro(void)
{
	int cal_int = 0;
	uint8_t axis = 0;

	for (cal_int = 0; cal_int < 2000; cal_int ++){
    if (cal_int % 125 == 0) {
      RGB_G_TOGGLE;                                         //Change the led status to indicate calibration.
      sprintf(Buf, ".");
      HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
    }

		Gyro_getADC();
		Mag_getADC();
		for(axis=0; axis<3; axis++)
		{
			imu.gyro_cal[axis] += (float)imu.gyroADC[axis];
		}
	}
	PrintData(2);
	for(axis=0; axis<3; axis++)
	{
		imu.gyro_cal[axis] /= 2000.0f;
	}
	HAL_Delay(100);
	PrintData(2);
}

void calibrate_compass(void)
{

}

void Gyro_getADC(void)
{
	int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_XOUT_H, 1, rawADC, 6);
	 /* Get Angular rate */
	x = ((int16_t)rawADC[0]<<8) | rawADC[1];
	y = ((int16_t)rawADC[2]<<8) | rawADC[3];
	z = ((int16_t)rawADC[4]<<8) | rawADC[5];
	
	GYRO_ORIENTATION(x, y, z);
	
	if(1) GYRO_Common(); //Flight_Status >= 1
}

void GYRO_Common(void)
{
  uint8_t axis =0;
  for(axis=0; axis<3; axis++)
  {
    imu.gyroRaw[axis] = ((float)imu.gyroADC[axis] - imu.gyro_cal[axis]) * gRes;// - gyroBias[axis];

    if(abs(imu.gyroRaw[axis]) <= 5)
      {
        imu.gyroRaw[axis] = 0;
      }
  }
}

void ACC_getADC(void)
{
  int16_t x = 0;
	int16_t y = 0;
	int16_t z = 0;
	I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_XOUT_H, 1, rawADC, 6);
	/* Get acceleration */
	x = ((int16_t)rawADC[0]<<8) | rawADC[1];
	y = ((int16_t)rawADC[2]<<8) | rawADC[3];
	z = ((int16_t)rawADC[4]<<8) | rawADC[5];
	
	ACC_ORIENTATION( x, y, z);
	
	if(1) ACC_Common(); //Flight_Status >= 1
}

void ACC_Common(void)
{
  uint8_t axis = 0;
  static int32_t a[3];

  if(calibratingA>0){
    for(axis=0; axis <3; axis++){
    // Reset a[axis] at start of calibration
      if (calibratingA == 512) a[axis]=0;
      // Sum up 512 readings
      a[axis] +=imu.accADC[axis];
      // Clear global variables for next reading
      imu.accADC[axis]=0;
      imu.acc_cal[axis]=0;
  }
     // Calculate average, shift Z down by acc_1G and store values in EEPROM at end of calibration
  if (calibratingA == 1) {
    imu.acc_cal[ROLL]  = a[ROLL]>>9;
    imu.acc_cal[PITCH] = a[PITCH]>>9;
    imu.acc_cal[YAW]   = (a[YAW]>>9)-(int32_t)aRes;
    sprintf(Buf, "ACC_CALI_DONE : %ld, %ld, %ld\r\n ", imu.acc_cal[ROLL], imu.acc_cal[PITCH], imu.acc_cal[YAW]);
    HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf),1000);
  }
  calibratingA--;
}

  for(axis=0;axis<3;axis++)
  {
    imu.accRaw[axis] = (float)imu.accADC[axis] * aRes;// - accBias[axis];
    //imu.accRaw[axis] = ((float)imu.accADC[axis]-(float)imu.acc_cal[axis]) * AMult;// - accelBias[axis];
  }
}

void Mag_getADC(void)
{
  int16_t x = 0;
  int16_t y = 0;
  int16_t z = 0;
	I2C_ByteRead(MPU9250_RA_MAG_ADDRESS, 0x02, 1, rawADC, 1);
	if( rawADC[0] & 0x01)
	{
	I2C_ByteRead(MPU9250_RA_MAG_ADDRESS, 0x03, 1, rawADC, 7);
	uint8_t c = rawADC[6];

	if(!(c & 0x08)){
	x = ((int16_t)rawADC[1]<<8) | rawADC[0];
	y = ((int16_t)rawADC[3]<<8) | rawADC[2];
	z = ((int16_t)rawADC[5]<<8) | rawADC[4];
	MAG_ORIENTATION( x, y, z);
	if(1) MAG_Common();//Flight_Status >= 1
		}
	}
}

void MAG_Common(void)
{
  uint8_t axis = 0;
  for(axis=0;axis<3;axis++)
  {
    imu.magRaw[axis] = (float)imu.magADC[axis] * mRes * magCalibration[axis] - magBias[axis];
  }
}

void Temp_getADC(void)
{
	I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_TEMP_OUT_H, 1, rawADC, 2);
	/* Get acceleration */
	imu.rawTemp = ((int16_t)rawADC[0]<<8) | rawADC[1];
	imu.Temp = ((float)imu.rawTemp / 337.87f) + 21.0f;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(float * dest1, float * dest2)
{
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x80);// Write a one to bit 7 reset bit; toggle reset device
  HAL_Delay(100); // for stability
// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x01);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_PWR_MGMT_2, 7, 8, 0x00);
  HAL_Delay(200); // for stability

// Configure device for bias calculation
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_INT_ENABLE, 7, 8, 0x00);// Disable all interrupts
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_FIFO_EN, 7, 8, 0x00);// Disable FIFO
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_PWR_MGMT_1, 7, 8, 0x00);// Turn on internal clock source
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_I2C_MST_CTRL, 7, 8, 0x00);// Disable I2C master
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_USER_CTRL, 7, 8, 0x00);// Disable FIFO and I2C master modes
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_USER_CTRL, 7, 8, 0x0C);// Reset FIFO and DMP
  HAL_Delay(15); // for stability

// Configure MPU9250 gyro and accelerometer for bias calculation.

  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_CONFIG, 7, 8, 0x01); // Set low-pass filter to 188 Hz
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_SMPLRT_DIV, 7, 8, 0x00);// Set sample rate to 1 kHz
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, 0x00);// Set gyro full-scale to 250 degrees per second, maximum sensitivity
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, 0x00);// Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_USER_CTRL, 7, 8, 0x40); // Enable FIFO
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_FIFO_EN, 7, 8, 0x78); // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
  HAL_Delay(40); // accumulate 40 samples in 80 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_FIFO_EN, 7, 8, 0x00);// Disable gyro and accelerometer sensors for FIFO
  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_FIFO_COUNTH, 1, rawADC, 2);// read FIFO sample count
  fifo_count = ((uint16_t)rawADC[0] << 8) | rawADC[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH,MPU9250_RA_FIFO_R_W, 1, rawADC, 12);// read data for averaging

    accel_temp[0] = (int16_t) (((int16_t)rawADC[0] << 8) | rawADC[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)rawADC[2] << 8) | rawADC[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)rawADC[4] << 8) | rawADC[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)rawADC[6] << 8) | rawADC[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)rawADC[8] << 8) | rawADC[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)rawADC[10] << 8) | rawADC[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

}
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

/// Push gyro biases to hardware registers
/*  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_XG_OFFS_USRH, 7, 8, data[0]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_XG_OFFS_USRL, 7, 8, data[1]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_YG_OFFS_USRH, 7, 8, data[2]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_YG_OFFS_USRL, 7, 8, data[3]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ZG_OFFS_USRH, 7, 8, data[4]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ZG_OFFS_USRL, 7, 8, data[5]);
*/
	
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_XA_OFFSET_H, 1, rawADC, 2);// Read factory accelerometer trim values
  accel_bias_reg[0] = (int16_t) ((int16_t)rawADC[0] << 8) | rawADC[1];
  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_YA_OFFSET_H, 1, rawADC, 2);
  accel_bias_reg[1] = (int16_t) ((int16_t)rawADC[0] << 8) | rawADC[1];
  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ZA_OFFSET_H, 1, rawADC, 2);
  accel_bias_reg[2] = (int16_t) ((int16_t)rawADC[0] << 8) | rawADC[1];

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
    if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
/*  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_XA_OFFSET_H, 7, 8, data[0]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_XA_OFFSET_L, 7, 8, data[1]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_YA_OFFSET_H, 7, 8, data[2]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_YA_OFFSET_L, 7, 8, data[3]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ZA_OFFSET_H, 7, 8, data[4]);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ZA_OFFSET_L, 7, 8, data[5]);*/


// Output scaled accelerometer biases for manual subtraction in the main program
   dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
   dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
   dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}
void initAK8963(float * destination)
{
	// First extract the factory calibration for each magnetometer axis
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, AK8963_CNTL, 7, 8, 0x00);// Power down magnetometer
	  HAL_Delay(10);
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, AK8963_CNTL, 7, 8, 0x0F);// Enter Fuse ROM access mode
	  HAL_Delay(10);
	  I2C_ByteRead(MPU9250_RA_MAG_ADDRESS, AK8963_ASAX, 1, rawADC, 3);// Read the x-, y-, and z-axis calibration values
	  destination[0] =  (float)(rawADC[0] - 128)/256.0f + 1.0f;   // Return x-axis sensitivity adjustment values, etc.
	  destination[1] =  (float)(rawADC[1] - 128)/256.0f + 1.0f;
	  destination[2] =  (float)(rawADC[2] - 128)/256.0f + 1.0f;
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, AK8963_CNTL, 7, 8, 0x00);// Power down magnetometer
	  HAL_Delay(10);
	  // Configure the magnetometer for continuous read and highest resolution
	  // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	  // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	  I2C_ByteWrite(MPU9250_RA_MAG_ADDRESS, AK8963_CNTL, 7, 8, Mscale << 4 | Mmode); // Set magnetometer data resolution and sample ODR
	  HAL_Delay(10);
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU9250SelfTest(float * destination) // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
   uint8_t selfTest[6] = {0, 0, 0, 0, 0, 0};
   uint16_t i, ii;
   int16_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
   float factoryTrim[6];
   uint8_t FS = GFS_250DPS;

   I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_SMPLRT_DIV, 7, 8, 0x00);// Set gyro sample rate to 1 kHz
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_CONFIG, 7, 8, 0x02);// Set gyro sample rate to 1 kHz and DLPF to 92 Hz
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, FS<<3);// Set full scale range for the gyro to 250 dps
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, 0x1D, 7, 8, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, FS<<3);// Set full scale range for the accelerometer to 2 g

  for(ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_XOUT_H, 1, rawADC, 6);  // Read the six raw data registers into data array
  aAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  aAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;

  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_XOUT_H, 1, rawADC, 6); // Read the six raw data registers sequentially into data array
  gAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  gAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;
  }

  for (ii =0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
  aAvg[ii] /= 200;
  gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
   I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
   HAL_Delay(25); // Delay a while to let the device stabilize

  for(ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_XOUT_H, 1, rawADC, 6); // Read the six raw data registers into data array
  aSTAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  aSTAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  aSTAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;

  I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_XOUT_H, 1, rawADC, 6); // Read the six raw data registers sequentially into data array
  gSTAvg[0] += (int16_t)(((int16_t)rawADC[0] << 8) | rawADC[1]) ; // Turn the MSB and LSB into a signed 16-bit value
  gSTAvg[1] += (int16_t)(((int16_t)rawADC[2] << 8) | rawADC[3]) ;
  gSTAvg[2] += (int16_t)(((int16_t)rawADC[4] << 8) | rawADC[5]) ;
  }

  for (ii =0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
  aSTAvg[ii] /= 200;
  gSTAvg[ii] /= 200;
  }

 // Configure the gyro and accelerometer for normal operation
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_ACCEL_CONFIG, 7, 8, 0x00);
  I2C_ByteWrite(MPU9250_ADDRESS_AD0_HIGH, MPU9250_RA_GYRO_CONFIG, 7, 8, 0x00);
  HAL_Delay(25); // Delay a while to let the device stabilize

   // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
   I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, 0x0D, 1, rawADC, 3);
   selfTest[0] = rawADC[0]; // X-axis accel self-test results
   selfTest[1] = rawADC[1]; // Y-axis accel self-test results
   selfTest[2] = rawADC[2]; // Z-axis accel self-test results
   I2C_ByteRead(MPU9250_ADDRESS_AD0_HIGH, 0x00, 1, rawADC, 3);
   selfTest[3] = rawADC[0]; // X-axis gyro self-test results
   selfTest[4] = rawADC[1]; // Y-axis gyro self-test results
   selfTest[5] = rawADC[2]; // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
   factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
   factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
   factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
   factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
   factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
   factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

 // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
 // To get percent, must multiply by 100
   for (i = 0; i < 3; i++) {
     destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i] - 100.0; // Report percent differences
     destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3] - 100.0; // Report percent differences
   }
}

void magcalMPU9250(float * dest1, float * dest2)
 {
	int jj = 0;
 uint16_t ii = 0, sample_count = 0;
 int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
 int16_t mag_max[3] = {0x8000, 0x8000, 0x8000}, mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF}, mag_temp[3] = {0, 0, 0};

 sprintf(Buf, "Mag Calibration: Wave device in a figure 8 until done!\r\n");
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 sprintf(Buf, "4 seconds to get ready followed by 15 seconds of sampling\r\n");
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 HAL_Delay(4000);

 // shoot for ~fifteen seconds of mag data
 // at 8 Hz ODR, new mag data is available every 125 ms
 if (Mmode == M_8HZ)
 {
   sample_count = 128;
 }
 // at 100 Hz ODR, new mag data is available every 10 ms
 if (Mmode == M_100HZ)
 {
   sample_count = 1500;
 }

 for(ii = 0; ii < sample_count; ii++) {
  Mag_getADC();  // Read the mag data
  mag_temp[0] = imu.magADC[0];
  mag_temp[1] = imu.magADC[1];
  mag_temp[2] = imu.magADC[2];
 for (jj = 0; jj < 3; jj++) {
  if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
  if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
 }

 if (Mmode == M_8HZ)
 {
   HAL_Delay(135); // At 8 Hz ODR, new mag data is available every 125 ms
 }
 if (Mmode == M_100HZ)
 {
   HAL_Delay(12);  // At 100 Hz ODR, new mag data is available every 10 ms
 }
 }

// Get hard iron correction
 mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
 mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
 mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

 dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
 dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
 dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

// Get soft iron correction estimate
 mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
 mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
 mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

 float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
 avg_rad /= 3.0;

 dest2[0] = avg_rad/((float)mag_scale[0]);
 dest2[1] = avg_rad/((float)mag_scale[1]);
 dest2[2] = avg_rad/((float)mag_scale[2]);
 sprintf(Buf, "Mag Calibration done!\r\n");
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

 sprintf(Buf, " AK8963 mag biase(x): %f mG\r\n",dest1[0]);
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 sprintf(Buf, " AK8963 mag biase(y): %f mG\r\n",dest1[1]);
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 sprintf(Buf, " AK8963 mag biase(z): %f mG\r\n",dest1[2]);
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

 sprintf(Buf, " AK8963 mag Scale(x): %f mG\r\n",dest2[0]);
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 sprintf(Buf, " AK8963 mag Scale(y): %f mG\r\n",dest2[1]);
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 sprintf(Buf, " AK8963 mag Scale(z): %f mG\r\n",dest2[2]);
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 sprintf(Buf, "gyro calibration start wait 5 second.!\r\n");
 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
 HAL_Delay(4000);

 }

alt_t alt;

void MS5611_Init(ms5611_osr_t osr)
{
	sprintf(Buf, "Initialize MS-5611 Sensor\r\n");
	HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

  I2C_Write(MS5611_ADDRESS, MS5611_CMD_RESET, 1);
  // Set oversampling value
  switch (osr)
  {
    case MS5611_ULTRA_LOW_POWER:
      ms5611.ct = 1;
      break;
    case MS5611_LOW_POWER:
      ms5611.ct = 2;
      break;
    case MS5611_STANDARD:
      ms5611.ct = 3;
      break;
    case MS5611_HIGH_RES:
      ms5611.ct = 5;
      break;
    case MS5611_ULTRA_HIGH_RES:
      ms5611.ct = 10;
      break;
  }
  ms5611.uosr = osr;
  HAL_Delay(100);

  for (uint8_t offset = 0; offset < 6; offset++)
  {
    ms5611.fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
  }

  sprintf(Buf, "Oversampling: %d\r\n", ms5611.uosr);
  HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

}

void Baro_Common(void)
{
  static int32_t baroHistTab[BARO_TAB_SIZE_MAX];
  static int baroHistIdx = 0;

  uint8_t indexplus1 = (baroHistIdx + 1);
  if (indexplus1 == BARO_TAB_SIZE_MAX) indexplus1 = 0;
  baroHistTab[baroHistIdx] = ms5611.realPressure;
  baroPressureSum += baroHistTab[baroHistIdx];
  baroPressureSum -= baroHistTab[indexplus1];
  baroHistIdx = indexplus1;
}

uint8_t Baro_update(void)
{
    static uint32_t baroDeadline = 0;
    static int state = 0;

       if(state >= 2){
         state = 0;
         MS561101BA_Calculate();
         return 1;
       }
       if ((int32_t)(loop_timer - baroDeadline) < 0) return 0;
       baroDeadline = loop_timer;
       if (state == 0) {
         Baro_Common();
         ms5611.rawTemp = readRegister24(MS5611_CMD_ADC_READ);
        //Request pressure data
         I2C_Write(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + ms5611.uosr, 1);
         baroDeadline +=10000;
       } else {
         ms5611.rawPressure = readRegister24(MS5611_CMD_ADC_READ);
        //Request temperature data
         I2C_Write(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + ms5611.uosr, 1);
         baroDeadline +=10000;
       }
       state ++;
    return 1;
}

uint8_t getEstimatedAltitude(void)
{
  static float baroGroundTemperatureScale,logBaroGroundPressureSum;
  static uint16_t previousT;
  uint16_t currentT = micros();
  uint16_t dTime;

  dTime = currentT - previousT;
  if (dTime < 25000) return 0;
  previousT = currentT;
  if (calibratingB > 0) {

    logBaroGroundPressureSum = log(baroPressureSum);
    baroGroundTemperatureScale = ((int32_t)ms5611.realTemperature + 27315) * (2 * 29.271267f); // 2 *  is included here => no need for * 2  on BaroAlt in additional LPF
    calibratingB--;
  }

  // baroGroundPressureSum is not supposed to be 0 here
  // see: https://code.google.com/p/ardupilot-mega/source/browse/libraries/AP_Baro/AP_Baro.cpp
  ms5611.BaroAlt = ( logBaroGroundPressureSum - log(baroPressureSum) ) * baroGroundTemperatureScale;

  alt.EstAlt = (alt.EstAlt * 6 + ms5611.BaroAlt ) >> 3; // additional LPF to reduce baro noise (faster by 30 µs)

  return 1;
}
uint32_t readRawTemperature(void)
{
    I2C_Write(MS5611_ADDRESS, MS5611_CMD_CONV_D2 + ms5611.uosr, 1);

    HAL_Delay(ms5611.ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t readRawPressure(void)
{
    I2C_Write(MS5611_ADDRESS, MS5611_CMD_CONV_D1 + ms5611.uosr, 1);

    HAL_Delay(ms5611.ct);

    return readRegister24(MS5611_CMD_ADC_READ);
}

void MS561101BA_Calculate(void){
  uint32_t D1 = ms5611.rawPressure;

  uint32_t D2 = ms5611.rawTemp;
  int32_t dT = D2 - (uint32_t)ms5611.fc[4] * 256;

  int64_t OFF = (int64_t)ms5611.fc[1] * 65536 + (int64_t)ms5611.fc[3] * dT / 128;
  int64_t SENS = (int64_t)ms5611.fc[0] * 32768 + (int64_t)ms5611.fc[2] * dT / 256;

  int32_t TEMP = 2000 + ((int64_t) dT * ms5611.fc[5]) / 8388608;
  ms5611.realTemperature = TEMP;

  ms5611.OFF2 = 0;
  ms5611.SENS2 = 0;

  if (TEMP < 2000)
  {
    ms5611.OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
    ms5611.SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
  }

  if (TEMP < -1500)
  {
    ms5611.OFF2 = ms5611.OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
    ms5611.SENS2 = ms5611.SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
  }

  OFF = OFF - ms5611.OFF2;
  SENS = SENS - ms5611.SENS2;

  ms5611.realPressure = (D1 * SENS / 2097152 - OFF) / 32768;
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t readRegister16(uint8_t reg)
{
    uint16_t value;
    I2C_Write(MS5611_ADDRESS, reg, 1);

    I2C_Read(MS5611_ADDRESS, rawADC, 2);

    uint8_t vha = rawADC[0];
    uint8_t vla = rawADC[1];

    value = vha << 8 | vla;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t readRegister24(uint8_t reg)
{
    uint32_t value;
    I2C_Write(MS5611_ADDRESS, reg, 1);

    I2C_Read(MS5611_ADDRESS, rawADC, 3);

    uint8_t vxa = rawADC[0];
    uint8_t vha = rawADC[1];
    uint8_t vla = rawADC[2];

    value = ((int32_t)vxa << 16) | ((int32_t)vha << 8) | vla;

    return value;
}
