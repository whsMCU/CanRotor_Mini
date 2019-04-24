//#include "I2C.h"
#include "Board.h"

void I2C_ByteWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitStart, uint8_t length, uint8_t data)
{
	 // 010 value to write
	// 76543210 bit numbers
	// xxx   args: bitStart=4, length=3
	// 00011100 mask byte
	// 10101111 original value (sample)
	// 10100011 original & ~mask
	// 10101011 masked | value
	uint8_t tmp;
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
	data <<= (bitStart - length + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tmp &= ~(mask); // zero all important bits in existing byte
	tmp |= data; // combine data with existing byte
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 10);
}

void I2C_BitWrite(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitNum, uint8_t data)
{
	uint8_t tmp;
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
	tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
	HAL_I2C_Mem_Write(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
}

//void I2C_ByteRead(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitStart, uint8_t length, uint8_t *data)
//{
//	// 01101001 read byte
//	// 76543210 bit numbers
//	//    xxx   args: bitStart=4, length=3
//	//    010   masked
//	//   -> 010 shifted
//	uint8_t tmp;
//	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
//	uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
//	tmp &= mask;
//	tmp >>= (bitStart - length + 1);
//	*data = tmp;
//}

void I2C_ByteRead(uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size)
{
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, MemAddSize, pData, Size, 1);
}

void I2C_BitRead(uint8_t DevAddress, uint8_t MemAddress, uint8_t bitNum, uint8_t *data)
{
	uint8_t tmp;
	HAL_I2C_Mem_Read(&hi2c2, DevAddress, MemAddress, 1, &tmp, 1, 1);
	*data = tmp & (1 << bitNum);
}

void I2C_Write(uint16_t DevAddress, uint8_t data, uint16_t Size)
{
  HAL_StatusTypeDef state;
  uint32_t ErrorCode;

  if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY){

  state = HAL_I2C_Master_Transmit(&hi2c2, DevAddress, &data, Size, 1000);
  ErrorCode = hi2c2.ErrorCode;
   while(state)
   {
     sprintf(Buf, "MS5611_Tx_Error : %d, %ld\r\n", state, ErrorCode);
     HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
     while (Error.error !=0)
     {
       Error.error = 2;
       error_signal();
       HAL_Delay(4);
     }
   }
  }
  //while(HAL_I2C_Master_Transmit(&hi2c2, DevAddress, &data, Size, 1) != HAL_OK);
}

void I2C_Read(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
  HAL_StatusTypeDef state;
  uint32_t ErrorCode;

  if(HAL_I2C_GetState(&hi2c2) == HAL_I2C_STATE_READY){

  state = HAL_I2C_Master_Receive(&hi2c2, DevAddress, pData, Size, 1000);
  ErrorCode = hi2c2.ErrorCode;
    while(state)
    {
      sprintf(Buf, "MS5611_Rx_Error : %d, %ld\r\n", state, ErrorCode);
      HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
      while (Error.error !=0)
      {
        Error.error = 3;
        error_signal();
        HAL_Delay(4);
      }
    }
  }
  //while(HAL_I2C_Master_Receive(&hi2c2, DevAddress, pData, Size, 1) != HAL_OK);
}
