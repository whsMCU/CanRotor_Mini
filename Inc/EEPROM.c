/*
 * EEPROM.c
 *
 *  Created on: 2019. 1. 20.
 *      Author: WANG
 */
//#include "EEPROM.h"
#include "Board.h"

eeprom_t eeprom;
uint8_t _b[8];


void EEPROM_Test()
{
  // read and write byte
	 sprintf(Buf, "Write 42 to address 12\r\n");
	 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	 write(12, 42);
	 sprintf(Buf, "Read byte from address 12 ...\r\n");
	 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
     uint8_t b = read(12);
	 sprintf(Buf, "read value : %d\r\n\n", b);
	 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

	 // read and write integer
	 sprintf(Buf, "Write 65000 to address 15\r\n");
	 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	 writeInt(15, 65000);
	 sprintf(Buf, "Read integer from address 15 ...\r\n");
	 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
	 unsigned int i = readInt(15);
	 sprintf(Buf, "read value : %d\r\n\n", i);
	 HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

	 // read and write long
   sprintf(Buf, "Write 3293732729 to address 20\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   writeLong(20, 3293732729UL);
   sprintf(Buf, "Read long from address 20 ...\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   unsigned long l = readLong(20);
   sprintf(Buf, "read value : %lu\r\n\n", l);
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);


   // read and write long
   sprintf(Buf, "Write 1111111111 to address 31\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   writeLong(31, 1111111111);
   sprintf(Buf, "Read long from address 31 ...\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   unsigned long l2 = readLong(31);
   sprintf(Buf, "read value : %lu\r\n\n", l2);
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);


   // read and write float
   sprintf(Buf, "Write 3.14 to address 40\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   writeFloat(40, 3.14);
   sprintf(Buf, "Read float from address 40 ...\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   float f = readFloat(40);
   sprintf(Buf, "read value : %f\r\n\n", f);
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

   // read and write double
   sprintf(Buf, "Write 3.14159265359 to address 50\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   writeDouble(50, 3.14159265359);
   sprintf(Buf, "Read double from address 50 ...\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   double d = readDouble(50);
   sprintf(Buf, "read value : %f\r\n\n", d);
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

   // read and write char
   sprintf(Buf, "Write chars: '");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   char msg[] = "This is a message";
   sprintf(Buf, "%s", msg);
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   sprintf(Buf, "' to address 200\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   writeChars(200, msg, sizeof(msg));
   sprintf(Buf, "Read chars from address 200 ...\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   char msg2[30];
   readChars(200, msg2, sizeof(msg2));
   sprintf(Buf, "read value : '%s'\r\n\n", msg2);
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

   // write array of bytes
   sprintf(Buf, "Write array of 80 bytes at address 1000\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   uint8_t xy[] = {0,0,0,1,1,1,2,2,2,3,3,3,4,4,4,5,5,5,6,6,6,7,7,7,8,8,8,9,9,9,    // 10 x 3 = 30
                  10,11,12,13,14,15,16,17,18,19,                                   //          10
                  120,121,122,123,124,125,126,127,128,129,                         //          10
                  130,131,132,133,134,135,136,137,138,139,                         //          10
                  200,201,202,203,204,205,206,207,208,209,                         //          10
                  210,211,212,213,214,215,216,217,218,219};                        //          10
   write_1(1000, (uint8_t*)xy, sizeof(xy));

   // read bytes with multiple steps
   sprintf(Buf, "Read 80 single bytes starting at address 1000\r\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   for (int i=0; i<sizeof(xy); i++) {
     uint8_t sb = read(1000+i);
     sprintf(Buf, "[ %d ] = %d\r\n", 1000+i, sb);
     HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   }
   sprintf(Buf, "\n");
   HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);

   // read bytes with one step
   uint8_t z[80];
   memset(&z[0], 32, sizeof(z));
   read_1(1000, z, sizeof(z));
   for (int i=0; i<sizeof(z); i++) {
     sprintf(Buf, "[ %d ] = %d\r\n", 1000+i, z[i]);
     HAL_UART_Transmit(&huart2, (uint8_t*)Buf, strlen(Buf), 1000);
   }
}

void EEPROM_Init()
{
   for(int i = 0; i < 3; i++){
     pid.kp[i] = readFloat( 0 + (4*i));
     pid.ki[i] = readFloat(12 + (4*i));
     pid.kd[i] = readFloat(24 + (4*i));

     pid.kp1[i] = readFloat(36 + (4*i));
     pid.ki1[i] = readFloat(48 + (4*i));
     pid.kp2[i] = readFloat(60 + (4*i));
     pid.ki2[i] = readFloat(72 + (4*i));
     pid.kd2[i] = readFloat(84 + (4*i));

     pid.kp_rate[i] = readFloat(96 + (4*i));
     pid.ki_rate[i] = readFloat(108 + (4*i));
     pid.kd_rate[i] = readFloat(120 + (4*i));

   }
   magBias[0] = readFloat(132);
   magBias[1] = readFloat(136);
   magBias[2] = readFloat(140);
   magScale[0] = readFloat(144);
   magScale[1] = readFloat(148);
   magScale[2] = readFloat(152);
}

uint8_t EEPROM_read(uint32_t dataAddress)
{
  uint8_t pData[1] = {0};
  HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, dataAddress, ADDRESS_SIZE, pData, 1, 1000);
  HAL_Delay(1);
  return pData[0];
}

void EEPROM_readPage(uint32_t dataAddress, uint8_t *pData, uint16_t ReadSize)
{
  HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, dataAddress, ADDRESS_SIZE, pData, ReadSize, 1000);
}

/**
 * Write byte
 */
void write(unsigned int address, uint8_t data) {
  uint8_t* tmp;
  tmp = &data;
  HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, address, ADDRESS_SIZE, tmp, 1, 1000);
  HAL_Delay(5);
}

/**
 * Write integer
 */
void writeInt(unsigned int address, unsigned int data) {
  write_1(address, (uint8_t*)&data, 2);
}

/**
 * Write long
 */
void writeLong(unsigned int address, unsigned long data) {
  write_1(address, (uint8_t*)&data, 4);
}

/**
 * Write float
 */
void writeFloat(unsigned int address, float data) {
  write_1(address, (uint8_t*)&data, 4);
}

/**
 * Write double
 */
void writeDouble(unsigned int address, double data) {
  write_1(address, (uint8_t*)&data, 8);
}

/**
 * Write chars
 */
void writeChars(unsigned int address, char *data, int length) {
  write_1(address, (uint8_t*)data, length);
}

/**
 * Read byte
 */
uint8_t read(unsigned int address) {
  uint8_t b[1];
  HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, address, ADDRESS_SIZE, b, 1, 1000);
  return *b;
}

/**
 * Read integer
 */
unsigned int readInt(unsigned int address) {
  read_1(address, _b, 2);
  return *(unsigned int*)&_b[0];
}

/**
 * Read long
 */
unsigned long readLong(unsigned int address) {
  read_1(address, _b, 4);
  return *(unsigned long*)&_b[0];
}

/**
 * Read float
 */
float readFloat(unsigned int address) {
  read_1(address, _b, 4);
  return *(float*)&_b[0];
}

/**
 * Read double
 */
double readDouble(unsigned int address) {
  read_1(address, _b, 8);
  return *(double*)&_b[0];
}

/**
 * Read chars
 */
void readChars(unsigned int address, char *data, int n) {
  read_1(address, (uint8_t*)data, n);
}

/**
 * Write sequence of n bytes
 */
void write_1(unsigned int address, uint8_t *data, int n) {
  // status quo
  int c = n;            // bytes left to write
  int offD = 0;         // current offset in data pointer
  int offP;           // current offset in page
  int nc = 0;           // next n bytes to write

  // write alle bytes in multiple steps
  while (c > 0) {
    // calc offset in page
    offP = address % PAGE_SIZE;
    // maximal 30 bytes to write
    nc = min(min(c, 30), PAGE_SIZE - offP);
    write_2(address, data, offD, nc);
    c-=nc;
    offD+=nc;
    address+=nc;
  }
}

/**
 * Write sequence of n bytes from offset
 */
void write_2(unsigned int address, uint8_t *data, int offset, int n) {
    uint8_t *adr = data+offset;
    HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, address, ADDRESS_SIZE, adr, n, 1000);
    HAL_Delay(10);
}

/**
 * Read sequence of n bytes
 */
void read_1(unsigned int address, uint8_t *data, int n) {
  int c = n;
  int offD = 0;
  // read until are n bytes read
  while (c > 0) {
    // read maximal 32 bytes
    int nc = c;
    if (nc > 32)
      nc = 32;
    read_2(address, data, offD, nc);
    address+=nc;
    offD+=nc;
    c-=nc;
  }
}


/**
 * Read sequence of n bytes to offset
 */
void read_2(unsigned int address, uint8_t *data, int offset, int n) {
  int r = 0;
  uint8_t pData[80];
  memset(&pData[0], n, sizeof(pData));
  HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, address, ADDRESS_SIZE, pData, n, 1000);
  while (r<n) {
    data[offset+r] = pData[r];
    r++;
  }
}

