#ifndef __EEPROM_H
#define __EEPROM_H

typedef struct {
	unsigned long errors;
	unsigned long address;
} eeprom_t;

#define EEPROM_ADDRESS 0xA0
#define ADDRESS_SIZE 2
//#define PAGE_SIZE 32 //AT24C32, 64
#define PAGE_SIZE 64  //AT24C128, 256

void EEPROM_Test();

void EEPROM_Init();

void write(unsigned int address, uint8_t data);
void write_1(unsigned int address, uint8_t *data, int n);
void writeInt(unsigned int address, unsigned int data);
void writeLong(unsigned int address, unsigned long data);
void writeFloat(unsigned int address, float data);
void writeDouble(unsigned int address, double data);
void writeChars(unsigned int address, char *data, int length);
uint8_t read(unsigned int address);
void read_1(unsigned int address, uint8_t *data, int n);
unsigned int readInt(unsigned int address);
unsigned long readLong(unsigned int address);
float readFloat(unsigned int address);
double readDouble(unsigned int address);
void readChars(unsigned int address, char *data, int n);

void read_2(unsigned int address, uint8_t *data, int offset, int n);
void write_2(unsigned int address, uint8_t *data, int offset, int n);

#endif
