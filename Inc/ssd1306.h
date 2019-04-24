#ifndef SSD1306_H_
#define SSD1306_H_

#include "PHan_Lib.h"

#define BLACK 	0
#define WHITE 	1
#define INVERSE 2

#define SSD1306_I2C_ADDRESS           0x3C  // 011110+SA0+RW - 0x3C or 0x3D

#define SSD1306_128_64

#if defined SSD1306_128_64
  #define SSD1306_LCDWIDTH		      	128
  #define SSD1306_LCDHEIGHT		      	64
#endif

#define SSD1306_SETCONTRAST 	      	0x81
#define SSD1306_DISPLAYALLON_RESUME   0xA4
#define SSD1306_DISPLAYALLON 		      0xA5
#define SSD1306_NORMALDISPLAY 	    	0xA6
#define SSD1306_INVERTDISPLAY 		    0xA7
#define SSD1306_DISPLAYOFF 			      0xAE
#define SSD1306_DISPLAYON 			      0xAF

#define SSD1306_SETDISPLAYOFFSET 	    0xD3
#define SSD1306_SETCOMPINS 			      0xDA

#define SSD1306_SETVCOMDETECT 		    0xDB

#define SSD1306_SETDISPLAYCLOCKDIV  	0xD5
#define SSD1306_SETPRECHARGE 		      0xD9

#define SSD1306_SETMULTIPLEX 		      0xA8

#define SSD1306_SETLOWCOLUMN 		      0x00
#define SSD1306_SETHIGHCOLUMN 		    0x10

#define SSD1306_SETSTARTLINE 		      0x40

#define SSD1306_MEMORYMODE 			      0x20
#define SSD1306_COLUMNADDR 			      0x21
#define SSD1306_PAGEADDR   			      0x22

#define SSD1306_COMSCANINC	 		      0xC0
#define SSD1306_COMSCANDEC 			      0xC8

#define SSD1306_SEGREMAP 			        0xA0

#define SSD1306_CHARGEPUMP 			      0x8D

#define SSD1306_EXTERNALVCC 		      0x01
#define SSD1306_SWITCHCAPVCC 		      0x02

// Scrolling #defines
#define SSD1306_ACTIVATE_SCROLL 					          	0x2F
#define SSD1306_DEACTIVATE_SCROLL 					        	0x2E
#define SSD1306_SET_VERTICAL_SCROLL_AREA 			      	0xA3
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 			      	0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 					      0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 	0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 	0x2A


int8_t _i2caddr, _vccstate;
uint8_t _width, _height;

typedef struct {
	uint16_t CurrentX;
	uint16_t CurrentY;
	uint8_t Inverted;
	uint8_t Initialized;
} SSD1306_t;

void ssd1306_command(uint8_t command);
void ssd1306_begin(uint8_t vccstate, uint8_t i2caddr);
void display(void);
void clearDisplay(void);
void drawPixel(uint8_t x, uint8_t y, uint8_t color);
void invertDisplay(uint8_t i);
void startscrollright(uint8_t start, uint8_t stop);
void startscrollleft(uint8_t start, uint8_t stop);
void startscrolldiagright(uint8_t start, uint8_t stop);
void startscrolldiagleft(uint8_t start, uint8_t stop);
void stopscroll(void);
void dim(uint8_t dim);

void OLed_printf(int x, int y,  const char *fmt, ...);
void disHanFont(int x, int y, PHAN_FONT_OBJ *FontPtr);


#endif /* SSD1306_H_ */
