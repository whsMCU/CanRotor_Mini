/*
 * GFX.h
 *
 *  Created on: 2017. 5. 16.
 *      Author: KSKim
 */

#ifndef GFX_H_
#define GFX_H_

#include "ssd1306.h"

void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void drawCircleHelper( int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
void fillScreen(uint16_t color);
void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color);
void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
int16_t width(void);
int16_t height(void);


#endif /* GFX_H_ */
