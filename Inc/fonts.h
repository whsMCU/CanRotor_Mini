/*
 * fonts.h
 *
 *  Created on: 2017. 5. 15.
 *      Author: KSKim
 */

#ifndef FONTS_H_
#define FONTS_H_

typedef struct {
	const uint8_t FontWidth;
	uint8_t FontHeight;
	const uint16_t *data;
} FontDef;

extern FontDef Font_7x10;
extern FontDef Font_11x18;
extern FontDef Font_16x26;


#endif /* FONTS_H_ */
