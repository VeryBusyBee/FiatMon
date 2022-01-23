/*
 * fonts.h
 *
 *  Created on: 13 січ. 2021 р.
 *      Author: Victor
 */

#ifndef DISP_FONTS_H_
#define DISP_FONTS_H_

#include "stdint.h"

class Font
{
public:
	uint8_t width;
    uint8_t height;
    uint8_t first_char;
    uint8_t count;
    uint8_t *widths;
    uint8_t *data;
    uint8_t space;

private:
    uint16_t curr_char_ptr;	//the address of the glyph is being drawn
    uint8_t curr_char_bytes;	//total bytes of the glyph is being drawn

public:
    Font();
    Font(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t *, uint8_t *, uint8_t);

	uint8_t getWidth(uint8_t);
	uint8_t getWidth();
	uint8_t setChar(uint8_t);
	uint8_t getCharBytes(uint8_t);
	uint8_t getCharData(uint8_t, uint8_t);
	uint8_t getCharData(uint8_t);
};

#endif /* DISP_FONTS_H_ */
