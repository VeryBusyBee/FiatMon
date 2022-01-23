/*
 * fonts.cpp
 *
 *  Created on: 26 січ. 2021 р.
 *      Author: Victor
 */
#include "fonts.h"


Font::Font()
{
	width = 5;
	height = 7;
    first_char = ' ';
    count = 96;
    widths = 0;
    data = 0;
    space = 0;
}

Font::Font(uint8_t w, uint8_t h, uint8_t fc, uint8_t cn, uint8_t *wds, uint8_t *dt, uint8_t sp)
{
	width = w;
	height = h;
    first_char = fc;
    count = cn;
    widths = wds;
    data = dt;
    space = sp;
}

uint8_t Font::getWidth()
{
	return this->width;
}

uint8_t Font::getWidth(uint8_t ch)
{
	uint8_t w;

	if (this->widths != 0)
	{
		if (ch < this->first_char || ch > (this->first_char + this->count)) {
			ch = '4';	//bytes for a widest char
		}

		w = this->widths[ch - this->first_char];
		return w;
	}
	else return this->width;

}

uint8_t Font::getCharData(uint8_t ch, uint8_t idx)
{
	uint8_t dt, w, char_bytes;
	uint16_t ch_addr = 0;

	ch -= this->first_char;

	if (this->widths != 0)
	{
		for (uint8_t i=0;i<ch;i++)
		{
			w = this->widths[i];

			char_bytes = w*(this->height/8);
			if (this->height%8 >0) char_bytes += w;

			ch_addr += char_bytes;
		}
	}
	else
	{
		w = this->width;

		char_bytes = w*(this->height/8);
		if (this->height%8 >0) char_bytes += w;

		ch_addr = char_bytes * ch;
	}

	if (ch > this->count) return 0;
	else dt = this->data[ch_addr + idx];

	return dt;
}

uint8_t Font::getCharData(uint8_t idx)
{
	uint8_t dt;

	if (idx > this->curr_char_bytes) return 0;
	else dt = this->data[curr_char_ptr + idx];

	return dt;
}

uint8_t Font::getCharBytes(uint8_t ch)
{
	uint8_t w, char_bytes;

	if (this->widths != 0)
	{
		w = this->widths[ch - this->first_char];
	}
	else w = this->width;

	char_bytes = w*(this->height/8);
	if (this->height%8 >0) char_bytes += w;


	return char_bytes;

}

uint8_t Font::setChar(uint8_t ch)
{
	uint8_t w, curr_char, char_bytes;

	curr_char  = ch - this->first_char;
	this->curr_char_ptr = 0;

	if (this->widths != 0)
	{
		for (uint8_t i=0;i<curr_char; i++)
		{
			w = this->widths[i];

			char_bytes = w*(this->height/8);
			if (this->height%8 >0) char_bytes += w;

			this->curr_char_ptr += char_bytes;	//sum of all widths before
		}
	}
	else
		{
			char_bytes = this->getCharBytes(curr_char);

			this->curr_char_ptr = curr_char * char_bytes;
		}
	char_bytes = this->curr_char_bytes = this->getCharBytes(ch);

	return char_bytes;
}

