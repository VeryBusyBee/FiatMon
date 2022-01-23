#include "disp.h"
#include "screens.h"

#include "Arial16.h"
#include "fonts.h"
#include "SystemFont5x7.h"
#include "Arial32.h"

uint8_t scr_buff[SCREEN_BYTES+8];	//screen buffer in memory

/**
 * @brief Write command to ST7789 controller
 * @param cmd -> command to write
 * @return none
 */
void DISP_WriteCommand(uint8_t cmd)
{
	DISP_Select();
	DISP_DC_Clr();
	HAL_SPI_Transmit(&DISP_SPI_PORT, &cmd, sizeof(cmd), HAL_MAX_DELAY);
	DISP_UnSelect();
}

/**
 * @brief Write data to DISP controller
 * @param buff -> pointer of data buffer
 * @param buff_size -> size of the data buffer
 * @return none
 */
void DISP_WriteData(uint8_t *buff, size_t buff_size)
{
#ifdef UC1609
	DISP_Select();
	DISP_DC_Set();

	// split data in small chunks because HAL can't send more than 64K at once

	if (buff_size > 0 && buff_size <= SCREEN_BYTES) {
		HAL_SPI_Transmit_IT(&hspi1, buff, buff_size);
	}

#else //for both St7565 and SSD1306/SPD0301
	uint8_t p;

	for (p=0;p<8;p++)
	{
		DISP_WriteCommand(0xb0 | p);	//Set page
//		DISP_WriteCommand(0x0 | 4);	//Set lower column for rotated ST7565
		DISP_WriteCommand(0x0 | 0);	//Set lower column
		DISP_WriteCommand(0x10 | 0);	//Set upper column
		DISP_WriteCommand(0xE0);	//Set RMW

		DISP_Select();
		DISP_DC_Set();
		HAL_SPI_Transmit_IT(&hspi1, buff + SCREEN_WIDTH*p, SCREEN_WIDTH);
	}

#endif


	DISP_UnSelect();
}
/**
 * @brief Write data to DISP controller, simplify for 8/16 bit data.
 * data -> data to write
 * @return none
 */
void DISP_WriteSmallData(uint8_t data)
{
	DISP_Select();
	DISP_DC_Set();
	HAL_SPI_Transmit(&DISP_SPI_PORT, &data, sizeof(data), HAL_MAX_DELAY);
	DISP_UnSelect();
}

/**
 * @brief Initialize UC1609 controller
 * @param none
 * @return none
 */
void DISP_Init(void)
{
//	HAL_Delay(25);
    DISP_RST_Clr();
    HAL_Delay(100);
    DISP_RST_Set();
    HAL_Delay(20);
		
#ifdef UC1609
    // initialization of UC1609C controller
	DISP_WriteCommand(0xe2);	//system reset -> 0x0e2 (5ms)
	DISP_WriteCommand(0xa3);	//[A0: 76fps, A1b: 95fps, A2b: 132fps, A3b: 168fps(fps: frame-per-second)]
	DISP_WriteCommand(0xeb);	//Set LCD Bias Ratio = 9
	DISP_WriteCommand(0x2f);	//power control, bit 1,2: PC2, PC1 - internal charge pump, bit 0: PC0: cap load
	DISP_WriteCommand(0xc2);	//Set LCD Control LC[2:1]
	DISP_WriteCommand(0x81);	//Set VBIAS Potentiometer PM [7:0] (Double-byte command)
	DISP_WriteCommand(180);		//Potentiometer value
	DISP_WriteCommand(0xaf);	//Set Display Enable
#endif

#ifdef ST7565
    // initialization of ST7565 controller
	DISP_WriteCommand(0xe2);	//system reset -> 0x0e2 (5ms)
	DISP_WriteCommand(0xa3);	//Set bias = 7, 0xa2 = 9
	DISP_WriteCommand(0x2f);	//power control, bit 1,2: PC2, PC1 - internal charge pump, bit 0: PC0: cap load
//	DISP_WriteCommand(0xa0);	//Set SEG direction norm
	DISP_WriteCommand(0xa1);	//Set SEG direction reverse
	DISP_WriteCommand(0xc0);	//Set COM style
//	DISP_WriteCommand(0xc8);	//Set COM style
	DISP_WriteCommand(0x40);	//Set display start line
	DISP_WriteCommand(0x81);	//Set VBIAS Potentiometer PM [7:0] (Double-byte command)
	DISP_WriteCommand(100);		//Potentiometer value
	DISP_WriteCommand(0xa7);	//Set Display style
	DISP_WriteCommand(0xaf);	//Set Display Enable
#endif


#ifdef SSD1306
//    DISP_WriteCommand(0xAE); //display off

// Set multiplex ratio.
    DISP_WriteCommand(0xA8); //--set multiplex ratio(1 to 64) - CHECK
    DISP_WriteCommand(0x3F); // Seems to work for 128px high displays too.

    DISP_WriteCommand(0xD3); //-set display vertical offset - CHECK
    DISP_WriteCommand(0x00); //-not offset

    DISP_WriteCommand(0x40); //--set start line address - CHECK

#ifdef DISP_MIRROR_HORIZ
    DISP_WriteCommand(0xA0); // Mirror horizontally
#else
    DISP_WriteCommand(0xA1); //--set segment re-map 0 to 127 - CHECK
#endif

#ifdef DISP_MIRROR_VERT
    DISP_WriteCommand(0xC0); // Mirror vertically
#else
    DISP_WriteCommand(0xC8); //Set COM Output Scan Direction
#endif

    DISP_WriteCommand(0xDA); //--set com pins hardware configuration - CHECK
#if (SCREEN_HEIGHT == 32)
    DISP_WriteCommand(0x02);
#elif (SCREEN_HEIGHT == 64)
    DISP_WriteCommand(0x12);
#elif (SCREEN_HEIGHT == 128)
    DISP_WriteCommand(0x12);
#endif

    DISP_SetContrast(0x0);

    DISP_WriteCommand(0xA4); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content
//    DISP_WriteCommand(0xA5); //0xa4,Output follows RAM content;0xa5,Output ignores RAM content

#ifdef DISP_INVERSE_COLOR
    DISP_WriteCommand(0xA7); //--set inverse color
#else
    DISP_WriteCommand(0xA6); //--set normal color
#endif

    DISP_WriteCommand(0xD5); //--set display clock divide ratio/oscillator frequency
    DISP_WriteCommand(0x80); //--set divide ratio

    DISP_WriteCommand(0xB0); //Set Page Start Address for Page Addressing Mode,0-7

    DISP_WriteCommand(0x00); //---set low column address
    DISP_WriteCommand(0x10); //---set high column address

    DISP_WriteCommand(0x8D); //--set DC-DC enable
    DISP_WriteCommand(0x14); //

    DISP_WriteCommand(0xDB); //--set vcomh
    DISP_WriteCommand(0x34); //0x20,0.77xVcc

    DISP_WriteCommand(0xAF); //--turn on SSD1306 panel

/*
    DISP_WriteCommand(0x20); //Set Memory Addressing Mode
    DISP_WriteCommand(0x00); // 00b,Horizontal Addressing Mode; 01b,Vertical Addressing Mode;
                                // 10b,Page Addressing Mode (RESET); 11b,Invalid

    DISP_WriteCommand(0xD9); //--set pre-charge period
    DISP_WriteCommand(0x22); //

    DISP_WriteCommand(0xDB); //--set vcomh
    DISP_WriteCommand(0x20); //0x20,0.77xVcc
*/
    // Clear screen
    //DISP_Fill(Black);

    // Flush buffer to screen
    //DISP_UpdateScreen();

#endif

 	HAL_Delay(50);
	DISP_ClearScr(0);				//	Fill with Black.
}

/**
 * @brief Fill the DisplayWindow with single style
 * @param style -> style to Fill with
 * @return none
 */
void DISP_ClearScr(uint8_t style)
{
	for (uint16_t i = 0; i < SCREEN_BYTES; i++)
		scr_buff[i] = style ? 0xff : 0x00;
}

void DISP_Standby(void)
{
#ifdef ST7565
	DISP_WriteCommand(0xac);	//static indicator OFF
	DISP_WriteCommand(0xae);	//display OFF
	DISP_Bklit(0x100);
#endif

#ifdef SSD1306
	DISP_WriteCommand(0xae);	//display OFF
#endif
}

void DISP_Resume(void)
{
	DISP_WriteCommand(0xaf);	//display ON

#ifdef ST7565
	DISP_WriteCommand(0xad);	//static indicator ON
	DISP_Bklit(0xb000);
#endif
}

void DISP_Bklit(uint16_t bri)
{
#ifndef SSD1306
	TIM3->CCR4 = bri;	//initial LCD backlit brightness
#endif
}

void DISP_SetContrast(const uint8_t value) {

    DISP_WriteCommand(0x81);
    DISP_WriteCommand(value);
}



/**
 * @brief Draw a Pixel
 * @param x&y -> coordinate to Draw
 * @param style -> style of the Pixel
 * @return none
 */
void DISP_DrawPixel(uint16_t x, uint16_t y, uint16_t style)
{
	if ((x < 0) || (x >= SCREEN_WIDTH) ||
		 (y < 0) || (y >= SCREEN_HEIGHT))	return;

	uint16_t byte_ptr = y * SCREEN_WIDTH + (x / 8);	//
	uint8_t bit = x % 8;

	if (style == 0) scr_buff[byte_ptr] &= ~(1 << (7 - bit));
	else scr_buff[byte_ptr] |= 1 << (7 - bit);
}

/**
 * @brief Fill an Area with single style
 * @param xSta&ySta -> coordinate of the start point
 * @param xEnd&yEnd -> coordinate of the end point
 * @param style -> style to Fill with
 * @return none
 */
void DISP_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t style)
{
	if ((xEnd < 0) || (xEnd >= SCREEN_WIDTH) ||
		 (yEnd < 0) || (yEnd >= SCREEN_HEIGHT))	return;
/*
	for (i = ySta; i <= yEnd; i++)
		for (j = xSta; j <= xEnd; j++) {
			uint8_t data[] = {style >> 8, style & 0xFF};

		}
		*/
}

/**
 * @brief Draw a line with single style
 * @param x1&y1 -> coordinate of the start point
 * @param x2&y2 -> coordinate of the end point
 * @param style -> style of the line to Draw
 * @return none
 */
void DISP_DrawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1,
        uint16_t style) {
	uint16_t swap;
    uint16_t steep = ABS(y1 - y0) > ABS(x1 - x0);
    if (steep) {
		swap = x0;
		x0 = y0;
		y0 = swap;

		swap = x1;
		x1 = y1;
		y1 = swap;
        //_swap_int16_t(x0, y0);
        //_swap_int16_t(x1, y1);
    }

    if (x0 > x1) {
		swap = x0;
		x0 = x1;
		x1 = swap;

		swap = y0;
		y0 = y1;
		y1 = swap;
        //_swap_int16_t(x0, x1);
        //_swap_int16_t(y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = ABS(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0<=x1; x0++) {
        if (steep) {
            DISP_DrawPixel(y0, x0, style);
        } else {
            DISP_DrawPixel(x0, y0, style);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

/**
 * @brief Draw a Rectangle with single style
 * @param xi&yi -> 2 coordinates of 2 top points.
 * @param style -> style of the Rectangle line
 * @return none
 */
void DISP_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t style)
{
	DISP_DrawLine(x1, y1, x2, y1, style);
	DISP_DrawLine(x1, y1, x1, y2, style);
	DISP_DrawLine(x1, y2, x2, y2, style);
	DISP_DrawLine(x2, y1, x2, y2, style);
}

/** 
 * @brief Draw a circle with single style
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param style -> style of circle line
 * @return  none
 */
void DISP_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t style)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	DISP_DrawPixel(x0, y0 + r, style);
	DISP_DrawPixel(x0, y0 - r, style);
	DISP_DrawPixel(x0 + r, y0, style);
	DISP_DrawPixel(x0 - r, y0, style);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		DISP_DrawPixel(x0 + x, y0 + y, style);
		DISP_DrawPixel(x0 - x, y0 + y, style);
		DISP_DrawPixel(x0 + x, y0 - y, style);
		DISP_DrawPixel(x0 - x, y0 - y, style);

		DISP_DrawPixel(x0 + y, y0 + x, style);
		DISP_DrawPixel(x0 - y, y0 + x, style);
		DISP_DrawPixel(x0 + y, y0 - x, style);
		DISP_DrawPixel(x0 - y, y0 - x, style);
	}
	DISP_UnSelect();
}

/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Image
 * @param w&h -> width & height of the Image to Draw
 * @param data -> pointer of the Image array
 * @return none
 */
void DISP_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data)
{
	if ((x >= SCREEN_WIDTH) || (y >= SCREEN_HEIGHT))
		return;
	if ((x + w - 1) >= SCREEN_WIDTH)
		return;
	if ((y + h - 1) >= SCREEN_HEIGHT)
		return;

}

/**
 * @brief Draw an Image on the screen
 * @param x&y -> start point of the Bitmap
 * @param w&h -> width (max 64 pixels) & height of the Bitmap to Draw
 * @param style&bgstyle -> front & back styles
 * @param data -> pointer of the Bitmap array
 * @return none
 */
void DISP_DrawBitmap(uint16_t x, uint16_t y, BitmapDef *bmp, uint16_t color)
{
	uint8_t *bdata;
	uint16_t i;
	uint8_t j, b, w, h;
	uint16_t mem_ptr = y * SCREEN_WIDTH/8 + x + 1;

	bdata = (uint8_t *)bmp->data;
	w = bmp->width;
	h = bmp->height;

	if ((x >= SCREEN_WIDTH) || (y >= SCREEN_HEIGHT))
		return;
	if ((x + w - 1) >= SCREEN_WIDTH)
		return;
	if ((y + h - 1) >= SCREEN_HEIGHT)
		return;

	j = 0;
	uint16_t bmpBytes = h*w/8;
	for (i = 0; i < bmpBytes; i++) {
		b = (color & DISPLAY_REV) ? ~bdata[i] : bdata[i];
		scr_buff[mem_ptr + j] = b;
		if (++j >= w) {
			j = 0;
			mem_ptr += SCREEN_WIDTH;	//next row of bytes
		}
	}

}

/** 
 * @brief Write a char
 * @param  x&y -> cursor of the start point.
 * @param ch -> char to write
 * @param font -> fontstyle of the string
 * @param style -> style of the char
 * @return  none
 */
void DISP_DrawChar(uint16_t x, uint16_t y, char ch, Font *font, uint8_t style)
{
	uint16_t i, ch_bytes;
	uint8_t j, b, w, sp = 0;

	y /= 8;	//vertical position should be a multiple of 8
	uint16_t mem_ptr = y * SCREEN_WIDTH + x + 1;

	if (ch < font->first_char || ch > (font->first_char + font->count)) {
		ch = font->first_char;
		sp = 1;	//all chars out of font range are shown as spaces
		ch_bytes = font->getCharBytes('4');	//bytes for a widest char
//		ch_bytes = font->setChar('4');	//bytes for a widest char
	}
	else
	{
//		ch_bytes = font->getCharBytes(ch);	//bytes per char
		ch_bytes = font->setChar(ch);	//bytes per char
		sp = 0;
	}

	w = font->getWidth(ch);

	i = 0;
	while ( i < ch_bytes) {			//
		for (j = 0; j < w; j++) {		//row  of vertical bytes
//			if (sp == 0) b = style ? ~font->getCharData(ch, i+j) : font->getCharData(ch, i+j);
			if (sp == 0) b = (style & DISPLAY_REV) ? ~font->getCharData(i+j) : font->getCharData(i+j);
			else b = (style & DISPLAY_REV) ? 0xff : 0;

			scr_buff[mem_ptr + j] = b;
//			HAL_SPI_Transmit(&hspi1, &b, sizeof(b), HAL_MAX_DELAY);

		}
		b = (style & DISPLAY_REV) ? 0xff : 0;	//add space after the char
		for (j = 0; j < font->space; j++) scr_buff[mem_ptr + w + j] = b;

		i += w;
		mem_ptr += SCREEN_WIDTH;	//next row of bytes
	}

}

/** 
 * @brief Write a string 
 * @param  x&y -> cursor of the start point.
 * @param str -> string to write
 * @param font -> fontstyle of the string
 * @param style -> style of the string
 * @param bgstyle -> background style of the string
 * @return  none
 */
void DISP_DrawString(uint16_t x, uint16_t y, const char *str, uint8_t font, uint8_t style)
{
	Font *fnt;
	uint8_t w, len=0;
	uint8_t *wstr = (uint8_t *)str;

	switch(font){
	case FNT_SM:	fnt = FontSmall; break;
	case FNT_MED:	fnt = FontMedium; break;
	case FNT_BIG:	fnt = FontBig; break;
	default:	fnt = FontSmall;
	}
	//calculate full length of the string if it is right aligned
	if (style & DISPLAY_RALIGN)
	{
		while (*wstr) {
			len += (fnt->getWidth(*wstr) + fnt->space);
			wstr++;
		}
		x -= (len - fnt->space);
	}

	while (*str) {
		if (fnt->widths != 0) w = fnt->getWidth(*str);
		else w = fnt->width;

		if (x + w > SCREEN_WIDTH) {
			x = 0;
			y += fnt->height;
			if (y + fnt->height > SCREEN_HEIGHT) {
				break;
			}
		}
		DISP_DrawChar(x, y, *str, fnt, style);
		x += w + fnt->space;	//add 1 bit space after
		str++;

	}
}

/** 
 * @brief Draw a filled Rectangle with single style
 * @param  x&y -> coordinates of the starting point
 * @param w&h -> width & height of the Rectangle
 * @param style -> style of the Rectangle
 * @return  none
 */
void DISP_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t style)
{
	uint8_t i;

	/* Check input parameters */
	if (x >= SCREEN_WIDTH ||
		y >= SCREEN_HEIGHT) {
		/* Return error */
		return;
	}

	/* Check width and height */
	if ((x + w) >= SCREEN_WIDTH) {
		w = SCREEN_WIDTH - x;
	}
	if ((y + h) >= SCREEN_HEIGHT) {
		h = SCREEN_HEIGHT - y;
	}

	/* Draw lines */
	for (i = 0; i <= h; i++) {
		/* Draw lines */
		DISP_DrawLine(x, y + i, x + w, y + i, style);
	}
}

/** 
 * @brief Draw a Triangle with single style
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param style ->style of the lines
 * @return  none
 */
void DISP_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t style)
{
	/* Draw lines */
	DISP_DrawLine(x1, y1, x2, y2, style);
	DISP_DrawLine(x2, y2, x3, y3, style);
	DISP_DrawLine(x3, y3, x1, y1, style);
}

/** 
 * @brief Draw a filled Triangle with single style
 * @param  xi&yi -> 3 coordinates of 3 top points.
 * @param style ->style of the triangle
 * @return  none
 */
void DISP_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t style)
{
	int16_t deltax = 0, deltay = 0, x = 0, y = 0, xinc1 = 0, xinc2 = 0,
			yinc1 = 0, yinc2 = 0, den = 0, num = 0, numadd = 0, numpixels = 0,
			curpixel = 0;

	deltax = ABS(x2 - x1);
	deltay = ABS(y2 - y1);
	x = x1;
	y = y1;

	if (x2 >= x1) {
		xinc1 = 1;
		xinc2 = 1;
	}
	else {
		xinc1 = -1;
		xinc2 = -1;
	}

	if (y2 >= y1) {
		yinc1 = 1;
		yinc2 = 1;
	}
	else {
		yinc1 = -1;
		yinc2 = -1;
	}

	if (deltax >= deltay) {
		xinc1 = 0;
		yinc2 = 0;
		den = deltax;
		num = deltax / 2;
		numadd = deltay;
		numpixels = deltax;
	}
	else {
		xinc2 = 0;
		yinc1 = 0;
		den = deltay;
		num = deltay / 2;
		numadd = deltax;
		numpixels = deltay;
	}

	for (curpixel = 0; curpixel <= numpixels; curpixel++) {
		DISP_DrawLine(x, y, x3, y3, style);

		num += numadd;
		if (num >= den) {
			num -= den;
			x += xinc1;
			y += yinc1;
		}
		x += xinc2;
		y += yinc2;
	}
}

/** 
 * @brief Draw a Filled circle with single style
 * @param x0&y0 -> coordinate of circle center
 * @param r -> radius of circle
 * @param style -> style of circle
 * @return  none
 */
void DISP_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t style)
{
	int16_t f = 1 - r;
	int16_t ddF_x = 1;
	int16_t ddF_y = -2 * r;
	int16_t x = 0;
	int16_t y = r;

	DISP_DrawPixel(x0, y0 + r, style);
	DISP_DrawPixel(x0, y0 - r, style);
	DISP_DrawPixel(x0 + r, y0, style);
	DISP_DrawPixel(x0 - r, y0, style);
	DISP_DrawLine(x0 - r, y0, x0 + r, y0, style);

	while (x < y) {
		if (f >= 0) {
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		DISP_DrawLine(x0 - x, y0 + y, x0 + x, y0 + y, style);
		DISP_DrawLine(x0 + x, y0 - y, x0 - x, y0 - y, style);

		DISP_DrawLine(x0 + y, y0 + x, x0 - y, y0 + x, style);
		DISP_DrawLine(x0 + y, y0 - x, x0 - y, y0 - x, style);
	}
}

void DISP_Refresh(void)
{
	DISP_WriteData(scr_buff, SCREEN_BYTES);
}
