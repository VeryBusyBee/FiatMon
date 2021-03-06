/*
 * screens.h
 *
 *  Created on: 5 лист. 2020 р.
 *      Author: Victor
 */

#ifndef INC_SCREENS_H_
#define INC_SCREENS_H_

#include "main.h"
#include "../disp/disp.h"

#define FNT_SM 12
#define FNT_MED 18
#define FNT_BIG 24

#define SITEM_TYPE_UINUM 0
#define SITEM_TYPE_INUM 1
#define SITEM_TYPE_FNUM 4
#define SITEM_TYPE_LTEXT 8
#define SITEM_TYPE_BITMAP 10

#define HI_SCREEN 1
#define BY_SCREEN 2
#define NORM_SCREEN 3

typedef struct {
	uint8_t type;	//0 - string, 1 - bitmap
	uint8_t x0;		//left offset
	uint8_t y0;		//top offset
	uint8_t size;	//screen height
	uint8_t par1;	//parameter 1
	uint8_t par2;	//parameter 2
	uint8_t style;	//style
	void * src;
	void * src_Prev;
} sitem_t;

void InitScreen(void);
void FormatItems(void);
void UpdateScreen(void);
void SwitchScreen(uint8_t scr_num);
void ByScreen(void);
void HiScreen(void);

void DisplayInit(void);
void WriteString(uint16_t x, uint16_t y, const char *str, uint8_t font, uint8_t style);
void DrawBitmap(uint8_t x, uint8_t  y, BitmapDef *bitmap, uint8_t style);
void FillRect(uint8_t x, uint8_t  y, uint8_t w, uint8_t  h, uint8_t  color);
void ClearScr();
void StandbyScr();
void ResumeScr();
void SetBklit(uint16_t bri);

class ScreenItem
{
protected:
	uint8_t x0 = 0;		//left/right offset
	uint8_t y0 = 0;		//top offset
	uint8_t size = 8;	//screen height
	uint8_t option = 0;	//show/hide, blink, right-align

public:
	ScreenItem() {};
	ScreenItem(uint8_t x, uint8_t y, uint8_t sz, uint8_t opt) :
		x0(x), y0(y), size(sz), option(opt) {};
//	~ScreenItem(){}

	void hide(void) {option |= DISPLAY_HIDE;}
	void show(void) {option &= ~DISPLAY_HIDE;}

	void rev(void) {option |= DISPLAY_REV;}
	void revNo(void) {option &= ~DISPLAY_REV;}

	void blinkNo(void) {option &= ~DISPLAY_BLINK_SLOW;option &= ~DISPLAY_BLINK_FAST;}
	void blinkSlow(void) {option |= DISPLAY_BLINK_SLOW;option &= ~DISPLAY_BLINK_FAST;}
	void blinkFast(void) {option |= DISPLAY_BLINK_FAST;option &= ~DISPLAY_BLINK_SLOW;}

	void beep(void) {option |= DISPLAY_BEEP;}
	void beepNo(void) {option &= ~DISPLAY_BEEP;}

	void setOption(uint8_t opt) {option |= opt;}
	void clrOption(uint8_t opt) {option &= ~opt;}
	bool getOption(uint8_t opt) {return ((option & opt) > 0) ? true : false;};

	virtual void DrawItem(void) const = 0;

};

class IntNumItem : public ScreenItem
{
public:
	int16_t *value;
	uint8_t dig = 3;		//number of digits

public:
	IntNumItem(int16_t *val) :
		value(val) {};
	IntNumItem(int16_t *val, uint8_t x, uint8_t y, uint8_t sz, uint8_t intd, uint8_t opt) :
		ScreenItem(x, y, sz, opt), value(val), dig(intd) {};

	int16_t *getValue(void) {return value;}

	void DrawItem(void) const;
};

class RealNumItem : public ScreenItem
{
public:
	int32_t *value;
	uint8_t idig = 3;	//digits in integer part
	uint8_t fdig = 1;	//digits in fractional part

public:
	RealNumItem(int32_t *val) :
		value(val) {};
	RealNumItem(int32_t *val, uint8_t x, uint8_t y, uint8_t sz, uint8_t intd, uint8_t frad, uint8_t opt) :
		ScreenItem(x, y, sz, opt), value(val), idig(intd), fdig(frad) {};

	int32_t *getValue(void) {return value;}

	void DrawItem(void) const;
};

class TextItem : public ScreenItem
{
public:
	const char *str;

public:
	TextItem(char *st) :
		str(st) {};
	TextItem(char *st, uint8_t x, uint8_t y, uint8_t sz, uint8_t opt) :
		ScreenItem(x, y, sz, opt), str(st) {};

	const char * getText(void) {return str;}
	void setText(const char *text) {str = text;}

	void DrawItem(void) const;
};

class BitmapItem : public ScreenItem
{
public:
	BitmapDef *bmp;

public:
	BitmapItem(BitmapDef *bm) :
		bmp(bm) {};
	BitmapItem(BitmapDef *bm, uint8_t x, uint8_t y, uint8_t opt) :
		ScreenItem(x, y, 8, opt), bmp(bm) {};

	BitmapDef * getValue(void) {return bmp;}

	void DrawItem(void) const;
};

#endif /* INC_SCREENS_H_ */
