/*
 * screens.c
 *
 *  Created on: 5 лист. 2020 р.
 *      Author: Victor
 */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "screens.h"
#include "main.h"
#include "obd.h"
#include "../disp/disp.h"

//uint8_t currScreen = 0;
uint8_t PrevScreen = 0;

extern bool carBrake;
extern bool carConn;
extern bool carIgnON;
extern bool carEngON;
extern bool carLightON;
extern int16_t carSpeed;	//integer part of speed value
extern int16_t carRPM;
extern float carFSpeed;	//full speed value - including fractional part
extern int16_t carTemp;
extern int16_t DispDBri;	//Day brightness
extern int16_t DispNBri;	//Night brightness
extern uint64_t carDate;
extern int32_t carEcon1;
extern int32_t carEcon2;
extern bool econMode;	//0 - LPH (Liters Per Hour), 1 - LPK (Liters Per 100 Km)

extern int16_t CANfifo1, CANfifo2;
extern int16_t canQueueMax;

extern int16_t DispDBri;	//Day brightness
extern int16_t DispNBri;	//Night brightness

uint8_t blinkCounter = 0, prevBlinkCounter = 0;

IntNumItem *speedItem = new IntNumItem(&carSpeed, 72, 8, FNT_BIG, 3, DISPLAY_RALIGN);
TextItem *spdLabelItem = new TextItem((char *)"km/h", 70, 0, FNT_SM, DISPLAY_RALIGN);

RealNumItem *econ1Item = new RealNumItem(&carEcon1, 121, 8, FNT_MED, 3, 1, DISPLAY_HIDE | DISPLAY_RALIGN);
RealNumItem *econ2Item = new RealNumItem(&carEcon2, 121, 24, FNT_MED, 3, 1, DISPLAY_HIDE | DISPLAY_RALIGN);
TextItem *econLabelItem = new TextItem((char *)"l/100km", 121, 0, FNT_SM, DISPLAY_HIDE | DISPLAY_RALIGN);

IntNumItem *tempItem = new IntNumItem(&carTemp, 121, 48, FNT_MED, 3, DISPLAY_RALIGN);
TextItem *tempLabelItem = new TextItem((char *)"o", 127, 48, FNT_SM, DISPLAY_RALIGN);
BitmapItem *ETbmpItem = new BitmapItem(&bmpETemp, 71, 48, DISPLAY_HIDE);

TextItem *noConnItem = new TextItem((char *)"x", 0, 0, FNT_SM, DISPLAY_HIDE);

BitmapItem *brakeItem = new BitmapItem(&bmpBrake, 43, 8, DISPLAY_HIDE | DISPLAY_BLINK_SLOW | DISPLAY_REV);

IntNumItem *Can1Item = new IntNumItem(&CANfifo1, 20, 40, FNT_SM, 5, DISPLAY_HIDE);
IntNumItem *Can2Item = new IntNumItem(&carRPM, 80, 40, FNT_SM, 5, DISPLAY_NORM);
IntNumItem *Can3Item = new IntNumItem(&canQueueMax, 0, 40, FNT_SM, 5, DISPLAY_NORM);

TextItem *HiItem = new TextItem((char *)"Hi!", 48, 24, FNT_SM, DISPLAY_NORM);
TextItem *ByItem = new TextItem((char *)"By...", 48, 24, FNT_SM, DISPLAY_NORM);

ScreenItem **currScreen;

ScreenItem *scrItems[] = {	noConnItem,
							spdLabelItem, speedItem,
							econLabelItem, econ1Item, econ2Item,
							tempLabelItem, tempItem,
							brakeItem,
							ETbmpItem,
							Can1Item,
							Can2Item,
							Can3Item,
						};

ScreenItem *scrHi[] = {	HiItem };
ScreenItem *scrBy[] = {	ByItem, noConnItem };

uint8_t items = sizeof(scrItems)/sizeof(*scrItems);	//number of items in array

void DecodeDateVal(uint64_t *data, char *msg);
void Str2Digits(char *msg);
void StrFormat(char *msg, uint8_t itg, uint8_t fra);

void InitScreen(void)
{
	DisplayInit();
	WriteString(43, 16, "FiatMon", FNT_SM, DISPLAY_NORM);
	WriteString(48, 24, FM_VERSION, FNT_SM, DISPLAY_NORM);
	DISP_Refresh();
	HAL_Delay(800);
//	ClearScr();
}

void SwitchScreen(uint8_t scr_num)
{
	currScreen = scrItems;

	switch(scr_num){
		case HI_SCREEN:
			currScreen = scrHi;
			items = sizeof(scrHi)/sizeof(*scrHi);
			break;
		case BY_SCREEN:
			currScreen = scrBy;
			items = sizeof(scrBy)/sizeof(*scrBy);
			break;
		case NORM_SCREEN:
			currScreen = scrItems;
			items = sizeof(scrItems)/sizeof(*scrItems);
		default:
			break;

	}
}

void ByScreen(void)
{
	SwitchScreen(BY_SCREEN);
}

void HiScreen(void)
{
	SwitchScreen(HI_SCREEN);
}

void FormatItems(void)
{
	if (carConn) noConnItem->hide();
	else noConnItem->show();

	//Car engine coolant temperature
	if (carTemp < 40)
	{
		ETbmpItem->show();

		tempItem->show();
		tempLabelItem->show();
	}
	else if (carTemp > 84)
	{
		ETbmpItem->show();
		ETbmpItem->blinkSlow();

		tempItem->show();
		tempLabelItem->show();
	}
	else if (carTemp > 95)
	{
		ETbmpItem->beep();
		ETbmpItem->blinkFast();

		tempItem->blinkSlow();
		tempLabelItem->show();
	}
	else
	{
		ETbmpItem->hide();
		ETbmpItem->beepNo();
		ETbmpItem->blinkNo();

		tempItem->hide();
		tempItem->blinkNo();
		tempLabelItem->hide();
	}

//Handbrake
	if (carBrake == true)
		{
		brakeItem->show();
		}
	else
		{
		brakeItem->hide();
		}

//Ignition
//	if (carIgnON) SwitchScreen(NORM_SCREEN);
//	else SwitchScreen(BY_SCREEN);

//Light
	if (carLightON) SetBklit(DispNBri);
	else SetBklit(DispDBri);

//Engine ON
	if (carEngON)
	{
		Can2Item->show();
	}
	else
	{
		Can2Item->hide();
	}

	if (econMode) //LPK
	{
		econLabelItem->revNo();
		econLabelItem->setText("l/Hk");
	}
	else
	{
		econLabelItem->rev();
		econLabelItem->setText("l/h");
	}
//	if (carEngON)
//	{
		econ1Item->show();
		econ2Item->show();
		econLabelItem->show();

//	}
//	else
/*	{
		econ1Item->hide();
		econ2Item->hide();
		econLabelItem->hide();
	}
*/
}

void UpdateScreen(void)
{
	//update blink counter
	++blinkCounter;
	blinkCounter &= 0x0F;
	if (blinkCounter == 0 && prevBlinkCounter > 0)
		{
			updateAvgEcon();	//update averages every 2 sec
		}
	prevBlinkCounter = blinkCounter;

	ClearScr();
	for(int i=0;i<items;i++) {
		if (!currScreen[i]->getOption(DISPLAY_HIDE))
		{
			if (currScreen[i]->getOption(DISPLAY_BLINK_FAST))
			{
				if ((blinkCounter & 2) > 0 ) currScreen[i]->DrawItem();
			}
			else if (currScreen[i]->getOption(DISPLAY_BLINK_SLOW))
			{
				if ((blinkCounter & 4) > 0 ) currScreen[i]->DrawItem();
			}
			else currScreen[i]->DrawItem();
		}
	}
	DISP_Refresh();
}

void DecodeDateVal(uint64_t *data, char *msg)
{

	char *pdata;

	pdata = (char *)data;

		msg[0] = '0' + ((pdata[0]>>4)&0x0F);//hours high
		msg[1] = '0' + (pdata[0]&0x0F);//hours low

		msg[2] = ':';

		msg[3] = '0' + ((pdata[1]>>4)&0x0F);//minutes high
		msg[4] = '0' + (pdata[1]&0x0F);//minutes low

		msg[5] = ' ';

		msg[6] = '0' + ((pdata[2]>>4)&0x0F);//day high
		msg[7] = '0' + (pdata[2]&0x0F);//day low

		msg[8] = '.';

		msg[9] = '0' + ((pdata[3]>>4)&0x0F);//month high
		msg[10] = '0' + (pdata[3]&0x0F);//month low

		msg[11] = '.';

		msg[12] = '0' + ((pdata[4]>>4)&0x0F);//year th
		msg[13] = '0' + (pdata[4]&0x0F);//year hu
		msg[14] = '0' + ((pdata[5]>>4)&0x0F);//year dec
		msg[15] = '0' + (pdata[5]&0x0F);//year

		msg[16] = 0;
}

void Str2Digits(char *msg)
{
	uint8_t i, len;

	len = strlen(msg);

	for (i=0;i<len;i++) {
		if (msg[i] == '.') msg[i] = (char)43;
		else if (msg[i] == '0') msg[i] = (char)33;
		else if (msg[i] == '1') msg[i] = (char)34;
		else if (msg[i] == '2') msg[i] = (char)35;
		else if (msg[i] == '3') msg[i] = (char)36;
		else if (msg[i] == '4') msg[i] = (char)37;
		else if (msg[i] == '5') msg[i] = (char)38;
		else if (msg[i] == '6') msg[i] = (char)39;
		else if (msg[i] == '7') msg[i] = (char)40;
		else if (msg[i] == '8') msg[i] = (char)41;
		else if (msg[i] == '9') msg[i] = (char)42;
		else msg[i] = 32;
	}
}

void StrFormat(char *msg, uint8_t itg, uint8_t fra)
{
	uint8_t i, len, sh;

	len = strlen(msg);

	sh = itg - len;
	if (fra >0) sh += fra+1;

	for (i=0;i<len+1;i++)
	{
		msg[len-i+sh] = msg[len-i];
	}
	for (i=0;i<sh;i++) msg[i] = ' ';
}


void DisplayInit(void)
{
	DISP_Init();
}

void WriteString(uint16_t x, uint16_t y, const char *str, uint8_t font, uint8_t style)
{
	DISP_DrawString(x, y, str, font, style);
}
void DrawBitmap(uint8_t x, uint8_t  y, BitmapDef *bitmap, uint8_t style)
{
	DISP_DrawBitmap(x, y, bitmap, style);
}

void FillRect(uint8_t x, uint8_t  y, uint8_t w, uint8_t  h, uint8_t style)
{

}

void ClearScr()
{
	DISP_ClearScr(0);
}

void StandbyScr()
{
	DISP_Standby();
}

void ResumeScr()
{
	DISP_Resume();
}

void SetBklit(uint16_t bri)
{
	DISP_Bklit(bri);
}


void IntNumItem :: DrawItem(void) const
{
	char txt[6];

	itoa(*this->value, (char *)&txt, 10);
	WriteString(this->x0, this->y0, txt, this->size, this->option);
}

void RealNumItem :: DrawItem(void) const
{
	char txt[10];
	int32_t val = *this->value;
	uint8_t pw = 0;

		if (this->fdig > 0) pw = (uint8_t)powf(10, (float)this->fdig);//
		uint8_t frac = val % pw;

		itoa(val/pw, (char *)&txt, 10);
		uint8_t len = strlen(txt);
		if (pw > 0) {
			strcat(txt, ".");
			itoa(frac, (char *)&txt+len+1, 10);
		}
		WriteString(this->x0, this->y0, txt, this->size, this->option);
}

void TextItem :: DrawItem(void) const
{
	WriteString(this->x0, this->y0, str, this->size, this->option);

}


void BitmapItem :: DrawItem(void) const
{
	DrawBitmap(this->x0, this->y0, this->bmp, this->option);
}

