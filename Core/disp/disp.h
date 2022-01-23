#ifndef __DISP_H
#define __DISP_H

#include "main.h"
#include "fonts.h"
#include "bitmaps.h"

/* choose a Hardware SPI port to use. */
#define DISP_SPI_PORT hspi1
extern SPI_HandleTypeDef DISP_SPI_PORT;

//#define UC1609
//#define ST7565
#define SSD1306 //OLED

/* Choose a type you are using */
//#define USING_135X240
//#define USING_240X240
//#define USING_192X64
#define USING_128X64

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_BYTES SCREEN_WIDTH*SCREEN_HEIGHT/8

#define DISP_MIRROR_HORIZ
#define DISP_MIRROR_VERT
/**
 * if you predefined pin names in CubeMX, 
 * you could find them in main.h 
 * and use them below  
 */   

/* Define the pins tp connect */
#define DISP_RST_PORT DISP_RST_GPIO_Port
#define DISP_RST_PIN DISP_RST_Pin
#define DISP_DC_PORT DISP_DC_GPIO_Port
#define DISP_DC_PIN DISP_DC_Pin
#define DISP_CS_PORT DISP_CS_GPIO_Port
#define DISP_CS_PIN DISP_CS_Pin

/***** Use if need backlight control *****
#define BLK_PORT 
#define BLK_PIN 
******************************************/


/* Choose a display rotation you want to use: (0-3) */
//#define DISP_ROTATION 0
//#define DISP_ROTATION 1
#define DISP_ROTATION 2				//  use Normally on 240x240
//#define DISP_ROTATION 3


/* Control Registers and constant codes */
#define DISP_NOP     0x00
#define DISP_SWRESET 0x01
#define DISP_RDDID   0x04
#define DISP_RDDST   0x09

#define DISP_SLPIN   0x10
#define DISP_SLPOUT  0x11
#define DISP_PTLON   0x12
#define DISP_NORON   0x13

#define DISP_INVOFF  0x20
#define DISP_INVON   0x21
#define DISP_DISPOFF 0x28
#define DISP_DISPON  0x29
#define DISP_CASET   0x2A
#define DISP_RASET   0x2B
#define DISP_RAMWR   0x2C
#define DISP_RAMRD   0x2E

#define DISP_PTLAR   0x30
#define DISP_COLMOD  0x3A
#define DISP_MADCTL  0x36

/* Advanced options */
/**
 * Caution: Do not operate these settings
 * You know what you are doing 
 */

/* Basic operations */
#define DISP_RST_Clr() HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_RESET)
#define DISP_RST_Set() HAL_GPIO_WritePin(DISP_RST_PORT, DISP_RST_PIN, GPIO_PIN_SET)

#define DISP_DC_Clr() HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_RESET)
#define DISP_DC_Set() HAL_GPIO_WritePin(DISP_DC_PORT, DISP_DC_PIN, GPIO_PIN_SET)

#define DISP_Select() HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_RESET)
#define DISP_UnSelect() HAL_GPIO_WritePin(DISP_CS_PORT, DISP_CS_PIN, GPIO_PIN_SET)

#define ABS(x) ((x) > 0 ? (x) : -(x))

/* Basic functions. */
void DISP_Init(void);
void DISP_SetRotation(uint8_t m);
void DISP_ClearScr(uint8_t style);
void DISP_WriteCommand(uint8_t cmd);
void DISP_WriteData(uint8_t *buff, size_t buff_size);
void DISP_Refresh(void);
void DISP_Standby(void);
void DISP_Resume(void);
void DISP_Bklit(uint16_t bri);
void DISP_SetContrast(const uint8_t value);

void DISP_DrawPixel(uint16_t x, uint16_t y, uint16_t style);
void DISP_Fill(uint16_t xSta, uint16_t ySta, uint16_t xEnd, uint16_t yEnd, uint16_t style);

/* Graphical functions. */
void DISP_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t style);
void DISP_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t style);
void DISP_DrawCircle(uint16_t x0, uint16_t y0, uint8_t r, uint16_t style);
void DISP_DrawImage(uint16_t x, uint16_t y, uint16_t w, uint16_t h, const uint16_t *data);
void DISP_DrawBitmap(uint16_t x, uint16_t y, BitmapDef *bmp, uint16_t color);

/* Text functions. */
void DISP_DrawChar(uint16_t x, uint16_t y, char ch, Font *font, uint8_t style);
void DISP_DrawString(uint16_t x, uint16_t y, const char *str, uint8_t font, uint8_t style);

/* Extented Graphical functions. */
void DISP_DrawFilledRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t style);
void DISP_DrawTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t style);
void DISP_DrawFilledTriangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, uint16_t x3, uint16_t y3, uint16_t style);
void DISP_DrawFilledCircle(int16_t x0, int16_t y0, int16_t r, uint16_t style);

#endif
