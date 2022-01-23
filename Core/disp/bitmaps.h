/*
 * bitmaps.h
 *
 *  Created on: Dec 12, 2020
 *      Author: Victor
 */

#ifndef INC_BITMAPS_H_
#define INC_BITMAPS_H_

#include "stdint.h"

typedef struct {
    const uint8_t width;
    uint8_t height;
    const uint8_t *data;
} BitmapDef;

extern BitmapDef bmpNoConnect;
extern BitmapDef bmpConnect;
extern BitmapDef bmpEcono;
extern BitmapDef bmpETemp;
extern BitmapDef bmpBrake ;



#endif /* INC_BITMAPS_H_ */
