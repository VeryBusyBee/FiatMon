/*
 * obd.h
 *
 *  Created on: 7 лист. 2020 р.
 *      Author: Victor
 */

#ifndef INC_OBD_H_
#define INC_OBD_H_

#include "main.h"

typedef struct {
	CAN_RxHeaderTypeDef pRxHeader;
	uint8_t rxData[10];
} CANMsg_t;

void HandleOBDMsg(CAN_RxHeaderTypeDef pRxHeader, uint8_t *rxData);
void RequestOBD(void);

void updateAvgEcon();

#endif /* INC_OBD_H_ */
