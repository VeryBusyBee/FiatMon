/*
 * can.h
 *
 *  Created on: 7 лист. 2020 р.
 *      Author: Victor
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"

extern CAN_HandleTypeDef hcan;

void CANInit(void);
void CAN_Send(uint8_t *message_out, uint8_t len, uint32_t address);
void CAN_Sleep();
void CAN_Resume();


#endif /* INC_CAN_H_ */
