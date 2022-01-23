/*
 * usart.h
 *
 *  Created on: Jun 26, 2021
 *      Author: Victor
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include "obd.h"

void UARTWait4Char();
void Send2CANHacker(CANMsg_t);
void Send2CANSniffer(CANMsg_t);

#endif /* INC_USART_H_ */
