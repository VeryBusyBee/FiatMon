/*
 * can.c
 *
 *  Created on: 7 лист. 2020 р.
 *      Author: Victor
 */
#include "can.h"
#include <stdio.h>

#define CAN_TIMEOUT_VALUE 100

CAN_TxHeaderTypeDef pHeader; //declare a specific header for message transmittions
CAN_RxHeaderTypeDef pRxHeader; //declare header for message reception
uint32_t TxMailbox;
uint8_t rxData[16]; //declare a receive data buffer
CAN_FilterTypeDef sFilterConfig; //declare CAN filter structure


void CANInit(void){


}

//function to send CAN message (using HAL)
void CAN_Send(uint8_t *message_out, uint8_t len, uint32_t address){
	HAL_StatusTypeDef status;

	pHeader.ExtId = address;
	pHeader.DLC = (uint32_t) len;

	status = HAL_CAN_AddTxMessage(&hcan, &pHeader, message_out, &TxMailbox);
}

void CAN_Sleep()
{
	SET_BIT(hcan.Instance->IER, CAN_IER_WKUIE); //enable the wake up interrupt
	SET_BIT(hcan.Instance->MCR, CAN_MCR_SLEEP); //sleep bxCAN
	SET_BIT(hcan.Instance->MCR, CAN_MCR_AWUM); //activate automatic bxCAN wakeup

	HAL_GPIO_WritePin(TJA_S_GPIO_Port, TJA_S_Pin, GPIO_PIN_SET);	//set TJA1050 silent mode

}

void CAN_Resume()
{
	CLEAR_BIT(hcan.Instance->IER, CAN_IER_WKUIE); //disable the wake up interrupt
	CLEAR_BIT(hcan.Instance->MCR, CAN_MCR_SLEEP); //disable sleep bxCAN

	HAL_GPIO_WritePin(TJA_S_GPIO_Port, TJA_S_Pin, GPIO_PIN_RESET);	//set TJA1050 hi-speed mode
	HAL_CAN_MspInit(&hcan);
}
