

#include "main.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;

#define APP_RX_DATA_SIZE  32
#define APP_TX_DATA_SIZE  30

uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];
uint8_t RXbuffIdx = 0;
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

uint8_t interface_state;
extern bool CANHacker_active;

uint8_t halfbyte_to_hexascii(uint8_t _halfbyte)
{
 _halfbyte &= 0x0F ;
 if(_halfbyte >= 10) return('A' + _halfbyte - 10) ;
	else               return('0' + _halfbyte) ;
}

uint8_t hexascii_to_halfbyte(uint8_t _ascii)
{
 if((_ascii >= '0') && (_ascii <= '9')) return(_ascii - '0') ;
 if((_ascii >= 'a') && (_ascii <= 'f')) return(_ascii - 'a') ;
 if((_ascii >= 'A') && (_ascii <= 'F')) return(_ascii - 'A') ;
 return(0xFF)	;
}

void UARTWait4Char()
{
	  HAL_UART_Receive_IT (&huart1, UserRxBufferFS, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if (huart == &huart1)
	{
/*
	    switch(UserRxBufferFS[RXbuffIdx])
		 {
			case 'V':
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)"V0101\r", 6);
				RXbuffIdx = 0;
			break ;

			case 'v':
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)"vFM04\r", 7);
				RXbuffIdx = 0;
			break ;

			case 'O':
				CANHacker_active = true;
				RXbuffIdx = 0;
			break ;

			case 'C':
				CANHacker_active = false;
				RXbuffIdx = 0;
			break ;

			case 'm':
				CANHacker_active = false;
				RXbuffIdx = 0;
			break ;

			case 'M':
				CANHacker_active = false;
				RXbuffIdx = 0;
			break ;

			case 'M':
				CANHacker_active = false;
				RXbuffIdx = 0;
			break ;

			default :
				HAL_UART_Transmit_IT(&huart1, (uint8_t*)"\r", 1);
				RXbuffIdx++;
			break ;
		 }
*/
	    switch(RXbuffIdx)
		 {
			case 0: case 1:
//				UserRxBufferFS[] = 0;
			break ;
		 }

	    RXbuffIdx++;

	    HAL_UART_Receive_IT (&huart1, UserRxBufferFS+RXbuffIdx, 1);

	}
}

void Send2CANHacker(CANMsg_t CANMsg)
{
	  uint8_t i;

	  UserTxBufferFS[0] = 'T' ;
	  UserTxBufferFS[1] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId)>>28);
	  UserTxBufferFS[2] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId)>>24);
	  UserTxBufferFS[3] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId)>>20);
	  UserTxBufferFS[4] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId)>>16);
	  UserTxBufferFS[5] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId)>>12);
	  UserTxBufferFS[6] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId)>>8);
	  UserTxBufferFS[7] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId)>>4);
		UserTxBufferFS[8] = halfbyte_to_hexascii((CANMsg.pRxHeader.ExtId));
		UserTxBufferFS[9] = halfbyte_to_hexascii((CANMsg.pRxHeader.DLC));

		for (i=0;i<CANMsg.pRxHeader.DLC;i++)
		{
			UserTxBufferFS[i*2+10] = halfbyte_to_hexascii((CANMsg.rxData[i])>>4);
			UserTxBufferFS[i*2+11] = halfbyte_to_hexascii((CANMsg.rxData[i]));
		}
		UserTxBufferFS[i*2+10] = halfbyte_to_hexascii(((uint16_t)uwTick)>>12);
		UserTxBufferFS[i*2+11] = halfbyte_to_hexascii(((uint16_t)uwTick)>>8);
		UserTxBufferFS[i*2+12] = halfbyte_to_hexascii(((uint16_t)uwTick)>>4);
		UserTxBufferFS[i*2+13] = halfbyte_to_hexascii(((uint16_t)uwTick)>>0);
		UserTxBufferFS[i*2+14] = '\r';

		while( HAL_UART_GetState (&huart1) == HAL_UART_STATE_BUSY_TX ) ;
		HAL_UART_Transmit_IT(&huart1, UserTxBufferFS, i*2+15);

}

void Send2CANSniffer(CANMsg_t CANMsg)
{
	  uint8_t i;

	  UserTxBufferFS[0] = 0xAA ;
	  UserTxBufferFS[1] = 0x55 ;
	  UserTxBufferFS[2] = 0xAA ;
	  UserTxBufferFS[3] = 0x55 ;
	  UserTxBufferFS[4] = (CANMsg.pRxHeader.ExtId);
	  UserTxBufferFS[5] = (CANMsg.pRxHeader.ExtId)>>8;
	  UserTxBufferFS[6] = (CANMsg.pRxHeader.ExtId)>>16;
	  UserTxBufferFS[7] = (CANMsg.pRxHeader.ExtId)>>24;
		UserTxBufferFS[8] = (CANMsg.pRxHeader.DLC);

		for (i=0;i<CANMsg.pRxHeader.DLC;i++)
		{
			UserTxBufferFS[i+9] = (CANMsg.rxData[i]);
		}

		while( HAL_UART_GetState (&huart1) == HAL_UART_STATE_BUSY_TX ) ;
		HAL_UART_Transmit_IT(&huart1, UserTxBufferFS, i+9);
}
