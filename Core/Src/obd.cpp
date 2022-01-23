/*
 * obd.c
 *
 *  Created on: 7 лист. 2020 р.
 *      Author: Victor
 */
#include "main.h"
#include "screens.h"
#include "can.h"
#include "obd.h"

extern bool carBrake;
extern bool carConn;
extern bool carIgnON;
extern bool carEngON;
extern bool carLightON;
extern int16_t carSpeed;	//integer part of speed value
extern int16_t carRPM;
extern float carFSpeed;	//full speed value - including fractional part
extern int16_t carTemp;
extern uint64_t carDate;
extern int32_t carEcon1;
extern int32_t carEcon2;

extern uint8_t blinkCounter;



#define ECON_AVG_BUF_SIZE 8

void HandleOBDMsg(CAN_RxHeaderTypeDef pRxHeader, uint8_t *rxData)
{
	uint16_t dt;
	uint16_t *ptr;

	ptr = (uint16_t *)rxData;

	switch (pRxHeader.ExtId)
	{
		case CAR_DATE_ID:
			if (pRxHeader.DLC == 6)
			{
				carDate = *(uint64_t *)rxData;
			}
			break;
		case CAR_SPEED_ID:
			if (pRxHeader.DLC == 8 && rxData[7] == 0)
			{
				dt = (uint16_t)rxData[5];		//low byte
				dt += ((uint16_t)rxData[4] << 8) & 0xFF00;
//				dt = ptr[2];

				carSpeed = (int16_t)(dt >> 7);
				carFSpeed = dt;
				carFSpeed /= 128;
			}
			break;
		case CAR_ECTEMP_ID:
			if (pRxHeader.DLC == 8)
			{
				carTemp = (int16_t)rxData[3] - 40;
			}
			break;
		case CAR_ECON_ID:
			if (pRxHeader.DLC == 6)
			{
				dt = ((((uint16_t)rxData[4]<<8) & 0xFF00) + rxData[5]);

				if (carFSpeed > 3) carEcon1 = (uint32_t)((dt*1000/512) / carFSpeed );
				else carEcon1 = (uint32_t)(dt*10/512);	//if speed=0 show economy in l/h

				carEcon1 = CalcAvgEcon(carEcon1);	//sliding average by 8 points
			}
			break;
		case CAR_STAT1_ID:
			if (pRxHeader.DLC == 8)
			{
				ptr = (uint16_t *)rxData;

				if (rxData[6] & 0x20)	//Handbrake
					{
						carBrake = true;
					}
				else {
						carBrake = false;
					}

				if (rxData[1] & 0x20)	//Light
					{
						carLightON = 1;
					}
				else {
						carLightON = 0;
					}

				if ((rxData[5] & 0x18) == 0x18)	//Ignition
				{
					carIgnON = true;
//					SendDebugMsg("I\r");
				}
				else {
					carIgnON = false;
//					SendDebugMsg("i\r");
				}
			}
			break;
		case CAR_STAT3_ID:
			if (pRxHeader.DLC == 8)
			{
				ptr = (uint16_t *)rxData;
				if (rxData[0] == 0x80)	//Engine ON
				{
					carEngON = 1;
				}
				else {
					carEngON = 0;
				}
			}
			break;
		case CAR_RPM_ID:
			if (pRxHeader.DLC == 8)
			{
				carRPM = ((((uint16_t)rxData[2]<<8) & 0xFF00) + rxData[3]);
			}
			break;
		default:
			break;
	}

}

void RequestOBD(void)
{
	//request a fuel consumption value
//	uint8_t message[] = {0x3,0x22,0x19,0x42,0x0,0x0,0x0,0x0};
//	uint32_t address = CAR_REQ_ID;
	uint8_t message[] = {0x5,0x62,0x19,0x42,0x3,0xDA};
	uint32_t address = CAR_ECON_ID;

	CAN_Send(message, sizeof(message), address);

}


uint16_t CalcAvgEcon(uint16_t val)
{
	static uint16_t buff[ECON_AVG_BUF_SIZE];
	static uint8_t idx;
	static uint32_t avg;
	uint32_t s_avg;

	if (idx >= ECON_AVG_BUF_SIZE) idx = 0;
	buff[idx++] = val;

	s_avg = 0;
	for(uint8_t i=0;i<ECON_AVG_BUF_SIZE;i++) s_avg += buff[i];
	s_avg /= ECON_AVG_BUF_SIZE;

	if (blinkCounter == 0) avg = s_avg;

	return (uint16_t)avg;
}

