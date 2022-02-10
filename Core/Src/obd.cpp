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
extern bool econMode;	//0 - LPH (Liters Per Hour), 1 - LPK (Liters Per 100 Km)


extern uint8_t blinkCounter;

void CalcAvgEcon(uint16_t, bool);

#define ECON_AVG_BUF_SIZE 15

void HandleOBDMsg(CAN_RxHeaderTypeDef pRxHeader, uint8_t *rxData)
{
	uint16_t dt;
	bool ecModePrev = econMode;	//to check if changed


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

				carSpeed = (int16_t)(dt >> 7);
				carFSpeed = dt / 128;	//speed value as float

				//economy display mode
				econMode = (carSpeed >0)?true:false;

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

				int32_t econVal;

				if (econMode)
					econVal = (uint32_t)((dt*1000/512) / carFSpeed );	//economy in Lp100km
				else
					econVal = (uint32_t)(dt*10/512);	//if speed=0 show economy in lph

				CalcAvgEcon(econVal, (econMode != ecModePrev));	//clear the avg buf if mode changed
			}
			break;
		case CAR_STAT1_ID:
			if (pRxHeader.DLC == 8)
			{

				carBrake = (rxData[6] & 0x20)?true:false;	//Handbrake

				carLightON = (rxData[1] & 0x20)?true:false;	//Light

				carIgnON = ((rxData[5] & 0x18) == 0x18)?true:false;	//Ignition
			}
			break;
		case CAR_STAT3_ID:
			if (pRxHeader.DLC == 8)
			{
				carEngON = ((rxData[0] & 0x80) == 0x80)?true:false;	//Engine ON
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
	uint8_t message[] = {0x3,0x22,0x19,0x42,0x0,0x0,0x0,0x0};
	uint32_t address = CAR_REQ_ID;
//	uint8_t message[] = {0x5,0x62,0x19,0x42,0x3,0xDA};
//	uint32_t address = CAR_ECON_ID;

	CAN_Send(message, sizeof(message), address);

}

//calculation of average values for economy

	static uint16_t buff[ECON_AVG_BUF_SIZE];
	static uint8_t idx = 0;

	static uint32_t iavg = 0;	//instant (2 sec) average
	static uint16_t count = 0;	//instant average samples count

void updateAvgEcon()
{

	if (count >0) carEcon1 = iavg / count;	//instant (2 sec) economy
	else carEcon1 = 0;

	iavg = 0;
	count = 0;

	buff[idx++] = carEcon1;	//update the long-term avg buffer
	if (idx >= ECON_AVG_BUF_SIZE) idx = 0;

	uint32_t s_avg = 0;	//calc the sliding avg for the long-term economy value
	for(int8_t i=ECON_AVG_BUF_SIZE-1;i>=0;--i) s_avg += buff[i];

	carEcon2 = s_avg / ECON_AVG_BUF_SIZE;

}

void CalcAvgEcon(uint16_t val, bool resAvg)
{

	if (resAvg) 	//reset average buffers on economy calculation mode change
		{
			iavg = 0;
			count = 0;
			for(int8_t i=ECON_AVG_BUF_SIZE-1;i>=0;--i) buff[i] = val;
			idx = 0;
		}

	iavg += val;
	++count;

}

