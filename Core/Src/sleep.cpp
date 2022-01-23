/*
 * sleep.cpp
 *
 *  Created on: 26 лип. 2021 р.
 *      Author: Victor
 */
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "sleep.h"
#include "screens.h"
//#include <stm32l0xx.h>

void GotoSleep(void)
{
	ByScreen();
	osDelay(800);

	//put screen and MC to stop mode
	StandbyScr();

	CAN_Sleep();
	HAL_SuspendTick();

    /* TIM2 clock DeInit */
    __HAL_RCC_TIM2_CLK_DISABLE();
    /* TIM2 interrupt DeInit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);

    EXTI_HandleTypeDef hexti_canrx = { 0 };
    EXTI_ConfigTypeDef hexti_canrx_config, hexti_canrx_conf_bk;

    // Get the handle of the EXTI 11 ( bxCAN RX line )
    HAL_EXTI_GetHandle(&hexti_canrx, EXTI_LINE_11);

    HAL_EXTI_GetConfigLine(
                           &hexti_canrx,
                           &hexti_canrx_config
                           );
    hexti_canrx_conf_bk = hexti_canrx_config;	//make a backup copy

    hexti_canrx_config.Mode = EXTI_MODE_EVENT;
    hexti_canrx_config.Trigger = EXTI_TRIGGER_RISING_FALLING;

    __disable_irq(); //disable so that we don't end up in the ISR until we reconfigure the clocks after wakeup

    HAL_EXTI_SetConfigLine(&hexti_canrx, &hexti_canrx_config);

    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFE); // STOP mode, wake up at event

    HAL_EXTI_ClearPending(&hexti_canrx, EXTI_TRIGGER_RISING_FALLING);
    HAL_EXTI_SetConfigLine(&hexti_canrx, &hexti_canrx_conf_bk); //restore line 11 config

	__enable_irq(); //now the EXTI interrupt will fire

    HAL_ResumeTick();
}

void ResumeFromSleep(void)
{

    /* Peripheral clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 9, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    CAN_Resume();

//	HiScreen();
	ResumeScr();
//	osDelay(100);

	SwitchScreen(NORM_SCREEN);
}
