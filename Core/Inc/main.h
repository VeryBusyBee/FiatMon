/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define FM_VERSION "v.0.5"

#define CAR_DATE_ID 0x0c28a000
#define CAR_SPEED_ID 0x0210A006
#define CAR_RPM_ID 0x0618A001
#define CAR_STAT1_ID 0x0a18a000
#define CAR_STAT2_ID 0x0a18a006
#define CAR_STAT3_ID 0x0c2ca000
#define CAR_ECTEMP_ID 0x0018a001
#define CAR_ECON_ID 0x18DAF110

#define CAR_REQ_ID 0x18DA10F1


#define DISPLAY_NORM 0
#define DISPLAY_REV 1
#define DISPLAY_RALIGN 2
#define DISPLAY_BLINK_FAST 4
#define DISPLAY_BLINK_SLOW 8
#define DISPLAY_HIDE 0x10
#define DISPLAY_BEEP 0x20

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void SendDebugMsg(char *msg);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOARD_LED_Pin GPIO_PIN_13
#define BOARD_LED_GPIO_Port GPIOC
#define DISP_RST_Pin GPIO_PIN_2
#define DISP_RST_GPIO_Port GPIOA
#define DISP_DC_Pin GPIO_PIN_3
#define DISP_DC_GPIO_Port GPIOA
#define DISP_CS_Pin GPIO_PIN_4
#define DISP_CS_GPIO_Port GPIOA
#define Norm_LB_Pin GPIO_PIN_8
#define Norm_LB_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define TJA_S_Pin GPIO_PIN_14
#define TJA_S_GPIO_Port GPIOC

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
