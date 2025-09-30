/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LD1_CAN_RX_Pin GPIO_PIN_13
#define LD1_CAN_RX_GPIO_Port GPIOC
#define LD2_CAN_TX_Pin GPIO_PIN_14
#define LD2_CAN_TX_GPIO_Port GPIOC
#define LD5_RUN_Pin GPIO_PIN_0
#define LD5_RUN_GPIO_Port GPIOA
#define USART2_EN_Pin GPIO_PIN_1
#define USART2_EN_GPIO_Port GPIOA
#define ID_08_Pin GPIO_PIN_4
#define ID_08_GPIO_Port GPIOA
#define ID_04_Pin GPIO_PIN_5
#define ID_04_GPIO_Port GPIOA
#define ID_02_Pin GPIO_PIN_6
#define ID_02_GPIO_Port GPIOA
#define ID_01_Pin GPIO_PIN_7
#define ID_01_GPIO_Port GPIOA
#define USART1_EN_Pin GPIO_PIN_8
#define USART1_EN_GPIO_Port GPIOA
#define USART3_EN_Pin GPIO_PIN_15
#define USART3_EN_GPIO_Port GPIOA
#define LD4_485_RX_Pin GPIO_PIN_5
#define LD4_485_RX_GPIO_Port GPIOB
#define LD3_485_TX_Pin GPIO_PIN_6
#define LD3_485_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Version */
// 0.0.1 : 231215 ÏµúÏ¥à?ûë?Ñ±
// 0.0.2 : 240116 Î≤ÑÏ†Ñ?ùΩÍ∏? jog Í∏∞Îä• Ï∂îÍ?
// 0.0.3 : 240116 dxl ?Üµ?ã† ?Üç?èÑ 1Mbps Î≥?Í≤?
// 0.0.4 : 240127 queue ÏµúÏ†Å?ôî
// 0.0.5 : Î∞òÏùë?Üç?èÑ Ï¶ùÍ? FreeRTOS ?àò?ã†Queue Î™®Îëê ÎπÑÏ?
// 0.0.6 : ?èô?ûëÏ§? Ï¥àÍ∏∞?úÑÏπ? ?ù¥?èô ?èô?ûë

#define VERSION_MAJOR 0
#define VERSION_MINOR 0
#define VERSION_PATCH 6

/* Master CAN ID */
#define MASTER_CAN_ID 0

/*----------------------------------------------------------------------------*/
#define CAN_Q_BUFF_SIZE 	512   //  ((id 4 Byte + data 8 Byte) x 512(CAN_Q_BUFF_SIZE)) x 2(rx,tx) = 12,288 Byte
// CAN 1Í∞úÏùº Í≤ΩÏö∞
#define CAN_1	0 //
#define CAN_CNT 1

void HAL_CAN_RxFifo0MsgPendingCallback1(CAN_HandleTypeDef *hcan);
#include "dl_can.h"
#include "net_phd_pid.h"


// 1 : PDO CSP, 2 : PDO PP, 3 : SDO PP
#define PDO_CSP			1
#define PDO_PP			2
#define SDO_PP			3
#define CANOPEN_MODE	PDO_PP


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
