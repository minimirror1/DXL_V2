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
#define LD1_Pin GPIO_PIN_13
#define LD1_GPIO_Port GPIOC
#define LD2_Pin GPIO_PIN_14
#define LD2_GPIO_Port GPIOC
#define LD3_Pin GPIO_PIN_15
#define LD3_GPIO_Port GPIOC
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
#define LD5_Pin GPIO_PIN_6
#define LD5_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_7
#define LD4_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/* Version */
// 0.0.1 : 231215 최초작성
// 0.0.2 : 240116 버전읽기 jog 기능 추가
// 0.0.3 : 240116 dxl 통신 속도 1Mbps 변경
// 0.0.4 : 240127 queue 최적화
// 0.0.5 : 반응속도 증가 FreeRTOS 수신Queue 모두 비움
// 0.0.6 : 동작중 초기위치 이동 동작

#define VERSION_MAJOR 0
#define VERSION_MINOR 0
#define VERSION_PATCH 6

/* Master CAN ID */
#define MASTER_CAN_ID 0

/*----------------------------------------------------------------------------*/
#define CAN_Q_BUFF_SIZE 	512   //  ((id 4 Byte + data 8 Byte) x 512(CAN_Q_BUFF_SIZE)) x 2(rx,tx) = 12,288 Byte
// CAN 1개일 경우
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
