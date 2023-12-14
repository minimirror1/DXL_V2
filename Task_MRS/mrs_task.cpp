/*
 * mrs_task.cpp
 *
 *  Created on: Dec 14, 2023
 *      Author: minim
 */

#include "main.h"
#include "can.h"

/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mrs_task.h"
#include "cpp_tick.h"

/* MRS -----------------------------------------------------------------------*/
#include "app_pid_motion_cmd.h"
#include "app_pid_init_cmd.h"

uint8_t my_gid = 0;

uint8_t idRead(
		GPIO_TypeDef* GPIO_01, uint16_t Pin_01,
		GPIO_TypeDef* GPIO_02, uint16_t Pin_02,
		GPIO_TypeDef* GPIO_04, uint16_t Pin_04,
		GPIO_TypeDef* GPIO_08, uint16_t Pin_08,
		GPIO_PinState setState);

void MrsTask(void *argument)
{
	/* USER CODE BEGIN MrsTask */
	/* MRS protocol */
	can_init_data_save (&hcan1);
	gm_motion_TX_LED_init(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	gm_motion_RX_LED_init(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);

	my_gid = idRead(
			ID_01_GPIO_Port, ID_01_Pin,
			ID_02_GPIO_Port, ID_02_Pin,
			ID_04_GPIO_Port, ID_04_Pin,
			ID_08_GPIO_Port, ID_08_Pin,
			GPIO_PIN_RESET);


	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
	/* USER CODE END MrsTask */
}

uint8_t idRead(
		GPIO_TypeDef* GPIO_01, uint16_t Pin_01,
		GPIO_TypeDef* GPIO_02, uint16_t Pin_02,
		GPIO_TypeDef* GPIO_04, uint16_t Pin_04,
		GPIO_TypeDef* GPIO_08, uint16_t Pin_08,
		GPIO_PinState setState){
	uint8_t ret = 0;
	ret += (HAL_GPIO_ReadPin(GPIO_01, Pin_01) == setState)? 1:0;
	ret += (HAL_GPIO_ReadPin(GPIO_02, Pin_02) == setState)? 2:0;
	ret += (HAL_GPIO_ReadPin(GPIO_04, Pin_04) == setState)? 4:0;
	ret += (HAL_GPIO_ReadPin(GPIO_08, Pin_08) == setState)? 8:0;

	return ret;
}
