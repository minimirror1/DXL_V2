/*
 * dxl_task.cpp
 *
 *  Created on: Dec 13, 2023
 *      Author: minim
 */


#include "main.h"

/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* User Task -----------------------------------------------------------------*/
#include "dxl_task.h"

/* Component -----------------------------------------------------------------*/
#include "UART_Class.h"
#include "DXL_Class.h"
#include "DXL_Manager.h"
#include "cpp_tick.h"


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
Serial serial1;
Serial serial2;
Serial serial3;

DXL_Manager dxlManager;

void DXL_Manager_Init(void);
void Serial_Init(void);
uint8_t readGroupID(void);

void DxlTask(void *argument)
{
	/* USER CODE BEGIN DxlTask */
	osDelay(1000);

	Tick t_Run;

	Serial_Init();
	DXL_Manager_Init();
	//DXL_motor dxl1(1, 2, Motor::Robotis_Type, &serial1);
	//dxl1.init();
	dxlManager.initializeAll();

	/* Infinite loop */
	for (;;) {
		osDelay(1);

		serial1.rxLed_Check();
		serial2.rxLed_Check();
		serial3.rxLed_Check();

		if(t_Run.delay(500))
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	}
	/* USER CODE END DxlTask */
}

void DXL_Manager_Init(void){
	uint8_t gID = readGroupID();

	for(int i = 1; i <= 10; i++)
		dxlManager.addDXLObject(gID, i, Motor::Robotis_Type, &serial1);
	for(int i = 11; i <= 20; i++)
		dxlManager.addDXLObject(gID, i, Motor::Robotis_Type, &serial2);
	for(int i = 21; i <= 30; i++)
		dxlManager.addDXLObject(gID, i, Motor::Robotis_Type, &serial3);
}

void Serial_Init(void){
	/* serial1 [uart2] 1~10 */
	serial1.init(&huart2, USART2_IRQn);
	serial1.init_txLed(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	serial1.init_rxLed(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	serial1.init_rs485(USART2_EN_GPIO_Port, USART2_EN_Pin);

	/* serial2 [uart3] 11~20 */
	serial2.init(&huart3, USART3_IRQn);
	serial2.init_txLed(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	serial2.init_rxLed(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	serial2.init_rs485(USART3_EN_GPIO_Port, USART3_EN_Pin);

	/* serial3 [uart1] 1~10 */
	serial3.init(&huart1, USART1_IRQn);
	serial3.init_txLed(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	serial3.init_rxLed(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	serial3.init_rs485(USART1_EN_GPIO_Port, USART1_EN_Pin);
}


uint8_t readGroupID(void){
	return 1;
}

/* HAL Driver Callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	serial1.TxCpltCallback(huart);
	serial2.TxCpltCallback(huart);
	serial3.TxCpltCallback(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	serial1.RxCpltCallback(huart);
	serial2.RxCpltCallback(huart);
	serial3.RxCpltCallback(huart);
}

