/*
 * dxl_task.cpp
 *
 *  Created on: Dec 13, 2023
 *      Author: minim
 */


#include "main.h"


#include "string.h"
/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

/* User Task -----------------------------------------------------------------*/
#include "dxl_task.h"
#include "mrs_task.h"

/* Component -----------------------------------------------------------------*/
#include "UART_Class.h"
#include "DXL_Class.h"
#include "DXL_Manager.h"
#include "cpp_tick.h"


extern osMessageQueueId_t txQueueHandle;
extern osMessageQueueId_t rxQueueHandle;

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
Serial serial1;
Serial serial2;
Serial serial3;

DXL_Manager dxlManager;

void DXL_Manager_Init(void);
void Serial_Init(void);
static uint8_t readGroupID(void);
void mrs_rx_bypass(BypassPacket_TypeDef *cmd_rx);

void DxlTask(void *argument)
{
	/* USER CODE BEGIN DxlTask */
	BypassPacket_TypeDef rx_bypass;
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

		/* os queue의 모든 메시지를 꺼냄*/
		osStatus_t status = osStatusReserved;
	    do {
	    	status = osMessageQueueGet(rxQueueHandle, &rx_bypass, NULL, 0U); // wait for message
	        if (status == osOK) {
	        	mrs_rx_bypass(&rx_bypass);
			}
	    } while (status == osOK); // 큐가 비어있지 않는 동안 계속 반복

		dxlManager.allMotorProcess();

		//dxlManager.allTimeCheckPosi();

		serial1.rxLed_Check();
		serial2.rxLed_Check();
		serial3.rxLed_Check();

		if(t_Run.delay(500))
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);

	}
	/* USER CODE END DxlTask */
}


void mrs_rx_bypass(BypassPacket_TypeDef *cmd_rx) {

	//240119 init error -> 응답 하지 않음
	if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_InitError)
		return;

	switch (cmd_rx->cmd) {
	case MRS_RX_DATA1: {
		prtc_data_ctl_init_driver_data1_t *pData = (prtc_data_ctl_init_driver_data1_t*) cmd_rx->data;

		//if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_PreRun
		//		||dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_SettingInfo){

			dxlManager.setSettingInfo(
					cmd_rx->gid,
					cmd_rx->sid,
					(pData->direction == 0 ? DXL_ROT_CW : DXL_ROT_CCW),
					(float) pData->angle / 100,
					pData->init_position,
					0
					);

			BypassPacket_TypeDef msg;
			msg.gid = cmd_rx->gid;
			msg.sid = cmd_rx->sid;
			msg.cmd = MRS_TX_DATA1_ACK;
			memcpy(msg.data, (uint8_t *)pData, 8);
			osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
		//}
		break;
	}

	case MRS_RX_DATA_OP : {
		prtc_data_ctl_init_driver_data_op_dxl_t *pData = (prtc_data_ctl_init_driver_data_op_dxl_t*) cmd_rx->data;

		if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_SettingInfo
			|| dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_SettingData_op){

			dxlManager.setSettingData_op(
					cmd_rx->gid,
					cmd_rx->sid,
					pData->home_cnt,
					0
					);

			BypassPacket_TypeDef msg;
			msg.gid = cmd_rx->gid;
			msg.sid = cmd_rx->sid;
			msg.cmd = MRS_TX_DATA_OP_ACK;
			memcpy(msg.data, (uint8_t *)pData, 8);
			osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
		}
		//240119 재전송시 응답만
		else if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Statis_SettingOk){
			BypassPacket_TypeDef msg;
			msg.gid = cmd_rx->gid;
			msg.sid = cmd_rx->sid;
			msg.cmd = MRS_TX_DATA_OP_ACK;
			memcpy(msg.data, (uint8_t *)pData, 8);
			osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
		}
		break;
	}
	case MRS_RX_MOVE_DEFAULT_POSI: {
		if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Statis_SettingOk
				|| dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_Run){ // 241105 Run 상태에서 다시 초기위치 이동을 위하여 추가
			dxlManager.setDefaultPosi_Ready(cmd_rx->gid, cmd_rx->sid);
		}
		break;
	}

	case MRS_RX_MOVE_DEFAULT_POSI_CHECK: {
		// MRS_RX_MOVE_DEFAULT_POSI 를 수신 받지 못했을 경우 초기위치 설정
		if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Statis_SettingOk){
			dxlManager.setDefaultPosi_Ready(cmd_rx->gid, cmd_rx->sid);
		}

		BypassPacket_TypeDef msg = {0,};
		msg.gid = cmd_rx->gid;
		msg.sid = cmd_rx->sid;
		msg.cmd = MRS_TX_MOVE_DEFAULT_POSI_CHECK;
		if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_Run)
			msg.data[0] = 1;
		else
			msg.data[0] = 0;
		osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
		break;
	}

	case MRS_RX_MOTION : {
		prtc_data_ctl_motion_adc_t *pData = (prtc_data_ctl_motion_adc_t*) cmd_rx->data;
		dxlManager.setPosition(cmd_rx->sid, pData->adc_val);
		break;
	}
	case MRS_RX_JOG_MOVE : {
		prtc_data_ctl_motion_direction_t *pData = (prtc_data_ctl_motion_direction_t*) cmd_rx->data;
		if(pData->direction == MOTION_DIRECTION_CCW)
			dxlManager.setJogMove(cmd_rx->sid, -pData->val);
		else
		{
			dxlManager.setJogMove(cmd_rx->sid, pData->val);
		}
		break;
	}




	default:
		break;

	}

}

//---------------------------------------

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

	/* serial3 [uart1] 21~30 */
	serial3.init(&huart1, USART1_IRQn);
	serial3.init_txLed(LD1_GPIO_Port, LD1_Pin, GPIO_PIN_RESET);
	serial3.init_rxLed(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	serial3.init_rs485(USART1_EN_GPIO_Port, USART1_EN_Pin);
}

static uint8_t idRead(
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

static uint8_t readGroupID(void){
	return idRead(
			ID_01_GPIO_Port, ID_01_Pin,
			ID_02_GPIO_Port, ID_02_Pin,
			ID_04_GPIO_Port, ID_04_Pin,
			ID_08_GPIO_Port, ID_08_Pin,
			GPIO_PIN_RESET);
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

