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
#include "dxl_error_handler.h"

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
	// 에러 핸들러 초기화 (모터 타이머 및 카운터 초기화)
	DXL_ErrorHandler_Init();

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
		// Status_Run 중 주기적 Hardware Error READ는 HardFault 유발.
		// 0.0.6과 같이 동작 중에는 호출하지 않는다.

		serial1.rxLed_Check();
		serial2.rxLed_Check();
		serial3.rxLed_Check();

		if(t_Run.delay(500))
			HAL_GPIO_TogglePin(LD5_RUN_GPIO_Port, LD5_RUN_Pin);

	}
	/* USER CODE END DxlTask */
}


void mrs_rx_bypass(BypassPacket_TypeDef *cmd_rx) {

	//240119 init error -> 응답 하지 않음
	//if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_InitError)
	//	return;

	switch (cmd_rx->cmd) {
	case MRS_RX_DATA1: {
		prtc_data_ctl_init_driver_data1_t *pData = (prtc_data_ctl_init_driver_data1_t*) cmd_rx->data;

		//if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_PreRun
		//		||dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_SettingInfo){

			if(cmd_rx->sid > MAX_MOTOR_COUNT)
				return;
			
			if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) != Motor::Status_PreRun){
				dxlManager.initializeOne(cmd_rx->sid);
				DXL_ErrorHandler_ResetCounters(cmd_rx->sid);
			}
		
			if(dxlManager.getStatus(cmd_rx->gid, cmd_rx->sid) == Motor::Status_InitError){
				BypassPacket_TypeDef msg = {0,};
				msg.gid = cmd_rx->gid;
				msg.sid = cmd_rx->sid;
				msg.cmd = MRS_TX_HARDWARE_ERROR_EVENT;
				strcpy((char*)msg.data, "InitErr");
				osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
			}
			else{
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
			}
		break;
	}

	case MRS_RX_DATA_OP : {
		prtc_data_ctl_init_driver_data_op_dxl2_t *pData = (prtc_data_ctl_init_driver_data_op_dxl2_t*) cmd_rx->data;

		if(cmd_rx->sid > MAX_MOTOR_COUNT)
			return;

		BypassPacket_TypeDef msg = {0,};
		msg.gid = cmd_rx->gid;
		msg.sid = cmd_rx->sid;
		msg.cmd = MRS_TX_DATA_OP_ACK;

		if(DXL_motor::isSupportedCase(pData->case_num) == false){
			// 미지원 case_num : 설정을 반영하지 않고, case_num 자리에 거부 표시를 실어 응답한다.
			// 나머지 슬롯은 0 으로 남긴다.
			prtc_data_rsp_init_driver_data_op_dxl2_t *pRsp =
					(prtc_data_rsp_init_driver_data_op_dxl2_t*) msg.data;
			pRsp->case_num = DXL_MAP_CASE_REJECT;
		}
		else{
			// 상위 재부팅이나 설정값 변경으로 언제든 재수신될 수 있으므로
			// 이전 상태를 따지지 않고 항상 반영한다.
			// (240119 에 추가되었던 "재전송시 응답만" 가드는 설정 변경이 반영되지 않아 제거함)
			dxlManager.setSettingData_op(
					cmd_rx->gid,
					cmd_rx->sid,
					pData->case_num,
					pData->min,
					pData->ref_position,
					pData->max
					);

			// 상위가 반영 결과를 검증할 수 있도록 수신값을 그대로 되돌려준다.
			memcpy(msg.data, (uint8_t *)pData, 8);
		}

		osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
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

	case MRS_RX_SCALE_RAW_RQT: {
		BypassPacket_TypeDef msg = {0,};
		msg.gid = cmd_rx->gid;
		msg.sid = cmd_rx->sid;
		msg.cmd = MRS_TX_SCALE_RAW_RSP;

		int64_t raw = 0;
		if(cmd_rx->sid <= MAX_MOTOR_COUNT)
			raw = (int64_t)dxlManager.getRawCommandPosition(cmd_rx->sid);
		memcpy(msg.data, &raw, sizeof(raw));

		osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
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
	/* serial1 [uart2] 1~10 - IT 모드 사용 */
	serial1.init(&huart2, USART2_IRQn);  // IT 모드
	serial1.init_txLed(LD3_485_TX_GPIO_Port, LD3_485_TX_Pin, GPIO_PIN_RESET);
	serial1.init_rxLed(LD4_485_RX_GPIO_Port, LD4_485_RX_Pin, GPIO_PIN_RESET);
	serial1.init_rs485(USART2_EN_GPIO_Port, USART2_EN_Pin);

	/* serial2 [uart3] 11~20 */
	serial2.init(&huart3, USART3_IRQn);  // IT 모드
	serial2.init_txLed(LD3_485_TX_GPIO_Port, LD3_485_TX_Pin, GPIO_PIN_RESET);
	serial2.init_rxLed(LD4_485_RX_GPIO_Port, LD4_485_RX_Pin, GPIO_PIN_RESET);
	serial2.init_rs485(USART3_EN_GPIO_Port, USART3_EN_Pin);

	/* serial3 [uart1] 21~30 */
	serial3.init(&huart1, USART1_IRQn);  // IT 모드
	serial3.init_txLed(LD3_485_TX_GPIO_Port, LD3_485_TX_Pin, GPIO_PIN_RESET);
	serial3.init_rxLed(LD4_485_RX_GPIO_Port, LD4_485_RX_Pin, GPIO_PIN_RESET);
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
	serial1.RxCpltCallback(huart);  // IT, DMA 모드 공용
	serial2.RxCpltCallback(huart);
	serial3.RxCpltCallback(huart);
}

