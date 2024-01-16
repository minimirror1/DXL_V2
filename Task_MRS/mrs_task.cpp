/*
 * mrs_task.cpp
 *
 *  Created on: Dec 14, 2023
 *      Author: minim
 */

#include "main.h"
#include "can.h"

#include "string.h"
/* RTOS ----------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"

#include "mrs_task.h"
#include "cpp_tick.h"

/* MRS -----------------------------------------------------------------------*/
#include "app_pid_motion_cmd.h"
#include "app_pid_init_cmd.h"

extern osMessageQueueId_t txQueueHandle;
extern osMessageQueueId_t rxQueueHandle;

extern can_q_buff_t can_rx_ring_buff[CAN_CNT];

uint8_t my_gid = 0;

void mrs_tx_Bypass(BypassPacket_TypeDef *cmd_tx);
void CAN_FilterConfig(CAN_HandleTypeDef *hcan);
static uint8_t idRead(
		GPIO_TypeDef* GPIO_01, uint16_t Pin_01,
		GPIO_TypeDef* GPIO_02, uint16_t Pin_02,
		GPIO_TypeDef* GPIO_04, uint16_t Pin_04,
		GPIO_TypeDef* GPIO_08, uint16_t Pin_08,
		GPIO_PinState setState);

void MrsTask(void *argument)
{
	/* USER CODE BEGIN MrsTask */
	BypassPacket_TypeDef tx_bypass;


	/* MRS protocol */
	can_init_data_save (&hcan1);
	gm_motion_TX_LED_init(LD4_GPIO_Port, LD4_Pin, GPIO_PIN_RESET);
	gm_motion_RX_LED_init(LD5_GPIO_Port, LD5_Pin, GPIO_PIN_RESET);
	CAN_FilterConfig(&hcan1);

	my_gid = idRead(
			ID_01_GPIO_Port, ID_01_Pin,
			ID_02_GPIO_Port, ID_02_Pin,
			ID_04_GPIO_Port, ID_04_Pin,
			ID_08_GPIO_Port, ID_08_Pin,
			GPIO_PIN_RESET);

	set_my_can_id(my_gid);
	add_my_can_sub_id(1, 30);

	osDelay(3000);
	can_rx_ring_buff[0].head = 0;
	can_rx_ring_buff[0].tail = 0;
	app_tx_init_sub_pid_boot_ctl(
			0,
			0,
			my_gid,
			MASTER_CAN_ID,
			1,
			0);

	/* Infinite loop */
	for (;;) {
		osDelay(1);

		osStatus_t status;
		status = osMessageQueueGet(txQueueHandle, &tx_bypass, NULL, 0U); // wait for message
		if (status == osOK) {
			mrs_tx_Bypass(&tx_bypass);
		}


		/* MRS Protocol */
		proc_can_rx();
		proc_can_tx();

	}
	/* USER CODE END MrsTask */
}


void mrs_tx_Bypass(BypassPacket_TypeDef *cmd_tx){

	if (cmd_tx->sid > 30)
			return;

	switch (cmd_tx->cmd) {
	case MRS_TX_DATA1_ACK: {

		app_tx_init_sub_pid_driver_data1_rsp(
			0,
			0,
			cmd_tx->gid,
			MASTER_CAN_ID,
			cmd_tx->sid,
			0,
			0,
			0,
			0,
			0);
		break;
	}
	case MRS_TX_DATA2_ACK: {
		app_tx_init_sub_pid_driver_data2_rsp(
			0,
			0,
			cmd_tx->gid,
			MASTER_CAN_ID,
			cmd_tx->sid,
			0,
			0,
			0);
		break;
	}
	case MRS_TX_DATA_OP_ACK : {
		uint8_t data;
		app_tx_init_sub_pid_driver_data_op_rsp(
			0,
			0,
			cmd_tx->gid,
			MASTER_CAN_ID,
			cmd_tx->sid,
			0,
			&data);
		break;
	}
	case MRS_TX_MOVE_DEFAULT_POSI_CHECK: {
		app_tx_init_sub_pid_status_rsp(
			0,
			0,
			cmd_tx->gid,
			MASTER_CAN_ID,
			cmd_tx->sid,
			0,
			MOVE_INIT_POSITION,
			cmd_tx->data[0]);//status 1
		break;
	}

	default :
		break;
	}
}

void app_rx_motion_sub_pid_adc_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_motion_adc_t *pData)
{
	uint8_t axleId = (uint8_t)pPh->target_sub_id;
	axleId -= 1;
	if (32 < axleId) return;

	BypassPacket_TypeDef bypassRx;

	bypassRx.gid = pPh->target_id;
	bypassRx.sid = pPh->target_sub_id;
	bypassRx.cmd = MRS_RX_MOTION;
	memcpy(
	    bypassRx.data,
	    pData,
	    sizeof(*pData)
	);
	osMessageQueuePut(rxQueueHandle, &bypassRx, 0U, 0U);
}

/* jog move */
void app_rx_motion_sub_pid_direction_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_motion_direction_t *pData)
{
	uint8_t axleId = (uint8_t)pPh->target_sub_id;
	axleId -= 1;
	if (32 < axleId) return;

	BypassPacket_TypeDef bypassRx;

	bypassRx.gid = pPh->target_id;
	bypassRx.sid = pPh->target_sub_id;
	bypassRx.cmd = MRS_RX_JOG_MOVE;
	memcpy(
		bypassRx.data,
		pData,
		sizeof(*pData)
	);
	osMessageQueuePut(rxQueueHandle, &bypassRx, 0U, 0U);
}


/* init sequnce */
void app_rx_init_sub_pid_driver_data1_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_init_driver_data1_t *pData)
{
	BypassPacket_TypeDef bypassRx;
	bypassRx.gid = pPh->target_id;
	bypassRx.sid = pPh->target_sub_id;
	bypassRx.cmd = MRS_RX_DATA1;
	memcpy(bypassRx.data, (uint8_t *)pData, 8);

	osMessageQueuePut(rxQueueHandle, &bypassRx, 0U, 0U);
}
//op 로 변경
void app_rx_init_sub_pid_driver_data2_ctl(uint8_t num, prtc_header_t *pPh, prtc_data_ctl_init_driver_data2_t *pData)
{
	BypassPacket_TypeDef bypassRx;
	bypassRx.gid = pPh->target_id;
	bypassRx.sid = pPh->target_sub_id;
	bypassRx.cmd = MRS_RX_DATA2;
	memcpy(bypassRx.data, (uint8_t *)pData, 8);

	osMessageQueuePut(rxQueueHandle, &bypassRx, 0U, 0U);
}

void app_rx_init_sub_pid_driver_data_op_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	BypassPacket_TypeDef bypassRx;
	bypassRx.gid = pPh->target_id;
	bypassRx.sid = pPh->target_sub_id;
	bypassRx.cmd = MRS_RX_DATA_OP;
	memcpy(bypassRx.data, (uint8_t *)pData, 8);

	osMessageQueuePut(rxQueueHandle, &bypassRx, 0U, 0U);
}

void app_rx_init_sub_pid_move_sensor_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	app_tx_init_sub_pid_move_sensor_rsp(
		num,
		0,
		pPh->target_id,
		MASTER_CAN_ID,
		pPh->target_sub_id,
		0,
		0);
}

void app_rx_init_sub_pid_status_rqt(uint8_t num, prtc_header_t *pPh, prtc_data_rqt_init_status_t *pData)
{
	uint8_t status = 0;

	prtc_data_rqt_init_status_t *temp = (prtc_data_rqt_init_status_t *) pData;

	switch(temp->step)
	{
	//init step 1 : 1. vattery check
	case ABSOLUTE_BATTERY:
		app_tx_init_sub_pid_status_rsp(
			num,
			0,
			pPh->target_id,
			MASTER_CAN_ID,
			pPh->target_sub_id,
			0,
			ABSOLUTE_BATTERY,
			1); // ok;
		break;
	case DRIVER_DATA1:

		break;
	case DRIVER_DATA2:

		break;
	case MOVE_SENSOR:

		status = 1;//ok
		app_tx_init_sub_pid_status_rsp(
			num,
			0,
			pPh->target_id,
			MASTER_CAN_ID,
			pPh->target_sub_id,
			0,
			MOVE_SENSOR,
			status);

		break;
	case MOVE_INIT_POSITION:

		BypassPacket_TypeDef bypassRx;
		bypassRx.gid = pPh->target_id;
		bypassRx.sid = pPh->target_sub_id;
		bypassRx.cmd = MRS_RX_MOVE_DEFAULT_POSI_CHECK;
		memcpy(bypassRx.data, (uint8_t *)pData, 8);

		//초기위치로 이동 완료하였는지 확인
		osMessageQueuePut(rxQueueHandle, &bypassRx, 0U, 0U);

		break;
	}
}
void app_rx_init_sub_pid_move_init_position_ctl(uint8_t num, prtc_header_t *pPh, uint8_t *pData)
{
	BypassPacket_TypeDef bypassRx;
	bypassRx.gid = pPh->target_id;
	bypassRx.sid = pPh->target_sub_id;
	bypassRx.cmd = MRS_RX_MOVE_DEFAULT_POSI;

	osMessageQueuePut(rxQueueHandle, &bypassRx, 0U, 0U);

 	app_tx_init_sub_pid_move_init_position_rsp(
		num,
		0,
		pPh->target_id,
		MASTER_CAN_ID,
		pPh->target_sub_id,
		0);
}


void CAN_FilterConfig(CAN_HandleTypeDef *hcan)
{
	CAN_FilterTypeDef  sFilterConfig;

	/*##-2- Configure the CAN Filter ###########################################*/
#ifdef CAN1
	if(hcan->Instance == CAN1)
#else
	if(hcan->Instance == CAN)
#endif
	{
		sFilterConfig.FilterBank = 0;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.SlaveStartFilterBank = 14;
	}
	else
	{
		sFilterConfig.FilterBank = 14;
		sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
		sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
		sFilterConfig.FilterIdHigh = 0x0000;
		sFilterConfig.FilterIdLow = 0x0000;
		sFilterConfig.FilterMaskIdHigh = 0x0000;
		sFilterConfig.FilterMaskIdLow = 0x0000;
		sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
		sFilterConfig.FilterActivation = ENABLE;
		sFilterConfig.SlaveStartFilterBank = 14;
	}

	if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK)
	{
		/* Filter configuration Error */
		Error_Handler();
	}

	/*##-3- Start the CAN peripheral ###########################################*/
	if (HAL_CAN_Start(hcan) != HAL_OK)
	{
		/* Start Error */
		Error_Handler();
	}

	/*##-4- Activate CAN RX notification #######################################*/
	if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}
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
