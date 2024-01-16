/*
 * mrs_task.h
 *
 *  Created on: Dec 14, 2023
 *      Author: minim
 */

#ifndef MRS_TASK_H_
#define MRS_TASK_H_



#ifdef __cplusplus
extern "C" {
#endif


typedef enum _TxBypassCmd_TypeDef{
	MRS_TX_DATA1_ACK,
	MRS_TX_DATA2_ACK,
	MRS_TX_DATA_OP_ACK,
	MRS_TX_MOVE_DEFAULT_POSI_ACK,
	MRS_TX_MOVE_DEFAULT_POSI_CHECK
}TxBypassCmd_TypeDef;

typedef enum _RxBypassCmd_TypeDef{
	MRS_RX_DATA1 = MRS_TX_MOVE_DEFAULT_POSI_CHECK + 1,
	MRS_RX_DATA2,
	MRS_RX_DATA_OP,
	MRS_RX_MOVE_DEFAULT_POSI,
	MRS_RX_MOVE_DEFAULT_POSI_CHECK,
	MRS_RX_MOTION,
	MRS_RX_JOG_MOVE
}RxBypassCmd_TypeDef;

typedef struct _BypassPacket_TypeDef{
	uint8_t gid;
	uint8_t sid;
	uint8_t cmd;
	uint8_t data[8];
}BypassPacket_TypeDef;

void MrsTask(void *argument);


#ifdef __cplusplus
}
#endif


#endif /* MRS_TASK_H_ */
