/*
 * dxl_error_handler.h
 *
 *  Created on: Sep 30, 2025
 *      Author: AI Assistant
 *  Description: Dynamixel 모터 에러 처리 모듈
 */

#ifndef DXL_ERROR_HANDLER_H_
#define DXL_ERROR_HANDLER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* 상수 정의 ------------------------------------------------------------------*/
#define MAX_MOTOR_COUNT         30
#define MAX_ERROR_BITS          8
#define ERROR_SEND_LIMIT        3
#define ERROR_CHECK_INTERVAL_MS 100

/* 에러 정의 구조체 -----------------------------------------------------------*/
typedef struct {
    uint8_t errorBit;       // 에러 비트 마스크
    uint8_t bitIndex;       // 배열 인덱스
    const char* errorMsg;   // 에러 메시지
} ErrorDefinition_t;

/* Public 함수 선언 -----------------------------------------------------------*/

/**
 * @brief 에러 핸들러 초기화 (모든 모터의 타이머 및 카운터 초기화)
 */
void DXL_ErrorHandler_Init(void);

/**
 * @brief 특정 모터의 에러 카운터 리셋
 * @param motorID 모터 ID (1~MAX_MOTOR_COUNT)
 */
void DXL_ErrorHandler_ResetCounters(uint8_t motorID);

/**
 * @brief 개별 모터의 하드웨어 에러 체크 및 전송 처리
 * @param motorID 모터 ID (1~MAX_MOTOR_COUNT)
 */
void DXL_ErrorHandler_CheckMotor(uint8_t motorID);

/**
 * @brief 모든 모터의 하드웨어 에러 주기적 체크
 * @note 각 모터별로 ERROR_CHECK_INTERVAL_MS 간격으로 체크
 */
void DXL_ErrorHandler_CheckAll(void);

/**
 * @brief 모터가 모션을 수신했을 때 호출 (타이머 업데이트)
 * @param motorID 모터 ID (1~MAX_MOTOR_COUNT)
 */
void DXL_ErrorHandler_UpdateMotionTimer(uint8_t motorID);

#ifdef __cplusplus
}
#endif

#endif /* DXL_ERROR_HANDLER_H_ */
