/*
 * dxl_error_handler.cpp
 *
 *  Created on: Sep 30, 2025
 *      Author: AI Assistant
 *  Description: Dynamixel 모터 에러 처리 모듈 구현
 */

#include "dxl_error_handler.h"
#include "main.h"
#include "cmsis_os.h"
#include "string.h"

/* Component -----------------------------------------------------------------*/
#include "cpp_tick.h"

/* Private 변수 --------------------------------------------------------------*/

// 모터별 마지막 모션 수신 타이머 배열 (모터 ID 1~MAX_MOTOR_COUNT)
static Tick t_rxLastRxMotion[MAX_MOTOR_COUNT + 1];

// 에러 전송 카운터 배열 [모터ID][에러비트]
static uint8_t errorSendCount[MAX_MOTOR_COUNT + 1][MAX_ERROR_BITS] = {0};

/* Public 함수 구현 -----------------------------------------------------------*/

/**
 * @brief 에러 핸들러 초기화
 */
void DXL_ErrorHandler_Init(void) {
    for(uint8_t i = 1; i <= MAX_MOTOR_COUNT; i++) {
        t_rxLastRxMotion[i].tickUpdate();
    }

    for(uint8_t i = 0; i <= MAX_MOTOR_COUNT; i++) {
        for(uint8_t j = 0; j < MAX_ERROR_BITS; j++) {
            errorSendCount[i][j] = 0;
        }
    }
}

/**
 * @brief 에러 카운터 리셋 (모터의 모든 에러 카운터 초기화)
 */
void DXL_ErrorHandler_ResetCounters(uint8_t motorID) {
    if(motorID > MAX_MOTOR_COUNT) return;

    for(int i = 0; i < MAX_ERROR_BITS; i++) {
        errorSendCount[motorID][i] = 0;
    }
}

/**
 * @brief 개별 모터의 하드웨어 에러 체크 및 전송 처리
 */
void DXL_ErrorHandler_CheckMotor(uint8_t motorID) {
    (void)motorID;
    // Status_Run 중 주기 READ는 HardFault 유발. 0.0.6과 같이 동작 중에는 읽지 않는다.
}

/**
 * @brief 모든 모터의 하드웨어 에러 주기적 체크
 */
void DXL_ErrorHandler_CheckAll(void) {
    // 0.0.6과 동일: Status_Run(모션 중) 주기적 Hardware Error READ 하지 않음
}

/**
 * @brief 모터가 모션을 수신했을 때 타이머 업데이트
 */
void DXL_ErrorHandler_UpdateMotionTimer(uint8_t motorID) {
    if(motorID > 0 && motorID <= MAX_MOTOR_COUNT) {
        t_rxLastRxMotion[motorID].tickUpdate();
    }
}
