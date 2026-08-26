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
#include "DXL_Manager.h"
#include "cpp_tick.h"

/* Task ----------------------------------------------------------------------*/
#include "mrs_task.h"

/* External 변수 -------------------------------------------------------------*/
extern osMessageQueueId_t txQueueHandle;
extern DXL_Manager dxlManager;

/* Private 함수 선언 ----------------------------------------------------------*/
static void sendHardwareErrorMessage(uint8_t gID, uint8_t motorID, 
                                     const ErrorDefinition_t* errorDef);
static bool isMotorReadyForErrorCheck(Motor::Status status);
static uint8_t readGroupID(void);

/* Private 변수 --------------------------------------------------------------*/

// 하드웨어 에러 정의 테이블
static const ErrorDefinition_t HW_ERROR_TABLE[] = {
    {0x01, 0, "Voltage"},   // Bit 0: Input Voltage Error
    {0x04, 2, "HotTemp"},   // Bit 2: Overheating Error
    {0x08, 3, "Encoder"},   // Bit 3: Motor Encoder Error
    {0x10, 4, "EShock"},    // Bit 4: Electrical Shock Error
    {0x20, 5, "OvrLoad"},   // Bit 5: Overload Error
    {0x40, 6, "CommErr"},   // Bit 6: Communication Failure
    {0x80, 7, "NotAsgn"}    // Bit 7: Motor Not Assigned
};
#define HW_ERROR_COUNT (sizeof(HW_ERROR_TABLE) / sizeof(ErrorDefinition_t))

// 모터별 마지막 모션 수신 타이머 배열 (모터 ID 1~MAX_MOTOR_COUNT)
static Tick t_rxLastRxMotion[MAX_MOTOR_COUNT + 1];

// 에러 전송 카운터 배열 [모터ID][에러비트]
static uint8_t errorSendCount[MAX_MOTOR_COUNT + 1][MAX_ERROR_BITS] = {0};

/* Public 함수 구현 -----------------------------------------------------------*/

/**
 * @brief 에러 핸들러 초기화
 */
void DXL_ErrorHandler_Init(void) {
    // 모든 모터의 타이머 초기화
    for(uint8_t i = 1; i <= MAX_MOTOR_COUNT; i++) {
        t_rxLastRxMotion[i].tickUpdate();
    }
    
    // 에러 카운터 초기화 (이미 0으로 초기화되어 있지만 명시적으로)
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
    if(motorID > MAX_MOTOR_COUNT) return;
    
    uint8_t gID = readGroupID();
    Motor::Status status = dxlManager.getStatus(gID, motorID);
    
    // 연결되어 있고 Status_Run 상태인 모터만 체크
    if(!isMotorReadyForErrorCheck(status)) {
        return;
    }
    
    uint8_t errorStatus = dxlManager.getHardwareErrorStatus(motorID);
    
    // 에러가 해결된 경우 카운터 리셋
    if(errorStatus == 0) {
        DXL_ErrorHandler_ResetCounters(motorID);
        return;
    }
    
    // 하드웨어 에러 테이블을 순회하며 에러 체크 및 전송
    for(uint8_t i = 0; i < HW_ERROR_COUNT; i++) {
        if(errorStatus & HW_ERROR_TABLE[i].errorBit) {
            sendHardwareErrorMessage(gID, motorID, &HW_ERROR_TABLE[i]);
        }
    }
}

/**
 * @brief 모든 모터의 하드웨어 에러 주기적 체크
 */
void DXL_ErrorHandler_CheckAll(void) {
    // 모든 모터 (1~MAX_MOTOR_COUNT)에 대해 개별적으로 체크
    for(uint8_t motorID = 1; motorID <= MAX_MOTOR_COUNT; motorID++) {
        // 해당 모터의 마지막 모션 수신 후 ERROR_CHECK_INTERVAL_MS가 지났는지 확인
        if(t_rxLastRxMotion[motorID].tickCheck(ERROR_CHECK_INTERVAL_MS)) {
            DXL_ErrorHandler_CheckMotor(motorID);
            t_rxLastRxMotion[motorID].tickUpdate();
        }
    }
}

/**
 * @brief 모터가 모션을 수신했을 때 타이머 업데이트
 */
void DXL_ErrorHandler_UpdateMotionTimer(uint8_t motorID) {
    if(motorID > 0 && motorID <= MAX_MOTOR_COUNT) {
        t_rxLastRxMotion[motorID].tickUpdate();
    }
}

/* Private 함수 구현 ----------------------------------------------------------*/

/**
 * @brief 개별 에러 전송 처리
 */
static void sendHardwareErrorMessage(uint8_t gID, uint8_t motorID, 
                                     const ErrorDefinition_t* errorDef) {
    if(motorID > MAX_MOTOR_COUNT) return;
    
    // 에러 전송 횟수 제한 체크
    if(errorSendCount[motorID][errorDef->bitIndex] < ERROR_SEND_LIMIT) {
        BypassPacket_TypeDef msg = {0,};
        msg.gid = gID;
        msg.sid = motorID;
        msg.cmd = MRS_TX_HARDWARE_ERROR_EVENT;
        strcpy((char*)msg.data, errorDef->errorMsg);
        osMessageQueuePut(txQueueHandle, &msg, 0U, 0U);
        errorSendCount[motorID][errorDef->bitIndex]++;
    }
}

/**
 * @brief 모터가 에러 체크 가능한 상태인지 확인
 */
static bool isMotorReadyForErrorCheck(Motor::Status status) {
    return (status != Motor::Status_None && 
            status != Motor::Status_InitError && 
            status == Motor::Status_Run);
}

/**
 * @brief Group ID 읽기 (GPIO 핀에서)
 */
static uint8_t readGroupID(void) {
    uint8_t ret = 0;
    ret += (HAL_GPIO_ReadPin(ID_01_GPIO_Port, ID_01_Pin) == GPIO_PIN_RESET) ? 1 : 0;
    ret += (HAL_GPIO_ReadPin(ID_02_GPIO_Port, ID_02_Pin) == GPIO_PIN_RESET) ? 2 : 0;
    ret += (HAL_GPIO_ReadPin(ID_04_GPIO_Port, ID_04_Pin) == GPIO_PIN_RESET) ? 4 : 0;
    ret += (HAL_GPIO_ReadPin(ID_08_GPIO_Port, ID_08_Pin) == GPIO_PIN_RESET) ? 8 : 0;
    return ret;
}
