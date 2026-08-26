# DXL-DYNAMIXEL 보드 펌웨어

![GNU Toolchain](https://img.shields.io/badge/GNU%20Tools%20for%20STM32%20(11.3.rel1)-fail-red)
![GNU Toolchain](https://img.shields.io/badge/GNU%20Tools%20for%20STM32%20(12.3.rel1)-passing-brightgreen)

![STM32CubeIDE](https://img.shields.io/badge/STM32CubeIDE%2014.0-fail-red)
![STM32CubeIDE](https://img.shields.io/badge/STM32CubeIDE%2016.0-passing-brightgreen)



## 소개

DXL-DYNAMIXEL 보드 펌웨어는 STM32F446RET 마이크로컨트롤러를 사용하여 작성되었습니다.

[GM_MOTION Protocol](https://github.com/KimJunHyung1991/GM_MOTION)과 호환되는 펌웨어입니다.


## 목차

1. [펌웨어 빌드](#1-펌웨어-빌드)
2. [하드웨어 에러 코드](#2-하드웨어-에러-코드)


## 1. 펌웨어 빌드

STM32CubeIDE 를 사용하여 작성되었습니다.

PC에 폴더를 복사하여 사용하세요.

git clone + submodule 복제
```bash
git clone --recursive https://github.com/minimirror1/DXL_V2.git
```

PC로 복제된 폴더의 Component 안에 의존성 라이브러리가(submodule) 있습니다.

만약 Component 안의 라이브러리 폴더 안이 비어있을 경우 (서브모듈이 업데이트 되지않을경우.)
폴더를 비우고 다시 복제하세요.
```bash
git clone https://github.com/minimirror1/DXL_V2.git
```

서브모듈 리스트를 확인하세요.
```bash
git submodule status
```

서브모듈을 하나씩 업데이트 하세요.
```bash
git submodule update --init --recursive
```

## 2. 하드웨어 에러 코드

`DXL_ErrorHandler_CheckAll()` (`Task_DXL/dxl_error_handler.cpp`)에서 사용되는 7글자 에러 코드 목록입니다.

각 모터의 하드웨어 에러 상태는 8비트 플래그로 전송되며, 비트별로 다음과 같은 에러 코드가 할당됩니다:

| 비트 | 7글자 코드 | 원래 풀 스펠링 | 설명 |
|------|-----------|---------------|------|
| Bit 0 (0x01) | `Voltage` | Voltage | 입력 전압이 설정된 동작전압 범위를 벗어난 경우 |
| Bit 2 (0x04) | `HotTemp` | Hot + Temp | 설정된 온도를 벗어난 경우 |
| Bit 3 (0x08) | `Encoder` | Encoder | 모터의 엔코더가 동작하지 않는 경우 |
| Bit 4 (0x10) | `EShock` | E + Shock | 전기적으로 회로가 충격을 받았거나 입력 전력이 부족한 경우 |
| Bit 5 (0x20) | `OvrLoad` | Ovr + Load | 최대 출력으로 제어할 수 없는 하중이 지속적으로 발생한 경우 |
| Bit 6 (0x40) | `CommErr` | Comm + Err | 통신 실패 (3회 연속 실패 시 발생) |
| Bit 7 (0x80) | `NotAsgn` | Not + Asgn | 할당되지 않은 모터 |

### 추가 에러 코드

| 에러 코드 | 원래 풀 스펠링 | 설명 |
|-----------|---------------|------|
| `InitErr` | Init + Err | 모터 초기화 실패 (Status_InitError 발생 시) |

### 에러 감지 주기 (모터별 개별 관리)

- **개별 타이머**: 각 모터별로 독립적인 타이머 관리 (`Tick t_rxLastRxMotion[MAX_MOTOR_COUNT + 1]`, 현재 `MAX_MOTOR_COUNT = 30`)
- **타이머 갱신**: `MRS_RX_MOTION` 수신 시 `DXL_ErrorHandler_UpdateMotionTimer()`로 해당 모터의 타이머만 갱신
- **100ms 간격 체크**: 타이머가 마지막으로 갱신된 후 100ms가 지나면 해당 모터를 체크하고 타이머를 다시 갱신
- **체크 대상 조건**: `Status_Run` 상태인 모터만 실제로 에러를 읽음 (`isMotorReadyForErrorCheck()`)
- 각 에러는 `MRS_TX_HARDWARE_ERROR_EVENT` 명령으로 상위 시스템에 전송
- **중복 전송 방지**: 각 에러별로 최대 3번까지만 전송하여 CAN 네트워크 부하 감소
- **자동 복구**: 에러가 해결되면 전송 카운터가 자동으로 리셋되어 재발 시 다시 전송

#### 동작 예시
```
모터 1: Status_Run  → 100ms 간격으로 에러 체크
모터 2: Status_Run  → 100ms 간격으로 에러 체크
모터 3: Status_PreRun (초기 위치 동기화 미완료) → 에러 체크 안함
모터 4: 미연결 (dxlObjects[4] == nullptr)       → 에러 체크 안함
...
```

`DXL_ErrorHandler_Init()`이 모든 모터의 타이머를 한 번에 초기화하므로, 100ms 주기 자체는
모션 수신 여부와 무관하게 전 모터에 대해 돌아갑니다. 실제로 에러를 읽을지 결정하는 것은
`Status_Run` 조건이며, 모션 타이머는 모션을 받은 모터의 체크 시점을 뒤로 미루는 역할을 합니다.

### 통신 실패 안정성 강화

- **통신 실패 판정**: 3회 연속 통신 실패 시에만 `COMM` 에러로 판정
- **통신 복구**: 3회 미만에서 통신이 성공하면 실패 카운터가 0으로 리셋
- **일시적 노이즈 무시**: 일회성 통신 장애는 에러로 처리하지 않아 안정성 향상



