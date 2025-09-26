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

`allHardwareErrorCheck()` 함수에서 사용되는 4글자 에러 코드 목록입니다.

각 모터의 하드웨어 에러 상태는 8비트 플래그로 전송되며, 비트별로 다음과 같은 에러 코드가 할당됩니다:

| 비트 | 4글자 코드 | 원래 풀 스펠링 | 설명 |
|------|-----------|---------------|------|
| Bit 0 (0x01) | `VOLT` | Voltage | 입력 전압이 설정된 동작전압 범위를 벗어난 경우 |
| Bit 2 (0x04) | `HEAT` | Overheating | 설정된 온도를 벗어난 경우 |
| Bit 3 (0x08) | `ENCR` | Encoder | 모터의 엔코더가 동작하지 않는 경우 |
| Bit 4 (0x10) | `SHCK` | Electrical Shock | 전기적으로 회로가 충격을 받았거나 입력 전력이 부족한 경우 |
| Bit 5 (0x20) | `LOAD` | Overload | 최대 출력으로 제어할 수 없는 하중이 지속적으로 발생한 경우 |
| Bit 6 (0x40) | `COMM` | Communication | 통신 실패 |
| Bit 7 (0x80) | `NASS` | Not Assigned | 할당되지 않은 모터 |

### 에러 감지 주기

- 100ms마다 Status_Run 상태의 모터들을 순환하며 하드웨어 에러 체크
- 연결되지 않은 모터는 자동으로 건너뛰어 효율성 향상
- 각 에러는 `MRS_TX_HARDWARE_ERROR_EVENT` 명령으로 상위 시스템에 전송



