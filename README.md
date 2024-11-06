# DXL-DYNAMIXEL 보드 펌웨어

## 목차

1. [펌웨어 빌드](#1-펌웨어-빌드)


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



