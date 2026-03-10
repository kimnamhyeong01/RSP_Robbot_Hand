<<<<<<< HEAD
# RSP Robot Hand — 가위바위보 로봇 손

AVR 마이크로컨트롤러 기반의 **묵찌빠 게임 로봇 손** 임베디드 프로젝트입니다.
사용자가 플렉스 센서로 손 모양을 입력하면, 로봇 손(서보모터)이 컴퓨터의 선택을 물리적으로 표현하며 게임을 진행합니다.

---

## 시연 결과물

![임베디드 최종 결과물](./임베디드%20최종%20결과물.jpg)

---

## 주요 기능

| 기능 | 설명 |
|------|------|
| **손 모양 인식** | ADC 플렉스 센서 2개(PD1, PD3)로 묵·찌·빠 판별 |
| **로봇 손 구동** | 서보모터 2개(PF4, PF5)로 묵·찌·빠 3가지 손 모양 구현 |
| **게임 시작 인식** | MPU6050 자이로 센서 — 위아래 2회 흔들기 제스처로 게임 시작 |
| **자세 보정** | 가속도 + 자이로 데이터를 상보 필터(α=0.6)로 융합하여 각도 계산 |
| **결과 피드백** | 승리(흰색 LED + 승리 멜로디) / 패배(빨간 LED + 패배 멜로디) / 무승부(양쪽 LED) |
| **카운트다운** | 3·2·1 카운트다운과 함께 부저 알림음 출력 |
| **시리얼 통신** | USART(9600 baud)로 게임 진행 상황 실시간 출력 |

---

## 게임 규칙 (묵찌빠)

1. **1라운드 (가위바위보)**: 사용자와 컴퓨터가 동시에 손 모양을 선택
   - 이긴 쪽이 **공격자**가 됨
   - 무승부이면 재시작

2. **2라운드 이후 (묵찌빠)**: 공격자와 수비자가 계속 대결
   - **공격자가 같은 손 모양을 내면** → 공격자 최종 승리
   - 공격자가 이기면 → 공격자 유지 (상대방은 공격자로 전환 가능)
   - 수비자가 이기면 → 수비자가 공격자로 전환

---

## 하드웨어 구성

```
MCU         : AVR (ATmega 계열, 16MHz)
자이로/가속도 : MPU6050 (I2C, TWI0, PA2=SDA / PA3=SCL)
서보모터 1   : TCB0 PWM → PF4 (묵·찌·빠 손 모양)
서보모터 2   : TCB1 PWM → PF5 (묵·찌·빠 손 모양)
플렉스 센서A : ADC0 AIN1 (PD1)
플렉스 센서B : ADC0 AIN3 (PD3)
흰색 LED    : PA5
빨간 LED    : PA4
부저        : PD7
시리얼 출력  : Serial1 (USART, 9600 baud)
```

### 손 모양별 서보 각도

| 손 모양 | 서보1 각도 | 서보2 각도 |
|---------|-----------|-----------|
| 묵 (바위) | 0° | 0° |
| 찌 (가위) | 180° | 0° |
| 빠 (보)   | 180° | 180° |

---

## 센서 입력 판별 로직

```
플렉스 센서A 값 >= 600              → 묵 (바위)
340 <= 센서A <= 530 & 340 <= 센서B <= 530  → 찌 (가위)
그 외                               → 빠 (보)
```

---

## 소프트웨어 구조

```
FinalCode_Embeded.ino
├── setup()              : 핀 초기화, MPU6050 초기화, 랜덤 시드 설정
├── loop()               : 메인 게임 루프 (묵찌빠 진행)
├── gameStartCountdown() : 3·2·1 카운트다운 및 사용자 입력 수집
├── getUserInput()       : ADC 센서 읽어 손 모양 판별
├── getComputerChoice()  : 랜덤 컴퓨터 선택 (직전 값 제외)
├── checkWin()           : 승패 판정 (0=승, 1=패, 2=무)
├── displayResult()      : LED + 부저로 결과 표시
├── detectGesture()      : MPU6050 자이로로 흔들기 제스처 감지
├── mpu6050_*()          : TWI(I2C) 드라이버 및 MPU6050 초기화/데이터 읽기
├── setServo1/2Angle()   : TCB PWM으로 서보모터 각도 제어
├── muk() / jji() / bba(): 묵·찌·빠 손 모양 함수
└── buzz*()              : 부저 멜로디 (승리/패배/카운트다운)
```

### 상보 필터 (Complementary Filter)

자이로 드리프트와 가속도계 노이즈를 보완하기 위해 상보 필터를 적용합니다.

```
angle = α × (angle + gyro × dt) + (1 - α) × accel_angle
α = 0.6,  dt = 0.2534s
```

---

## 빌드 및 업로드

1. Arduino IDE 또는 AVR 호환 툴체인을 준비합니다.
2. `FinalCode_Embeded.ino`를 열고 대상 보드를 선택합니다.
3. 컴파일 후 MCU에 업로드합니다.
4. 시리얼 모니터(9600 baud)를 열면 게임 상태를 확인할 수 있습니다.

---

## 동작 흐름

```
전원 ON
  └─> MPU6050 초기화 및 오프셋 캘리브레이션
  └─> 부저 시작음 출력
  └─> "위아래로 두 번 흔들어 게임 시작" 대기
        └─> 제스처 감지
              └─> 3·2·1 카운트다운
              └─> 플렉스 센서로 사용자 입력 수집
              └─> 컴퓨터 랜덤 선택 + 서보모터 손 모양 구현
              └─> 승패 판정 → 공격자 결정
                    └─> 묵찌빠 라운드 반복
                          └─> 최종 승패 → LED + 멜로디 출력
                                └─> 게임 재시작
```

---

## 파일 구성

```
RSP_Robbot_Hand-main/
├── FinalCode_Embeded.ino   # 메인 임베디드 소스코드
├── 발표자료.md              # 프로젝트 발표 슬라이드
├── 임베디드 최종 결과물.jpg  # 완성 하드웨어 사진
└── README.md               # 프로젝트 설명 (이 파일)
```
=======
# RSP_Robot_Hand

<img width="1854" height="1358" alt="image" src="https://github.com/user-attachments/assets/3063ab18-9510-4e0d-91e8-79e95f52ee8d" />
<img width="1872" height="1379" alt="image" src="https://github.com/user-attachments/assets/e686cabb-c10e-48f6-a9ac-3c90d3ca2007" />
<img width="1838" height="1366" alt="image" src="https://github.com/user-attachments/assets/9c58f111-55f3-4fdf-b71b-92762468d9ff" />
<img width="1845" height="1384" alt="image" src="https://github.com/user-attachments/assets/0853ab78-270b-4cd0-80ad-31e66ce86a8f" />
<img width="1856" height="1352" alt="image" src="https://github.com/user-attachments/assets/d41777ab-9d00-4505-8275-e7a36000a2b2" />
<img width="1854" height="1386" alt="image" src="https://github.com/user-attachments/assets/1c2856df-8a6e-4c28-b9f6-a4bcd266373d" />
<img width="1860" height="1371" alt="image" src="https://github.com/user-attachments/assets/26b167d4-982e-4bb9-a179-24036d9cb5e4" />
<img width="1834" height="1374" alt="image" src="https://github.com/user-attachments/assets/fe4fd61d-d819-4906-946e-899acfb503b2" />
<img width="1847" height="1371" alt="image" src="https://github.com/user-attachments/assets/bddcf347-25cd-4a39-8f5a-e8c1b5a88062" />
<img width="1843" height="1364" alt="image" src="https://github.com/user-attachments/assets/c5764456-975e-4e9a-b81b-6b02ae940529" />
>>>>>>> e9fb9936b5531600855a2f9dedb3969a09938685
