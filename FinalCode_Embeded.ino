#define F_CPU 16000000UL // CPU 클럭을 16MHz로 정의

#include <avr/io.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#define WHITE_LED PIN5_bm // PA5
#define RED_LED PIN4_bm // PA4
#define BUZZER_PIN PIN7_bm // PD7

#define THRESHOLD 1000 // 센서 값 임계값 

#define _PORTA (*(volatile unsigned char*) 0x0400)
#define _PORTA_DIR (*(volatile unsigned char*) (0x0400 + 0x00))
#define _PORTA_OUT (*(volatile unsigned char*) (0x0400 + 0x04))
#define _PORTC (*(volatile unsigned char*) 0x0440)
#define _PORTC_DIR (*(volatile unsigned char*) (0x0440 + 0x00))
#define _PORTC_OUT (*(volatile unsigned char*) (0x0440 + 0x04))
#define _PORTC_PIN2CTRL (*(volatile unsigned char*) (0x0440 + 0x12))
#define _PORTC_PIN2CTRL_PULLUPEN_bm ((unsigned char) 0b00001000)
#define _PORTC_PIN3CTRL (*(volatile unsigned char*) (0x0440 + 0x13))
#define _PORTC_PIN3CTRL_PULLUPEN_bm ((unsigned char) 0b00001000)
#define _PORTF (*(volatile unsigned char*) 0x04A0)
#define _PORTF_DIR (*(volatile unsigned char*) (0x04A0 + 0x00))
#define _PORTF_OUT (*(volatile unsigned char*) (0x04A0 + 0x04))
#define _PORTMUX (*(volatile unsigned char*) 0x05E0)
#define _PORTMUX_TWISPIROUTEA (*(volatile unsigned char*) (0x05E0 + 0x03))
#define _PORTMUX_TWI0_ALT2_bm ((unsigned char) 0b00100000)
#define _PORTMUX_TWI0_ALT3_bm ((unsigned char) 0b01000000) // 추가된 부분
#define _PORTMUX_TCBROUTEA (*(volatile unsigned char*) (0x05E0 + 0x05))
#define _PORTMUX_TCB0_ALT1_gc ((unsigned char) 0b00000001)
#define _PORTMUX_TCB1_ALT1_gc ((unsigned char) 0b00000010)

#define _TWI0 (*(volatile unsigned char*) 0x08A0)
#define _TWI0_MCTRLA (*(volatile unsigned char*) (0x08A0 + 0x03))
#define _TWI_ENABLE_bm ((unsigned char) 0b00000001)
#define _TWI0_MCTRLB (*(volatile unsigned char*) (0x08A0 + 0x04))
#define _TWI_ACKACT_NACK_gc ((unsigned char) 0b00000100)
#define _TWI_MCMD_RECVTRANS_gc ((unsigned char) 0b00000010)
#define _TWI_MCMD_STOP_gc ((unsigned char) 0b00000011)
#define _TWI0_MSTATUS (*(volatile unsigned char*) (0x08A0 + 0x05))
#define _TWI_RIF_bm ((unsigned char) 0b10000000)
#define _TWI_WIF_bm ((unsigned char) 0b01000000)
#define _TWI_RXACK_bm ((unsigned char) 0b00010000)
#define _TWI_ARBLOST_bm ((unsigned char) 0b00001000)
#define _TWI_BUSERR_bm ((unsigned char) 0b00000100)
#define _TWI_BUSSTATE_IDLE_gc ((unsigned char) 0b00000001)
#define _TWI0_MBAUD (*(volatile unsigned char*) (0x08A0 + 0x06))
#define _TWI0_MADDR (*(volatile unsigned char*) (0x08A0 + 0x07))
#define _TWI0_MDATA (*(volatile unsigned char*) (0x08A0 + 0x08))

#define _TWI_READ true
#define _TWI_WRITE false

#define MPU6050 0x68
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_CLOCK_PLL_XGYRO_bm 0b00000001

#define PI 3.141592
#define ALPHA 0.6
#define DT 0.2534

void _twi_init();
bool _twi_start(unsigned char device, bool read);
void _twi_stop();
bool _twi_read(unsigned char* data, bool last);
bool _twi_write(unsigned char data);

void mpu6050_init();
void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz);

short mpu6050_offsets[6];
float angle_gx = 0, angle_gy = 0, angle_gz = 0;
float angle_x = 0, angle_y = 0, angle_z = 0;

void initADC() {
    // ADC 초기화
    VREF.CTRLA = VREF_ADC0REFSEL_4V34_gc; // VDD 참조 전압 사용
    VREF.CTRLB = VREF_ADC0REFEN_bm; // ADC0 참조 전압 활성화
    ADC0.CTRLC = ADC_REFSEL_INTREF_gc | ADC_PRESC_DIV4_gc; // 내부 참조 전압 및 분주 설정
    ADC0.CTRLA = ADC_ENABLE_bm | ADC_RESSEL_10BIT_gc; // ADC 활성화 및 10비트 해상도 설정
    PORTD.DIRCLR = PIN1_bm | PIN3_bm; // PD1, PD3 입력 설정
    PORTD.PIN1CTRL |= PORT_PULLUPEN_bm; // 풀업 저항 활성화
    PORTD.PIN3CTRL |= PORT_PULLUPEN_bm;
}

uint16_t readADC(uint8_t sensor) {
    if (sensor == 0) {
        ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc; // PD1
    } else {
        ADC0.MUXPOS = ADC_MUXPOS_AIN3_gc; // PD3
    }
    ADC0.COMMAND = ADC_STCONV_bm; // 변환 시작
    while (!(ADC0.INTFLAGS & ADC_RESRDY_bm)); // 변환 완료 대기
    ADC0.INTFLAGS = ADC_RESRDY_bm; // 플래그 클리어
    return ADC0.RES;
}

void my_delay_ms(unsigned int ms) {
    while (ms--) {
        for (unsigned int i = 0; i < 1000; i++) {
            __asm__("nop");
        }
    }
}

void my_delay_us(unsigned int us) {
    while (us--) {
        for (unsigned int i = 0; i < 16; i++) {
            __asm__("nop");
        }
    }
}

void buzz(uint16_t frequency, uint16_t duration) {
    uint16_t delay = 1000000 / frequency;
    uint16_t cycles = frequency * duration / 1000;
    for (uint16_t i = 0; i < cycles; i++) {
        VPORTD.OUT |= BUZZER_PIN; // 부저 켜기
        my_delay_us(duration); // 짧은 대기 시간
        VPORTD.OUT &= ~BUZZER_PIN; // 부저 끄기
        my_delay_us(duration); // 짧은 대기 시간
    }
}

void buzzShort() {
    buzz(1000, 100); // 100ms 동안 1kHz
}

void buzzStart() {
    for (int i = 0; i < 2; ++i) {
        buzz(2000, 50); // 50ms 동안 2kHz, 2번 울림
        _delay_ms(100);
    }
}

void buzzCountdown() {
    buzz(1500, 50); // 50ms 동안 1.5kHz, 카운트다운 시
}

void buzzVariableDelay(int duration, uint16_t delay_time) {
    for (int i = 0; i < duration; ++i) {
        VPORTD.OUT |= BUZZER_PIN; // 부저 켜기
        my_delay_us(delay_time); // 지정된 대기 시간
        VPORTD.OUT &= ~BUZZER_PIN; // 부저 끄기
        my_delay_us(delay_time); // 지정된 대기 시간
    }
}

void buzzVictoryMelody() {
    for (int i = 0; i < 5; ++i) {
        buzzVariableDelay(100, 10);
        my_delay_ms(100); // 적절한 지연 시간을 설정
        buzzVariableDelay(100, 80);
        my_delay_ms(100); // 적절한 지연 시간을 설정
        buzzVariableDelay(100, 100);
        my_delay_ms(100); // 적절한 지연 시간을 설정 
        buzzVariableDelay(100, 50);
        my_delay_ms(100); // 적절한 지연 시간을 설정          
        buzzVariableDelay(100, 10);
        my_delay_ms(100); // 적절한 지연 시간을 설정     
    }
}

void buzzDefeatMelody() {
    for (int i = 0; i < 2; ++i) {
        buzzVariableDelay(40, 300);
        my_delay_ms(100); // 적절한 지연 시간을 설정
        buzzVariableDelay(40, 800);
        my_delay_ms(100); // 적절한 지연 시간을 설정
        buzzVariableDelay(40, 1000);
        my_delay_ms(100); // 적절한 지연 시간을 설정 
        buzzVariableDelay(40, 500);
        my_delay_ms(100); // 적절한 지연 시간을 설정          
        buzzVariableDelay(40, 700);
        my_delay_ms(100); // 적절한 지연 시간을 설정   
    }
}

void displayResult(int result) {
    switch (result) {
        case 0: // 승리
            buzzVictoryMelody(); // 승리 멜로디
            VPORTA.OUT |= WHITE_LED;
            VPORTA.OUT &= ~RED_LED;
            _delay_ms(2000);
            VPORTA.OUT &= ~WHITE_LED;
            break;
        case 1: // 패배
            buzzDefeatMelody(); // 패배 멜로디
            VPORTA.OUT &= ~WHITE_LED;
            VPORTA.OUT |= RED_LED;
            _delay_ms(2000);
            VPORTA.OUT &= ~RED_LED;
            break;
        case 2: // 무승부
            for (int i = 0; i < 5; ++i) {
                buzzShort(); // 0.2초 울림
                _delay_ms(100);
            }
            VPORTA.OUT |= WHITE_LED | RED_LED;
            _delay_ms(2000);
            VPORTA.OUT &= ~(WHITE_LED | RED_LED);
            break;
    }
}

int checkWin(int user, int com) {
    if (user == com) return 2;
    else if ((user == 0 && com == 1) || (user == 1 && com == 2) || (user == 2 && com == 0)) return 0;
    else return 1;
}     

const char* getHandName(int hand) {
    switch (hand) {
        case 0: return "묵";
        case 1: return "찌";
        case 2: return "빠";
        default: return "알 수 없음";
    }
}

int gameStartCountdown() {
    Serial1.println("게임 시작하겠습니다. 3, 2, 1에 맞춰 값을 입력해주세요.");
    int userInput = -1;
    for (int i = 3; i > 0; i--) {
        Serial1.println(i);
        if (i == 1) {
            buzzVariableDelay(100, 50);
        } else {
            buzzVariableDelay(400, 30);
        }
        _delay_ms(1000); // 1초 대기
        for (int j = 0; j < 10; j++) {
            if (userInput == -1) {  // 처음 입력된 값을 저장
                userInput = getUserInput();
            }
            _delay_ms(100);   
        }
    }
    return userInput; // 카운트다운 후 사용자 입력 반환
}

int getUserInput() {
    uint16_t currentADCValue_a = readADC(0);
    uint16_t currentADCValue_b = readADC(1);

    Serial1.print("Current Sensor a: ");
    Serial1.print(currentADCValue_a);
    Serial1.print(" Current Sensor b: ");
    Serial1.println(currentADCValue_b);

    if (currentADCValue_a >= 600) {
        return 0; // 묵
    } else if ((currentADCValue_a >= 340 && currentADCValue_a <= 530) && (currentADCValue_b >= 340 && currentADCValue_b <= 530)) {
        return 1; // 찌
    } else {
        return 2; // 빠
    }
}


void setup() {
    PORTA.DIRSET = WHITE_LED | RED_LED; // PA4, PA5를 출력으로 설정
    PORTD.DIRSET = BUZZER_PIN; // PD7을 출력으로 설정
    mpu6050_init();

    // 시리얼 통신 초기화
    USART0.BAUD = 103; // 9600 baud rate 설정 (16MHz 기준)
    USART0.CTRLB |= USART_TXEN_bm | USART_RXEN_bm; // TX 및 RX 활성화
    Serial1.begin(9600); // 시리얼 통신 초기화

    // 시리얼 통신 초기화 확인 메시지
    Serial1.println("Serial communication initialized.");

    // ADC 초기화 및 랜덤 시드 설정
    initADC();
    // 복잡한 시드 설정: ADC값, 타이머값 및 자이로 데이터를 결합
    uint16_t adcValue = readADC(0);
    uint32_t seed = adcValue ^ millis() ^ (readADC(1) << 16) ^ (mpu6050_offsets[0] << 8);
    srand(seed); // 복잡한 시드 설정

    // 부저 초기화 알림 (승리 시와 같은 소리)
    buzzStart(); // 게임 시작 부저 소리

    // 게임 시작을 위해 자이로 센서로 동작 인식
    Serial1.println("위아래로 두 번 흔들어 게임을 시작하세요.");
    while (!detectGesture()); // 제스처가 감지될 때까지 대기
}

void _twi_init() {
    _PORTMUX_TWISPIROUTEA |= _PORTMUX_TWI0_ALT3_bm; // PA2: SDA, PA3: SCL
    _PORTA_DIR &= ~((1 << 2) | (1 << 3)); // PA2, PA3 입력으로 설정
    _PORTA_OUT |= (1 << 2) | (1 << 3); // PA2, PA3 PULLUP 활성화

    unsigned int frequency = 400000; // 400kHz
    unsigned short t_rise = 300; // 300ns
    unsigned int baud = (F_CPU / frequency - F_CPU / 1000 / 1000 * t_rise / 1000 - 10) / 2;
    _TWI0_MBAUD = (unsigned char) baud;

    _TWI0_MCTRLA = _TWI_ENABLE_bm;
    _TWI0_MSTATUS = _TWI_BUSSTATE_IDLE_gc;
}

bool _twi_start(unsigned char device, bool read) {
    _TWI0_MADDR = device << 1 | (read ? 1 : 0);

    while (!(_TWI0_MSTATUS & (_TWI_WIF_bm | _TWI_RIF_bm)));

    if (_TWI0_MSTATUS & _TWI_ARBLOST_bm) {
        while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc));
        return false;
    }
    if (_TWI0_MSTATUS & _TWI_RXACK_bm) {
        _TWI0_MCTRLB |= _TWI_MCMD_STOP_gc;
        while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc));
        return false;
    }
    return true;
}

void _twi_stop() {
    _TWI0_MCTRLB |= _TWI_MCMD_STOP_gc;
    while (!(_TWI0_MSTATUS & _TWI_BUSSTATE_IDLE_gc));
}

bool _twi_read(unsigned char* data, bool last) {
    while (!(_TWI0_MSTATUS & _TWI_RIF_bm));

    *data = _TWI0_MDATA;
    if (last) {
        _TWI0_MCTRLB = _TWI_ACKACT_NACK_gc; // NACK 전송
    } else {
        _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc; // ACK 전송
    }
    return true;
}

bool _twi_write(unsigned char data) {
    _TWI0_MCTRLB = _TWI_MCMD_RECVTRANS_gc;
    _TWI0_MDATA = data;

    while (!(_TWI0_MSTATUS & _TWI_WIF_bm));
    if (_TWI0_MSTATUS & (_TWI_ARBLOST_bm | _TWI_BUSERR_bm)) {
        return false; // 오류 발생
    }
    return !(_TWI0_MSTATUS & _TWI_RXACK_bm); // ACK 확인
}

void mpu6050_init() {
    _twi_init();

    _twi_start(MPU6050, _TWI_WRITE);
    _twi_write(MPU6050_PWR_MGMT_1); // 레지스터 주소
    _twi_write(0); // 레지스터 값
    _twi_stop();

    // 센서 오프셋 초기화
    short sum[6] = { 0 };
    for (int i = 0; i < 10; i += 1) {
        short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
        mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);
        sum[0] += raw_ax;
        sum[1] += raw_ay;
        sum[2] += raw_az;
        sum[3] += raw_gx;
        sum[4] += raw_gy;
        sum[5] += raw_gz;
    }

    for (int i = 0; i < 6; i += 1) {
        mpu6050_offsets[i] = sum[i] / 10;
    }
}

void mpu6050_fetch(short* raw_ax, short* raw_ay, short* raw_az, short* raw_gx, short* raw_gy, short* raw_gz) {
    unsigned char buf[14];
    _twi_start(MPU6050, _TWI_WRITE);
    _twi_write(MPU6050_ACCEL_XOUT_H); // 레지스터 주소
    _twi_stop();
    _twi_start(MPU6050, _TWI_READ);
    for (int i = 0; i < 14; i += 1) {
        _twi_read(&buf[i], (i == 13)); // 레지스터 값
    }
    _twi_stop();
    *raw_ax = (buf[0] << 8) | buf[1];
    *raw_ay = (buf[2] << 8) | buf[3];
    *raw_az = (buf[4] << 8) | buf[5];
    *raw_gx = (buf[8] << 8) | buf[9];
    *raw_gy = (buf[10] << 8) | buf[11];
    *raw_gz = (buf[12] << 8) | buf[13];
}

bool detectGesture() {
    static float prev_gz = 0;
    static int shake_count = 0;

    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    float gz = ((float)(raw_gz - mpu6050_offsets[5])) / 131;

    if (abs(gz - prev_gz) > 20) { // 특정 값 이상 변화가 있을 경우
        shake_count++;
        if (shake_count >= 2) { // 두 번 흔듦을 인식 (위아래 2번씩 총 4번)
            shake_count = 0;
            return true;
        }
    }
    prev_gz = gz;
    _delay_ms(100); // 디바운스 딜레이
    return false;
}

void setupServo() {
    // 모든 인터럽트 사용금지

    // 시스템 클럭 설정 (기본 클럭 사용)
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL_MCLKCTRLB = CLKCTRL_PDIV_64X_gc | CLKCTRL_PEN_bm;
    CLKCTRL_MCLKCTRLA = CLKCTRL_CLKSEL_OSCULP32K_gc;
    while (CLKCTRL_MCLKSTATUS & CLKCTRL_SOSC_bm);    // 시스템 클럭 설정 (기본 클럭 사용)

    // PF4와 PF5를 출력으로 설정
    PORTF.DIR |= PIN4_bm | PIN5_bm;
    PORTF.OUT &= ~(PIN4_bm | PIN5_bm); // PF4와 PF5를 LOW로 설정

    // TCB0 설정 (PF4용)
    TCB0.CCMPL = 0xFF; // PWM 주기 설정 (8비트 모드에서 최대값 255)
    TCB0.CCMPH = 0x07; // 초기 듀티 사이클 설정 (2.7% 듀티 사이클)

    TCB0.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // 클럭 분주를 2로 설정하고 타이머 활성화
    TCB0.CTRLB = TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc; // 비교 출력 활성화, 8비트 PWM 모드 설정

    // TCB1 설정 (PF5용)
    TCB1.CCMPL = 0xFF; // PWM 주기 설정 (8비트 모드에서 최대값 255)
    TCB1.CCMPH = 0x07; // 초기 듀티 사이클 설정 (2.7% 듀티 사이클)

    TCB1.CTRLA = TCB_CLKSEL_CLKDIV2_gc | TCB_ENABLE_bm; // 클럭 분주를 2로 설정하고 타이머 활성화
    TCB1.CTRLB = TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc; // 비교 출력 활성화, 8비트 PWM 모드 설정

    // TCB0 출력을 PF4로 라우팅
    _PORTMUX_TCBROUTEA |= _PORTMUX_TCB0_ALT1_gc;

    // TCB1 출력을 PF5로 라우팅
    _PORTMUX_TCBROUTEA |= _PORTMUX_TCB1_ALT1_gc;
}

void restoreClock() {
    // 시스템 클럭을 원래 상태로 복구 (16MHz 설정)
    CPU_CCP = CCP_IOREG_gc;
    CLKCTRL_MCLKCTRLB = 0; // 분주 비활성화
    CLKCTRL_MCLKCTRLA = CLKCTRL_CLKSEL_OSC20M_gc; // 20MHz 내부 오실레이터 사용
    while (CLKCTRL_MCLKSTATUS & CLKCTRL_SOSC_bm); // 복구 완료 대기
    CLKCTRL_MCLKCTRLB = CLKCTRL_PDIV_2X_gc; // 분주기 2로 설정하여 10MHz로 변경
    CLKCTRL_MCLKCTRLB = 0; // 분주기 비활성화 (기본 클럭 설정으로 16MHz 설정)
}

void setServo1Angle(int angle) {
    // 모터 제어를 위해 시스템 클럭 변경
    setupServo();

    uint16_t pulseWidth;
    // 각도를 펄스 폭으로 변환 (0도에서 180도는 500µs에서 2400µs에 해당)
    pulseWidth = 500 + ((angle * 1900) / 180); // 각도에 따라 펄스 폭을 조정
    TCB0.CCMPH = pulseWidth; // TCB0의 펄스 폭 설정
    _delay_ms(10);
    // 모터 제어 후 클럭 원상 복구
    restoreClock();
}

void setServo2Angle(int angle) {
    // 모터 제어를 위해 시스템 클럭 변경
    setupServo();

    uint16_t pulseWidth;
    // 각도를 펄스 폭으로 변환 (0도에서 180도는 500µs에서 2400µs에 해당)
    pulseWidth = 500 + ((angle * 1900) / 180); // 각도에 따라 펄스 폭을 조정
    TCB1.CCMPH = pulseWidth; // TCB1의 펄스 폭 설정
    _delay_ms(10);

    // 모터 제어 후 클럭 원상 복구
    restoreClock();
}

void muk() {
    setServo1Angle(0);
    setServo2Angle(0);
}

void jji() {
    setServo1Angle(180);
    setServo2Angle(0);
}

void bba() {
    setServo1Angle(180);
    setServo2Angle(180);
}

int getComputerChoice() {
    static int lastChoice = -1;
    int newChoice;

    // 최근 나온 값을 피하기 위해 다른 값을 선택
    do {
        newChoice = rand() % 3;
    } while (newChoice == lastChoice);

    lastChoice = newChoice;
    return newChoice;
}

void loop() {
    static int isSetup = 0;
    static int isFirstTime = 1; // 게임 처음 시작인지 여부를 나타내는 변수

    if (!isSetup) {
        setup();
        isSetup = 1;
    }

    if (isFirstTime) {
        Serial1.println("위아래로 두 번 흔들어 게임을 시작하세요.");
        while (!detectGesture()); // 제스처가 감지될 때까지 대기
        isFirstTime = 0; // 게임 시작 후, 더 이상 제스처를 요구하지 않음
    }

    char buffer[100];
    int user = -1, com, result;
    bool isUserAttacker;

    short raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz;
    mpu6050_fetch(&raw_ax, &raw_ay, &raw_az, &raw_gx, &raw_gy, &raw_gz);

    // 가속도 데이터 각도 계산
    float ax = raw_ax - mpu6050_offsets[0];
    float ay = raw_ay - mpu6050_offsets[1];
    float az = raw_az - mpu6050_offsets[2];
    float angle_ax = atan(ay / sqrt(ax * ax + az * az)) * (180 / PI);
    float angle_ay = atan(ax / sqrt(ay * ay + az * az)) * (180 / PI);
    float angle_az = atan(sqrt(ax * ax + ay * ay) / az) * (180 / PI);

    float gx = ((float) (raw_gx - mpu6050_offsets[3])) / 131;
    float gy = ((float) (raw_gy - mpu6050_offsets[4])) / 131;
    float gz = ((float) (raw_gz - mpu6050_offsets[5])) / 131;

    // 상보 필터 적용
    angle_x = ALPHA * (angle_x + gx * DT) + (1 - ALPHA) * angle_ax;
    angle_y = ALPHA * (angle_y + gy * DT) + (1 - ALPHA) * angle_ay;
    angle_z = ALPHA * (angle_z + gz * DT) + (1 - ALPHA) * angle_az;

    _delay_ms(500);  // 0.5초 대기

    buzzStart(); // 게임 시작 부저 소리
    Serial1.println("가위바위보! 컴퓨터의 손 모양을 선택합니다.");

    // 첫 번째 게임 시작 카운트다운
    user = gameStartCountdown();
    if (user == -1) {
        Serial1.println("3초 안에 입력이 없어 게임을 다시 시작합니다.");
        return;
    }

    com = getComputerChoice();
    snprintf(buffer, sizeof(buffer), "컴퓨터가 선택한 값: %s", getHandName(com));
    Serial1.println(buffer);

    switch (com) {
        case 0: muk(); 
        _delay_ms(20);
        break;
        case 1: jji(); 
        _delay_ms(20);
        break;
        case 2: bba(); 
        _delay_ms(20);
        break; 

    }

    snprintf(buffer, sizeof(buffer), "사용자가 선택한 값: %s", getHandName(user));
    Serial1.println(buffer);

    result = checkWin(user, com);
    if (result == 2) {
        Serial1.println("무승부입니다. 게임을 다시 시작하세요.");
        return;
    } else if (result == 0) {
        Serial1.println("사용자가 이겼습니다! 사용자가 공격자가 됩니다.");
        isUserAttacker = true;
    } else {
        Serial1.println("컴퓨터가 이겼습니다! 컴퓨터가 공격자가 됩니다.");
        isUserAttacker = false;
    }

    while (1) {
        user = gameStartCountdown();
        if (user == -1) {
            Serial1.println("3초 안에 입력이 없어 게임을 다시 시작합니다.");
            continue;
        }

        snprintf(buffer, sizeof(buffer), "사용자가 선택한 값: %s", getHandName(user));
        Serial1.println(buffer);

        com = getComputerChoice();
        snprintf(buffer, sizeof(buffer), "컴퓨터가 선택한 값: %s", getHandName(com));
        Serial1.println(buffer);

        switch (com) {
            case 0: muk(); 
            _delay_ms(20);
            break;
            case 1: jji(); 
            _delay_ms(20);
            break;
            case 2: bba(); 
            _delay_ms(20);
            break;
        }

        result = checkWin(user, com);
        if (result == 2) {
            if (isUserAttacker) {
                Serial1.println("사용자가 이겼습니다!");
                displayResult(0);
                break;
            } else {
                Serial1.println("컴퓨터가 이겼습니다!");
                displayResult(1);
                break;
            }
        } else if (result == 0) {
            if (isUserAttacker) {
                Serial1.println("사용자가 공격자가 됩니다.");
            } else {
                Serial1.println("사용자가 공격자가 됩니다.");
                isUserAttacker = true;
            }
        } else {
            if (isUserAttacker) {
                Serial1.println("컴퓨터가 공격자가 됩니다.");
                isUserAttacker = false;
            } else {
                Serial1.println("컴퓨터가 공격자가 됩니다.");
            }
        }
        _delay_ms(100); // 디바운싱을 위한 지연
    }
}

void setup_and_loop() {
    setup();

    while (1) {
        loop();
    }
}
