/* =====================================================================
   AA-12V 탐사로버  —  Arduino Nano 메인 제어기
   PUMPKIN IDEA FACTORY MAKERSPACE / SRC 교육용
   ---------------------------------------------------------------------
   하드웨어
     - AA 8개 배터리홀더(알카라인 12V / 니켈수소 9.6V)
     - Arduino Nano (ATmega328P)
     - L298N 모터드라이버 1개 : 왼쪽 TT모터 2개 병렬(OUT1/2), 오른쪽 2개 병렬(OUT3/4)
     - MPU6050(자이로/가속도), BME280(온도/습도/기압)  → I2C (A4 SDA, A5 SCL)
     - HM-10 BLE 모듈 → SoftwareSerial (D2=RX, D3=TX), 9600bps
     - ESP32-CAM 은 L298N 5V/GND 에서 전원만 받음 (데이터선 연결 없음)

   핀 배치
     D2  ← HM-10 TXD          D3  → HM-10 RXD (1k/2k 분압 권장)
     D5  → L298N ENA (왼쪽 PWM)  D7 → IN1   D8 → IN2
     D6  → L298N ENB (오른쪽 PWM) D9 → IN3   D10 → IN4
     A4  ↔ SDA (MPU6050, BME280)  A5 ↔ SCL
     A0  ← 배터리 전압 분압(선택, USE_BATT 1)  D11/D12 초음파(선택, USE_SONAR 1)
     D13 → 상태 LED

   BLE 명령 (GCS → 로버, 한 줄 끝 '\n')
     M,l,r          수동주행 PWM(-255~255). 0.7초 안에 다시 안 오면 자동 정지
     S              즉시 정지(모든 동작 취소)
     T,dir,ms,pwm   시간 주행. dir = F/B/L/R (L/R은 제자리 회전)
     D,cm           직진 거리 주행(음수 = 후진), 자이로로 방향 유지
     A,deg          제자리 회전(+ 왼쪽/반시계, - 오른쪽/시계)
     R,cm,deg       회전반경 cm 로 deg 만큼 원호 주행 (0 이면 제자리 회전)
     G,trackmm,basemm  바퀴 접지점 간 거리 입력 → 회전 기하 계산, EEPROM 저장
     X              스키드 계수(chi) 자동 보정 — 2초 제자리 회전하며 자이로로 측정
     V,pwm,cm,ms    속도 보정 결과 입력 (pwm 으로 ms 동안 간 거리 cm)
     P,pwm          기본 주행 PWM          W,dead,max  PWM 데드밴드/최대값
     J,g100         충돌감지 임계값(×100 g, 0=끔)
     Y,sign         요(yaw) 방향 부호(1/-1)  Z  위치 원점 재설정   C  자이로 재보정
     Q              설정값·센서상태 요청  H  하트비트(응답 h)
     U              I2C 센서 다시 찾기 (배선을 고친 뒤 재부팅 없이)

   텔레메트리 (로버 → GCS)
     I,roll10,pitch10,yaw10,x10,y10,mode,sonar      5Hz  (각도·위치 ×10)
     E,temp100,hum10,press10,alt10,vib100,batt100   1Hz
     K,cmd  명령 접수   F,cmd 동작 완료   B,x10,y10 충돌/장애물 이벤트
     G,track,base,chi1000,beff,sweep,vcal10,pwmcal,dead,max,base,yawsign  기하/설정
     U,mpu,chip,bme,주소...  I2C 센서 상태 (16진수, 0=없음)

   ■ 2026-11 수정
     1) 오도메트리(거리·좌표)를 MPU6050 검사 밖으로 옮김
        — 자이로가 없거나 읽기에 실패해도 D 명령의 거리가 쌓이고 지도가 그려집니다.
     2) I2C 버스 자동 복구 추가 (i2cRecover)
        — 센서 한쪽이 버스를 LOW 로 물고 멎는 "하나만 인식되는" 증상을 스스로 풉니다.
        — MPU 와 BME 를 각각 따로 복구하고, 반복되면 50kHz 로 낮춥니다.
     3) BME280 보정값이 0 이면 '센서 없음'으로 처리 (엉뚱한 온도·기압 방지)
     4) 못 찾은 센서를 5초마다 다시 탐색 (배선을 고치면 재부팅 없이 살아남)
     5) Z 명령이 travelCm·holdYaw·targetYaw 까지 초기화
     6) C 명령 뒤 dt 기준 시각(lastImu) 재설정
   ===================================================================== */
#include <Wire.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#include <math.h>

// ---------------- 팀 설정 ----------------
#define TEAM_ID 1          // ESP32-CAM 의 TEAM_ID 와 같게. BLE 이름이 PUMPKIN-01 처럼 바뀝니다
#define HM10_SET_NAME 1    // 1 = 부팅 때 HM-10 이름을 자동으로 바꿈 (한 번 바뀌면 0 으로 해도 유지)

// ---------------- 선택 기능 ----------------
#define USE_BATT   0      // A0 배터리 분압(10k/4.7k) 연결 시 1
#define USE_SONAR  0      // HC-SR04 추가 시 1 (D11 TRIG, D12 ECHO)
#define BATT_RATIO 3.128  // (10k+4.7k)/4.7k

// ---------------- 핀 ----------------
const uint8_t PIN_BT_RX = 2, PIN_BT_TX = 3;
const uint8_t ENA = 5, IN1 = 7, IN2 = 8;     // 왼쪽
const uint8_t ENB = 6, IN3 = 9, IN4 = 10;    // 오른쪽
const uint8_t PIN_LED = 13, PIN_TRIG = 11, PIN_ECHO = 12;

SoftwareSerial bt(PIN_BT_RX, PIN_BT_TX);

// ---------------- 동작 모드 (함수보다 먼저 선언) ----------------
// Arduino IDE 는 함수 선언(프로토타입)을 첫 함수 앞에 자동으로 넣습니다.
// 함수 인자에 쓰는 자료형은 그보다 위에 있어야 컴파일됩니다.
enum Mode : uint8_t { IDLE = 0, MANUAL, TIMED, DIST, TURN, ARC, CALIB };
Mode mode = IDLE;

// ---------------- 설정(EEPROM) ----------------
struct Cfg {
  uint16_t magic;
  int16_t trackMm;   // 좌우 바퀴 접지점 간 거리(윤거)
  int16_t baseMm;    // 앞뒤 바퀴 접지점 간 거리(축거)
  float chi;         // 스키드 계수 (유효윤거 = chi × 윤거)
  float vcal;        // pwmCal 에서의 직진 속도 cm/s
  int16_t pwmCal;    // 속도 보정 PWM
  int16_t dead;      // 모터가 안 도는 PWM 하한
  int16_t maxPwm;    // 최대 PWM (TT모터 과전압 보호)
  int16_t basePwm;   // 자율주행 기본 PWM
  int8_t yawSign;    // 자이로 방향
};
// 스키드계수(chi) 추정.
//  예전 식 1+(L/T)^2 는 축거가 윤거보다 길면 끝없이 커집니다.
//  (T150mm·L400mm 이면 8.11 → 유효윤거 1.2m 라는 값이 나와, 반경 30cm 원호에서
//   안쪽 바퀴가 거꾸로 도는 엉뚱한 주행이 됩니다.)
//  제자리 회전 때 바퀴 접지점이 그리는 원을 기준으로 sqrt(1+(L/T)^2) 로 바꾸고
//  실측에서 나오는 범위인 1.0~3.0 으로 묶습니다.
//  정확한 값은 X 명령(2초 회전 자동보정)으로 실제 측정해 덮어씁니다.
float chiEstimate(int16_t trackMm, int16_t baseMm) {
  if (trackMm <= 0) return 1.0;
  float r = (float)baseMm / (float)trackMm;
  float c = sqrt(1.0 + r * r);
  if (c < 1.0) c = 1.0;
  if (c > 3.0) c = 3.0;
  return c;
}

const uint16_t CFG_MAGIC = 0xB0A7;
Cfg cfg;

void cfgDefault() {
  cfg.magic = CFG_MAGIC;
  cfg.trackMm = 150; cfg.baseMm = 120;
  cfg.chi = chiEstimate(cfg.trackMm, cfg.baseMm);   // 초기 추정값, X 명령으로 실측 보정
  cfg.vcal = 25.0; cfg.pwmCal = 150;
  cfg.dead = 60; cfg.maxPwm = 180; cfg.basePwm = 140;
  cfg.yawSign = 1;
}
void cfgLoad() { EEPROM.get(0, cfg); if (cfg.magic != CFG_MAGIC) { cfgDefault(); EEPROM.put(0, cfg); } }
void cfgSave() { EEPROM.put(0, cfg); }

// ---------------- I2C 버스 관리 ----------------
// 센서 두 개가 같은 A4/A5 를 나눠 쓰므로, 한쪽이 전송 도중 리셋되면
// SDA 가 LOW 에 물려 버스 전체가 멎습니다. 그러면 "하나를 꽂으면 다른 하나가
// 안 보이는" 증상이 납니다. 아래 두 함수가 그 상황을 스스로 푸는 장치입니다.
// .ino 자동 프로토타입은 '함수'만 만들어 줍니다. 아래 변수·함수는 정의보다
// 먼저 쓰이므로 여기에 미리 선언해 둡니다.
void sensorsInit();
void sendSensors();
void mpuInit();
void bmeInit();
void gyroCalibrate();
unsigned long lastImu = 0, lastTel = 0, lastEnv = 0, lastSonar = 0, lastScan = 0;

bool i2cSlow = false;            // 복구가 반복되면 50kHz 로 낮춥니다
uint8_t mpuFail = 0, bmeFail = 0;
uint16_t i2cRecoverCnt = 0;
unsigned long lastRecover = 0;         // 복구가 연달아 돌면 주행명령을 놓칩니다
bool recoverDue() {
  unsigned long ms = millis();
  if (ms - lastRecover < 1500) return false;
  lastRecover = ms; return true;
}

void i2cStart() {
  Wire.begin();
  Wire.setClock(i2cSlow ? 50000UL : 100000UL);
#if defined(ARDUINO_ARCH_AVR)
  // 배선이 잘못돼 응답이 없어도 25ms 뒤에는 버스를 풀고 프로그램이 계속 돕니다
  Wire.setWireTimeout(3000, true);   // 3ms. 100kHz I2C 한 바이트는 0.1ms 이므로 충분하고,
                                     // 센서가 없을 때 루프가 길게 멎지 않습니다
#endif
}

// SDA 가 LOW 에 붙어 있으면 SCL 을 9번 흔들어 상대 장치가 남은 비트를
// 흘려보내게 한 뒤 STOP 조건을 만들어 버스를 풀어 줍니다.
bool i2cRecover() {
  pinMode(SCL, INPUT_PULLUP);
  pinMode(SDA, INPUT_PULLUP);
  delayMicroseconds(20);
  if (digitalRead(SDA) == HIGH) { i2cStart(); return false; }   // 버스는 멀쩡
  for (uint8_t i = 0; i < 9 && digitalRead(SDA) == LOW; i++) {
    pinMode(SCL, OUTPUT); digitalWrite(SCL, LOW); delayMicroseconds(6);
    pinMode(SCL, INPUT_PULLUP);                   delayMicroseconds(6);
  }
  pinMode(SDA, OUTPUT); digitalWrite(SDA, LOW); delayMicroseconds(6);
  pinMode(SCL, INPUT_PULLUP);                   delayMicroseconds(6);
  pinMode(SDA, INPUT_PULLUP);                   delayMicroseconds(6);   // STOP
  if (i2cRecoverCnt < 65535) i2cRecoverCnt++;
  if (i2cRecoverCnt >= 3) i2cSlow = true;        // 자꾸 멎으면 속도를 낮춘다
  i2cStart();
  return true;
}

// ---------------- MPU6050 ----------------
uint8_t MPU = 0;          // 찾은 주소 (0x68, AD0 핀이 HIGH 면 0x69). 0 = 없음
bool mpuOk = false;
float gzBias = 0, gxBias = 0, gyBias = 0;
float roll = 0, pitch = 0, yaw = 0, gzDps = 0;
float axG = 0, ayG = 0, azG = 1;
float vibSum = 0, vibSq = 0; uint16_t vibN = 0;   // 진동(지형 거칠기) 통계
float lastAccMag = 1.0;

void mpuWrite(uint8_t r, uint8_t v) { Wire.beginTransmission(MPU); Wire.write(r); Wire.write(v); Wire.endTransmission(); }
bool mpuRead(int16_t *d) {
  Wire.beginTransmission(MPU); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, (uint8_t)14) != 14) return false;
  for (uint8_t i = 0; i < 7; i++) { int16_t h = Wire.read(); d[i] = (h << 8) | Wire.read(); }
  return true;
}
bool i2cHas(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }
void mpuInit() {
  MPU = i2cHas(0x68) ? 0x68 : (i2cHas(0x69) ? 0x69 : 0);
  mpuOk = (MPU != 0);
  if (!mpuOk) return;
  mpuWrite(0x6B, 0x00);   // 깨우기
  mpuWrite(0x1A, 0x03);   // DLPF 44Hz
  mpuWrite(0x1B, 0x00);   // ±250 dps
  mpuWrite(0x1C, 0x00);   // ±2 g
  delay(50);
}
void gyroCalibrate() {
  if (!mpuOk) return;
  int16_t d[7]; float sx = 0, sy = 0, sz = 0; uint16_t n = 0;
  digitalWrite(PIN_LED, HIGH);
  for (uint16_t i = 0; i < 400; i++) {
    if (mpuRead(d)) { sx += d[4]; sy += d[5]; sz += d[6]; n++; }
    delay(3);
  }
  if (n) { gxBias = sx / n; gyBias = sy / n; gzBias = sz / n; }
  digitalWrite(PIN_LED, LOW);
}

// ---------------- BME280 (라이브러리 없이 데이터시트 보정식) ----------------
uint8_t bmeAddr = 0;
uint8_t bmeChip = 0;      // 0x60 = BME280 (온도·습도·기압), 0x58 = BMP280 (습도 없음)
uint16_t T1, P1; int16_t T2, T3, P2, P3, P4, P5, P6, P7, P8, P9;
uint8_t H1, H3; int16_t H2, H4, H5; int8_t H6;
float tempC = NAN, humP = NAN, pressHpa = NAN, altM = NAN;
float p0Hpa = 1013.25;   // 고도 기준(첫 측정값으로 0m 설정)
bool p0Set = false;

// 레지스터 읽기: 주소를 쓴 뒤 STOP 없이 바로 읽기(repeated start) — 데이터시트 권장 방식
uint8_t bmeR8(uint8_t r) { Wire.beginTransmission(bmeAddr); Wire.write(r); Wire.endTransmission(false); if (Wire.requestFrom(bmeAddr, (uint8_t)1) != 1) return 0; return Wire.read(); }
uint16_t bmeR16LE(uint8_t r) { uint16_t lo = bmeR8(r); uint16_t hi = bmeR8(r + 1); return (hi << 8) | lo; }
void bmeW8(uint8_t r, uint8_t v) { Wire.beginTransmission(bmeAddr); Wire.write(r); Wire.write(v); Wire.endTransmission(); }

void bmeInit() {
  uint8_t cands[2] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2; i++) {
    bmeAddr = cands[i];
    Wire.beginTransmission(bmeAddr);
    if (Wire.endTransmission() == 0) {
      bmeChip = bmeR8(0xD0);
      if (bmeChip == 0x60 || bmeChip == 0x58) break;   // 0x60 BME280, 0x58 BMP280
    }
    bmeAddr = 0;
  }
  if (!bmeAddr) { bmeChip = 0; return; }
  T1 = bmeR16LE(0x88); T2 = bmeR16LE(0x8A); T3 = bmeR16LE(0x8C);
  // 보정값이 전부 0 이면 I2C 가 읽히지 않은 것입니다.
  // 이대로 두면 온도·기압이 엉뚱한 숫자로 나오므로 '없음' 으로 처리합니다.
  // 전부 0 일 때만 '못 읽었다' 로 봅니다.
  // (하나만 0 인 것으로 판정하면, 한 번 삐끗한 읽기에도 멀쩡한 센서를 버리게 됩니다)
  P1 = bmeR16LE(0x8E); P2 = bmeR16LE(0x90); P3 = bmeR16LE(0x92); P4 = bmeR16LE(0x94);
  if (T1 == 0 && T2 == 0 && T3 == 0 && P1 == 0 && P2 == 0) { bmeAddr = 0; bmeChip = 0; return; }
  P5 = bmeR16LE(0x96); P6 = bmeR16LE(0x98); P7 = bmeR16LE(0x9A); P8 = bmeR16LE(0x9C); P9 = bmeR16LE(0x9E);
  if (bmeChip == 0x60) {
  H1 = bmeR8(0xA1); H2 = bmeR16LE(0xE1); H3 = bmeR8(0xE3);
  uint8_t e4 = bmeR8(0xE4), e5 = bmeR8(0xE5), e6 = bmeR8(0xE6);
  H4 = (int16_t)(((int16_t)(int8_t)e4 << 4) | (e5 & 0x0F));
  H5 = (int16_t)(((int16_t)(int8_t)e6 << 4) | (e5 >> 4));
  H6 = (int8_t)bmeR8(0xE7);
  bmeW8(0xF2, 0x01);   // 습도 x1  (ctrl_meas 보다 먼저)
  }
  bmeW8(0xF5, 0xA0);   // standby 1000ms
  bmeW8(0xF4, 0x27);   // 온도 x1, 기압 x1, normal mode
}

void bmeReadAll() {
  if (!bmeAddr) return;
  Wire.beginTransmission(bmeAddr); Wire.write(0xF7); Wire.endTransmission(false);
  uint8_t n = (bmeChip == 0x60) ? 8 : 6;            // BMP280 은 습도 레지스터가 없음
  if (Wire.requestFrom(bmeAddr, n) != n) {
    // 연속으로 실패하면 버스를 풀고 이 센서만 다시 잡습니다 (MPU 는 건드리지 않음)
    if (++bmeFail >= 5 && recoverDue()) { bmeFail = 0; i2cRecover(); bmeInit(); }
    return;
  }
  bmeFail = 0;
  uint8_t d[8] = {0}; for (uint8_t i = 0; i < n; i++) d[i] = Wire.read();
  int32_t adcP = ((int32_t)d[0] << 12) | ((int32_t)d[1] << 4) | (d[2] >> 4);
  int32_t adcT = ((int32_t)d[3] << 12) | ((int32_t)d[4] << 4) | (d[5] >> 4);
  int32_t adcH = ((int32_t)d[6] << 8) | d[7];
  // 온도
  int32_t v1 = ((((adcT >> 3) - ((int32_t)T1 << 1))) * ((int32_t)T2)) >> 11;
  int32_t v2 = (((((adcT >> 4) - ((int32_t)T1)) * ((adcT >> 4) - ((int32_t)T1))) >> 12) * ((int32_t)T3)) >> 14;
  int32_t tFine = v1 + v2;
  tempC = ((tFine * 5 + 128) >> 8) / 100.0;
  // 기압 (float 보정식)
  float f1 = tFine / 2.0 - 64000.0;
  float f2 = f1 * f1 * P6 / 32768.0;
  f2 = f2 + f1 * P5 * 2.0;
  f2 = f2 / 4.0 + P4 * 65536.0;
  f1 = (P3 * f1 * f1 / 524288.0 + P2 * f1) / 524288.0;
  f1 = (1.0 + f1 / 32768.0) * P1;
  if (f1 != 0) {
    float p = 1048576.0 - adcP;
    p = (p - f2 / 4096.0) * 6250.0 / f1;
    f1 = P9 * p * p / 2147483648.0;
    f2 = p * P8 / 32768.0;
    p = p + (f1 + f2 + P7) / 16.0;
    pressHpa = p / 100.0;
    if (!p0Set && pressHpa > 300) { p0Hpa = pressHpa; p0Set = true; }
    altM = 44330.0 * (1.0 - pow(pressHpa / p0Hpa, 0.1903));
  }
  // 습도 (BME280 만)
  if (bmeChip != 0x60) { humP = NAN; return; }
  float h = tFine - 76800.0;
  h = (adcH - (H4 * 64.0 + H5 / 16384.0 * h)) *
      (H2 / 65536.0 * (1.0 + H6 / 67108864.0 * h * (1.0 + H3 / 67108864.0 * h)));
  h = h * (1.0 - H1 * h / 524288.0);
  if (h > 100) h = 100; if (h < 0) h = 0;
  humP = h;
}

// ---------------- 모터 ----------------
int curL = 0, curR = 0;   // 실제 출력 PWM (부호 = 방향)
int tgtL = 0, tgtR = 0;   // 목표 PWM
// 소프트스타트: PWM 을 10ms 마다 RAMP_STEP 씩만 올립니다.
// 모터 기동전류가 한꺼번에 흐르지 않아 배터리 전압이 덜 떨어지고 ESP32-CAM 통신이 안정됩니다.
// 줄이거나 멈추는 쪽은 즉시 반영 (안전). 방향이 바뀔 때는 한 번 0 을 거칩니다.
const int RAMP_STEP = 15;    // 0 → 180 까지 약 0.12초

void setSide(uint8_t en, uint8_t a, uint8_t b, int pwm) {
  if (pwm > 0) { digitalWrite(a, HIGH); digitalWrite(b, LOW); }
  else if (pwm < 0) { digitalWrite(a, LOW); digitalWrite(b, HIGH); }
  else { digitalWrite(a, LOW); digitalWrite(b, LOW); }
  analogWrite(en, abs(pwm));
}
int slew(int cur, int tgt) {
  if (tgt == 0) return 0;
  if (cur != 0 && ((cur > 0) != (tgt > 0))) return 0;          // 방향 전환: 먼저 정지
  if (abs(tgt) <= abs(cur)) return tgt;                          // 감속은 즉시
  int step = min(RAMP_STEP, abs(tgt) - abs(cur));
  return tgt > 0 ? cur + step : cur - step;
}
void applyMotors() {
  int l = slew(curL, tgtL), r = slew(curR, tgtR);
  if (l != curL) { curL = l; setSide(ENA, IN1, IN2, l); }
  if (r != curR) { curR = r; setSide(ENB, IN3, IN4, r); }
}
void drive(int l, int r) {
  tgtL = constrain(l, -cfg.maxPwm, cfg.maxPwm);
  tgtR = constrain(r, -cfg.maxPwm, cfg.maxPwm);
  applyMotors();             // 감속·정지는 여기서 바로 반영
}

// PWM → 바퀴 선속도(cm/s) 모델 (데드밴드 이상에서 선형)
float speedOf(int pwm) {
  int a = abs(pwm);
  if (a <= cfg.dead) return 0;
  float v = cfg.vcal * (float)(a - cfg.dead) / (float)(cfg.pwmCal - cfg.dead);
  return pwm > 0 ? v : -v;
}
// 바퀴 선속도 → PWM (역모델)
int pwmOf(float v) {
  if (fabs(v) < 0.05) return 0;
  float a = cfg.dead + fabs(v) / cfg.vcal * (cfg.pwmCal - cfg.dead);
  int p = (int)(a + 0.5);
  if (p > cfg.maxPwm) p = cfg.maxPwm;
  return v > 0 ? p : -p;
}

// ---------------- 회전 기하 ----------------
// 유효 윤거 B = chi × T.  반경 R 원호에서  vL = v(R - B/2)/R, vR = v(R + B/2)/R
float bEffCm() { return cfg.chi * cfg.trackMm / 10.0; }
float pivotSweepCm() {   // 제자리 회전 시 바퀴 접지점이 그리는 원의 반지름
  float t = cfg.trackMm / 20.0, b = cfg.baseMm / 20.0;
  return sqrt(t * t + b * b);
}

// ---------------- 위치(오도메트리) ----------------
float posX = 0, posY = 0;    // cm. x = 처음 앞방향, y = 처음 왼쪽
float travelCm = 0;          // 현재 동작 중 이동거리

// ---------------- 동작 상태 ----------------
// (enum Mode 와 변수 mode 는 파일 위쪽에 선언되어 있습니다)
char curCmd = 0;
unsigned long modeStart = 0, lastCmdMs = 0, timeout = 0;
float targetYaw = 0, targetVal = 0, holdYaw = 0;
int basePl = 0, basePr = 0;
bool headingHold = false;
float calYaw0 = 0; bool calMeasuring = false;
const float K_HEAD = 4.0;          // 직진 방향유지 비례 게인 (PWM/deg)
float bumpG = 0.60;                // 충돌 감지 가속도 급변 임계값(g). J 명령으로 변경, 0 = 끔

void reply(char c, char cmd) { bt.print(c); bt.print(','); bt.println(cmd); }

void stopAll(bool notify) {
  drive(0, 0);
  if (notify && mode != IDLE && mode != MANUAL) reply('F', curCmd);
  mode = IDLE; headingHold = false;
}

void startMode(Mode m, char cmd, unsigned long tmo) {
  mode = m; curCmd = cmd; modeStart = millis(); timeout = tmo; travelCm = 0;
  reply('K', cmd);
}

void sendGeo() {
  bt.print(F("G,")); bt.print(cfg.trackMm); bt.print(','); bt.print(cfg.baseMm); bt.print(',');
  bt.print((long)(cfg.chi * 1000)); bt.print(','); bt.print((long)(bEffCm() * 10)); bt.print(',');
  bt.print((long)(pivotSweepCm() * 10)); bt.print(','); bt.print((long)(cfg.vcal * 10)); bt.print(',');
  bt.print(cfg.pwmCal); bt.print(','); bt.print(cfg.dead); bt.print(','); bt.print(cfg.maxPwm); bt.print(',');
  bt.print(cfg.basePwm); bt.print(','); bt.println(cfg.yawSign);
}

// ---------------- 초음파(선택) ----------------
int sonarCm = -1;
#if USE_SONAR
int readSonar() {
  digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10); digitalWrite(PIN_TRIG, LOW);
  unsigned long d = pulseIn(PIN_ECHO, HIGH, 12000UL);   // 최대 약 2m
  return d ? (int)(d / 58) : -1;
}
#endif

// ---------------- 명령 처리 ----------------
char buf[48]; uint8_t bufLen = 0;

float argF(uint8_t idx) {   // buf 를 ',' 로 나눈 idx 번째 값
  char *p = buf; uint8_t n = 0;
  while (*p && n < idx) { if (*p == ',') n++; p++; }
  return (n == idx) ? atof(p) : 0;
}
char argC(uint8_t idx) {
  char *p = buf; uint8_t n = 0;
  while (*p && n < idx) { if (*p == ',') n++; p++; }
  return (n == idx) ? *p : 0;
}

void beginArc(float radiusCm, float deg, char cmd) {
  float vc = speedOf(cfg.basePwm);
  float B = bEffCm();
  if (radiusCm < 0.5) {   // 제자리 회전
    targetYaw = yaw + deg; targetVal = deg;
    startMode(TURN, cmd, 1000UL + (unsigned long)(fabs(deg) * 60));
    return;
  }
  float vIn = vc * (radiusCm - B / 2) / radiusCm;
  float vOut = vc * (radiusCm + B / 2) / radiusCm;
  float vMax = speedOf(cfg.maxPwm);
  if (fabs(vOut) > vMax) { float s = vMax / fabs(vOut); vIn *= s; vOut *= s; }
  if (deg >= 0) { basePl = pwmOf(vIn); basePr = pwmOf(vOut); }   // 왼쪽(반시계) 회전 → 오른쪽 바퀴가 바깥
  else          { basePl = pwmOf(vOut); basePr = pwmOf(vIn); }
  targetYaw = yaw + deg; targetVal = deg;
  float arcLen = radiusCm * fabs(deg) * DEG_TO_RAD;
  float vCenter = (vIn + vOut) / 2;
  unsigned long est = vCenter > 1 ? (unsigned long)(arcLen / vCenter * 1000) : 8000;
  startMode(ARC, cmd, est * 2 + 2000);
  drive(basePl, basePr);
}

void handleLine() {
  if (bufLen == 0) return;
  char c = buf[0];
  switch (c) {
    case 'M': {
      int l = (int)argF(1), r = (int)argF(2);
      if (mode != MANUAL && mode != IDLE) stopAll(true);
      mode = MANUAL; curCmd = 'M';
      drive(l, r); lastCmdMs = millis();
      break; }
    case 'S': stopAll(true); reply('K', 'S'); break;
    case 'H': bt.println('h'); break;
    case 'T': {   // T,dir,ms,pwm
      char d = argC(1); unsigned long ms = (unsigned long)argF(2); int p = (int)argF(3);
      if (p <= 0) p = cfg.basePwm;
      startMode(TIMED, 'T', ms);
      headingHold = (d == 'F' || d == 'B'); holdYaw = yaw;
      if (d == 'F') { basePl = p; basePr = p; }
      else if (d == 'B') { basePl = -p; basePr = -p; }
      else if (d == 'L') { basePl = -p; basePr = p; }
      else if (d == 'R') { basePl = p; basePr = -p; }
      else { basePl = basePr = 0; }
      drive(basePl, basePr);
      break; }
    case 'D': {   // D,cm
      float cm = argF(1);
      targetVal = fabs(cm);
      int p = cm >= 0 ? cfg.basePwm : -cfg.basePwm;
      basePl = basePr = p; headingHold = true; holdYaw = yaw;
      float v = fabs(speedOf(p));
      unsigned long est = v > 1 ? (unsigned long)(targetVal / v * 1000) : 5000;
      startMode(DIST, 'D', est * 2 + 2000);
      drive(basePl, basePr);
      break; }
    case 'A': {   // A,deg
      float deg = argF(1);
      targetYaw = yaw + deg; targetVal = deg;
      startMode(TURN, 'A', 1500UL + (unsigned long)(fabs(deg) * 60));
      break; }
    case 'R': beginArc(argF(1), argF(2), 'R'); break;
    case 'G': {
      int t = (int)argF(1), b = (int)argF(2);
      if (t >= 30 && t <= 1000 && b >= 0 && b <= 1000) {
        cfg.trackMm = t; cfg.baseMm = b;
        cfg.chi = chiEstimate(t, b);                     // 기하 기반 초기 추정
        cfgSave();
      }
      sendGeo(); break; }
    case 'X': {   // chi 자동 보정: 2초 제자리 왼쪽 회전
      startMode(CALIB, 'X', 2600);
      calMeasuring = false;
      drive(-cfg.basePwm, cfg.basePwm);
      break; }
    case 'V': {   // V,pwm,cm,ms
      int p = (int)argF(1); float cm = argF(2); float ms = argF(3);
      if (p > cfg.dead && cm > 0 && ms > 100) { cfg.pwmCal = p; cfg.vcal = cm / (ms / 1000.0); cfgSave(); }
      sendGeo(); break; }
    case 'P': { int p = (int)argF(1); if (p > cfg.dead && p <= 255) { cfg.basePwm = p; cfgSave(); } sendGeo(); break; }
    case 'W': {
      int d = (int)argF(1), mx = (int)argF(2);
      if (d >= 0 && d < 200) cfg.dead = d;
      if (mx > cfg.dead && mx <= 255) cfg.maxPwm = mx;
      cfgSave(); sendGeo(); break; }
    case 'Y': { int s = (int)argF(1); cfg.yawSign = (s < 0) ? -1 : 1; cfgSave(); sendGeo(); break; }
    case 'Z':   // 위치·방향 원점. 진행 중이던 목표값도 함께 지웁니다
      posX = posY = 0; yaw = 0; travelCm = 0;
      holdYaw = 0; targetYaw = 0;
      reply('K', 'Z'); break;
    case 'C':   // 자이로 재보정 (약 1.2초). 끝난 뒤 dt 기준 시각을 다시 잡습니다
      stopAll(false); gyroCalibrate(); lastImu = micros(); reply('K', 'C'); break;
    case 'J': bumpG = argF(1) / 100.0; reply('K', 'J'); break;
    case 'Q': sendGeo(); sendSensors(); break;
    case 'U': stopAll(false); sensorsInit(); sendSensors(); break;   // 센서 다시 찾기 (2초)
  }
}

void readBT() {
  while (bt.available()) {
    char ch = bt.read();
    if (ch == '\n' || ch == '\r') { buf[bufLen] = 0; handleLine(); bufLen = 0; }
    else if (bufLen < sizeof(buf) - 1) buf[bufLen++] = ch;
  }
}

// ---------------- 주기 작업 ----------------

void updateImu() {
  int16_t d[7];
  unsigned long now = micros();
  float dt = (now - lastImu) / 1e6; lastImu = now;
  if (dt <= 0 || dt > 0.1) dt = 0.01;

  // ★ 오도메트리는 자이로가 없어도 반드시 돌아가야 합니다.
  //   D 명령의 거리 적산(travelCm)과 지도 좌표(posX/posY)가 여기서 만들어지므로,
  //   예전처럼 MPU 검사 뒤에 두면 MPU6050 이 없을 때 거리가 영원히 0 이 되어
  //   D 명령이 '타임아웃으로만' 끝나고 지도에 아무것도 그려지지 않습니다.
  float v = (speedOf(curL) + speedOf(curR)) / 2.0;
  float hRad = yaw * DEG_TO_RAD;
  posX += v * cos(hRad) * dt;
  posY += v * sin(hRad) * dt;
  travelCm += fabs(v) * dt;

  if (!mpuOk) return;
  if (!mpuRead(d)) {
    // 연속 실패하면 버스를 풀고 MPU 만 다시 잡습니다 (BME 는 건드리지 않음)
    if (++mpuFail >= 20 && recoverDue()) { mpuFail = 0; i2cRecover(); mpuInit(); }
    return;
  }
  mpuFail = 0;
  axG = d[0] / 16384.0; ayG = d[1] / 16384.0; azG = d[2] / 16384.0;
  float gx = (d[4] - gxBias) / 131.0, gy = (d[5] - gyBias) / 131.0;
  float gzRaw = d[6];
  gzDps = (gzRaw - gzBias) / 131.0 * cfg.yawSign;
  // 정지 상태에서 자이로 바이어스 천천히 추적 (드리프트 감소)
  if (curL == 0 && curR == 0 && fabs(gzDps) < 1.5) { gzBias += 0.002 * (gzRaw - gzBias); gzDps = 0; }
  float rollAcc = atan2(ayG, azG) * RAD_TO_DEG;
  float pitchAcc = atan2(-axG, sqrt(ayG * ayG + azG * azG)) * RAD_TO_DEG;
  roll = 0.98 * (roll + gx * dt) + 0.02 * rollAcc;
  pitch = 0.98 * (pitch + gy * dt) + 0.02 * pitchAcc;
  yaw += gzDps * dt;

  float mag = sqrt(axG * axG + ayG * ayG + azG * azG);
  vibSum += mag; vibSq += mag * mag; vibN++;
  float jerk = fabs(mag - lastAccMag); lastAccMag = mag;

  // 충돌 감지: 전진/후진 중 가속도 급변
  if ((mode == DIST || mode == TIMED || mode == ARC) && millis() - modeStart > 400 && bumpG > 0 && jerk > bumpG) {
    bt.print(F("B,")); bt.print((long)(posX * 10)); bt.print(','); bt.println((long)(posY * 10));
    stopAll(true);
  }
}

void updateMotion() {
  unsigned long el = millis() - modeStart;
  switch (mode) {
    case MANUAL:
      if (millis() - lastCmdMs > 700) { drive(0, 0); mode = IDLE; }   // 통신 끊김 보호
      break;
    case TIMED:
    case DIST: {
      if ((mode == TIMED && el >= timeout) || (mode == DIST && (travelCm >= targetVal || el >= timeout))) { stopAll(true); break; }
#if USE_SONAR
      if (basePl > 0 && basePr > 0 && sonarCm > 0 && sonarCm < 15) {
        bt.print(F("B,")); bt.print((long)(posX * 10)); bt.print(','); bt.println((long)(posY * 10));
        stopAll(true); break;
      }
#endif
      if (headingHold) {
        float err = holdYaw - yaw;
        int corr = (int)(K_HEAD * err);
        corr = constrain(corr, -40, 40);
        drive(basePl - corr, basePr + corr);
      }
      break; }
    case TURN: {
      float rem = targetYaw - yaw;
      if (fabs(rem) <= 3.0 || el >= timeout) { stopAll(true); break; }
      int p = cfg.basePwm;
      if (fabs(rem) < 25) p = max(cfg.dead + 25, cfg.basePwm * 2 / 3);
      if (rem > 0) drive(-p, p); else drive(p, -p);
      break; }
    case ARC: {
      float rem = targetYaw - yaw;
      if ((targetVal >= 0 && rem <= 2.0) || (targetVal < 0 && rem >= -2.0) || el >= timeout) stopAll(true);
      break; }
    case CALIB: {
      if (!calMeasuring && el >= 500) { calYaw0 = yaw; calMeasuring = true; }
      if (el >= 2500) {
        float dYaw = (yaw - calYaw0) * DEG_TO_RAD;      // 2.0초 동안 회전량
        float w = fabs(dYaw) / 2.0;                     // rad/s
        float v = fabs(speedOf(cfg.basePwm));
        stopAll(true);
        if (w > 0.05 && v > 0.5) {
          float B = 2.0 * v / w;                        // ω = 2v / B
          float c = B / (cfg.trackMm / 10.0);
          if (c > 0.8 && c < 5.0) { cfg.chi = c; cfgSave(); }
        }
        sendGeo();
      }
      break; }
    default: break;
  }
}

void sendTelemetry() {
  bt.print(F("I,")); bt.print((int)(roll * 10)); bt.print(','); bt.print((int)(pitch * 10)); bt.print(',');
  bt.print((long)(yaw * 10)); bt.print(','); bt.print((long)(posX * 10)); bt.print(',');
  bt.print((long)(posY * 10)); bt.print(','); bt.print((int)mode); bt.print(','); bt.println(sonarCm);
}

void sendEnv() {
  bmeReadAll();
  float vib = 0;
  if (vibN > 1) { float m = vibSum / vibN; float var = vibSq / vibN - m * m; vib = var > 0 ? sqrt(var) : 0; }
  vibSum = vibSq = 0; vibN = 0;
  float vb = 0;
#if USE_BATT
  vb = analogRead(A0) * 5.0 / 1023.0 * BATT_RATIO;
#endif
  bt.print(F("E,"));
  bt.print(isnan(tempC) ? -9999L : (long)(tempC * 100)); bt.print(',');
  bt.print(isnan(humP) ? -1L : (long)(humP * 10)); bt.print(',');
  bt.print(isnan(pressHpa) ? -1L : (long)(pressHpa * 10)); bt.print(',');
  bt.print(isnan(altM) ? 0L : (long)(altM * 10)); bt.print(',');
  bt.print((long)(vib * 100)); bt.print(',');
  bt.println((long)(vb * 100));
}

// HM-10 이름을 PUMPKIN-팀번호 로 설정 (연결 전 상태에서만 AT 명령이 먹힙니다)
// 정품 HM-10 은 줄바꿈 없이, 호환 모듈(BT05·CC41·JDY 등)은 \r\n 이 있어야 응답하므로 먼저 종류를 확인합니다.
// 이미 같은 이름이면 아무것도 바꾸지 않습니다.
uint8_t btResp(char *out, uint8_t len, uint16_t ms) {
  uint8_t n = 0; unsigned long t0 = millis();
  while (millis() - t0 < ms) { while (bt.available() && n < len - 1) out[n++] = bt.read(); }
  out[n] = 0; return n;
}
void hm10Cmd(const char *cmd, const char *arg, bool crlf) {
  bt.print(cmd); if (arg) bt.print(arg); if (crlf) bt.print(F("\r\n"));
}
char btName[20] = "?";          // 마지막으로 확인한 BLE 이름 (GCS 로도 보냅니다)
void hm10SetName() {
  char name[14], r[48];
  snprintf(name, sizeof(name), "PUMPKIN-%02d", TEAM_ID);
  delay(900);                    // 모듈이 켜진 직후에는 AT 를 못 받습니다
  while (bt.available()) bt.read();

  // AT 응답 확인 — 정품(줄바꿈 없음)·호환모듈(\r\n) 둘 다, 최대 3번 시도
  bool crlf = false, alive = false;
  for (uint8_t i = 0; i < 3 && !alive; i++) {
    hm10Cmd("AT", NULL, false); btResp(r, sizeof(r), 600);
    if (strstr(r, "OK")) { alive = true; break; }
    bt.print(F("\r\n")); btResp(r, sizeof(r), 600);
    if (strstr(r, "OK")) { alive = true; crlf = true; break; }
    delay(200);
  }
  if (!alive) {
    // 여기로 오는 흔한 경우: ① GCS 가 이미 BLE 로 연결되어 있음(연결 중에는 AT 가 안 먹힘)
    //                      ② HM-10 RXD 분압저항/배선 문제  ③ 통신속도가 9600 이 아님
    Serial.println(F("HM-10 AT 응답 없음 - 이름을 바꾸지 않았습니다"));
    Serial.println(F("  (GCS 연결을 끊고 로버만 전원을 다시 넣어 보세요. D2/D3 배선과 9600bps 도 확인)"));
    snprintf(btName, sizeof(btName), "확인못함");
    return;
  }

  hm10Cmd("AT+NAME?", NULL, crlf); btResp(r, sizeof(r), 700);
  Serial.print(F("HM-10 현재 이름 응답: ")); Serial.println(r);
  if (strstr(r, name)) {
    snprintf(btName, sizeof(btName), "%s", name);
    Serial.print(F("BLE 이름 확인: ")); Serial.println(name);
    return;
  }

  hm10Cmd("AT+NAME", name, crlf); btResp(r, sizeof(r), 700);
  if (!strstr(r, "OK")) {        // 바꾸지 못했으면 RESET 도 보내지 않습니다
    Serial.print(F("BLE 이름 변경 실패 (응답: ")); Serial.print(r); Serial.println(F(")"));
    snprintf(btName, sizeof(btName), "변경실패");
    return;
  }
  hm10Cmd("AT+RESET", NULL, crlf); delay(1400);
  while (bt.available()) bt.read();
  snprintf(btName, sizeof(btName), "%s", name);
  Serial.print(F("BLE 이름 변경 완료: ")); Serial.println(name);
}

// ---------------- I2C 센서 시작 · 진단 ----------------
void sensorsInit() {
  i2cSlow = false; i2cRecoverCnt = 0; mpuFail = bmeFail = 0;
  i2cStart();
  i2cRecover();             // 지난번에 멎은 채로 남아 있으면 먼저 풀어 줍니다
  mpuInit();
  bmeInit();
  // 한쪽만 잡혔다면 버스를 한 번 풀고 못 잡은 쪽만 다시 시도합니다
  if (!mpuOk || !bmeAddr) { i2cRecover(); if (!mpuOk) mpuInit(); if (!bmeAddr) bmeInit(); }
  delay(300);
  gyroCalibrate();
  lastImu = micros();
}

// U,MPU주소,칩ID,BME주소,찾은주소들...   (16진수, 0 = 없음)
// 예) U,68,60,76,68,76   → MPU6050 0x68, BME280 0x76 정상
//     U,0,0,0            → I2C 장치가 하나도 없음 (A4/A5 배선·센서 전원 확인)
void sendSensors() {
  char line[72]; uint8_t n = snprintf(line, sizeof(line), "U,%X,%X,%X", MPU, bmeChip, bmeAddr);
  // SDA 가 LOW 에 물려 있으면 112개 주소를 훑는 동안 루프가 멎습니다. 그때는 건너뜁니다.
  pinMode(SDA, INPUT_PULLUP); delayMicroseconds(20);
  bool busOk = (digitalRead(SDA) == HIGH);
  i2cStart();
  if (busOk)
    for (uint8_t a = 0x08; a < 0x78 && n < sizeof(line) - 4; a++)
      if (i2cHas(a)) n += snprintf(line + n, sizeof(line) - n, ",%X", a);
  bt.println(line);
  Serial.println(line);
  Serial.print(F("MPU6050: ")); Serial.println(mpuOk ? F("OK") : F("없음 - A4(SDA)/A5(SCL), VCC/GND 확인"));
  Serial.print(F("BME/BMP280: "));
  Serial.println(bmeChip == 0x60 ? F("BME280 OK") : bmeChip == 0x58 ? F("BMP280 (습도 없음)") : F("없음 - 3V3/GND, A4/A5 확인"));
  Serial.print(F("I2C 버스 복구 횟수: ")); Serial.print(i2cRecoverCnt);
  Serial.println(i2cSlow ? F("  (50kHz 로 낮춤)") : F("  (100kHz)"));
}

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT); pinMode(PIN_LED, OUTPUT);
#if USE_SONAR
  pinMode(PIN_TRIG, OUTPUT); pinMode(PIN_ECHO, INPUT);
#endif
  cfgLoad();                 // drive() 가 cfg.maxPwm 을 쓰므로 먼저 읽습니다
  drive(0, 0);
  Serial.begin(115200);
  bt.begin(9600);
#if HM10_SET_NAME
  hm10SetName();
#endif
  sensorsInit();            // 부팅 후 2초간 로버를 움직이지 마세요 (자이로 보정)
  bt.print(F("N,PUMPKIN-AA12V TEAM ")); bt.print(TEAM_ID);
  bt.print(F(" BLE=")); bt.println(btName);
  Serial.print(F("BLE 이름: ")); Serial.println(btName);
  sendGeo();
  sendSensors();
}

void loop() {
  readBT();
  unsigned long ms = millis();
  if (micros() - lastImu >= 10000UL) { updateImu(); applyMotors(); }   // 100Hz
  updateMotion();
#if USE_SONAR
  if (ms - lastSonar >= 100) { lastSonar = ms; sonarCm = readSonar(); }
#endif
  if (ms - lastTel >= 200) { lastTel = ms; sendTelemetry(); }   // 5Hz
  if (ms - lastEnv >= 1000) { lastEnv = ms; sendEnv(); }        // 1Hz
  // 못 찾은 센서는 5초마다, 로버가 멈춰 있을 때만 다시 찾아봅니다.
  // (배선을 고쳐 꽂으면 재부팅 없이 살아납니다)
  if ((!mpuOk || !bmeAddr) && mode == IDLE && curL == 0 && curR == 0 && ms - lastScan >= 5000) {
    lastScan = ms;
    bool had = mpuOk && bmeAddr;
    // 한 번에 한 센서만 봅니다. 두 개를 연달아 초기화하면 수백 ms 동안
    // readBT() 가 멈춰서, 그 사이에 누른 주행버튼 명령이 사라집니다.
    if (!mpuOk) mpuInit();
    else if (!bmeAddr) bmeInit();
    readBT();                                     // 재탐색 중에 들어온 명령을 먼저 받습니다
    if (!had && mpuOk && bmeAddr) sendSensors();
  }
  digitalWrite(PIN_LED, mode != IDLE);
}
