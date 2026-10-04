#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <ESP_I2S.h>
#include "esp32-hal-rmt.h"
#include "esp_sleep.h"
#include <math.h>

// ============================================================
// PAIRING VOICE CUBE
// CLEAN VOICE 32kHz
// ROBUST BUTTON + AUDIO MATRIX
//
// XIAO ESP32-C3
//
// PIN MAP
//
// D0 / A0 / GPIO2 = Battery ADC
//
// D1 / GPIO3      = POWER + RECORD
// D2 / GPIO4      = INMP441 SD
// D3 / GPIO5      = MAX98357A DIN
// D4 / GPIO6      = BCLK / SCK
// D5 / GPIO7      = WS / LRC
//
// D6 / GPIO21     = PLAY BUTTON
// D7 / GPIO20     = WS2812 DATA
//
// D10 / GPIO10    = MAX98357A SD
//
// ============================================================


// ============================================================
// PIN
// ============================================================

static const int PIN_BATTERY = A0;

static const int PIN_CONTROL = 3;   // D1
static const int PIN_MIC_SD  = 4;   // D2
static const int PIN_AMP_DIN = 5;   // D3
static const int PIN_BCLK    = 6;   // D4
static const int PIN_WS      = 7;   // D5

static const int PIN_PLAY    = 21;  // D6
static const int PIN_LED     = 20;  // D7

static const int PIN_AMP_SD  = 10;  // D10


// ============================================================
// HARDWARE OPTION
// ============================================================

// MAX98357A SD -> D10 이면 true.
//
// MAX SD가 VIN에 직접 연결되어 있으면 false.
//
// SD를 VIN과 D10에 동시에 연결하지 마십시오.
static const bool AMP_SD_CONTROL = true;


// BAT+ -> 47k -> A0 -> 47k -> GND
static const bool BATTERY_MONITOR_ENABLED = true;


// ============================================================
// AUDIO
// ============================================================

static const uint32_t SAMPLE_RATE = 32000;

static const uint32_t MAX_RECORD_SECONDS = 15;


// 현재 사용하는 녹음 파일
static const char *VOICE_PATH =
    "/voice_32k.raw";


// 이전 코드에서 남았을 수 있는 파일
static const char *OLD_TEMP_PATH =
    "/voice_temp.raw";

static const char *OLD_16K_PATH =
    "/voice.raw";


// LittleFS를 끝까지 꽉 채우지 않도록
// 128KB 정도 남겨둠.
static const size_t FLASH_RESERVE_BYTES =
    128 * 1024;


// ============================================================
// CLEAN VOICE DSP
// ============================================================

// 기본 입력 증폭
static const float DSP_INPUT_GAIN = 3.0f;


// 저역 충격 / 울림 제거
static const float HPF_FREQ = 90.0f;


// 음성 존재감
static const float PRESENCE_FREQ = 3000.0f;

static const float PRESENCE_GAIN_DB = 1.0f;

static const float PRESENCE_Q = 0.80f;


// 최대치까지 밀지 않음
static const float LIMITER_CEILING = 0.84f;


// 재생 단계에서도 약간의 headroom
static const float PLAYBACK_GAIN = 0.88f;


// ============================================================
// BUTTON
// ============================================================

static const uint32_t BUTTON_DEBOUNCE_MS = 35;

static const uint32_t INPUT_COOLDOWN_MS = 300;

static const uint32_t RECORD_HOLD_MS = 3000;

// 상단(D6) 2초 이상 홀드 = 배터리 잔량 표시
static const uint32_t BATTERY_HOLD_MS = 2000;

// 배터리 상태 LED 표시 시간
static const uint32_t BATTERY_STATUS_SHOW_MS = 2000;

// 5% 이하는 매우 낮음으로 간주하여 빨강 점멸
static const int VERY_LOW_BATTERY_PERCENT = 5;


// ============================================================
// LED MATRIX
// ============================================================

static const int LED_COUNT = 8;


// ------------------------------------------------------------
// 실제로 켤 LED 범위
//
// 전원선이 연결된 쪽부터 1,2,3...8로 셌을 때
// LED 2 ~ LED 6만 사용.
// LED 1, 7, 8은 항상 OFF.
//
// 중요:
// WS2812 데이터 순서는 DI 기준입니다.
// 전원선이 연결된 쪽과 DI가 같은 쪽이면 true.
// 반대쪽이면 false로 바꾸면 됩니다.
// ------------------------------------------------------------

static const int ACTIVE_LED_FIRST_NUMBER = 2;
static const int ACTIVE_LED_LAST_NUMBER  = 8;
static const int ACTIVE_LED_COUNT =
    ACTIVE_LED_LAST_NUMBER -
    ACTIVE_LED_FIRST_NUMBER +
    1;

static const bool POWER_END_IS_DATA_INPUT_SIDE = false;


// 웨이브 한 칸 이동시간
static const uint32_t MATRIX_STEP_MS = 55;


// 잔상
static const float MATRIX_DECAY = 0.84f;


// 음성 반응
static const float AUDIO_NOISE_FLOOR = 180.0f;

static const float AUDIO_FULL_LEVEL = 7000.0f;


// 대기 breathing
// 기존보다 조금 느리게 해서 무드등 느낌 강화
static const uint32_t IDLE_BREATHE_MS = 5200;


// ============================================================
// SYSTEM STATE
// ============================================================

enum SystemState {

  STATE_IDLE,
  STATE_RECORDING,
  STATE_PLAYING,
  STATE_SLEEPING
};


SystemState systemState = STATE_IDLE;


// ============================================================
// BUTTON STATE
//
// 사용자 정의 타입을 함수 인자로 넘기지 않고
// 전부 primitive global로 처리.
// Arduino .ino prototype 문제 회피.
// ============================================================

bool d1Raw = HIGH;
bool d1Stable = HIGH;

bool d6Raw = HIGH;
bool d6Stable = HIGH;

uint32_t d1LastRawChange = 0;
uint32_t d6LastRawChange = 0;

bool d1PressedEvent = false;
bool d1ReleasedEvent = false;

bool d6PressedEvent = false;
bool d6ReleasedEvent = false;

uint32_t inputLockUntil = 0;

bool comboQuarantine = false;

bool d1Tracking = false;
bool d6Tracking = false;

uint32_t d1PressedAt = 0;
uint32_t d6PressedAt = 0;


// ============================================================
// I2S
// ============================================================

I2SClass I2S;


// ============================================================
// BIQUAD
//
// ★ setupHighPass(Biquad&) 같은 외부 함수를 사용하지 않음.
//
// 필터 설정 함수 자체를 구조체 내부에 넣어서
// Arduino 자동 prototype 문제를 제거.
// ============================================================

struct Biquad {

  float b0 = 1.0f;
  float b1 = 0.0f;
  float b2 = 0.0f;

  float a1 = 0.0f;
  float a2 = 0.0f;

  float z1 = 0.0f;
  float z2 = 0.0f;


  float process(float x) {

    float y =
        b0 * x +
        z1;


    z1 =
        b1 * x -
        a1 * y +
        z2;


    z2 =
        b2 * x -
        a2 * y;


    return y;
  }


  void reset() {

    z1 = 0.0f;
    z2 = 0.0f;
  }


  void configureHighPass(
    float frequency,
    float q
  ) {

    float w0 =
        2.0f *
        PI *
        frequency /
        SAMPLE_RATE;


    float cw =
        cosf(w0);


    float sw =
        sinf(w0);


    float alpha =
        sw /
        (
          2.0f *
          q
        );


    float localB0 =
        (
          1.0f +
          cw
        ) /
        2.0f;


    float localB1 =
        -(
          1.0f +
          cw
        );


    float localB2 =
        (
          1.0f +
          cw
        ) /
        2.0f;


    float localA0 =
        1.0f +
        alpha;


    float localA1 =
        -2.0f *
        cw;


    float localA2 =
        1.0f -
        alpha;


    b0 =
        localB0 /
        localA0;


    b1 =
        localB1 /
        localA0;


    b2 =
        localB2 /
        localA0;


    a1 =
        localA1 /
        localA0;


    a2 =
        localA2 /
        localA0;


    reset();
  }


  void configurePeaking(
    float frequency,
    float q,
    float gainDB
  ) {

    float A =
        powf(
          10.0f,
          gainDB /
          40.0f
        );


    float w0 =
        2.0f *
        PI *
        frequency /
        SAMPLE_RATE;


    float cw =
        cosf(w0);


    float sw =
        sinf(w0);


    float alpha =
        sw /
        (
          2.0f *
          q
        );


    float localB0 =
        1.0f +
        alpha *
        A;


    float localB1 =
        -2.0f *
        cw;


    float localB2 =
        1.0f -
        alpha *
        A;


    float localA0 =
        1.0f +
        alpha /
        A;


    float localA1 =
        -2.0f *
        cw;


    float localA2 =
        1.0f -
        alpha /
        A;


    b0 =
        localB0 /
        localA0;


    b1 =
        localB1 /
        localA0;


    b2 =
        localB2 /
        localA0;


    a1 =
        localA1 /
        localA0;


    a2 =
        localA2 /
        localA0;


    reset();
  }
};


Biquad highPassFilter;
Biquad presenceEQ;


uint32_t limiterHitCount = 0;


// ============================================================
// DSP
// ============================================================

void initDSP() {

  highPassFilter.configureHighPass(
    HPF_FREQ,
    0.707f
  );


  presenceEQ.configurePeaking(
    PRESENCE_FREQ,
    PRESENCE_Q,
    PRESENCE_GAIN_DB
  );


  Serial.println();
  Serial.println("DSP OK");
  Serial.println("32kHz");
  Serial.println("Input Gain x3.0");
  Serial.println("90Hz HPF");
  Serial.println("3kHz +1.0dB");
  Serial.println("Compressor OFF");
}


void resetDSP() {

  highPassFilter.reset();

  presenceEQ.reset();

  limiterHitCount = 0;
}


int16_t processVoiceSample(
  int16_t input
) {

  float x =
      (float)input /
      32768.0f;


  // gain
  x *= DSP_INPUT_GAIN;


  // low-cut
  x =
      highPassFilter.process(x);


  // presence
  x =
      presenceEQ.process(x);


  // limiter
  if (
    x >
    LIMITER_CEILING
  ) {

    x =
        LIMITER_CEILING;


    limiterHitCount++;
  }

  else if (
    x <
    -LIMITER_CEILING
  ) {

    x =
        -LIMITER_CEILING;


    limiterHitCount++;
  }


  return
      (int16_t)(
        x *
        32767.0f
      );
}


// ============================================================
// AMP
// ============================================================

bool ampEnabled = false;


void ampOn() {

  if (
    ampEnabled
  ) {

    return;
  }


  if (
    AMP_SD_CONTROL
  ) {

    digitalWrite(
      PIN_AMP_SD,
      HIGH
    );


    // amp 안정화
    delay(15);
  }


  ampEnabled = true;
}


void ampOff() {

  if (
    !ampEnabled
  ) {

    // 실제 핀만큼은 LOW 보장
    if (
      AMP_SD_CONTROL
    ) {

      digitalWrite(
        PIN_AMP_SD,
        LOW
      );
    }


    return;
  }


  if (
    AMP_SD_CONTROL
  ) {

    digitalWrite(
      PIN_AMP_SD,
      LOW
    );
  }


  ampEnabled = false;
}


// ============================================================
// LED
// ============================================================

struct RGB {

  uint8_t r;
  uint8_t g;
  uint8_t b;
};


RGB pixels[
  LED_COUNT
];


static const int LED_BITS =
    LED_COUNT *
    24;


rmt_data_t ledData[
  LED_BITS
];


bool ledReady = false;


// matrix history
float audioMatrix[
  ACTIVE_LED_COUNT
] = {0};


float currentEnvelope = 0.0f;

float matrixWindowPeak = 0.0f;

uint32_t lastMatrixStep = 0;


// ============================================================
// UTIL
// ============================================================

float clamp01(
  float x
) {

  if (
    x <
    0.0f
  ) {

    return 0.0f;
  }


  if (
    x >
    1.0f
  ) {

    return 1.0f;
  }


  return x;
}


// 전원 연결부 쪽을 LED 1번으로 보았을 때,
// 1-based LED 번호를 WS2812 실제 데이터 index로 변환.
int powerSideNumberToPixelIndex(
  int ledNumber
) {

  if (
    ledNumber < 1 ||
    ledNumber > LED_COUNT
  ) {

    return -1;
  }


  if (
    POWER_END_IS_DATA_INPUT_SIDE
  ) {

    return
        ledNumber -
        1;
  }


  return
      LED_COUNT -
      ledNumber;
}


// ============================================================
// WS2812
// ============================================================

void clearPixels() {

  for (
    int i = 0;
    i < LED_COUNT;
    i++
  ) {

    pixels[i].r = 0;

    pixels[i].g = 0;

    pixels[i].b = 0;
  }
}


void setPixel(
  int index,
  uint8_t r,
  uint8_t g,
  uint8_t b
) {

  if (
    index <
    0 ||
    index >=
    LED_COUNT
  ) {

    return;
  }


  pixels[index].r = r;

  pixels[index].g = g;

  pixels[index].b = b;
}


void showPixels() {

  if (
    !ledReady
  ) {

    return;
  }


  // HARD MASK:
  // 전원 연결부 기준 물리 LED 2~6만 허용.
  // 다른 함수가 실수로 색을 넣어도 1/7/8은 전송 직전에 강제로 OFF.
  for (int pixelIndex = 0; pixelIndex < LED_COUNT; pixelIndex++) {

    int physicalNumberFromPowerSide;

    if (POWER_END_IS_DATA_INPUT_SIDE) {
      physicalNumberFromPowerSide = pixelIndex + 1;
    } else {
      physicalNumberFromPowerSide = LED_COUNT - pixelIndex;
    }

    if (
      physicalNumberFromPowerSide < ACTIVE_LED_FIRST_NUMBER ||
      physicalNumberFromPowerSide > ACTIVE_LED_LAST_NUMBER
    ) {
      pixels[pixelIndex].r = 0;
      pixels[pixelIndex].g = 0;
      pixels[pixelIndex].b = 0;
    }
  }


  int dataIndex = 0;


  for (
    int led = 0;
    led < LED_COUNT;
    led++
  ) {

    // WS2812 = G R B
    uint8_t colors[3] = {

      pixels[led].g,
      pixels[led].r,
      pixels[led].b
    };


    for (
      int c = 0;
      c < 3;
      c++
    ) {

      for (
        int bit = 7;
        bit >= 0;
        bit--
      ) {

        bool one =
            (
              colors[c] &
              (
                1 <<
                bit
              )
            );


        if (one) {

          ledData[
            dataIndex
          ].level0 = 1;

          ledData[
            dataIndex
          ].duration0 = 8;

          ledData[
            dataIndex
          ].level1 = 0;

          ledData[
            dataIndex
          ].duration1 = 4;
        }

        else {

          ledData[
            dataIndex
          ].level0 = 1;

          ledData[
            dataIndex
          ].duration0 = 4;

          ledData[
            dataIndex
          ].level1 = 0;

          ledData[
            dataIndex
          ].duration1 = 8;
        }


        dataIndex++;
      }
    }
  }


  rmtWrite(
    PIN_LED,
    ledData,
    LED_BITS,
    RMT_WAIT_FOR_EVER
  );


  delayMicroseconds(80);
}


void setAllLED(
  uint8_t r,
  uint8_t g,
  uint8_t b
) {

  // 먼저 8개 전체를 끔.
  // 이렇게 해야 LED 1 / 7 / 8이 이전 색을 유지하지 않음.
  clearPixels();


  // 전원 연결부 기준 LED 2 ~ 6만 켬.
  for (
    int ledNumber = ACTIVE_LED_FIRST_NUMBER;
    ledNumber <= ACTIVE_LED_LAST_NUMBER;
    ledNumber++
  ) {

    int pixelIndex =
        powerSideNumberToPixelIndex(
          ledNumber
        );


    if (
      pixelIndex >= 0
    ) {

      setPixel(
        pixelIndex,
        r,
        g,
        b
      );
    }
  }


  showPixels();
}


void ledOff() {

  // OFF일 때는 8개 전체를 확실히 0으로 보냄.
  clearPixels();

  showPixels();
}


void initLED() {

  Serial.println();
  Serial.println("LED INIT");


  bool result =
      rmtInit(
        PIN_LED,
        RMT_TX_MODE,
        RMT_MEM_NUM_BLOCKS_1,
        10000000
      );


  if (
    !result
  ) {

    Serial.println(
      "LED RMT INIT FAILED"
    );


    ledReady = false;

    return;
  }


  ledReady = true;


  // boot amber
  setAllLED(
    7,
    2,
    0
  );


  delay(180);


  ledOff();


  Serial.println(
    "LED OK"
  );
}


// ============================================================
// IDLE MOOD
// ============================================================

void updateIdleLED() {

  if (
    !ledReady ||
    systemState != STATE_IDLE
  ) {
    return;
  }


  static uint32_t lastUpdate = 0;

  if (
    millis() - lastUpdate < 50
  ) {
    return;
  }

  lastUpdate = millis();


  float phase =
      (millis() % IDLE_BREATHE_MS) /
      (float)IDLE_BREATHE_MS;

  phase *= 2.0f * PI;


  float breathe =
      (sinf(phase) + 1.0f) *
      0.5f;


  // 부드러운 breathing
  breathe =
      breathe *
      breathe;


  // =========================================================
  // RED CASE COMPENSATION
  //
  // 케이스 자체가 붉은 색이므로
  // LED의 R 비율을 낮추고 G/B를 올림.
  //
  // 실제 LED 원색은 살짝 크림/아이보리 쪽,
  // 케이스를 통과하면 따뜻한 웜톤으로 보이게 설정.
  // =========================================================

  const float BASE_R = 48.0f;
  const float BASE_G = 58.0f;
  const float BASE_B = 42.0f;


  // 완전히 꺼지지 않는 무드등
  // 45% ~ 100% 밝기로 천천히 변화
  float brightness =
      0.45f +
      breathe *
      0.55f;


  uint8_t r =
      (uint8_t)(
        BASE_R *
        brightness
      );


  uint8_t g =
      (uint8_t)(
        BASE_G *
        brightness
      );


  uint8_t b =
      (uint8_t)(
        BASE_B *
        brightness
      );


  setAllLED(
    r,
    g,
    b
  );
}


// ============================================================
// RECORD LED
//
// 녹음 중에는 고정 dim red.
//
// breathing을 없애 LED 전류 변화가
// microphone 쪽에 영향을 줄 가능성을 줄임.
// ============================================================

void showRecordLED() {

  // 기존보다 한 단계 밝게.
  setAllLED(
    50,
    0,
    0
  );
}


// ============================================================
// AUDIO MATRIX
// ============================================================

float calculateAudioLevel(
  const int16_t *samples,
  size_t count
) {

  if (
    samples ==
    nullptr ||
    count ==
    0
  ) {

    return 0.0f;
  }


  uint64_t sumAbs = 0;

  int32_t peak = 0;


  for (
    size_t i = 0;
    i < count;
    i++
  ) {

    int32_t value =
        samples[i];


    int32_t magnitude =
        (
          value <
          0
        )
        ?
        -value
        :
        value;


    sumAbs +=
        magnitude;


    if (
      magnitude >
      peak
    ) {

      peak =
          magnitude;
    }
  }


  float meanAbs =
      (float)sumAbs /
      (float)count;


  float energy =
      meanAbs *
      0.85f +
      peak *
      0.15f;


  float normalized =
      (
        energy -
        AUDIO_NOISE_FLOOR
      ) /
      (
        AUDIO_FULL_LEVEL -
        AUDIO_NOISE_FLOOR
      );


  normalized =
      clamp01(
        normalized
      );


  // 작은 신호도 보이게
  normalized =
      sqrtf(
        normalized
      );


  // fast attack
  if (
    normalized >
    currentEnvelope
  ) {

    currentEnvelope =
        currentEnvelope *
        0.20f +
        normalized *
        0.80f;
  }

  // slower release
  else {

    currentEnvelope =
        currentEnvelope *
        0.86f +
        normalized *
        0.14f;
  }


  return
      currentEnvelope;
}


void resetAudioMatrix() {

  for (
    int i = 0;
    i < ACTIVE_LED_COUNT;
    i++
  ) {

    audioMatrix[i] =
        0.0f;
  }


  currentEnvelope =
      0.0f;


  matrixWindowPeak =
      0.0f;


  lastMatrixStep =
      millis();
}


void pushMatrix(
  float newValue
) {

  for (
    int i =
        ACTIVE_LED_COUNT -
        1;
    i >=
        1;
    i--
  ) {

    audioMatrix[i] =
        audioMatrix[
          i - 1
        ] *
        MATRIX_DECAY;
  }


  audioMatrix[0] =
      clamp01(
        newValue
      );
}


void renderAudioMatrix() {

  // LED 1 / 7 / 8은 항상 OFF.
  clearPixels();


  for (
    int index = 0;
    index < ACTIVE_LED_COUNT;
    index++
  ) {

    float energy =
        clamp01(
          audioMatrix[
            index
          ]
        );


    // index 0 = 전원 연결부 기준 LED 2
    // index 4 = 전원 연결부 기준 LED 6
    int ledNumber =
        ACTIVE_LED_FIRST_NUMBER +
        index;


    int physicalLED =
        powerSideNumberToPixelIndex(
          ledNumber
        );


    if (
      physicalLED < 0
    ) {

      continue;
    }


    // 기본 golden amber
    float r =
        10.0f;


    float g =
        4.0f;


    float b =
        1.0f;


    // 기존보다 약 25~30% 정도 밝게.
    r +=
        energy *
        80.0f;


    g +=
        energy *
        50.0f;


    b +=
        energy *
        8.0f;


    setPixel(
      physicalLED,
      (uint8_t)r,
      (uint8_t)g,
      (uint8_t)b
    );
  }


  showPixels();
}


void updatePlaybackMatrix(
  const int16_t *samples,
  size_t count
) {

  float level =
      calculateAudioLevel(
        samples,
        count
      );


  if (
    level >
    matrixWindowPeak
  ) {

    matrixWindowPeak =
        level;
  }


  if (
    millis() -
    lastMatrixStep >=
    MATRIX_STEP_MS
  ) {

    pushMatrix(
      matrixWindowPeak
    );


    matrixWindowPeak =
        0.0f;


    lastMatrixStep =
        millis();


    renderAudioMatrix();
  }
}


// ============================================================
// BUTTON
// ============================================================

void syncButtons() {

  d1Raw =
      digitalRead(
        PIN_CONTROL
      );


  d1Stable =
      d1Raw;


  d6Raw =
      digitalRead(
        PIN_PLAY
      );


  d6Stable =
      d6Raw;


  uint32_t now =
      millis();


  d1LastRawChange =
      now;


  d6LastRawChange =
      now;


  d1PressedEvent = false;
  d1ReleasedEvent = false;

  d6PressedEvent = false;
  d6ReleasedEvent = false;
}


void updateButtons() {

  d1PressedEvent = false;
  d1ReleasedEvent = false;

  d6PressedEvent = false;
  d6ReleasedEvent = false;


  uint32_t now =
      millis();


  // ----------------------------------------
  // D1
  // ----------------------------------------

  bool newD1 =
      digitalRead(
        PIN_CONTROL
      );


  if (
    newD1 !=
    d1Raw
  ) {

    d1Raw =
        newD1;


    d1LastRawChange =
        now;
  }


  if (
    d1Raw !=
    d1Stable
    &&
    now -
    d1LastRawChange >=
    BUTTON_DEBOUNCE_MS
  ) {

    d1Stable =
        d1Raw;


    if (
      d1Stable ==
      LOW
    ) {

      d1PressedEvent =
          true;
    }

    else {

      d1ReleasedEvent =
          true;
    }
  }


  // ----------------------------------------
  // D6
  // ----------------------------------------

  bool newD6 =
      digitalRead(
        PIN_PLAY
      );


  if (
    newD6 !=
    d6Raw
  ) {

    d6Raw =
        newD6;


    d6LastRawChange =
        now;
  }


  if (
    d6Raw !=
    d6Stable
    &&
    now -
    d6LastRawChange >=
    BUTTON_DEBOUNCE_MS
  ) {

    d6Stable =
        d6Raw;


    if (
      d6Stable ==
      LOW
    ) {

      d6PressedEvent =
          true;
    }

    else {

      d6ReleasedEvent =
          true;
    }
  }
}


bool inputLocked() {

  return
      (int32_t)(
        millis() -
        inputLockUntil
      ) <
      0;
}


void lockInput(
  uint32_t duration
) {

  inputLockUntil =
      millis() +
      duration;
}


// ============================================================
// BUTTON RELEASE SAFETY
// ============================================================

void waitAllButtonsReleasedStable(
  bool applyCooldown
) {

  uint32_t releasedSince = 0;


  while (
    true
  ) {

    bool d1 =
        digitalRead(
          PIN_CONTROL
        );


    bool d6 =
        digitalRead(
          PIN_PLAY
        );


    if (
      d1 ==
      HIGH
      &&
      d6 ==
      HIGH
    ) {

      if (
        releasedSince ==
        0
      ) {

        releasedSince =
            millis();
      }


      if (
        millis() -
        releasedSince >=
        100
      ) {

        break;
      }
    }

    else {

      releasedSince =
          0;
    }


    delay(5);
  }


  syncButtons();


  if (
    applyCooldown
  ) {

    lockInput(
      INPUT_COOLDOWN_MS
    );
  }
}


void waitControlReleasedStable() {

  uint32_t releasedSince = 0;


  while (
    true
  ) {

    if (
      digitalRead(
        PIN_CONTROL
      ) ==
      HIGH
    ) {

      if (
        releasedSince ==
        0
      ) {

        releasedSince =
            millis();
      }


      if (
        millis() -
        releasedSince >=
        100
      ) {

        return;
      }
    }

    else {

      releasedSince =
          0;
    }


    delay(5);
  }
}


// ============================================================
// BATTERY
// ============================================================

static const uint32_t BATTERY_PRINT_INTERVAL =
    5000;


uint32_t lastBatteryPrint = 0;


float readBatteryVoltage() {

  if (
    !BATTERY_MONITOR_ENABLED
  ) {

    return -1.0f;
  }


  uint32_t totalMv = 0;


  for (
    int i = 0;
    i < 12;
    i++
  ) {

    totalMv +=
        analogReadMilliVolts(
          PIN_BATTERY
        );
  }


  float adcMv =
      (float)totalMv /
      12.0f;


  // 47k : 47k
  return
      (
        adcMv *
        2.0f
      ) /
      1000.0f;
}


int batteryPercent(
  float voltage
) {

  if (
    voltage >=
    4.20f
  ) {

    return 100;
  }


  if (
    voltage <=
    3.30f
  ) {

    return 0;
  }


  struct BatteryPoint {

    float voltage;
    int percent;
  };


  static const BatteryPoint table[] = {

    {4.20f, 100},
    {4.10f, 90},
    {4.00f, 80},
    {3.92f, 70},
    {3.85f, 60},
    {3.80f, 50},
    {3.75f, 40},
    {3.70f, 30},
    {3.65f, 20},
    {3.55f, 10},
    {3.30f, 0}
  };


  int count =
      sizeof(table) /
      sizeof(table[0]);


  for (
    int i = 0;
    i < count - 1;
    i++
  ) {

    if (
      voltage <=
      table[i].voltage
      &&
      voltage >=
      table[i + 1].voltage
    ) {

      float ratio =
          (
            voltage -
            table[i + 1].voltage
          ) /
          (
            table[i].voltage -
            table[i + 1].voltage
          );


      float percent =
          table[i + 1].percent +
          ratio *
          (
            table[i].percent -
            table[i + 1].percent
          );


      return
          (int)(
            percent +
            0.5f
          );
    }
  }


  return 0;
}


void printBattery() {

  if (
    !BATTERY_MONITOR_ENABLED
  ) {

    return;
  }


  float voltage =
      readBatteryVoltage();


  if (
    voltage <
    2.5f
    ||
    voltage >
    4.5f
  ) {

    Serial.print(
      "[BAT] INVALID "
    );


    Serial.println(
      voltage,
      3
    );


    return;
  }


  Serial.print(
    "[BAT] "
  );


  Serial.print(
    voltage,
    3
  );


  Serial.print(
    " V | "
  );


  Serial.print(
    batteryPercent(
      voltage
    )
  );


  Serial.println(
    " % approx."
  );
}


// ============================================================
// BATTERY STATUS LED
//
// D6(상단) 2초 이상 홀드 시:
//
// 80~100% : 초록
// 50~79%  : 웜 화이트 / 노랑
// 20~49%  : 주황
// 6~19%   : 빨강
// 0~5%    : 빨강 점멸
// ============================================================

void showBatteryStatusLED() {

  if (
    !BATTERY_MONITOR_ENABLED
  ) {

    return;
  }


  float voltage =
      readBatteryVoltage();


  if (
    voltage <
    2.5f ||
    voltage >
    4.5f
  ) {

    Serial.println(
      "[BAT] INVALID -> RED BLINK"
    );


    for (
      int i = 0;
      i < 4;
      i++
    ) {

      setAllLED(
        70,
        0,
        0
      );

      delay(180);

      ledOff();

      delay(180);
    }


    return;
  }


  int percent =
      batteryPercent(
        voltage
      );


  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.print(
    "BATTERY STATUS : "
  );

  Serial.print(
    voltage,
    3
  );

  Serial.print(
    " V | "
  );

  Serial.print(
    percent
  );

  Serial.println(
    " %"
  );

  Serial.println(
    "================================"
  );


  if (
    percent >=
    80
  ) {

    setAllLED(
      0,
      70,
      12
    );

    delay(
      BATTERY_STATUS_SHOW_MS
    );
  }

  else if (
    percent >=
    50
  ) {

    setAllLED(
      70,
      55,
      16
    );

    delay(
      BATTERY_STATUS_SHOW_MS
    );
  }

  else if (
    percent >=
    20
  ) {

    setAllLED(
      85,
      25,
      0
    );

    delay(
      BATTERY_STATUS_SHOW_MS
    );
  }

  else if (
    percent >
    VERY_LOW_BATTERY_PERCENT
  ) {

    setAllLED(
      75,
      0,
      0
    );

    delay(
      BATTERY_STATUS_SHOW_MS
    );
  }

  else {

    uint32_t startedAt =
        millis();


    while (
      millis() -
      startedAt <
      BATTERY_STATUS_SHOW_MS
    ) {

      setAllLED(
        85,
        0,
        0
      );

      delay(180);

      ledOff();

      delay(180);
    }
  }


  ledOff();
}


// ============================================================
// FLASH
// ============================================================

void printFlashInfo() {

  size_t total =
      LittleFS.totalBytes();


  size_t used =
      LittleFS.usedBytes();


  size_t freeBytes =
      (
        total >
        used
      )
      ?
      total -
      used
      :
      0;


  Serial.println();
  Serial.println(
    "---------- FLASH ----------"
  );


  Serial.print(
    "Total : "
  );

  Serial.println(
    total
  );


  Serial.print(
    "Used  : "
  );

  Serial.println(
    used
  );


  Serial.print(
    "Free  : "
  );

  Serial.println(
    freeBytes
  );


  Serial.println(
    "---------------------------"
  );
}


bool initStorage() {

  if (
    !LittleFS.begin(
      true
    )
  ) {

    Serial.println(
      "LittleFS FAILED"
    );


    return false;
  }


  Serial.println(
    "LittleFS OK"
  );


  // ----------------------------------------
  // 이전 버전 불필요 파일 정리
  // ----------------------------------------

  if (
    LittleFS.exists(
      OLD_TEMP_PATH
    )
  ) {

    Serial.println(
      "Delete old temp file"
    );


    LittleFS.remove(
      OLD_TEMP_PATH
    );
  }


  // 16kHz 시절 voice.raw는
  // 현재 코드에서 사용하지 않음.
  if (
    LittleFS.exists(
      OLD_16K_PATH
    )
  ) {

    Serial.println(
      "Delete old 16k voice"
    );


    LittleFS.remove(
      OLD_16K_PATH
    );
  }


  printFlashInfo();


  if (
    LittleFS.exists(
      VOICE_PATH
    )
  ) {

    File file =
        LittleFS.open(
          VOICE_PATH,
          FILE_READ
        );


    if (
      file
    ) {

      Serial.print(
        "Voice size : "
      );


      Serial.println(
        file.size()
      );


      Serial.print(
        "Voice time : "
      );


      Serial.print(
        (float)file.size() /
        (
          SAMPLE_RATE *
          sizeof(int16_t)
        ),
        2
      );


      Serial.println(
        " sec"
      );


      file.close();
    }
  }

  else {

    Serial.println(
      "No voice saved"
    );
  }


  return true;
}


// ============================================================
// I2S
// ============================================================

bool initI2S() {

  I2S.setPins(
    PIN_BCLK,
    PIN_WS,
    PIN_AMP_DIN,
    PIN_MIC_SD
  );


  bool result =
      I2S.begin(
        I2S_MODE_STD,
        SAMPLE_RATE,
        I2S_DATA_BIT_WIDTH_32BIT,
        I2S_SLOT_MODE_STEREO
      );


  if (
    !result
  ) {

    Serial.println(
      "I2S FAILED"
    );


    return false;
  }


  Serial.println(
    "I2S OK @ 32000Hz"
  );


  return true;
}


// ============================================================
// SILENCE
// ============================================================

void sendSilence(
  int blocks
) {

  int32_t silence[
    128
  ] = {0};


  for (
    int i = 0;
    i < blocks;
    i++
  ) {

    I2S.write(
      reinterpret_cast<
        const uint8_t *
      >(
        silence
      ),
      sizeof(
        silence
      )
    );
  }
}


// ============================================================
// RECORD
// ============================================================

bool recordVoice() {

  if (
    systemState !=
    STATE_IDLE
  ) {

    return false;
  }


  systemState =
      STATE_RECORDING;


  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "RECORD PREPARE"
  );

  Serial.println(
    "================================"
  );


  // speaker amp OFF
  ampOff();


  resetDSP();


  // 녹음 중 LED는 고정 빨강
  showRecordLED();


  // ----------------------------------------
  // 기존 음성 제거
  // ----------------------------------------

  if (
    LittleFS.exists(
      VOICE_PATH
    )
  ) {

    Serial.println(
      "Old voice -> DELETE"
    );


    if (
      !LittleFS.remove(
        VOICE_PATH
      )
    ) {

      Serial.println(
        "DELETE FAILED"
      );


      systemState =
          STATE_IDLE;


      waitAllButtonsReleasedStable(
        true
      );


      return false;
    }


    delay(20);
  }


  // ----------------------------------------
  // Flash 공간 계산
  // ----------------------------------------

  size_t totalFS =
      LittleFS.totalBytes();


  size_t usedFS =
      LittleFS.usedBytes();


  size_t freeFS =
      (
        totalFS >
        usedFS
      )
      ?
      totalFS -
      usedFS
      :
      0;


  Serial.print(
    "Flash free : "
  );


  Serial.println(
    freeFS
  );


  if (
    freeFS <=
    FLASH_RESERVE_BYTES +
    16384
  ) {

    Serial.println(
      "NOT ENOUGH FLASH"
    );


    systemState =
        STATE_IDLE;


    waitAllButtonsReleasedStable(
      true
    );


    return false;
  }


  size_t usableBytes =
      freeFS -
      FLASH_RESERVE_BYTES;


  size_t maxSamplesByFlash =
      usableBytes /
      sizeof(int16_t);


  size_t requestedSamples =
      SAMPLE_RATE *
      MAX_RECORD_SECONDS;


  size_t maxSamples;


  if (
    maxSamplesByFlash <
    requestedSamples
  ) {

    maxSamples =
        maxSamplesByFlash;
  }

  else {

    maxSamples =
        requestedSamples;
  }


  Serial.print(
    "Record limit : "
  );


  Serial.print(
    (float)maxSamples /
    SAMPLE_RATE,
    2
  );


  Serial.println(
    " sec"
  );


  File file =
      LittleFS.open(
        VOICE_PATH,
        FILE_WRITE
      );


  if (
    !file
  ) {

    Serial.println(
      "VOICE FILE OPEN FAILED"
    );


    systemState =
        STATE_IDLE;


    waitAllButtonsReleasedStable(
      true
    );


    return false;
  }


  // ----------------------------------------
  // D1 3초 hold로 들어왔기 때문에
  // 일단 버튼을 완전히 놓게 함.
  // ----------------------------------------

  Serial.println(
    "Release D1..."
  );


  waitControlReleasedStable();


  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "RECORD START"
  );

  Serial.println(
    "D1 = STOP"
  );

  Serial.println(
    "D6 ignored while recording"
  );

  Serial.println(
    "================================"
  );


  static const size_t FRAMES =
      256;


  int32_t rxBuffer[
    FRAMES *
    2
  ];


  int16_t pcmBuffer[
    FRAMES
  ];


  size_t totalSamples = 0;

  int32_t rawPeak = 0;

  int32_t processedPeak = 0;


  // 초기 DMA 폐기
  for (
    int i = 0;
    i < 4;
    i++
  ) {

    I2S.readBytes(
      reinterpret_cast<
        char *
      >(
        rxBuffer
      ),
      sizeof(
        rxBuffer
      )
    );
  }


  bool writeError = false;


  while (
    totalSamples <
    maxSamples
  ) {

    // ========================================================
    // RECORD 중:
    //
    // D6는 아예 읽지 않음.
    // D1만 STOP으로 사용.
    // ========================================================

    if (
      digitalRead(
        PIN_CONTROL
      ) ==
      LOW
    ) {

      delay(
        BUTTON_DEBOUNCE_MS
      );


      if (
        digitalRead(
          PIN_CONTROL
        ) ==
        LOW
      ) {

        Serial.println(
          "D1 -> RECORD STOP"
        );


        break;
      }
    }


    // ----------------------------------------
    // MIC READ
    // ----------------------------------------

    size_t bytesRead =
        I2S.readBytes(
          reinterpret_cast<
            char *
          >(
            rxBuffer
          ),
          sizeof(
            rxBuffer
          )
        );


    if (
      bytesRead ==
      0
    ) {

      Serial.println(
        "MIC READ FAILED"
      );


      writeError =
          true;


      break;
    }


    size_t framesRead =
        bytesRead /
        (
          2 *
          sizeof(int32_t)
        );


    size_t remaining =
        maxSamples -
        totalSamples;


    if (
      framesRead >
      remaining
    ) {

      framesRead =
          remaining;
    }


    // ----------------------------------------
    // MIC -> DSP
    // ----------------------------------------

    for (
      size_t i = 0;
      i < framesRead;
      i++
    ) {

      // INMP441 L/R = GND
      // left slot
      int32_t raw32 =
          rxBuffer[
            i *
            2
          ];


      int16_t raw16 =
          (int16_t)(
            raw32 >>
            16
          );


      int32_t rawMagnitude =
          (
            raw16 <
            0
          )
          ?
          -(int32_t)raw16
          :
          (int32_t)raw16;


      if (
        rawMagnitude >
        rawPeak
      ) {

        rawPeak =
            rawMagnitude;
      }


      int16_t processed =
          processVoiceSample(
            raw16
          );


      pcmBuffer[i] =
          processed;


      int32_t magnitude =
          (
            processed <
            0
          )
          ?
          -(int32_t)processed
          :
          (int32_t)processed;


      if (
        magnitude >
        processedPeak
      ) {

        processedPeak =
            magnitude;
      }
    }


    // ----------------------------------------
    // FLASH WRITE
    // ----------------------------------------

    size_t bytesToWrite =
        framesRead *
        sizeof(int16_t);


    size_t written =
        file.write(
          reinterpret_cast<
            const uint8_t *
          >(
            pcmBuffer
          ),
          bytesToWrite
        );


    if (
      written !=
      bytesToWrite
    ) {

      Serial.println(
        "FLASH WRITE FAILED"
      );


      writeError =
          true;


      break;
    }


    totalSamples +=
        framesRead;
  }


  file.flush();


  size_t fileSize =
      file.size();


  file.close();


  // ----------------------------------------
  // 에러 파일 폐기
  // ----------------------------------------

  if (
    writeError
  ) {

    LittleFS.remove(
      VOICE_PATH
    );


    Serial.println(
      "BROKEN VOICE FILE DELETED"
    );


    setAllLED(
      12,
      0,
      0
    );


    delay(250);


    systemState =
        STATE_IDLE;


    waitAllButtonsReleasedStable(
      true
    );


    return false;
  }


  // ----------------------------------------
  // RESULT
  // ----------------------------------------

  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "RECORD COMPLETE"
  );

  Serial.println(
    "================================"
  );


  Serial.print(
    "Length         : "
  );


  Serial.print(
    (float)totalSamples /
    SAMPLE_RATE,
    2
  );


  Serial.println(
    " sec"
  );


  Serial.print(
    "Size           : "
  );


  Serial.println(
    fileSize
  );


  Serial.print(
    "Raw peak       : "
  );


  Serial.println(
    rawPeak
  );


  Serial.print(
    "Processed peak : "
  );


  Serial.println(
    processedPeak
  );


  Serial.print(
    "Limiter hits   : "
  );


  Serial.println(
    limiterHitCount
  );


  if (
    totalSamples >
    0
  ) {

    float limiterPercent =
        (
          (float)limiterHitCount /
          (float)totalSamples
        ) *
        100.0f;


    Serial.print(
      "Limiter %      : "
    );


    Serial.print(
      limiterPercent,
      3
    );


    Serial.println(
      "%"
    );
  }


  // 완료 green
  setAllLED(
    0,
    6,
    0
  );


  delay(180);


  systemState =
      STATE_IDLE;


  // 연타 제거
  waitAllButtonsReleasedStable(
    true
  );


  printFlashInfo();


  return
      fileSize >
      0;
}


// ============================================================
// PLAY
// ============================================================

bool playVoice() {

  if (
    systemState !=
    STATE_IDLE
  ) {

    return false;
  }


  if (
    !LittleFS.exists(
      VOICE_PATH
    )
  ) {

    Serial.println(
      "NO VOICE FILE"
    );


    setAllLED(
      8,
      2,
      0
    );


    delay(200);


    waitAllButtonsReleasedStable(
      true
    );


    return false;
  }


  File file =
      LittleFS.open(
        VOICE_PATH,
        FILE_READ
      );


  if (
    !file
  ) {

    Serial.println(
      "VOICE OPEN FAILED"
    );


    return false;
  }


  systemState =
      STATE_PLAYING;


  size_t fileBytes =
      file.size();


  size_t fileSamples =
      fileBytes /
      sizeof(int16_t);


  Serial.println();
  Serial.println(
    "================================"
  );

  Serial.println(
    "PLAYBACK START"
  );

  Serial.println(
    "ALL BUTTON INPUT LOCKED"
  );

  Serial.println(
    "================================"
  );


  Serial.print(
    "File size : "
  );


  Serial.println(
    fileBytes
  );


  // =========================================================
  // AMP POP PROTECTION
  //
  // silence
  // ↓
  // amp ON
  // ↓
  // silence
  // ↓
  // audio fade-in
  // =========================================================

  sendSilence(
    8
  );


  ampOn();


  sendSilence(
    6
  );


  resetAudioMatrix();


  static const size_t FRAMES =
      256;


  int16_t pcmBuffer[
    FRAMES
  ];


  int32_t txBuffer[
    FRAMES *
    2
  ];


  size_t playedSamples = 0;


  // 15ms fade
  const size_t FADE_SAMPLES =
      SAMPLE_RATE *
      15 /
      1000;


  bool playbackError = false;


  while (
    file.available()
  ) {

    size_t bytesRead =
        file.read(
          reinterpret_cast<
            uint8_t *
          >(
            pcmBuffer
          ),
          sizeof(
            pcmBuffer
          )
        );


    if (
      bytesRead ==
      0
    ) {

      break;
    }


    size_t samples =
        bytesRead /
        sizeof(int16_t);


    // ----------------------------------------
    // PCM -> LED MATRIX
    // ----------------------------------------

    updatePlaybackMatrix(
      pcmBuffer,
      samples
    );


    // ----------------------------------------
    // PCM -> AMP
    // ----------------------------------------

    for (
      size_t i = 0;
      i < samples;
      i++
    ) {

      size_t absoluteSample =
          playedSamples +
          i;


      float value =
          (float)pcmBuffer[i] *
          PLAYBACK_GAIN;


      // ----------------------------
      // Fade in
      // ----------------------------

      if (
        absoluteSample <
        FADE_SAMPLES
      ) {

        float fade =
            (float)absoluteSample /
            (float)FADE_SAMPLES;


        value *=
            fade;
      }


      // ----------------------------
      // Fade out
      // ----------------------------

      if (
        fileSamples >
        absoluteSample
      ) {

        size_t remaining =
            fileSamples -
            absoluteSample;


        if (
          remaining <
          FADE_SAMPLES
        ) {

          float fade =
              (float)remaining /
              (float)FADE_SAMPLES;


          value *=
              fade;
        }
      }


      // safety
      if (
        value >
        32767.0f
      ) {

        value =
            32767.0f;
      }


      if (
        value <
        -32768.0f
      ) {

        value =
            -32768.0f;
      }


      int32_t sample32 =
          (
            (int32_t)value
          ) <<
          16;


      // mono -> L/R
      txBuffer[
        i *
        2
      ] =
          sample32;


      txBuffer[
        i *
        2 +
        1
      ] =
          sample32;
    }


    size_t txBytes =
        samples *
        2 *
        sizeof(int32_t);


    size_t written =
        I2S.write(
          reinterpret_cast<
            const uint8_t *
          >(
            txBuffer
          ),
          txBytes
        );


    if (
      written !=
      txBytes
    ) {

      Serial.println(
        "I2S PLAY FAILED"
      );


      playbackError =
          true;


      break;
    }


    playedSamples +=
        samples;


    // ★ 재생 중에는 D1 / D6를 아예 읽지 않습니다.
    //
    // 마구 눌러도:
    // AMP toggle 없음
    // Flash 접근 없음
    // sleep 없음
    // play 재진입 없음
  }


  file.close();


  // =========================================================
  // POP PROTECTION
  // =========================================================

  sendSilence(
    12
  );


  delay(15);


  ampOff();


  // ----------------------------------------
  // Wave tail
  // ----------------------------------------

  for (
    int i = 0;
    i < ACTIVE_LED_COUNT;
    i++
  ) {

    pushMatrix(
      0.0f
    );


    renderAudioMatrix();


    delay(
      MATRIX_STEP_MS
    );
  }


  Serial.println();
  Serial.println(
    "================================"
  );


  if (
    playbackError
  ) {

    Serial.println(
      "PLAYBACK ERROR"
    );
  }

  else {

    Serial.println(
      "PLAYBACK COMPLETE"
    );
  }


  Serial.println(
    "================================"
  );


  Serial.print(
    "Length : "
  );


  Serial.print(
    (float)playedSamples /
    SAMPLE_RATE,
    2
  );


  Serial.println(
    " sec"
  );


  systemState =
      STATE_IDLE;


  // 재생 중 마구 누른 버튼을 전부 폐기
  waitAllButtonsReleasedStable(
    true
  );


  return
      !playbackError;
}


// ============================================================
// DEEP SLEEP
// ============================================================

void enterDeepSleep() {

  if (
    systemState !=
    STATE_IDLE
  ) {

    return;
  }


  systemState =
      STATE_SLEEPING;


  Serial.println();
  Serial.println(
    "POWER OFF -> DEEP SLEEP"
  );


  // audio quiet
  sendSilence(
    8
  );


  ampOff();


  ledOff();


  // 두 버튼 모두 완전히 놓기
  waitAllButtonsReleasedStable(
    false
  );


  // D1 / GPIO3 LOW wake
  esp_deep_sleep_enable_gpio_wakeup(
    1ULL <<
    PIN_CONTROL,
    ESP_GPIO_WAKEUP_GPIO_LOW
  );


  Serial.flush();


  delay(30);


  esp_deep_sleep_start();
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(
    115200
  );


  delay(1000);


  Serial.println();
  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "PAIRING VOICE CUBE"
  );

  Serial.println(
    "32kHz CLEAN VOICE"
  );

  Serial.println(
    "ROBUST INPUT VERSION"
  );

  Serial.println(
    "======================================"
  );


  // ----------------------------------------
  // Buttons
  // ----------------------------------------

  pinMode(
    PIN_CONTROL,
    INPUT_PULLUP
  );


  pinMode(
    PIN_PLAY,
    INPUT_PULLUP
  );


  syncButtons();


  // ----------------------------------------
  // AMP
  // ----------------------------------------

  if (
    AMP_SD_CONTROL
  ) {

    pinMode(
      PIN_AMP_SD,
      OUTPUT
    );


    digitalWrite(
      PIN_AMP_SD,
      LOW
    );
  }


  ampEnabled = false;


  // ----------------------------------------
  // BATTERY
  // ----------------------------------------

  if (
    BATTERY_MONITOR_ENABLED
  ) {

    pinMode(
      PIN_BATTERY,
      INPUT
    );


    analogReadResolution(
      12
    );


    analogSetPinAttenuation(
      PIN_BATTERY,
      ADC_11db
    );
  }


  // ----------------------------------------
  // Wake
  // ----------------------------------------

  if (
    esp_sleep_get_wakeup_cause()
    ==
    ESP_SLEEP_WAKEUP_GPIO
  ) {

    Serial.println(
      "WAKE FROM D1"
    );


    // Wake 버튼을 계속 누르고 있을 수 있음
    waitAllButtonsReleasedStable(
      false
    );
  }


  // ----------------------------------------
  // LED
  // ----------------------------------------

  initLED();


  // ----------------------------------------
  // FLASH
  // ----------------------------------------

  if (
    !initStorage()
  ) {

    setAllLED(
      15,
      0,
      0
    );


    while (
      true
    ) {

      delay(1000);
    }
  }


  // ----------------------------------------
  // DSP
  // ----------------------------------------

  initDSP();


  // ----------------------------------------
  // I2S
  // ----------------------------------------

  if (
    !initI2S()
  ) {

    setAllLED(
      15,
      0,
      0
    );


    while (
      true
    ) {

      delay(1000);
    }
  }


  // 초기 amp 확실히 mute
  sendSilence(
    4
  );


  ampOff();


  systemState =
      STATE_IDLE;


  syncButtons();


  lockInput(
    500
  );


  Serial.println();
  Serial.println(
    "======================================"
  );

  Serial.println(
    "READY"
  );

  Serial.println(
    "======================================"
  );


  Serial.println(
    "D1 short = Deep Sleep"
  );


  Serial.println(
    "D1 3 sec = Record"
  );


  Serial.println(
    "Record 중 D1 = Stop"
  );


  Serial.println(
    "D6 short = Play"
  );


  Serial.println(
    "D6 2 sec = Battery Status"
  );


  Serial.println(
    "Play 중 모든 버튼 무시"
  );


  Serial.println(
    "Record 중 D6 무시"
  );


  Serial.println(
    "D1+D6 동시 입력 무시"
  );


  printBattery();


  lastBatteryPrint =
      millis();
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  if (
    systemState !=
    STATE_IDLE
  ) {

    delay(5);

    return;
  }


  // ----------------------------------------
  // Mood light
  // ----------------------------------------

  updateIdleLED();


  // ----------------------------------------
  // Battery
  // ----------------------------------------

  if (
    millis() -
    lastBatteryPrint >=
    BATTERY_PRINT_INTERVAL
  ) {

    lastBatteryPrint =
        millis();


    printBattery();
  }


  // ----------------------------------------
  // Button debounce
  // ----------------------------------------

  updateButtons();


  if (
    inputLocked()
  ) {

    delay(5);

    return;
  }


  bool d1Pressed =
      (
        d1Stable ==
        LOW
      );


  bool d6Pressed =
      (
        d6Stable ==
        LOW
      );


  // ==========================================================
  // D1 + D6 동시에 누름
  // ==========================================================

  if (
    d1Pressed &&
    d6Pressed
  ) {

    if (
      !comboQuarantine
    ) {

      Serial.println(
        "D1 + D6 -> IGNORE"
      );
    }


    comboQuarantine =
        true;


    d1Tracking =
        false;

    d6Tracking =
        false;


    delay(5);

    return;
  }


  // 둘 다 놓을 때까지 폐기
  if (
    comboQuarantine
  ) {

    if (
      !d1Pressed &&
      !d6Pressed
    ) {

      comboQuarantine =
          false;


      syncButtons();


      lockInput(
        INPUT_COOLDOWN_MS
      );


      Serial.println(
        "COMBO RELEASED"
      );
    }


    delay(5);

    return;
  }


  // ==========================================================
  // D6 DOWN
  //
  // 짧게 누름 = 재생
  // 2초 이상 홀드 = 배터리 상태 표시
  // ==========================================================

  if (
    d6PressedEvent
  ) {

    // D1이 이미 눌린 상태면 D1+D6 조합으로 폐기
    if (
      d1Pressed
    ) {

      comboQuarantine =
          true;


      d1Tracking =
          false;

      d6Tracking =
          false;


      delay(5);

      return;
    }


    d6PressedAt =
        millis();


    d6Tracking =
        true;
  }


  // ==========================================================
  // D6 LONG -> BATTERY STATUS
  // ==========================================================

  if (
    d6Tracking &&
    d6Pressed
  ) {

    if (
      millis() -
      d6PressedAt >=
      BATTERY_HOLD_MS
    ) {

      d6Tracking =
          false;


      Serial.println();
      Serial.println(
        "D6 LONG -> BATTERY STATUS"
      );


      showBatteryStatusLED();


      // 계속 누른 상태의 입력은 폐기
      waitAllButtonsReleasedStable(
        true
      );


      delay(5);

      return;
    }
  }


  // ==========================================================
  // D6 RELEASE BEFORE 2 SEC -> PLAY
  // ==========================================================

  if (
    d6Tracking &&
    d6ReleasedEvent
  ) {

    uint32_t duration =
        millis() -
        d6PressedAt;


    d6Tracking =
        false;


    // 너무 짧은 입력은 노이즈로 처리
    if (
      duration <
      60
    ) {

      Serial.println(
        "D6 GLITCH -> IGNORE"
      );


      lockInput(
        150
      );


      delay(5);

      return;
    }


    Serial.println();
    Serial.println(
      "D6 SHORT -> PLAY"
    );


    playVoice();


    delay(5);

    return;
  }


  // ==========================================================
  // D1 DOWN
  // ==========================================================

  if (
    d1PressedEvent
  ) {

    d1PressedAt =
        millis();


    d1Tracking =
        true;
  }


  // ==========================================================
  // D1 LONG -> RECORD
  // ==========================================================

  if (
    d1Tracking &&
    d1Pressed
  ) {

    if (
      millis() -
      d1PressedAt >=
      RECORD_HOLD_MS
    ) {

      d1Tracking =
          false;


      Serial.println();
      Serial.println(
        "D1 LONG -> RECORD"
      );


      recordVoice();


      delay(5);

      return;
    }
  }


  // ==========================================================
  // D1 RELEASE BEFORE 3 SEC -> SLEEP
  // ==========================================================

  if (
    d1Tracking &&
    d1ReleasedEvent
  ) {

    uint32_t duration =
        millis() -
        d1PressedAt;


    d1Tracking =
        false;


    // 60ms보다 짧은 것은 glitch 취급
    if (
      duration <
      60
    ) {

      Serial.println(
        "D1 GLITCH -> IGNORE"
      );


      lockInput(
        150
      );


      delay(5);

      return;
    }


    Serial.println();
    Serial.println(
      "D1 SHORT -> POWER OFF"
    );


    enterDeepSleep();


    return;
  }


  delay(5);
}