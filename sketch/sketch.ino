// Aeris - M33 sketch
// Reads MAX30102 (PPG), TMP102 (skin temp), DHT11 (ambient temp/RH) and
// MPU6050 (accel), computes heart rate / SpO2 / motion state on-device,
// and exposes the result to Python over the Bridge as "read_vitals".

#include "Arduino_RouterBridge.h"
#include <Wire.h>
#include <math.h>
#include <MAX30105.h>
#include <Arduino_LED_Matrix.h>

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
#define MPU6050_ADDR 0x68

#define DHT11_PIN 2
#define DHT11_READ_INTERVAL_MS 5000UL  // the part cannot be polled faster than this
#define DHT11_STALE_MS 15000UL         // keep last good reading valid for this long

#define PPG_FS 25                        // effective Hz after on-chip averaging
#define PPG_WINDOW_SAMPLES (4 * PPG_FS)  // 4 s rolling buffer

#define ACCEL_FS 25
#define ACCEL_PERIOD_MS (1000 / ACCEL_FS)
#define MOTION_WINDOW_SAMPLES (2 * ACCEL_FS)  // 2 s rolling buffer

#define HR_REFRACTORY_MS 300UL
#define HR_STALE_MS 3000UL
#define RR_HISTORY_LEN 8

// Tunable signal-quality gates. These depend on LED current / tissue
// reflectance and were picked conservatively; adjust after checking real
// baseline/AC readings on Serial if finger-presence detection misbehaves.
#define PPG_FINGER_DC_MIN 20000.0f
#define PPG_MIN_AC_AMPLITUDE 20.0f
#define PPG_MIN_AC_FOR_SPO2 20.0f

#define REPORT_PERIOD_MS 1000UL

// ---------------------------------------------------------------------------
// Alert output hardware
// The board's 3.3 V headers cannot drive a buzzer directly, and there is no
// board-documented pin/constant for an external ack button, so those two
// remain placeholders pending confirmation against the actual wiring. The
// LED matrix and user RGB LED, by contrast, are on-board peripherals driven
// straight from the STM32 - see the Arduino_LED_Matrix / LED4_R,G,B APIs
// confirmed below.
// ---------------------------------------------------------------------------
#define BUZZER_PIN 8              // TODO: confirm wiring. Drives a transistor
                                   // gate (NOT the buzzer coil itself) since
                                   // the header is 3.3V logic, not enough to
                                   // drive most buzzers directly.
#define ACK_BUTTON_PIN 4          // TODO: confirm wiring. External momentary
                                   // button to GND, uses INPUT_PULLUP.
                                   // (D2 now used by the DHT11 data line.)
#define ACK_BUTTON_ACTIVE_LOW true
#define BUTTON_DEBOUNCE_MS 50UL
#define BUZZ_ON_MS 150UL
#define BUZZ_OFF_MS 150UL

// On-board LED matrix (8 rows x 13 cols, 3-bit grayscale) - confirmed via
// Arduino_LED_Matrix.h / `Arduino_LED_Matrix matrix; matrix.draw(frame)` in
// the local examples/core-and-foundational/02-led-matrix/01-... sketch.
#define MATRIX_ROWS 8
#define MATRIX_COLS 13
#define MATRIX_SIZE (MATRIX_ROWS * MATRIX_COLS)

// On-board user RGB LED "LED4": one GPIO per channel, active-LOW, plain
// digitalWrite - confirmed via LED4_R/LED4_G/LED4_B in the local
// examples/inspirational/color-your-leds sketch (set_led4_color there).
// (LED3_R/G/B also exist there as a PWM-capable RGB LED via analogWrite, if
// smoother color mixing is ever wanted instead of these fixed hues.)
#define RGB_FLASH_HALF_PERIOD_MS 250UL     // VERY_HIGH red flash: 2 Hz full cycle
#define MATRIX_FLASH_HALF_PERIOD_MS 500UL  // VERY_HIGH matrix flash: 1 Hz full cycle

// ---------------------------------------------------------------------------
// Autonomous fail-safe (Stage 7)
// If Python goes quiet for this long, the M33 stops waiting for raise_alert()
// RPCs and starts grading its own HR readings against the last thresholds
// Python handed it, driving the same buzzer/LED/matrix path directly.
// ---------------------------------------------------------------------------
#define BRIDGE_TIMEOUT_MS 30000UL  // confirmed value from your spec (30 s)

// Corner-pixel blink rate for the "running on its own" marker. Deliberately
// a different rate from both existing matrix/RGB flashes (2 Hz RGB, 1 Hz
// matrix full-flash) so it can't be mistaken for a VERY_HIGH indication.
// Placeholder - pick any rate that reads clearly on the physical matrix.
#define AUTONOMOUS_MARKER_HALF_PERIOD_MS 100UL  // ~5 Hz blink

// Ring buffer for {t_ms, hr, band} events recorded while autonomous. 64 is a
// round power-of-two within the requested 50-100 range: at 12 bytes/entry
// (uint32 + float + uint8, padded) that's 768 bytes total. The STM32U585 has
// several hundred KB of SRAM and every other static buffer in this sketch
// (irBuf/redBuf/motionBuf/rrHistory) already totals under 1.5 KB, so this is
// a negligible addition, not a sizing risk.
#define AUTONOMOUS_LOG_CAPACITY 64
// Even with no band change, drop one entry at least this often while
// autonomous, so a long single-band outage still leaves a paper trail
// instead of one entry at the start and silence for the rest.
#define AUTONOMOUS_HEARTBEAT_MS 60000UL  // placeholder - tune to taste

// ---------------------------------------------------------------------------
// Sensor objects
// ---------------------------------------------------------------------------
MAX30105 ppgSensor;

bool maxOk = false;
bool tmp117Ok = false;
bool sht4Ok = false;
bool bmi323Ok = false;

// ---------------------------------------------------------------------------
// PPG rolling buffer (4 s) - used for SpO2 AC/DC statistics
// ---------------------------------------------------------------------------
float irBuf[PPG_WINDOW_SAMPLES];
float redBuf[PPG_WINDOW_SAMPLES];
uint16_t ppgIdx = 0;
uint16_t ppgCount = 0;
uint16_t ppgSamplesThisSecond = 0;  // FIFO samples drained since the last PPG health print

// ---------------------------------------------------------------------------
// Heart-rate streaming state (AC-coupled peak detector)
// ---------------------------------------------------------------------------
float hrBaseline = 0;
bool hrBaselineInit = false;
float hrEnvelope = 0;

bool hrHavePrev = false, hrHavePrevPrev = false;
float hrPrevAc = 0, hrPrevPrevAc = 0;

bool haveLastBeat = false;
unsigned long lastBeatMillis = 0;
unsigned long lastAcceptedBeatMillis = 0;

float rrHistory[RR_HISTORY_LEN];
uint8_t rrHistoryCount = 0;
uint8_t rrHistoryIdx = 0;

float currentHr = NAN;
bool currentHrValid = false;

// Ring buffer of accepted-beat timestamps, for the PPG health line's beats_30s.
#define BEAT_HISTORY_CAPACITY 128
unsigned long beatHistory[BEAT_HISTORY_CAPACITY];
uint8_t beatHistoryHead = 0;
uint8_t beatHistoryCount = 0;

// ---------------------------------------------------------------------------
// Motion (accel magnitude) rolling buffer (2 s)
// ---------------------------------------------------------------------------
float motionBuf[MOTION_WINDOW_SAMPLES];
uint16_t motionIdx = 0;
uint16_t motionCount = 0;

// ---------------------------------------------------------------------------
// Latest slow-sensor (1 Hz) readings
// ---------------------------------------------------------------------------
float skinC = NAN;
bool skinValid = false;
float ambC = NAN;
float rh = NAN;
bool envValid = false;
unsigned long dhtLastGoodMillis = 0;
unsigned long dhtLastAttemptMillis = 0;

unsigned long lastAccelMillis = 0;
unsigned long lastReportMillis = 0;

// ---------------------------------------------------------------------------
// Alert output state (buzzer pattern + LED + ack button)
// Driven from Python's "raise_alert" RPC; must never block, since the alert
// path (buzzer/LED/button) is owned by the M33 while Python keeps polling at
// 1 Hz through the same Bridge.
// ---------------------------------------------------------------------------
enum AlertBand { ALERT_NONE = 0, ALERT_LOW = 1, ALERT_MODERATE = 2, ALERT_HIGH = 3, ALERT_VERY_HIGH = 4 };

AlertBand activeAlertBand = ALERT_NONE;
AlertBand lastRenderedBand = ALERT_NONE;  // so static LED/matrix states redraw once per transition, not every loop()

Arduino_LED_Matrix matrix;

bool buzzerContinuous = false;     // true only while a VERY_HIGH alert is unacknowledged
uint8_t buzzTogglesRemaining = 0;  // remaining on/off toggles in the current one-shot pattern
bool buzzerOn = false;
unsigned long buzzNextToggleMillis = 0;

bool rgbFlashOn = false;           // VERY_HIGH: red on/off phase of the user RGB LED
unsigned long rgbNextToggleMillis = 0;
bool matrixFlashOn = false;        // VERY_HIGH: full-matrix on/off phase
unsigned long matrixNextToggleMillis = 0;

bool buttonLastRaw = HIGH;
bool buttonStableState = HIGH;
unsigned long buttonLastChangeMillis = 0;

// Marker-free matrix content (strain bar or VERY_HIGH full-flash), composed
// with the autonomous corner marker (if any) by renderMatrix() before every
// draw() call - see Stage 7 section below.
uint8_t matrixFrame[MATRIX_SIZE];

// ---------------------------------------------------------------------------
// Autonomous fail-safe state (Stage 7)
// ---------------------------------------------------------------------------
unsigned long lastBridgeContactMs = 0;  // last successful Python -> M33 call
bool autonomousMode = false;
bool lastRenderedAutonomousMode = false;  // forces one redraw on mode transitions
AlertBand autonomousActiveBand = ALERT_NONE;  // last band the autonomous grader latched

bool autonomousMarkerOn = false;
unsigned long autonomousMarkerNextToggleMillis = 0;

// HR-band thresholds cached from Python's set_thresholds(). RAM only - not
// persisted to flash (see set_thresholds() comment for why), so a fresh boot
// starts ungraded until Python calls in again.
int cachedHrLow = 0, cachedHrModerate = 0, cachedHrHigh = 0, cachedHrVeryHigh = 0;
bool thresholdsCached = false;

struct AutonomousLogEntry {
  uint32_t tMillis;
  float hr;    // NAN if HR was invalid at log time
  uint8_t band;  // AlertBand
};

AutonomousLogEntry autonomousLog[AUTONOMOUS_LOG_CAPACITY];
uint8_t autonomousLogHead = 0;             // index the NEXT entry will be written to
uint8_t autonomousLogCount = 0;            // valid unread entries, capped at capacity
uint16_t autonomousLogOverflowCount = 0;   // entries overwritten before being drained
unsigned long autonomousLastLogMillis = 0;

// ---------------------------------------------------------------------------
// TMP102 raw I2C driver
// (12-bit temperature register per TI TMP102 datasheet: TEMP=0x00)
// ---------------------------------------------------------------------------
bool tmp102Init() {
  Wire.beginTransmission(0x48);
  return Wire.endTransmission() == 0;
}

bool tmp102Read(float &tempC) {
  Wire.beginTransmission(0x48);
  Wire.write((uint8_t)0x00);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)0x48, (int)2);
  if (got != 2) return false;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  int16_t raw = (int16_t)((msb << 8) | lsb) >> 4;  // top 12 bits, sign-extended
  tempC = raw * 0.0625f;
  return true;
}

// ---------------------------------------------------------------------------
// MPU6050 raw I2C driver
// (register map per InvenSense MPU-6050 datasheet: PWR_MGMT_1=0x6B,
//  CONFIG=0x1A, ACCEL_CONFIG=0x1C, ACCEL_XOUT_H=0x3B, WHO_AM_I=0x75)
// ---------------------------------------------------------------------------
uint8_t mpu6050WriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission();  // 0 = success; nonzero = Wire error code
}

bool mpu6050ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  uint8_t got = Wire.requestFrom((int)MPU6050_ADDR, (int)len);
  if (got != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool mpu6050Init() {
  if (mpu6050WriteReg(0x6B, 0x00) != 0) return false;  // PWR_MGMT_1: wake from sleep
  delay(10);

  uint8_t who = 0;
  if (!mpu6050ReadRegs(0x75, &who, 1)) {  // WHO_AM_I
    Serial.println("MPU6050: WHO_AM_I read failed (no ACK)");
    return false;
  }
  Serial.print("MPU6050 WHO_AM_I = 0x");
  if (who < 0x10) Serial.print("0");
  Serial.println(who, HEX);

  // Many MPU6050 clones report a WHO_AM_I other than the datasheet 0x68. The
  // device already ACKed at 0x68, which is sufficient proof it's there, so an
  // unrecognized ID is logged and init continues rather than failing here.
  if (who != 0x68 && who != 0x70 && who != 0x72 && who != 0x73 && who != 0x98) {
    Serial.println("MPU6050: WHO_AM_I not in known set, continuing anyway (device ACKed on the bus)");
  }

  uint8_t writeCode = mpu6050WriteReg(0x1C, 0x00);  // ACCEL_CONFIG: +/-2g
  if (writeCode != 0) {
    Serial.print("MPU6050: write to 0x1C failed, code=");
    Serial.println(writeCode);
    return false;
  }

  writeCode = mpu6050WriteReg(0x1A, 0x03);  // CONFIG: 44Hz DLPF
  if (writeCode != 0) {
    Serial.print("MPU6050: write to 0x1A failed, code=");
    Serial.println(writeCode);
    return false;
  }

  delay(5);
  return true;
}

bool mpu6050Read(float &axg, float &ayg, float &azg) {
  uint8_t buf[6];
  if (!mpu6050ReadRegs(0x3B, buf, 6)) return false;  // ACCEL_XOUT_H..ACCEL_ZOUT_L

  int16_t rawAx = (int16_t)((buf[0] << 8) | buf[1]);
  int16_t rawAy = (int16_t)((buf[2] << 8) | buf[3]);
  int16_t rawAz = (int16_t)((buf[4] << 8) | buf[5]);

  axg = rawAx / 16384.0f;
  ayg = rawAy / 16384.0f;
  azg = rawAz / 16384.0f;
  return true;
}

// ---------------------------------------------------------------------------
// DHT11 raw single-wire driver (bit-banged - no library dependency)
// Protocol: host pulls the line low >=18ms then releases; the DHT11 pulls it
// low then high (~80us each) to ack, then sends 40 bits (RH int, RH dec,
// temp int, temp dec, checksum byte). Each bit is a ~50us LOW followed by a
// HIGH whose duration encodes the value: ~26-28us = 0, ~70us = 1.
// ---------------------------------------------------------------------------
bool dht11WaitForLevel(int level, unsigned long timeoutUs) {
  unsigned long start = micros();
  while (digitalRead(DHT11_PIN) != level) {
    if (micros() - start > timeoutUs) return false;
  }
  return true;
}

bool dht11Read(float &tempCOut, float &rhOut) {
  uint8_t data[5] = {0, 0, 0, 0, 0};
  const char *failReason = nullptr;
  int failBit = -1;

  Serial.print("DHT11 using pin ");
  Serial.println(DHT11_PIN);

  pinMode(DHT11_PIN, INPUT);
  bool dhtIdleHigh = digitalRead(DHT11_PIN) == HIGH;
  Serial.print("DHT11 idle line = ");
  Serial.println(dhtIdleHigh ? "HIGH" : "LOW");
  if (!dhtIdleHigh) {
    Serial.println("DHT11: line held low - check wiring");
    return false;
  }

  pinMode(DHT11_PIN, OUTPUT);
  digitalWrite(DHT11_PIN, LOW);
  delay(20);  // DHT11 needs >=18ms; delay() rather than delayMicroseconds() - this core's us delays may be inaccurate
  pinMode(DHT11_PIN, INPUT);  // let the fitted pull-up raise the line
  delayMicroseconds(40);

  unsigned long relDurationUs = 0, lowDurationUs = 0, highDurationUs2 = 0;

  // Interrupts off only for this window (~4-5ms): the ack handshake plus 40
  // bit-timings below are all sensitive to microsecond-scale scheduling jitter.
  noInterrupts();

  unsigned long relStart = micros();
  if (!dht11WaitForLevel(LOW, 300)) {  // sensor never pulls the line low at all (3x widened)
    relDurationUs = micros() - relStart;
    failReason = "no response to start pulse";
  } else {
    relDurationUs = micros() - relStart;
    unsigned long lowStart = micros();
    if (!dht11WaitForLevel(HIGH, 300)) {  // sensor's ~80us low ack pulse (3x widened)
      lowDurationUs = micros() - lowStart;
      failReason = "ack timeout";
    } else {
      lowDurationUs = micros() - lowStart;
      unsigned long highStart2 = micros();
      if (!dht11WaitForLevel(LOW, 300)) {  // sensor's ~80us high ack pulse (3x widened)
        highDurationUs2 = micros() - highStart2;
        failReason = "ack timeout";
      } else {
        highDurationUs2 = micros() - highStart2;

        for (uint8_t i = 0; i < 40; i++) {
          if (!dht11WaitForLevel(HIGH, 100)) {  // bit's low phase ends
            failReason = "bit timeout";
            failBit = i;
            break;
          }
          unsigned long highStart = micros();
          if (!dht11WaitForLevel(LOW, 100)) {   // bit's high phase ends
            failReason = "bit timeout";
            failBit = i;
            break;
          }
          unsigned long highDurationUs = micros() - highStart;

          data[i / 8] <<= 1;
          if (highDurationUs > 40) data[i / 8] |= 1;  // ~70us -> 1, ~26-28us -> 0
        }
      }
    }
  }

  interrupts();

  Serial.print("DHT11 timing: rel=");
  Serial.print(relDurationUs);
  Serial.print(" low=");
  Serial.print(lowDurationUs);
  Serial.print(" high=");
  Serial.println(highDurationUs2);

  // Raw bytes on every attempt, success or failure - unread bits stay 0.
  Serial.print("DHT11 raw: ");
  for (uint8_t i = 0; i < 5; i++) {
    if (data[i] < 0x10) Serial.print("0");
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();

  if (failReason != nullptr) {
    if (failBit >= 0) {
      Serial.print("DHT11: bit timeout at bit ");
      Serial.println(failBit);
    } else {
      Serial.print("DHT11: ");
      Serial.println(failReason);
    }
    return false;
  }

  uint8_t checksum = (uint8_t)(data[0] + data[1] + data[2] + data[3]);
  if (checksum != data[4]) {
    Serial.print("DHT11: checksum fail, got ");
    Serial.print(data[0]);
    Serial.print(" ");
    Serial.print(data[1]);
    Serial.print(" ");
    Serial.print(data[2]);
    Serial.print(" ");
    Serial.print(data[3]);
    Serial.print(" ");
    Serial.println(checksum);
    return false;
  }

  rhOut = (float)data[0];     // DHT11: integral RH only, decimal byte is always 0
  tempCOut = (float)data[2];  // DHT11: integral degC only, decimal byte is always 0

  Serial.print("DHT11: OK ");
  Serial.print(tempCOut, 0);
  Serial.print(" C ");
  Serial.print(rhOut, 0);
  Serial.println(" %");

  return true;
}

// ---------------------------------------------------------------------------
// Boot-time dual I2C bus scan (diagnostic only, runs first thing in setup())
// ---------------------------------------------------------------------------
bool bootScanSawTmp102 = false;
bool bootScanSawMax30102 = false;
bool bootScanSawMpu6050 = false;

void scanI2CBus(TwoWire &bus, const char *label) {
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      Serial.print(label);
      Serial.print(" FOUND: 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.println(addr, HEX);

      if (addr == 0x48) bootScanSawTmp102 = true;
      if (addr == 0x57) bootScanSawMax30102 = true;
      if (addr == 0x68) bootScanSawMpu6050 = true;
    }
  }
}

void bootI2cScan() {
  Wire.begin();
  scanI2CBus(Wire, "WIRE0");

#if defined(WIRE_INTERFACES_COUNT) && (WIRE_INTERFACES_COUNT > 1)
  Serial.println("WIRE1: compiled in");
  Wire1.begin();
  scanI2CBus(Wire1, "WIRE1");
#else
  Serial.println("WIRE1: not available on this core");
#endif

  Serial.println("SCAN COMPLETE");
}

// ---------------------------------------------------------------------------
// Repeating diagnostics (every DIAG_INTERVAL_MS from loop()): re-scans both
// I2C buses and re-attempts init for any sensor whose health flag is still
// false, since a sensor that failed at boot may come online later.
// ---------------------------------------------------------------------------
#define DIAG_INTERVAL_MS 20000UL
unsigned long lastDiagMillis = 0;

// Verbose per-bus sweep for runDiagnostics() only (bootI2cScan()'s scanI2CBus()
// is left as-is): sets the clock, always reports something for the bus even
// when nothing answers, and updates the same bootScanSaw* flags scanI2CBus()
// does so the INIT reason text below still reflects reality.
void scanBusVerbose(TwoWire &bus, const char *label) {
  bus.begin();
  bus.setClock(100000);

  bool foundAny = false;
  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    bus.beginTransmission(addr);
    if (bus.endTransmission() == 0) {
      foundAny = true;
      Serial.print(label);
      Serial.print(" FOUND: 0x");
      if (addr < 0x10) Serial.print("0");
      Serial.println(addr, HEX);

      if (addr == 0x48) bootScanSawTmp102 = true;
      if (addr == 0x57) bootScanSawMax30102 = true;
      if (addr == 0x68) bootScanSawMpu6050 = true;
    }
  }
  if (!foundAny) {
    Serial.print(label);
    Serial.println(": no devices");
  }
}

void runDiagnostics() {
  Serial.println("===== DIAG START =====");

  bootScanSawTmp102 = false;
  bootScanSawMax30102 = false;
  bootScanSawMpu6050 = false;

#if defined(SDA) && defined(SCL)
  Serial.print("HEADER SDA/SCL -> pins SDA=");
  Serial.print(SDA);
  Serial.print(" SCL=");
  Serial.print(SCL);
  Serial.println(" (bus object not derivable from these macros - see variant headers)");
#else
  Serial.println("HEADER SDA/SCL -> cannot determine: SDA/SCL pin macros not defined on this core");
#endif

  scanBusVerbose(Wire, "WIRE0");
  scanBusVerbose(Wire1, "WIRE1");
  scanBusVerbose(Wire2, "WIRE2");

  Serial.println("SCAN COMPLETE");

  if (!maxOk) {
    maxOk = ppgSensor.begin(Wire, I2C_SPEED_FAST, MAX30105_ADDRESS);
    if (maxOk) {
      ppgSensor.setup(0x3F, 2, 2, 50, 411, 16384);
    }
  }
  Serial.print("INIT max30102: ");
  if (maxOk) {
    Serial.println("OK");
  } else {
    Serial.println(bootScanSawMax30102
                        ? "FAIL (device present at 0x57 but init sequence failed)"
                        : "FAIL (no device found at 0x57 during scan)");
  }

  if (!tmp117Ok) {
    tmp117Ok = tmp102Init();
  }
  Serial.print("INIT tmp102: ");
  if (tmp117Ok) {
    Serial.println("OK");
  } else {
    Serial.println(bootScanSawTmp102
                        ? "FAIL (device present at 0x48 but init sequence failed)"
                        : "FAIL (no device found at 0x48 during scan)");
  }

  if (!sht4Ok) {
    float diagAmbC, diagRh;
    sht4Ok = dht11Read(diagAmbC, diagRh);
    if (sht4Ok) {
      ambC = diagAmbC;
      rh = diagRh;
      envValid = true;
      dhtLastGoodMillis = millis();
      dhtLastAttemptMillis = millis();
    }
  }
  Serial.print("INIT dht11: ");
  Serial.println(sht4Ok ? "OK" : "FAIL");

  if (!bmi323Ok) {
    bmi323Ok = mpu6050Init();
  }
  Serial.print("INIT mpu6050: ");
  if (bmi323Ok) {
    Serial.println("OK");
  } else {
    Serial.println(bootScanSawMpu6050
                        ? "FAIL (device present at 0x68 but init sequence failed)"
                        : "FAIL (no device found at 0x68 during scan)");
  }

  Serial.println("===== DIAG END =====");
}

// ---------------------------------------------------------------------------
// Motion magnitude
// ---------------------------------------------------------------------------
void pushMotionSample(float magG) {
  motionBuf[motionIdx] = magG;
  motionIdx = (motionIdx + 1) % MOTION_WINDOW_SAMPLES;
  if (motionCount < MOTION_WINDOW_SAMPLES) motionCount++;
}

const char *classifyMotion(float motionG) {
  if (motionG < 0.05f) return "STILL";
  if (motionG <= 0.25f) return "LIGHT";
  return "ACTIVE";
}

void getMotionForReport(float &motionGOut, const char *&stateOut) {
  if (!bmi323Ok || motionCount < 2) {
    motionGOut = NAN;
    stateOut = nullptr;
    return;
  }
  float sum = 0;
  for (uint16_t i = 0; i < motionCount; i++) sum += motionBuf[i];
  float mean = sum / motionCount;
  float sq = 0;
  for (uint16_t i = 0; i < motionCount; i++) {
    float d = motionBuf[i] - mean;
    sq += d * d;
  }
  motionGOut = sqrtf(sq / motionCount);
  stateOut = classifyMotion(motionGOut);
}

// ---------------------------------------------------------------------------
// Heart rate: running median of accepted RR intervals
// ---------------------------------------------------------------------------
float medianOfRrHistory() {
  float tmp[RR_HISTORY_LEN];
  for (uint8_t i = 0; i < rrHistoryCount; i++) tmp[i] = rrHistory[i];
  for (uint8_t i = 1; i < rrHistoryCount; i++) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }
  uint8_t n = rrHistoryCount;
  if (n % 2 == 1) return tmp[n / 2];
  return (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5f;
}

float averageRecentIntervals(uint8_t n) {
  uint8_t count = n < rrHistoryCount ? n : rrHistoryCount;
  if (count == 0) return NAN;
  float sum = 0;
  uint8_t idx = rrHistoryIdx;
  for (uint8_t i = 0; i < count; i++) {
    idx = (idx == 0) ? (RR_HISTORY_LEN - 1) : (idx - 1);
    sum += rrHistory[idx];
  }
  return sum / count;
}

void processPpgSample(uint32_t irRaw, uint32_t redRaw, unsigned long tMillis) {
  ppgSamplesThisSecond++;

  irBuf[ppgIdx] = (float)irRaw;
  redBuf[ppgIdx] = (float)redRaw;
  ppgIdx = (ppgIdx + 1) % PPG_WINDOW_SAMPLES;
  if (ppgCount < PPG_WINDOW_SAMPLES) ppgCount++;

  float ir = (float)irRaw;

  // Slow EMA baseline (~0.08 Hz cutoff) approximates the DC component; the
  // difference from it is the AC (pulsatile) component used for peak detection.
  const float BASELINE_ALPHA = 0.02f;
  if (!hrBaselineInit) {
    hrBaseline = ir;
    hrBaselineInit = true;
  } else {
    hrBaseline += BASELINE_ALPHA * (ir - hrBaseline);
  }
  float ac = ir - hrBaseline;

  const float ENV_ALPHA = 0.05f;
  hrEnvelope += ENV_ALPHA * (fabsf(ac) - hrEnvelope);

  if (hrHavePrev && hrHavePrevPrev) {
    bool isPeak = (hrPrevAc > hrPrevPrevAc) && (hrPrevAc >= ac);
    if (isPeak) {
      // The peak occurred at the previous sample (one PPG period ago).
      unsigned long peakMillis = tMillis - (1000UL / PPG_FS);
      float threshold = fmaxf(hrEnvelope * 0.5f, PPG_MIN_AC_AMPLITUDE);
      bool fingerPresent = hrBaseline > PPG_FINGER_DC_MIN;

      if (hrPrevAc > threshold && fingerPresent) {
        if (!haveLastBeat) {
          lastBeatMillis = peakMillis;
          haveLastBeat = true;
        } else if (peakMillis > lastBeatMillis) {
          unsigned long interval = peakMillis - lastBeatMillis;
          if (interval >= HR_REFRACTORY_MS) {
            bool accept = true;
            if (rrHistoryCount > 0) {
              float med = medianOfRrHistory();
              if (fabsf((float)interval - med) > 0.30f * med) accept = false;
            }
            if (accept) {
              rrHistory[rrHistoryIdx] = (float)interval;
              rrHistoryIdx = (rrHistoryIdx + 1) % RR_HISTORY_LEN;
              if (rrHistoryCount < RR_HISTORY_LEN) rrHistoryCount++;
              lastBeatMillis = peakMillis;
              lastAcceptedBeatMillis = peakMillis;

              beatHistory[beatHistoryHead] = peakMillis;
              beatHistoryHead = (beatHistoryHead + 1) % BEAT_HISTORY_CAPACITY;
              if (beatHistoryCount < BEAT_HISTORY_CAPACITY) beatHistoryCount++;

              float avgInterval = averageRecentIntervals(4);
              currentHr = 60000.0f / avgInterval;
              currentHrValid = true;
            }
            // else: rejected outlier beat - anchor stays at the last accepted beat
          }
          // else: within refractory period - ignored as noise/double-trigger
        }
      }
    }
  }

  hrHavePrevPrev = hrHavePrev;
  hrPrevPrevAc = hrPrevAc;
  hrHavePrev = true;
  hrPrevAc = ac;
}

// Counts accepted-beat timestamps within the last 30s. Order doesn't matter -
// every valid slot in beatHistory (wrapped or not) gets checked once.
uint8_t countBeatsInLast30s(unsigned long now) {
  uint8_t count = 0;
  for (uint8_t i = 0; i < beatHistoryCount; i++) {
    if (now - beatHistory[i] <= 30000UL) count++;
  }
  return count;
}

void getHrForReport(float &hrOut, bool &validOut) {
  bool fingerPresent = hrBaselineInit && hrBaseline > PPG_FINGER_DC_MIN;
  bool fresh = haveLastBeat && (millis() - lastAcceptedBeatMillis < HR_STALE_MS);
  if (maxOk && currentHrValid && fingerPresent && fresh) {
    hrOut = currentHr;
    validOut = true;
  } else {
    hrOut = NAN;
    validOut = false;
  }
}

// SpO2 via the empirical ratio-of-ratios formula from Maxim AN6409. This is
// an uncalibrated estimate, not a clinical measurement.
bool computeSpo2(float &spo2Out) {
  if (!maxOk || ppgCount < (PPG_FS * 2)) return false;

  float sumIr = 0, sumRed = 0;
  for (uint16_t i = 0; i < ppgCount; i++) {
    sumIr += irBuf[i];
    sumRed += redBuf[i];
  }
  float dcIr = sumIr / ppgCount;
  float dcRed = sumRed / ppgCount;
  if (dcIr < PPG_FINGER_DC_MIN || dcRed <= 0) return false;

  float varIr = 0, varRed = 0;
  for (uint16_t i = 0; i < ppgCount; i++) {
    float dIr = irBuf[i] - dcIr;
    float dRed = redBuf[i] - dcRed;
    varIr += dIr * dIr;
    varRed += dRed * dRed;
  }
  float acIr = sqrtf(varIr / ppgCount);
  float acRed = sqrtf(varRed / ppgCount);
  if (acIr < PPG_MIN_AC_FOR_SPO2) return false;

  bool fingerPresent = hrBaselineInit && hrBaseline > PPG_FINGER_DC_MIN;
  bool fresh = haveLastBeat && (millis() - lastAcceptedBeatMillis < HR_STALE_MS);
  if (!(currentHrValid && fingerPresent && fresh)) return false;

  float r = (acRed / dcRed) / (acIr / dcIr);
  float spo2 = -45.060f * r * r + 30.354f * r + 94.845f;
  if (spo2 < 0.0f) spo2 = 0.0f;
  if (spo2 > 100.0f) spo2 = 100.0f;
  spo2Out = spo2;
  return true;
}

// ---------------------------------------------------------------------------
// JSON building
// ---------------------------------------------------------------------------
void fmtFloatOrNull(char *out, size_t outSize, bool valid, float val, uint8_t decimals) {
  if (!valid || isnan(val)) {
    snprintf(out, outSize, "null");
    return;
  }
  char fmt[8];
  snprintf(fmt, sizeof(fmt), "%%.%uf", decimals);
  snprintf(out, outSize, fmt, val);
}

String buildVitalsJson(float hr, bool hrValid, float spo2, bool spo2Valid,
                        float motionG, const char *motionState) {
  char hrStr[16], spo2Str[16], skinStr[16], ambStr[16], rhStr[16], motionStr[16], motionStateStr[16];

  fmtFloatOrNull(hrStr, sizeof(hrStr), hrValid, hr, 1);
  fmtFloatOrNull(spo2Str, sizeof(spo2Str), spo2Valid, spo2, 1);
  fmtFloatOrNull(skinStr, sizeof(skinStr), skinValid, skinC, 2);
  fmtFloatOrNull(ambStr, sizeof(ambStr), envValid, ambC, 2);
  fmtFloatOrNull(rhStr, sizeof(rhStr), envValid, rh, 1);
  fmtFloatOrNull(motionStr, sizeof(motionStr), motionState != nullptr, motionG, 3);

  if (motionState != nullptr) {
    snprintf(motionStateStr, sizeof(motionStateStr), "\"%s\"", motionState);
  } else {
    snprintf(motionStateStr, sizeof(motionStateStr), "null");
  }

  char buf[320];
  snprintf(buf, sizeof(buf),
           "{\"hr\":%s,\"hr_valid\":%s,\"spo2\":%s,\"spo2_valid\":%s,"
           "\"skin_c\":%s,\"amb_c\":%s,\"rh\":%s,"
           "\"motion_g\":%s,\"motion_state\":%s,"
           "\"uptime_ms\":%lu}",
           hrStr, hrValid ? "true" : "false",
           spo2Str, spo2Valid ? "true" : "false",
           skinStr, ambStr, rhStr,
           motionStr, motionStateStr,
           millis());
  return String(buf);
}

// ---------------------------------------------------------------------------
// Bridge RPC: read_vitals
// ---------------------------------------------------------------------------
String read_vitals() {
  // Stage 7 liveness: any successful Python -> M33 call counts as contact.
  // This is the ONLY change in this function - sampling/return logic below
  // is untouched. Required here specifically: read_vitals() is Python's
  // only once-a-second call, so without this line the liveness timer would
  // never reset during completely normal operation and the board would
  // falsely declare itself autonomous every ~30s.
  lastBridgeContactMs = millis();

  float hr, spo2, motionG;
  bool hrValid, spo2Valid;
  const char *motionState;

  getHrForReport(hr, hrValid);
  spo2Valid = computeSpo2(spo2);
  getMotionForReport(motionG, motionState);

  return buildVitalsJson(hr, hrValid, spo2, spo2Valid, motionG, motionState);
}

// ---------------------------------------------------------------------------
// Bridge RPC: raise_alert / ack_alert
// ---------------------------------------------------------------------------

// User RGB LED "LED4": one GPIO per channel, active-LOW (see header comment
// for where this was confirmed).
void setUserLed(bool r, bool g, bool b) {
  digitalWrite(LED4_R, r ? LOW : HIGH);
  digitalWrite(LED4_G, g ? LOW : HIGH);
  digitalWrite(LED4_B, b ? LOW : HIGH);
}

// Fills a MATRIX_SIZE frame with a bar of `litRows` fully-bright rows, filled
// from the bottom row upward (like a level meter), the rest left dark.
void buildStrainBarFrame(uint8_t litRows, uint8_t *outFrame) {
  for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
    uint8_t brightness = (row >= (MATRIX_ROWS - litRows)) ? 7 : 0;
    for (uint8_t col = 0; col < MATRIX_COLS; col++) {
      outFrame[row * MATRIX_COLS + col] = brightness;
    }
  }
}

// Composes matrixFrame (the current strain bar / VERY_HIGH flash content)
// with the autonomous-mode corner marker, if active, and draws the result.
// All matrix writes go through this so the marker can never be silently
// clobbered by a band/flash redraw, or vice versa.
void renderMatrix() {
  uint8_t out[MATRIX_SIZE];
  for (uint16_t i = 0; i < MATRIX_SIZE; i++) out[i] = matrixFrame[i];
  if (autonomousMode) out[0] = autonomousMarkerOn ? 7 : 0;  // top-left pixel
  matrix.draw(out);
}

uint8_t litRowsForBand(AlertBand band) {
  switch (band) {
    case ALERT_LOW: return 2;
    case ALERT_MODERATE: return 4;
    case ALERT_HIGH: return 6;
    case ALERT_VERY_HIGH: return 8;
    default: return 0;
  }
}

uint8_t pulsesForBand(AlertBand band) {
  switch (band) {
    case ALERT_LOW: return 1;
    case ALERT_MODERATE: return 2;
    case ALERT_HIGH: return 3;
    default: return 0;  // ALERT_VERY_HIGH is continuous, not pulsed; ALERT_NONE is silent
  }
}

// Shared by raise_alert() (Python-driven) and serviceAutonomousGrading()
// (M33-driven, when Python is unreachable): latches the new band and
// (re)starts its buzzer pattern. Must return immediately - VERY_HIGH just
// latches state for serviceAlertOutputs() to keep flashing, it does not
// block here waiting for the button.
void activateAlertBand(AlertBand newBand) {
  activeAlertBand = newBand;

  buzzerContinuous = (newBand == ALERT_VERY_HIGH);
  if (buzzerContinuous) {
    buzzerOn = true;
    digitalWrite(BUZZER_PIN, HIGH);
  } else {
    buzzTogglesRemaining = pulsesForBand(newBand) * 2;  // each buzz = one ON phase + one OFF phase
    buzzerOn = false;
    digitalWrite(BUZZER_PIN, LOW);
    buzzNextToggleMillis = millis();  // fire the first toggle on the next loop() pass
  }
}

// Called by Python once the graded alert logic decides to notify.
void raise_alert(String band) {
  lastBridgeContactMs = millis();

  AlertBand newBand;
  if (band == "LOW") newBand = ALERT_LOW;
  else if (band == "MODERATE") newBand = ALERT_MODERATE;
  else if (band == "HIGH") newBand = ALERT_HIGH;
  else if (band == "VERY_HIGH") newBand = ALERT_VERY_HIGH;
  else {
    Serial.print("raise_alert: unknown band ");
    Serial.println(band);
    return;
  }

  activateAlertBand(newBand);

  Serial.print("ALERT raised: ");
  Serial.println(band);
}

// ---------------------------------------------------------------------------
// Bridge RPC: set_thresholds / drain_autonomous_log (Stage 7)
// ---------------------------------------------------------------------------

// Called by Python at startup and whenever its HR-band reference changes
// meaningfully, so the M33 always holds a reasonably current copy to grade
// against if it ever has to act on its own. int, not float: only
// int/bool/String/vector<int> Bridge.provide() parameter types are
// demonstrated in the local examples (color-your-leds' set_led3_color takes
// int r,g,b) - float args are unconfirmed, so thresholds are sent as whole
// bpm rather than guessing an unverified marshaling type.
//
// RAM only, deliberately not written to flash/EEPROM: this project has no
// confirmed flash-write API for this Zephyr-based core in any local example,
// and the goal here is surviving Python dying while the M33 keeps running,
// which needs no persistence. A fresh M33 boot starts ungraded
// (thresholdsCached=false) until Python's next set_thresholds() call -
// placeholder / needs your call if surviving an M33 reboot too matters.
void set_thresholds(int hr_low, int hr_moderate, int hr_high, int hr_very_high) {
  lastBridgeContactMs = millis();

  cachedHrLow = hr_low;
  cachedHrModerate = hr_moderate;
  cachedHrHigh = hr_high;
  cachedHrVeryHigh = hr_very_high;
  thresholdsCached = true;

  Serial.print("Thresholds cached: LOW>=");
  Serial.print(hr_low);
  Serial.print(" MODERATE>=");
  Serial.print(hr_moderate);
  Serial.print(" HIGH>=");
  Serial.print(hr_high);
  Serial.print(" VERY_HIGH>=");
  Serial.println(hr_very_high);
}

const char *bandName(AlertBand band) {
  switch (band) {
    case ALERT_LOW: return "LOW";
    case ALERT_MODERATE: return "MODERATE";
    case ALERT_HIGH: return "HIGH";
    case ALERT_VERY_HIGH: return "VERY_HIGH";
    default: return "NONE";
  }
}

// Grades the current HR against the cached thresholds using the same band
// names used everywhere else. Returns NONE (never fabricates a grade) if no
// thresholds have been cached yet.
AlertBand gradeHrAutonomous(float hr) {
  if (!thresholdsCached) return ALERT_NONE;
  if (hr >= cachedHrVeryHigh) return ALERT_VERY_HIGH;
  if (hr >= cachedHrHigh) return ALERT_HIGH;
  if (hr >= cachedHrModerate) return ALERT_MODERATE;
  if (hr >= cachedHrLow) return ALERT_LOW;
  return ALERT_NONE;
}

// Appends one {t_ms, hr, band} event, overwriting the oldest unread entry if
// the ring is full (autonomousLogOverflowCount tracks this so an overrun is
// visible to Python/Serial rather than silently dropped).
void appendAutonomousLogEntry(float hr, AlertBand band) {
  AutonomousLogEntry &slot = autonomousLog[autonomousLogHead];
  slot.tMillis = (uint32_t)millis();
  slot.hr = hr;
  slot.band = (uint8_t)band;

  autonomousLogHead = (autonomousLogHead + 1) % AUTONOMOUS_LOG_CAPACITY;
  if (autonomousLogCount < AUTONOMOUS_LOG_CAPACITY) {
    autonomousLogCount++;
  } else {
    autonomousLogOverflowCount++;
  }
  autonomousLastLogMillis = millis();
}

// Called by Python on startup and on reconnect to recover whatever happened
// while it was gone. Returns all buffered events oldest-first as JSON and
// clears the buffer - same String+JSON return convention as read_vitals(),
// not a new/unconfirmed marshaling type.
//
// "now_ms" (this call's own millis(), same field name pattern as
// read_vitals()'s "uptime_ms") lets Python anchor every event's boot-relative
// t_ms to wall-clock time using a timestamp taken at this exact instant,
// rather than reusing a stale anchor from before the outage.
String drain_autonomous_log() {
  lastBridgeContactMs = millis();
  unsigned long nowMs = millis();

  uint8_t count = autonomousLogCount;
  // Not yet wrapped: oldest is at index 0. Full/wrapped: the next write slot
  // (head) is exactly the oldest unread entry, since we always overwrite the
  // oldest first.
  uint8_t startIdx = (autonomousLogCount < AUTONOMOUS_LOG_CAPACITY) ? 0 : autonomousLogHead;

  String out = "{\"now_ms\":";
  out += String(nowMs);
  out += ",\"overflow_count\":";
  out += String(autonomousLogOverflowCount);
  out += ",\"events\":[";

  for (uint8_t i = 0; i < count; i++) {
    uint8_t idx = (startIdx + i) % AUTONOMOUS_LOG_CAPACITY;
    AutonomousLogEntry &e = autonomousLog[idx];
    if (i > 0) out += ",";
    out += "{\"t_ms\":";
    out += String(e.tMillis);
    out += ",\"hr\":";
    if (isnan(e.hr)) out += "null";
    else out += String(e.hr, 1);
    out += ",\"band\":\"";
    out += bandName((AlertBand)e.band);
    out += "\"}";
  }
  out += "]}";

  autonomousLogCount = 0;
  autonomousLogHead = 0;
  autonomousLogOverflowCount = 0;

  return out;
}

// Runs only while autonomousMode is true (see loop()). Grades the current HR
// against the cached thresholds and, on a band change, drives the same
// buzzer/LED/matrix path raise_alert() uses - one event per change, not a
// flood - and logs the transition. Also drops a heartbeat log entry at most
// once per AUTONOMOUS_HEARTBEAT_MS even without a change, so a long
// single-band outage still leaves a trail instead of one entry and silence.
void serviceAutonomousGrading() {
  float hr;
  bool hrValid;
  getHrForReport(hr, hrValid);

  AlertBand graded = hrValid ? gradeHrAutonomous(hr) : ALERT_NONE;
  unsigned long now = millis();
  float loggedHr = hrValid ? hr : NAN;

  if (graded != autonomousActiveBand) {
    autonomousActiveBand = graded;
    activateAlertBand(graded);
    appendAutonomousLogEntry(loggedHr, graded);
  } else if (now - autonomousLastLogMillis >= AUTONOMOUS_HEARTBEAT_MS) {
    appendAutonomousLogEntry(loggedHr, graded);
  }
}

// Non-blocking servicer for the buzzer pattern, the user RGB LED colour, and
// the LED matrix strain bar; call every loop() iteration. Static (non
// VERY_HIGH) LED/matrix states are only redrawn on a band transition; the
// VERY_HIGH flashes are re-serviced on their own schedules every call.
void serviceAlertOutputs() {
  unsigned long now = millis();

  // --- Buzzer: one-shot pattern for LOW/MODERATE/HIGH, continuous tone for VERY_HIGH ---
  if (buzzerContinuous) {
    digitalWrite(BUZZER_PIN, HIGH);
  } else if (buzzTogglesRemaining > 0 && now >= buzzNextToggleMillis) {
    buzzerOn = !buzzerOn;
    digitalWrite(BUZZER_PIN, buzzerOn ? HIGH : LOW);
    buzzTogglesRemaining--;
    buzzNextToggleMillis = now + (buzzerOn ? BUZZ_ON_MS : BUZZ_OFF_MS);
  }

  // --- User RGB LED + LED matrix ---
  bool bandChanged = (activeAlertBand != lastRenderedBand);

  if (activeAlertBand == ALERT_VERY_HIGH) {
    if (bandChanged || now >= rgbNextToggleMillis) {
      rgbFlashOn = bandChanged ? true : !rgbFlashOn;
      rgbNextToggleMillis = now + RGB_FLASH_HALF_PERIOD_MS;
      setUserLed(rgbFlashOn, false, false);  // flashing red
    }
    if (bandChanged || now >= matrixNextToggleMillis) {
      matrixFlashOn = bandChanged ? true : !matrixFlashOn;
      matrixNextToggleMillis = now + MATRIX_FLASH_HALF_PERIOD_MS;
      for (uint16_t i = 0; i < MATRIX_SIZE; i++) matrixFrame[i] = matrixFlashOn ? 7 : 0;
      renderMatrix();
    }
  } else if (bandChanged) {
    switch (activeAlertBand) {
      case ALERT_LOW: setUserLed(false, true, false); break;       // green
      case ALERT_MODERATE: setUserLed(true, true, false); break;   // amber (red+green)
      case ALERT_HIGH: setUserLed(true, false, false); break;      // red
      default: setUserLed(false, false, false); break;             // NONE: off
    }
    buildStrainBarFrame(litRowsForBand(activeAlertBand), matrixFrame);
    renderMatrix();
  }

  lastRenderedBand = activeAlertBand;

  // --- Autonomous-mode marker: top-left matrix pixel, ~5 Hz blink, present
  // only while autonomousMode is true. Independent of the band/flash timers
  // above, so it must trigger its own redraw both mid-blink and on every
  // mode transition (entry snaps it on immediately; exit forces one more
  // redraw so it actually disappears rather than lingering until the next
  // unrelated band change). ---
  bool autonomousChanged = (autonomousMode != lastRenderedAutonomousMode);
  if (autonomousChanged) {
    autonomousMarkerOn = autonomousMode;
    autonomousMarkerNextToggleMillis = now + AUTONOMOUS_MARKER_HALF_PERIOD_MS;
    renderMatrix();
  } else if (autonomousMode && now >= autonomousMarkerNextToggleMillis) {
    autonomousMarkerOn = !autonomousMarkerOn;
    autonomousMarkerNextToggleMillis = now + AUTONOMOUS_MARKER_HALF_PERIOD_MS;
    renderMatrix();
  }
  lastRenderedAutonomousMode = autonomousMode;
}

// Clears the active alert locally (buzzer + LED + matrix) and tells Python so
// it can log the acknowledgement; fire-and-forget since the M33 doesn't need
// a response and must not block on the Python-side SQLite write.
void handleAckButtonPress() {
  if (activeAlertBand == ALERT_NONE) return;

  activeAlertBand = ALERT_NONE;
  buzzerContinuous = false;
  buzzTogglesRemaining = 0;
  buzzerOn = false;
  digitalWrite(BUZZER_PIN, LOW);
  setUserLed(false, false, false);
  buildStrainBarFrame(0, matrixFrame);
  renderMatrix();  // through the shared path, so an autonomous marker survives an ack
  lastRenderedBand = activeAlertBand;

  Bridge.notify("ack_alert");
  Serial.println("Alert acknowledged via button");
}

// Debounced poll of the acknowledge button; call every loop() iteration.
void pollAckButton() {
  bool raw = digitalRead(ACK_BUTTON_PIN);
  unsigned long now = millis();

  if (raw != buttonLastRaw) {
    buttonLastChangeMillis = now;
    buttonLastRaw = raw;
  }

  if ((now - buttonLastChangeMillis) > BUTTON_DEBOUNCE_MS && raw != buttonStableState) {
    buttonStableState = raw;
    bool pressed = ACK_BUTTON_ACTIVE_LOW ? (buttonStableState == LOW) : (buttonStableState == HIGH);
    if (pressed) handleAckButtonPress();
  }
}

// ---------------------------------------------------------------------------
// Serial status line
// ---------------------------------------------------------------------------
void printStatusLine(float hr, bool hrValid, float spo2, bool spo2Valid,
                      float motionG, const char *motionState) {
  Serial.print("[");
  Serial.print(millis());
  Serial.print(" ms] HR: ");
  if (hrValid) {
    Serial.print(hr, 1);
    Serial.print(" bpm");
  } else {
    Serial.print("-- (invalid)");
  }

  Serial.print(" | SpO2: ");
  if (spo2Valid) {
    Serial.print(spo2, 1);
    Serial.print(" %");
  } else {
    Serial.print("-- (invalid)");
  }

  Serial.print(" | Skin: ");
  if (skinValid) {
    Serial.print(skinC, 2);
    Serial.print(" C");
  } else {
    Serial.print("-- (invalid)");
  }

  Serial.print(" | Amb: ");
  if (envValid) {
    Serial.print(ambC, 2);
    Serial.print(" C, RH ");
    Serial.print(rh, 1);
    Serial.print(" %");
  } else {
    Serial.print("-- (invalid)");
  }

  Serial.print(" | Motion: ");
  if (motionState != nullptr) {
    Serial.print(motionG, 3);
    Serial.print(" g ");
    Serial.print(motionState);
  } else {
    Serial.print("-- (invalid)");
  }

  Serial.println();
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  delay(6000);  // let the Serial Monitor attach before the boot diagnostics print

  bootI2cScan();

  Bridge.begin();
  Bridge.provide("read_vitals", read_vitals);
  Bridge.provide("raise_alert", raise_alert);
  Bridge.provide("set_thresholds", set_thresholds);
  Bridge.provide("drain_autonomous_log", drain_autonomous_log);

  // Liveness starts counting from boot, not from an unset 0 vs. an unset
  // millis() near 0 - both are ~0 at this point, so the 30s grace period
  // naturally covers Python's own startup time before autonomous mode could
  // ever falsely trigger.
  lastBridgeContactMs = millis();

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(ACK_BUTTON_PIN, ACK_BUTTON_ACTIVE_LOW ? INPUT_PULLUP : INPUT);
  buttonLastRaw = digitalRead(ACK_BUTTON_PIN);
  buttonStableState = buttonLastRaw;

  pinMode(LED4_R, OUTPUT);
  pinMode(LED4_G, OUTPUT);
  pinMode(LED4_B, OUTPUT);
  setUserLed(false, false, false);

  matrix.begin();
  matrix.setGrayscaleBits(3);
  matrix.clear();

  Wire.begin();

  maxOk = ppgSensor.begin(Wire, I2C_SPEED_FAST, MAX30105_ADDRESS);
  if (maxOk) {
    // powerLevel=0x1F (~6.2 mA/LED), sampleAverage=4, ledMode=2 (Red+IR),
    // sampleRate=100 sps -> averaged FIFO output rate = 100/4 = 25 Hz.
    ppgSensor.setup(0x1F, 4, 2, 100, 411, 16384);
  }
  Serial.print("INIT max30102: ");
  if (maxOk) {
    Serial.println("OK");
  } else {
    Serial.println(bootScanSawMax30102
                        ? "FAIL (device present at 0x57 but init sequence failed)"
                        : "FAIL (no device found at 0x57 during boot scan)");
  }

  tmp117Ok = tmp102Init();
  Serial.print("INIT tmp102: ");
  if (tmp117Ok) {
    Serial.println("OK");
  } else {
    Serial.println(bootScanSawTmp102
                        ? "FAIL (device present at 0x48 but init sequence failed)"
                        : "FAIL (no device found at 0x48 during boot scan)");
  }

  {
    float bootAmbC, bootRh;
    sht4Ok = dht11Read(bootAmbC, bootRh);
    if (sht4Ok) {
      ambC = bootAmbC;
      rh = bootRh;
      envValid = true;
      dhtLastGoodMillis = millis();
    }
    dhtLastAttemptMillis = millis();
  }
  Serial.print("INIT dht11: ");
  Serial.println(sht4Ok ? "OK" : "FAIL");

  bmi323Ok = mpu6050Init();
  Serial.print("INIT mpu6050: ");
  if (bmi323Ok) {
    Serial.println("OK");
  } else {
    Serial.println(bootScanSawMpu6050
                        ? "FAIL (device present at 0x68 but init sequence failed)"
                        : "FAIL (no device found at 0x68 during boot scan)");
  }

  unsigned long now = millis();
  lastAccelMillis = now;
  lastReportMillis = now;
  lastDiagMillis = now;
}

void loop() {
  unsigned long now = millis();

  // runDiagnostics() call disabled: sweeping 3 I2C buses inside loop() was
  // stalling the MAX30102 FIFO drain and breaking peak-detection timing.
  // Function kept, just not called - see runDiagnostics() above.

  // Stage 7: if Python hasn't successfully called in for BRIDGE_TIMEOUT_MS,
  // grade HR locally instead of waiting for a raise_alert() that will never
  // come. Leaves autonomous mode the instant contact resumes.
  bool wasAutonomous = autonomousMode;
  autonomousMode = (now - lastBridgeContactMs) > BRIDGE_TIMEOUT_MS;
  if (autonomousMode != wasAutonomous) {
    Serial.println(autonomousMode
                        ? "AUTONOMOUS MODE: Bridge contact lost, grading HR locally"
                        : "Bridge contact resumed - leaving autonomous mode");
  }
  if (autonomousMode) {
    serviceAutonomousGrading();
  }

  // Alert output path: non-blocking buzzer/LED/matrix pattern + debounced ack button.
  serviceAlertOutputs();
  pollAckButton();

  // Drain PPG FIFO as fast as possible; the sensor paces the effective 25 Hz
  // rate internally via on-chip sample averaging.
  if (maxOk) {
    ppgSensor.check();
    while (ppgSensor.available()) {
      uint32_t red = ppgSensor.getFIFORed();
      uint32_t ir = ppgSensor.getFIFOIR();
      processPpgSample(ir, red, now);
      ppgSensor.nextSample();
    }
  }

  // Accel/gyro at 25 Hz.
  if (bmi323Ok && (now - lastAccelMillis >= ACCEL_PERIOD_MS)) {
    lastAccelMillis += ACCEL_PERIOD_MS;
    float ax, ay, az;
    if (mpu6050Read(ax, ay, az)) {
      pushMotionSample(sqrtf(ax * ax + ay * ay + az * az));
    }
  }

  // Slow sensors (1 Hz) + status report.
  if (now - lastReportMillis >= REPORT_PERIOD_MS) {
    lastReportMillis += REPORT_PERIOD_MS;

    if (sht4Ok) {
      if (now - dhtLastAttemptMillis >= DHT11_READ_INTERVAL_MS) {
        dhtLastAttemptMillis = now;
        float dhtAmbC, dhtRh;
        if (dht11Read(dhtAmbC, dhtRh)) {
          ambC = dhtAmbC;
          rh = dhtRh;
          dhtLastGoodMillis = now;
          envValid = true;
        } else if (now - dhtLastGoodMillis > DHT11_STALE_MS) {
          envValid = false;  // checksum/timeout and the last good value is too old to keep reporting
        }
        // else: read failed but the last good reading is still within its
        // staleness window - keep envValid/ambC/rh as they are.
      }
    } else {
      envValid = false;
    }

    if (tmp117Ok) {
      skinValid = tmp102Read(skinC);
    } else {
      skinValid = false;
    }

    float hr, spo2, motionG;
    bool hrValid, spo2Valid;
    const char *motionState;
    getHrForReport(hr, hrValid);
    spo2Valid = computeSpo2(spo2);
    getMotionForReport(motionG, motionState);

    printStatusLine(hr, hrValid, spo2, spo2Valid, motionG, motionState);

    uint16_t ppgSamples1s = ppgSamplesThisSecond;
    ppgSamplesThisSecond = 0;
    Serial.print("PPG: ir_dc=");
    Serial.print(hrBaseline, 0);
    Serial.print(" samples_1s=");
    Serial.print(ppgSamples1s);
    Serial.print(" beats_30s=");
    Serial.print(countBeatsInLast30s(now));
    Serial.print(" last_beat_ms_ago=");
    Serial.println(now - lastAcceptedBeatMillis);
  }
}
