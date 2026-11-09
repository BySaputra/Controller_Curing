

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <esp_task_wdt.h>

// ================================================================
// BLOCK 01 - CONFIGURATION
// ================================================================

#define OLED_SDA     21   // GPIO SDA I2C
#define OLED_SCL     22   // GPIO SCL I2C
#define OLED_RESET  -1    // Reset pin (-1 jika berbagi dengan reset ESP32)
#define SCREEN_WIDTH 128  // Lebar OLED pixel
#define SCREEN_HEIGHT 64  // Tinggi OLED pixel
#define OLED_ADDRESS 0x3C // Alamat I2C umum OLED 0.96"

#define ENCODER_CLK  18   // GPIO Encoder CLK / Phase A
#define ENCODER_DT   19   // GPIO Encoder DT / Phase B
#define ENCODER_SW   23   // GPIO Encoder Push Button (SW)

#define SSR_HEATER_PIN         25   // GPIO Control SSR Heater via NPN transistor switch
#define FAN_RELAY_PIN          26   // GPIO Control Relay Module Fan
#define FAN_RELAY_ACTIVE_LOW   true // true: Active LOW (LOW=ON, HIGH=OFF), false: Active HIGH

#define MAX6675_SCK            32   // GPIO Serial Clock MAX6675
#define MAX6675_CS             27   // GPIO Chip Select MAX6675
#define MAX6675_SO             34   // GPIO Serial Data Out MAX6675 (Input Only pada ESP32)

#define BUZZER_PIN             33   // GPIO Output Active/Passive Buzzer Driver

#define WDT_TIMEOUT_SECONDS    5    // Reboot MCU jika loop() tidak feed WDT dalam 5 detik

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ================================================================
// BLOCK 02 - SYSTEM STATE & DATA
// ================================================================

enum EncoderEvent {
  ENCODER_NONE,
  ENCODER_CW,
  ENCODER_CCW,
  ENCODER_PRESS
};

enum ScreenState {
  SCREEN_BOOT,
  SCREEN_HOME,
  SCREEN_RUNNING,
  SCREEN_MAIN_MENU,
  SCREEN_START_CONFIRM,
  SCREEN_SETPOINT_EDIT,
  SCREEN_TIMER,
  SCREEN_PID_MENU,
  SCREEN_PID_EDIT,
  SCREEN_MANUAL_MENU,
  SCREEN_MANUAL_HEATER,
  SCREEN_MANUAL_FAN,
  SCREEN_CALIBRATION,
  SCREEN_SYSTEM_MENU,
  SCREEN_SYSTEM_SENSOR,
  SCREEN_SYSTEM_DEVICE,
  SCREEN_SYSTEM_VERSION,
  SCREEN_SYSTEM_RESET_CONFIRM
};

enum BuzzerPattern {
  BUZZER_OVER_TEMP,     // Priority 1 (tertinggi): Over-Temp Safety Alarm
  BUZZER_SENSOR_ERROR,  // Priority 2: Sensor Error Alarm
  BUZZER_FINISH,        // Priority 3: Curing Finished (3x Beeps)
  BUZZER_START,         // Priority 4: Start Curing (2x Quick Beeps)
  BUZZER_SAVE,          // Priority 5: Setting Saved (1x Confirmation Beep)
  BUZZER_PRESS,         // Priority 6 (terendah): Rotary Press (1x Short Beep)
  BUZZER_IDLE           // Bukan pattern aktif
};

enum SafetyState {
  SAFETY_OK,
  SAFETY_SENSOR_ERROR,
  SAFETY_OVER_TEMP
};

// UI Management Variables
ScreenState currentScreen = SCREEN_BOOT;
bool displayNeedsUpdate = true;
unsigned long bootStartTime = 0;
const unsigned long BOOT_DURATION_MS = 2500; // Durasi layar boot 2.5 detik

const unsigned long UI_INACTIVITY_TIMEOUT_MS = 10000; // Timeout 10 detik auto return
unsigned long lastUIActivityMs = 0;

// System State Variables
bool systemRunning = false;     // True saat status HEATING / RUNNING

// Safety System Config & Variables
const float MAX_SAFE_TEMP = 150.0;            // Batas Maksimum Temperatur Keselamatan (°C)
const float OVER_TEMP_HYSTERESIS = 5.0;       // Suhu harus turun MAX_SAFE_TEMP - hysteresis sebelum boleh recover
const int   SENSOR_OK_STREAK_REQUIRED = 5;    // Jumlah pembacaan valid berturut-turut sebelum sensor error dianggap pulih
SafetyState currentSafetyState = SAFETY_OK;   // Status Keselamatan Awal
bool heaterAllowed = true;                    // Flag Izin Pengontrolan Heater (true = Boleh ON)
bool overTempFaultLatched = false;            // Fault over-temp terkunci sampai user acknowledge & suhu turun
bool sensorErrorFaultLatched = false;         // Fault sensor terkunci sampai streak valid terpenuhi
int  sensorOkStreak = 0;                      // Penghitung pembacaan valid berturut-turut

// Buzzer Engine Variables
BuzzerPattern currentBuzzerPattern = BUZZER_IDLE;
unsigned long buzzerTimerMs = 0;
int buzzerStep = 0;
bool buzzerState = false;

// MAX6675 & Temperature Variables
float currentTemperature = 0.0;      // Suhu terfilter akhir (°C) yang digunakan oleh PID & Safety
float rawTemperature = 0.0;          // Bacaan suhu mentah dari MAX6675 (°C)
float calibratedTemperature = 0.0;   // Suhu setelah penambahan calibrationOffset (°C)
float medianTemperature = 0.0;       // Suhu setelah Median Filter (5 sampel) (°C)
float filteredTemperature = 0.0;     // Suhu setelah EMA Filter (°C)
bool sensorError = false;            // Flag error thermocouple/MAX6675 (true = Error/Disconnected)
bool filterInitialized = false;      // Status inisialisasi filter EMA

// Median Filter Buffer & Config
const int MEDIAN_FILTER_SIZE = 5;
float medianBuffer[MEDIAN_FILTER_SIZE];
int medianBufferIndex = 0;
int medianBufferCount = 0;

const unsigned long SENSOR_READ_INTERVAL_MS = 250;  // Interval pembacaan sensor non-blocking (250 ms)
const float EMA_ALPHA = 0.20;                        // Faktor smoothing Exponential Moving Average (0.20)

unsigned long lastSensorReadMs = 0;

// PID Controller & Time-Proportional Control Variables
float pidOutput = 0.0;                 // Output PID 0.0 - 100.0 %
float pidIntegralTerm = 0.0;           // Akumulasi komponen Integral
float lastPidInput = 0.0;              // Temperatur terfilter sebelumnya untuk Derivative
unsigned long lastPidComputeMs = 0;    // Waktu kalkulasi PID terakhir (ms)
const unsigned long PID_COMPUTE_INTERVAL_MS = 250; // Interval komputasi PID (250 ms)
float pidPTerm = 0.0;                  // Komponen Proportional terakhir (untuk logging)
float pidDTerm = 0.0;                  // Komponen Derivative terakhir (untuk logging)
bool  pidIntegralActive = false;       // Flag: apakah integral sedang aktif (untuk logging)
bool  pidFirstRun = true;              // Flag inisialisasi first-run PID (global)

// Anti-Windup: Conditional Integration (Clamping) pada batas saturasi 0-100%.
// Integral BOLEH unwind (turun) saat overshoot — tidak diblokir oleh zona aktif kaku.

const unsigned long SSR_WINDOW_MS = 2000;   // Total window time 2000 ms (2 detik)
unsigned long ssrWindowStartTime = 0;       // Waktu awal window SSR saat ini (ms)
unsigned long ssrWindowOnTimeMs = 0;        // ON-time yang diaplikasikan pada window SAAT INI

const unsigned long CSV_LOG_INTERVAL_MS = 1000; // Interval logging CSV 1000 ms (1 detik)
unsigned long lastCsvLogMs = 0;

// Parameter Kontrol (Disimpan di NVS)
float setpoint = 60.0;           // Default setpoint (°C)
const float SETPOINT_MIN = 30.0; // Batas minimum setpoint (°C)
const float SETPOINT_MAX = 120.0;// Batas maksimum setpoint (°C)

float Kp = 1.50;                 // Parameter PID - Proportional (Konservatif awal)
float Ki = 0.075;                 // Parameter PID - Integral (Konservatif awal)
float Kd = 1.00;                 // Parameter PID - Derivative (Konservatif awal)

bool heaterManual = false;       // Manual flag heater
bool fanManual = false;          // Manual flag fan

bool heaterState = false;        // Status fisik SSR Heater (true = ON, false = OFF)
bool fanState = false;           // Status fisik Relay Fan (true = ON, false = OFF)
float heaterOutputPercent = 0.0; // Output heater aktual yang ditampilkan (0.0 - 100.0 %)

Preferences preferences;

// Timer Control Variables
uint32_t timerDurationMinutes = 30; // Default durasi timer (menit)
uint32_t tempTimerMinutes = 30;     // Temporary edit timer (menit)

// Running Process Variables
unsigned long curingStartTime = 0;      // Waktu awal proses curing dimulai (ms)
uint32_t curingTotalSeconds = 1800;     // Total durasi proses curing (detik)
uint32_t curingRemainingSeconds = 1800; // Sisa waktu proses curing (detik)

// Calibration Control Variables
float calibrationOffset = 0.0;     // Default offset kalibrasi (°C)
float tempCalibOffset = 0.0;       // Temporary edit offset kalibrasi (°C)
bool isCalibSavedMessage = false;  // Flag pesan tersimpan
unsigned long calibSaveMsgStartTime = 0;

// Temporary Edit Variables
float tempSetpoint = 0.0;
float tempPIDValue = 0.0;
int selectedPIDParam = 0;        // 0: Kp, 1: Ki, 2: Kd
int confirmOption = 0;           // 0: YES, 1: NO
int manualToggleOption = 0;      // 0: OFF, 1: ON

// Carousel & Navigation Indexes
int mainMenuCursor = 0;
const int MAIN_MENU_COUNT = 7;
const char* mainMenuItems[] = {
  "START",
  "SETPOINT",
  "TIMER",
  "PID",
  "MANUAL",
  "CALIBRATION",
  "SYSTEM"
};

int pidMenuCursor = 0;
const int PID_MENU_COUNT = 4;

int manualMenuCursor = 0;
const int MANUAL_MENU_COUNT = 3;

int systemMenuCursor = 0;
const int SYSTEM_MENU_COUNT = 5;
const char* systemMenuItems[] = {
  "SENSOR",
  "DEVICE",
  "VERSION",
  "RISET",
  "BACK"
};

int resetConfirmOption = 1;        // 0: YES, 1: NO (Default NO)
bool isResetDoneMessage = false;   // Flag notifikasi reset selesai
unsigned long resetDoneMsgStartTime = 0;

// Function Prototypes
void emergencyPinsSafeState();
void setupEncoder();
EncoderEvent readEncoder();
void handleInput();
void updateUI();
void resetUIInactivityTimer();
void checkUIInactivity();
void handleEncoderEvent(EncoderEvent event);
void handleHomeInput(EncoderEvent event);
void handleRunningInput(EncoderEvent event);
void handleTimerInput(EncoderEvent event);
void handleCalibrationInput(EncoderEvent event);
void handleStartInput(EncoderEvent event);
void handleSetpointInput(EncoderEvent event);
void updateDisplay();
void renderScreen();
void showBootScreen();
void showHomeScreen();
void showRunningScreen();
void showMainMenu();
void showStartConfirmScreen();
void showSetpointScreen();
void showTimerScreen();
void showPIDMenu();
void showPIDEditScreen();
void showManualMenu();
void showManualToggleScreen(const char* title, int currentVal);
void showCalibrationScreen();
void showSystemMenu();
void showSystemSensorScreen();
void showSystemDeviceScreen();
void showSystemVersionScreen();
void showSystemResetConfirmScreen();
void setupActuators();
void setHeater(bool state);
void setFan(bool state);
void setupMAX6675();
uint16_t readMAX6675Raw();
void readTemperatureSensor();
void computePID();
void updateOutputs();
void checkSafetySystem();
void setupBuzzer();
void triggerBuzzer(BuzzerPattern pattern);
void updateBuzzer();
void logCSVData();
void updateSystem();
void loadSettings();
void saveSettings();
void savePID();
void saveCalibration();
void saveSetpoint();
void saveTimer();
void resetSettings();
void drawHeader(const char* title, const char* statusStr = NULL);
void drawMenuWindow(const char* title, const char* items[], int itemCount, int cursorIndex);

// ================================================================
// BLOCK 03 - STORAGE
// ================================================================

void loadSettings() {
  preferences.begin("oven", false);
  setpoint = preferences.getFloat("setpoint", 60.0);
  timerDurationMinutes = preferences.getUInt("timer_dur", 30);
  Kp = preferences.getFloat("pid_kp", 1.50);
  Ki = preferences.getFloat("pid_ki", 0.075);
  Kd = preferences.getFloat("pid_kd", 1.00);
  calibrationOffset = preferences.getFloat("cal_offset", 0.0);
  preferences.end();
}

void saveSettings() {
  preferences.begin("oven", false);
  preferences.putFloat("setpoint", setpoint);
  preferences.putUInt("timer_dur", timerDurationMinutes);
  preferences.putFloat("pid_kp", Kp);
  preferences.putFloat("pid_ki", Ki);
  preferences.putFloat("pid_kd", Kd);
  preferences.putFloat("cal_offset", calibrationOffset);
  preferences.end();
}

void savePID() {
  preferences.begin("oven", false);
  preferences.putFloat("pid_kp", Kp);
  preferences.putFloat("pid_ki", Ki);
  preferences.putFloat("pid_kd", Kd);
  preferences.end();
}

void saveCalibration() {
  preferences.begin("oven", false);
  preferences.putFloat("cal_offset", calibrationOffset);
  preferences.end();
}

void saveSetpoint() {
  preferences.begin("oven", false);
  preferences.putFloat("setpoint", setpoint);
  preferences.end();
}

void saveTimer() {
  preferences.begin("oven", false);
  preferences.putUInt("timer_dur", timerDurationMinutes);
  preferences.end();
}

void resetSettings() {
  setpoint = 60.0;
  timerDurationMinutes = 30;
  Kp = 1.50;
  Ki = 0.075;
  Kd = 1.00;
  calibrationOffset = 0.0;
  saveSettings();

  if (systemRunning) {
    pidIntegralTerm   = 0.0f;
    pidIntegralActive = false;
    lastPidInput      = currentTemperature;
    lastPidComputeMs  = millis();
    pidFirstRun       = false;

    float err = setpoint - currentTemperature;
    pidPTerm = Kp * err;
    pidDTerm = 0.0f;
    float total = pidPTerm + pidIntegralTerm + pidDTerm;

    if (total > 100.0f)      pidOutput = 100.0f;
    else if (total < 0.0f)   pidOutput = 0.0f;
    else                     pidOutput = total;

    heaterOutputPercent = pidOutput;
    ssrWindowStartTime  = millis();
    if (pidOutput <= 0.0f) {
      ssrWindowOnTimeMs = 0;
      setHeater(false);
    } else {
      ssrWindowOnTimeMs = (unsigned long)((pidOutput / 100.0f) * (float)SSR_WINDOW_MS);
    }
  }
}

// ================================================================
// BLOCK 04 - ENCODER
// ================================================================

const int PULSES_PER_DETENT = 4;            // Jumlah pulsa quadrature per 1 detent mekanis
const unsigned long ROTARY_DEBOUNCE_MS = 25; // Waktu tunggu minimum (ms) antar detent event
const unsigned long BUTTON_DEBOUNCE_MS = 50; // Waktu debounce minimum (ms) untuk push button SW

const int8_t encoderStateTable[] = {
  0,  -1,   1,   0,
  1,   0,   0,  -1,
 -1,   0,   0,   1,
  0,   1,  -1,   0
};

volatile int encoderPulseCount = 0;
volatile uint8_t encoderLastState = 0;

void IRAM_ATTR encoderISR() {
  uint8_t clk = digitalRead(ENCODER_CLK);
  uint8_t dt  = digitalRead(ENCODER_DT);
  uint8_t currentState = (clk << 1) | dt;

  uint8_t stateIndex = ((encoderLastState & 0x03) << 2) | (currentState & 0x03);
  int8_t step = encoderStateTable[stateIndex];

  if (step != 0) {
    encoderPulseCount += step;
    encoderLastState = currentState;
  }
}

void setupEncoder() {
  pinMode(ENCODER_CLK, INPUT_PULLUP);
  pinMode(ENCODER_DT, INPUT_PULLUP);
  pinMode(ENCODER_SW, INPUT_PULLUP);

  uint8_t clk = digitalRead(ENCODER_CLK);
  uint8_t dt  = digitalRead(ENCODER_DT);
  encoderLastState = (clk << 1) | dt;

  attachInterrupt(digitalPinToInterrupt(ENCODER_CLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_DT), encoderISR, CHANGE);
}

static unsigned long lastRotaryEventMs = 0;
static bool lastButtonState = HIGH;
static bool buttonPressedHandled = false;
static unsigned long lastButtonDebounceMs = 0;

EncoderEvent readEncoder() {
  EncoderEvent event = ENCODER_NONE;
  unsigned long now = millis();

  noInterrupts();
  int currentPulses = encoderPulseCount;
  interrupts();

  if (now - lastRotaryEventMs >= ROTARY_DEBOUNCE_MS) {
    if (currentPulses >= PULSES_PER_DETENT) {
      event = ENCODER_CW;
      noInterrupts();
      encoderPulseCount -= PULSES_PER_DETENT;
      interrupts();
      lastRotaryEventMs = now;
    } else if (currentPulses <= -PULSES_PER_DETENT) {
      event = ENCODER_CCW;
      noInterrupts();
      encoderPulseCount += PULSES_PER_DETENT;
      interrupts();
      lastRotaryEventMs = now;
    }
  }

  bool currentButtonReading = digitalRead(ENCODER_SW);

  if (currentButtonReading != lastButtonState) {
    lastButtonDebounceMs = now;
    lastButtonState = currentButtonReading;
  }

  if ((now - lastButtonDebounceMs) >= BUTTON_DEBOUNCE_MS) {
    if (currentButtonReading == LOW && !buttonPressedHandled) {
      buttonPressedHandled = true;
      event = ENCODER_PRESS;
      triggerBuzzer(BUZZER_PRESS);
    } else if (currentButtonReading == HIGH && buttonPressedHandled) {
      buttonPressedHandled = false;
    }
  }

  return event;
}

// ================================================================
// BLOCK 05 - SENSOR
// ================================================================

void setupMAX6675() {
  pinMode(MAX6675_SCK, OUTPUT);
  pinMode(MAX6675_CS, OUTPUT);
  pinMode(MAX6675_SO, INPUT);

  digitalWrite(MAX6675_CS, HIGH);
  digitalWrite(MAX6675_SCK, LOW);
}

uint16_t readMAX6675Raw() {
  digitalWrite(MAX6675_CS, LOW);
  delayMicroseconds(1);

  uint16_t value = 0;
  for (int i = 15; i >= 0; i--) {
    digitalWrite(MAX6675_SCK, HIGH);
    delayMicroseconds(1);
    if (digitalRead(MAX6675_SO)) {
      value |= (1 << i);
    }
    digitalWrite(MAX6675_SCK, LOW);
    delayMicroseconds(1);
  }

  digitalWrite(MAX6675_CS, HIGH);
  return value;
}

void readTemperatureSensor() {
  unsigned long now = millis();

  if (now - lastSensorReadMs >= SENSOR_READ_INTERVAL_MS) {
    lastSensorReadMs = now;

    uint16_t rawData = readMAX6675Raw();

    // Deteksi thermocouple terputus (Bit 2 == 1) atau jalur data bus floating (0xFFFF).
    // Nilai 0x0000 diizinkan karena merupakan pembacaan valid untuk temperatur 0 °C.
    if ((rawData & 0x04) || rawData == 0xFFFF) {
      sensorError = true;
      sensorOkStreak = 0;
      filterInitialized = false;
      medianBufferCount = 0;
      medianBufferIndex = 0;
      rawTemperature = -999.0;
      calibratedTemperature = -999.0;
      medianTemperature = -999.0;
      filteredTemperature = -999.0;
      currentTemperature = -999.0;
    } else {
      sensorOkStreak++;

      // 1. RAW TEMPERATURE (MAX6675 12-bit, 0.25 °C/LSB)
      uint16_t tempBits = (rawData >> 3) & 0x0FFF;
      rawTemperature = tempBits * 0.25;

      // 2. CALIBRATION OFFSET
      calibratedTemperature = rawTemperature + calibrationOffset;

      // 3. MEDIAN FILTER (5 Sampel Valid)
      medianBuffer[medianBufferIndex] = calibratedTemperature;
      medianBufferIndex = (medianBufferIndex + 1) % MEDIAN_FILTER_SIZE;
      if (medianBufferCount < MEDIAN_FILTER_SIZE) {
        medianBufferCount++;
      }

      // Hitung median dari sampel valid di buffer
      float tempSorted[MEDIAN_FILTER_SIZE];
      for (int i = 0; i < medianBufferCount; i++) {
        tempSorted[i] = medianBuffer[i];
      }
      for (int i = 0; i < medianBufferCount - 1; i++) {
        for (int j = 0; j < medianBufferCount - i - 1; j++) {
          if (tempSorted[j] > tempSorted[j + 1]) {
            float tmp = tempSorted[j];
            tempSorted[j] = tempSorted[j + 1];
            tempSorted[j + 1] = tmp;
          }
        }
      }
      medianTemperature = tempSorted[medianBufferCount / 2];

      // 4. EMA FILTER (α = 0.20)
      if (filterInitialized) {
        filteredTemperature = (EMA_ALPHA * medianTemperature) + ((1.0 - EMA_ALPHA) * filteredTemperature);
      } else {
        filteredTemperature = medianTemperature;
        filterInitialized = true;
      }

      // 5. FILTERED TEMPERATURE UNTUK PID & SAFETY SYSTEM
      currentTemperature = filteredTemperature;

      if (sensorOkStreak >= SENSOR_OK_STREAK_REQUIRED) {
        sensorError = false;
      }
    }
  }
}

// ================================================================
// BLOCK 06 - PID
// ================================================================

void computePID() {
  unsigned long now = millis();

  // --- SAFETY GUARD ---
  // Jika kondisi tidak aman: reset semua state PID dan matikan output
  if (!systemRunning || !heaterAllowed || sensorError || currentSafetyState != SAFETY_OK) {
    pidOutput           = 0.0f;
    pidIntegralTerm     = 0.0f;
    pidPTerm            = 0.0f;
    pidDTerm            = 0.0f;
    pidIntegralActive   = false;
    lastPidComputeMs    = now;
    pidFirstRun         = true;
    ssrWindowOnTimeMs   = 0;
    heaterOutputPercent = 0.0f;
    return;
  }

  // --- FIRST RUN INIT: HANYA inisialisasi state, TIDAK komputasi output ---
  //
  // FIX v1.4.2: Versi lama menjalankan komputasi PID penuh di siklus pertama
  // dengan dt nominal 0.25s. Ini menyebabkan D-Term spike besar karena
  // dInput = currentTemperature - lastPidInput (lastPidInput = 0.0 sebelum init),
  // menghasilkan lonjakan output positif besar di siklus pertama.
  //
  // FIX: Gunakan first run HANYA untuk inisialisasi state.
  // Output tetap 0 dan SSR tetap OFF sampai siklus komputasi berikutnya.
  if (pidFirstRun) {
    lastPidInput       = currentTemperature;
    lastPidComputeMs   = now;
    ssrWindowStartTime = now;
    ssrWindowOnTimeMs  = 0;
    pidFirstRun        = false;
    return;
  }

  // --- PID INTERVAL CHECK ---
  if (now - lastPidComputeMs < PID_COMPUTE_INTERVAL_MS) {
    return;
  }

  float dt = (float)(now - lastPidComputeMs) / 1000.0f;
  lastPidComputeMs = now;

  // Batas dt minimum & maksimum untuk stabilitas numerik
  if (dt < 0.01f) dt = 0.01f;
  if (dt > 2.0f)  dt = 2.0f;

  // --- 1. HITUNG ERROR (setpoint - PV) ---
  // Positif = suhu di bawah setpoint (perlu pemanasan)
  // Negatif = overshoot (suhu di atas setpoint)
  float error = setpoint - currentTemperature;

  // --- 2. P-TERM ---
  float pTerm = Kp * error;

  // --- 3. D-TERM (Derivative on Measurement) ---
  // Menggunakan -d(PV)/dt untuk menghindari derivative kick saat setpoint diubah
  float dInput = currentTemperature - lastPidInput;
  float dTerm = 0.0f;
  if (dt > 0.001f) {
    dTerm = -Kd * (dInput / dt);
  }
  lastPidInput = currentTemperature;

  // Clamp dTerm untuk meredam lonjakan akibat noise sensor sesaat
  if (dTerm >  50.0f) dTerm =  50.0f;
  if (dTerm < -50.0f) dTerm = -50.0f;

  // --- 4. CONDITIONAL INTEGRATION / CLAMPING ANTI-WINDUP ---
  //
  // FIX v1.4.2: Bug kritis pada versi sebelumnya:
  // rawOutput dievaluasi menggunakan pidIntegralTerm LAMA (sebelum di-update).
  // Skenario overshoot: T=70C, error=-10, pidIntegralTerm=20 (dari sesi sebelum overshoot)
  //   rawOutput = pTerm(-10) + 20 + dTerm(~0) = ~10 (dalam range linier 0-100!)
  //   Karena rawOutput > 0 dan error < 0:
  //     Kondisi "rawOutput >= 100 && error > 0" -> false
  //     Kondisi "rawOutput <= 0 && error < 0"   -> false (karena rawOutput=10, bukan <= 0)
  //   allowIntegration = true -> integral bertambah lagi (meskipun sedang overshoot)!
  //   Ini menyebabkan SSR tetap ON saat T sudah di atas setpoint.
  //
  // FIX: Hitung candidateIntegral terlebih dahulu, lalu evaluasi candidateOutput.
  // Anti-windup dievaluasi berdasarkan output KANDIDAT yang sudah mencerminkan
  // delta integral baru, sehingga unwind bisa terjadi secara konsisten.

  float candidateIntegral = pidIntegralTerm + (Ki * error * dt);

  // Clamp kandidat integral ke batas fisik [0, 100]% (heater unidirectional)
  if (candidateIntegral > 100.0f) candidateIntegral = 100.0f;
  if (candidateIntegral <   0.0f) candidateIntegral = 0.0f;

  // Evaluasi output menggunakan integral KANDIDAT
  float candidateOutput = pTerm + candidateIntegral + dTerm;

  // Aturan Anti-Windup (Conditional Integration):
  // - Blokir akumulasi integral HANYA saat output saturasi atas (>= 100%) DAN error > 0 (mencegah windup pemanasan).
  // - Saat error negatif (overshoot), integral SELALU diizinkan melakukan unwind (turun)
  //   tanpa diblokir oleh batas bawah output (batas minimum integral dijamin aman oleh clamping ke 0.0f).
  bool allowIntegration = true;
  if (candidateOutput >= 100.0f && error > 0.0f) {
    allowIntegration = false;
  }

  if (allowIntegration) {
    pidIntegralTerm   = candidateIntegral;
    pidIntegralActive = (Ki > 0.0f && fabsf(error) > 0.01f);
  } else {
    pidIntegralActive = false;
    // pidIntegralTerm tidak diubah saat diblokir oleh anti-windup saturasi atas
  }

  // --- 5. TOTAL OUTPUT & SATURASI AKHIR ---
  float totalOutput = pTerm + pidIntegralTerm + dTerm;

  if (totalOutput > 100.0f) {
    pidOutput = 100.0f;
  } else if (totalOutput < 0.0f) {
    pidOutput = 0.0f;
  } else {
    pidOutput = totalOutput;
  }

  // --- 6. SIMPAN DATA LOGGING & AKTUALISASI heaterOutputPercent ---
  pidPTerm = pTerm;
  pidDTerm = dTerm;
  heaterOutputPercent = pidOutput;

  // Catatan: ssrWindowOnTimeMs di-update di updateOutputs() pada awal setiap
  // window baru, menggunakan pidOutput terkini pada saat window berganti.
}

// ================================================================
// BLOCK 07 - ACTUATOR
// ================================================================


void emergencyPinsSafeState() {
  pinMode(SSR_HEATER_PIN, OUTPUT);
  digitalWrite(SSR_HEATER_PIN, LOW);

  pinMode(FAN_RELAY_PIN, OUTPUT);
  digitalWrite(FAN_RELAY_PIN, FAN_RELAY_ACTIVE_LOW ? HIGH : LOW);
}

void setHeater(bool state) {
  if (state) {
    bool manualModeActive = (currentScreen == SCREEN_MANUAL_MENU || currentScreen == SCREEN_MANUAL_HEATER || currentScreen == SCREEN_MANUAL_FAN);

    if (!heaterAllowed || sensorError || currentSafetyState != SAFETY_OK || (!systemRunning && !manualModeActive)) {
      heaterState = false;
      if (!systemRunning) {
        heaterManual = false;
        heaterOutputPercent = 0.0f;
      }
      digitalWrite(SSR_HEATER_PIN, LOW);
      return;
    }
  }

  heaterState = state;
  if (!systemRunning) {
    heaterManual = state;
    heaterOutputPercent = state ? 100.0f : 0.0f;
  }
  digitalWrite(SSR_HEATER_PIN, state ? HIGH : LOW);
}

void setFan(bool state) {
  fanState = state;
  fanManual = state;
  if (FAN_RELAY_ACTIVE_LOW) {
    digitalWrite(FAN_RELAY_PIN, state ? LOW : HIGH);
  } else {
    digitalWrite(FAN_RELAY_PIN, state ? HIGH : LOW);
  }
}

void setupActuators() {
  pinMode(SSR_HEATER_PIN, OUTPUT);
  pinMode(FAN_RELAY_PIN, OUTPUT);

  setHeater(false);
  setFan(false);
}

// ================================================================
// BLOCK 08 - SAFETY
// ================================================================

void checkSafetySystem() {
  bool sensorInvalid = sensorError || currentTemperature <= -900.0 || isnan(currentTemperature);

  if (sensorInvalid) {
    sensorErrorFaultLatched = true;
    currentSafetyState = SAFETY_SENSOR_ERROR;
    heaterAllowed = false;
  } else if (sensorErrorFaultLatched) {
    sensorErrorFaultLatched = false;
  }

  if (!sensorInvalid) {
    if (currentTemperature >= MAX_SAFE_TEMP) {
      overTempFaultLatched = true;
    } else if (overTempFaultLatched && currentTemperature <= (MAX_SAFE_TEMP - OVER_TEMP_HYSTERESIS)) {
      overTempFaultLatched = false;
    }

    if (overTempFaultLatched) {
      currentSafetyState = SAFETY_OVER_TEMP;
      heaterAllowed = false;
    } else {
      currentSafetyState = SAFETY_OK;
      heaterAllowed = true;
    }
  }

  if (!heaterAllowed && heaterState) {
    heaterState = false;
    heaterManual = false;
    digitalWrite(SSR_HEATER_PIN, LOW);
    heaterOutputPercent = 0.0;
  }

  if (currentSafetyState == SAFETY_OVER_TEMP) {
    if (currentBuzzerPattern != BUZZER_OVER_TEMP) {
      triggerBuzzer(BUZZER_OVER_TEMP);
    }
  } else if (currentSafetyState == SAFETY_SENSOR_ERROR) {
    if (currentBuzzerPattern != BUZZER_SENSOR_ERROR) {
      triggerBuzzer(BUZZER_SENSOR_ERROR);
    }
  } else {
    if (currentBuzzerPattern == BUZZER_SENSOR_ERROR || currentBuzzerPattern == BUZZER_OVER_TEMP) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
      currentBuzzerPattern = BUZZER_IDLE;
    }
  }
}

void updateOutputs() {
  unsigned long now = millis();

  // Jika sistem tidak aman, paksa heater OFF tanpa syarat
  if (!heaterAllowed || sensorError || currentSafetyState != SAFETY_OK) {
    setHeater(false);
    return;
  }

  if (systemRunning) {
    // --- EVALUASI INSTANT SHUTOFF JIKA PID OUTPUT 0% ---
    // Pastikan PID Output 0% langsung mematikan SSR seketika,
    // tanpa menunggu sisa SSR window yang sedang berjalan.
    if (pidOutput <= 0.0f) {
      ssrWindowOnTimeMs = 0;
      setHeater(false);
      return;
    }

    // --- SSR TIME-PROPORTIONAL CONTROL (UNTUK OUTPUT > 0%) ---
    // Snapshot on-time baru saat window berganti
    if (now - ssrWindowStartTime >= SSR_WINDOW_MS) {
      ssrWindowStartTime = now;
      ssrWindowOnTimeMs  = (unsigned long)((pidOutput / 100.0f) * (float)SSR_WINDOW_MS);
    }

    unsigned long windowElapsed = now - ssrWindowStartTime;

    // Untuk output 100%: selalu ON sepanjang window
    if (ssrWindowOnTimeMs >= SSR_WINDOW_MS) {
      setHeater(true);
    } else {
      // Time-proportional normal untuk 0% < output < 100%
      setHeater(windowElapsed < ssrWindowOnTimeMs);
    }

  } else {
    // Sistem tidak running: heater OFF kecuali mode manual aktif
    bool manualModeActive = (currentScreen == SCREEN_MANUAL_MENU ||
                             currentScreen == SCREEN_MANUAL_HEATER ||
                             currentScreen == SCREEN_MANUAL_FAN);
    if (!manualModeActive && heaterState) {
      setHeater(false);
    }
  }
}

// ================================================================
// BLOCK 09 - CURING PROCESS
// ================================================================

// Curing process timing & state management handled in updateUI() & handleStartInput()

// ================================================================
// BLOCK 10 - BUZZER
// ================================================================

void setupBuzzer() {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  buzzerState = false;
  currentBuzzerPattern = BUZZER_IDLE;
}

void triggerBuzzer(BuzzerPattern pattern) {
  if (currentBuzzerPattern != BUZZER_IDLE && pattern > currentBuzzerPattern) {
    return;
  }

  currentBuzzerPattern = pattern;
  buzzerStep = 0;
  buzzerTimerMs = millis();
}

void updateBuzzer() {
  if (currentBuzzerPattern == BUZZER_IDLE) {
    if (buzzerState) {
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
    }
    return;
  }

  unsigned long now = millis();
  unsigned long elapsed = now - buzzerTimerMs;

  switch (currentBuzzerPattern) {
    case BUZZER_PRESS:
      if (buzzerStep == 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 1;
        buzzerTimerMs = now;
      } else if (buzzerStep == 1 && elapsed >= 50) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        currentBuzzerPattern = BUZZER_IDLE;
      }
      break;

    case BUZZER_SAVE:
      if (buzzerStep == 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 1;
        buzzerTimerMs = now;
      } else if (buzzerStep == 1 && elapsed >= 100) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        currentBuzzerPattern = BUZZER_IDLE;
      }
      break;

    case BUZZER_START:
      if (buzzerStep == 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 1;
        buzzerTimerMs = now;
      } else if (buzzerStep == 1 && elapsed >= 60) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 2;
        buzzerTimerMs = now;
      } else if (buzzerStep == 2 && elapsed >= 60) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 3;
        buzzerTimerMs = now;
      } else if (buzzerStep == 3 && elapsed >= 60) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        currentBuzzerPattern = BUZZER_IDLE;
      }
      break;

    case BUZZER_FINISH:
      if (buzzerStep == 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 1;
        buzzerTimerMs = now;
      } else if (buzzerStep == 1 && elapsed >= 120) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 2;
        buzzerTimerMs = now;
      } else if (buzzerStep == 2 && elapsed >= 100) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 3;
        buzzerTimerMs = now;
      } else if (buzzerStep == 3 && elapsed >= 120) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 4;
        buzzerTimerMs = now;
      } else if (buzzerStep == 4 && elapsed >= 100) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 5;
        buzzerTimerMs = now;
      } else if (buzzerStep == 5 && elapsed >= 120) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        currentBuzzerPattern = BUZZER_IDLE;
      }
      break;

    case BUZZER_SENSOR_ERROR:
      if (buzzerStep == 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 1;
        buzzerTimerMs = now;
      } else if (buzzerStep == 1 && elapsed >= 80) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 2;
        buzzerTimerMs = now;
      } else if (buzzerStep == 2 && elapsed >= 80) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 3;
        buzzerTimerMs = now;
      } else if (buzzerStep == 3 && elapsed >= 80) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 4;
        buzzerTimerMs = now;
      } else if (buzzerStep == 4 && elapsed >= 800) {
        buzzerStep = 0;
        buzzerTimerMs = now;
      }
      break;

    case BUZZER_OVER_TEMP:
      if (buzzerStep == 0) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 1;
        buzzerTimerMs = now;
      } else if (buzzerStep == 1 && elapsed >= 70) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 2;
        buzzerTimerMs = now;
      } else if (buzzerStep == 2 && elapsed >= 70) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 3;
        buzzerTimerMs = now;
      } else if (buzzerStep == 3 && elapsed >= 70) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 4;
        buzzerTimerMs = now;
      } else if (buzzerStep == 4 && elapsed >= 70) {
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerState = true;
        buzzerStep = 5;
        buzzerTimerMs = now;
      } else if (buzzerStep == 5 && elapsed >= 70) {
        digitalWrite(BUZZER_PIN, LOW);
        buzzerState = false;
        buzzerStep = 6;
        buzzerTimerMs = now;
      } else if (buzzerStep == 6 && elapsed >= 600) {
        buzzerStep = 0;
        buzzerTimerMs = now;
      }
      break;

    default:
      digitalWrite(BUZZER_PIN, LOW);
      buzzerState = false;
      currentBuzzerPattern = BUZZER_IDLE;
      break;
  }
}

// ================================================================
// BLOCK 11 - UI
// ================================================================

void resetUIInactivityTimer() {
  lastUIActivityMs = millis();
}

void checkUIInactivity() {
  if (currentScreen == SCREEN_BOOT) return;
  if (!systemRunning && currentScreen == SCREEN_HOME) return;
  if (systemRunning && currentScreen == SCREEN_RUNNING) return;

  if (millis() - lastUIActivityMs >= UI_INACTIVITY_TIMEOUT_MS) {
    resetUIInactivityTimer();

    if (currentScreen == SCREEN_START_CONFIRM) {
      confirmOption = 1;
    } else if (currentScreen == SCREEN_SYSTEM_RESET_CONFIRM) {
      resetConfirmOption = 1;
      isResetDoneMessage = false;
    } else if (currentScreen == SCREEN_CALIBRATION) {
      isCalibSavedMessage = false;
    }

    if (systemRunning) {
      currentScreen = SCREEN_RUNNING;
    } else {
      currentScreen = SCREEN_HOME;
    }
    displayNeedsUpdate = true;
  }
}

void handleInput() {
  EncoderEvent event = readEncoder();
  if (event != ENCODER_NONE) {
    resetUIInactivityTimer();
    handleEncoderEvent(event);
    displayNeedsUpdate = true;
  }
}

void updateUI() {
  if (currentScreen == SCREEN_BOOT) {
    if (millis() - bootStartTime >= BOOT_DURATION_MS) {
      currentScreen = SCREEN_HOME;
      resetUIInactivityTimer();
      displayNeedsUpdate = true;
    }
  }

  if (currentScreen == SCREEN_CALIBRATION && isCalibSavedMessage) {
    if (millis() - calibSaveMsgStartTime >= 1200) {
      isCalibSavedMessage = false;
      currentScreen = SCREEN_MAIN_MENU;
      resetUIInactivityTimer();
      displayNeedsUpdate = true;
    }
  }

  if (currentScreen == SCREEN_SYSTEM_RESET_CONFIRM && isResetDoneMessage) {
    if (millis() - resetDoneMsgStartTime >= 1500) {
      isResetDoneMessage = false;
      currentScreen = SCREEN_SYSTEM_MENU;
      resetUIInactivityTimer();
      displayNeedsUpdate = true;
    }
  }

  if (systemRunning &&
      (currentScreen == SCREEN_MANUAL_MENU ||
       currentScreen == SCREEN_MANUAL_HEATER ||
       currentScreen == SCREEN_MANUAL_FAN)) {
    currentScreen = SCREEN_MAIN_MENU;
    mainMenuCursor = 0;
    resetUIInactivityTimer();
    displayNeedsUpdate = true;
  }

  if (systemRunning) {
    unsigned long elapsedSec = (millis() - curingStartTime) / 1000;
    if (elapsedSec >= curingTotalSeconds) {
      curingRemainingSeconds = 0;
      systemRunning = false;
      setHeater(false);
      setFan(false);
      triggerBuzzer(BUZZER_FINISH);
      currentScreen = SCREEN_HOME;
      resetUIInactivityTimer();
      displayNeedsUpdate = true;
    } else {
      curingRemainingSeconds = curingTotalSeconds - elapsedSec;
    }
  }

  static unsigned long lastRunningRefreshMs = 0;
  if (currentScreen == SCREEN_RUNNING || currentScreen == SCREEN_HOME) {
    if (millis() - lastRunningRefreshMs >= 1000) {
      lastRunningRefreshMs = millis();
      displayNeedsUpdate = true;
    }
  }

  checkUIInactivity();
}

void handleHomeInput(EncoderEvent event) {
  if (event == ENCODER_PRESS) {
    if (systemRunning) {
      currentScreen = SCREEN_RUNNING;
    } else {
      currentScreen = SCREEN_MAIN_MENU;
      mainMenuCursor = 0;
    }
  }
}

void handleRunningInput(EncoderEvent event) {
  if (event == ENCODER_PRESS) {
    currentScreen = SCREEN_MAIN_MENU;
    mainMenuCursor = 0;
  }
}

void handleTimerInput(EncoderEvent event) {
  if (event == ENCODER_CW) {
    tempTimerMinutes += 1;
    if (tempTimerMinutes > 999) tempTimerMinutes = 999;
  } else if (event == ENCODER_CCW) {
    if (tempTimerMinutes > 1) tempTimerMinutes -= 1;
  } else if (event == ENCODER_PRESS) {
    timerDurationMinutes = tempTimerMinutes;
    saveTimer();
    triggerBuzzer(BUZZER_SAVE);
    currentScreen = SCREEN_MAIN_MENU;
  }
}

void handleCalibrationInput(EncoderEvent event) {
  if (isCalibSavedMessage) {
    if (event == ENCODER_PRESS) {
      isCalibSavedMessage = false;
      currentScreen = SCREEN_MAIN_MENU;
    }
    return;
  }

  if (event == ENCODER_CW) {
    tempCalibOffset += 0.1;
    if (tempCalibOffset > 10.0) tempCalibOffset = 10.0;
  } else if (event == ENCODER_CCW) {
    tempCalibOffset -= 0.1;
    if (tempCalibOffset < -10.0) tempCalibOffset = -10.0;
  } else if (event == ENCODER_PRESS) {
    calibrationOffset = tempCalibOffset;
    saveCalibration();
    triggerBuzzer(BUZZER_SAVE);
    isCalibSavedMessage = true;
    calibSaveMsgStartTime = millis();
  }
}

void handleStartInput(EncoderEvent event) {
  if (event == ENCODER_CW || event == ENCODER_CCW) {
    confirmOption = 1 - confirmOption;
  } else if (event == ENCODER_PRESS) {
    if (confirmOption == 0) {
      systemRunning = !systemRunning;
      if (systemRunning) {
        curingStartTime = millis();
        curingTotalSeconds = (unsigned long)timerDurationMinutes * 60;
        curingRemainingSeconds = curingTotalSeconds;
        pidOutput = 0.0;
        pidIntegralTerm = 0.0;
        pidPTerm = 0.0;
        pidDTerm = 0.0;
        pidFirstRun = true;           // Pastikan branch init di computePID() aktif

        lastPidInput = currentTemperature;
        lastPidComputeMs = millis();
        ssrWindowStartTime = millis();
        ssrWindowOnTimeMs = 0;
        setFan(true);
        triggerBuzzer(BUZZER_START);
        currentScreen = SCREEN_RUNNING;
      } else {
        pidOutput = 0.0f;
        pidIntegralTerm = 0.0f;
        pidPTerm = 0.0f;
        pidDTerm = 0.0f;
        pidIntegralActive = false;
        pidFirstRun = true;
        ssrWindowOnTimeMs = 0;
        heaterOutputPercent = 0.0f;
        setHeater(false);
        setFan(false);
        currentScreen = SCREEN_HOME;
      }
    } else {
      currentScreen = SCREEN_MAIN_MENU;
    }
  }
}

void handleSetpointInput(EncoderEvent event) {
  if (event == ENCODER_CW) {
    tempSetpoint += 1.0;
    if (tempSetpoint > SETPOINT_MAX) tempSetpoint = SETPOINT_MAX;
  } else if (event == ENCODER_CCW) {
    tempSetpoint -= 1.0;
    if (tempSetpoint < SETPOINT_MIN) tempSetpoint = SETPOINT_MIN;
  } else if (event == ENCODER_PRESS) {
    setpoint = tempSetpoint;
    saveSetpoint();
    triggerBuzzer(BUZZER_SAVE);
    currentScreen = SCREEN_MAIN_MENU;
  }
}

void handleEncoderEvent(EncoderEvent event) {
  switch (currentScreen) {
    case SCREEN_HOME:
      handleHomeInput(event);
      break;

    case SCREEN_RUNNING:
      handleRunningInput(event);
      break;

    case SCREEN_MAIN_MENU:
      if (event == ENCODER_CW) {
        mainMenuCursor = (mainMenuCursor + 1) % MAIN_MENU_COUNT;
      } else if (event == ENCODER_CCW) {
        mainMenuCursor = (mainMenuCursor - 1 + MAIN_MENU_COUNT) % MAIN_MENU_COUNT;
      } else if (event == ENCODER_PRESS) {
        switch (mainMenuCursor) {
          case 0:
            confirmOption = 0;
            currentScreen = SCREEN_START_CONFIRM;
            break;
          case 1:
            tempSetpoint = setpoint;
            currentScreen = SCREEN_SETPOINT_EDIT;
            break;
          case 2:
            tempTimerMinutes = timerDurationMinutes;
            currentScreen = SCREEN_TIMER;
            break;
          case 3:
            pidMenuCursor = 0;
            currentScreen = SCREEN_PID_MENU;
            break;
          case 4:
            if (systemRunning) {
              triggerBuzzer(BUZZER_PRESS);
            } else {
              manualMenuCursor = 0;
              currentScreen = SCREEN_MANUAL_MENU;
            }
            break;
          case 5:
            tempCalibOffset = calibrationOffset;
            isCalibSavedMessage = false;
            currentScreen = SCREEN_CALIBRATION;
            break;
          case 6:
            systemMenuCursor = 0;
            currentScreen = SCREEN_SYSTEM_MENU;
            break;
        }
      }
      break;

    case SCREEN_TIMER:
      handleTimerInput(event);
      break;

    case SCREEN_CALIBRATION:
      handleCalibrationInput(event);
      break;

    case SCREEN_START_CONFIRM:
      handleStartInput(event);
      break;

    case SCREEN_SETPOINT_EDIT:
      handleSetpointInput(event);
      break;

    case SCREEN_PID_MENU:
      if (event == ENCODER_CW) {
        pidMenuCursor = (pidMenuCursor + 1) % PID_MENU_COUNT;
      } else if (event == ENCODER_CCW) {
        pidMenuCursor = (pidMenuCursor - 1 + PID_MENU_COUNT) % PID_MENU_COUNT;
      } else if (event == ENCODER_PRESS) {
        if (pidMenuCursor == 3) {
          currentScreen = SCREEN_MAIN_MENU;
        } else {
          selectedPIDParam = pidMenuCursor;
          if (selectedPIDParam == 0) tempPIDValue = Kp;
          else if (selectedPIDParam == 1) tempPIDValue = Ki;
          else if (selectedPIDParam == 2) tempPIDValue = Kd;
          // Snap nilai ke grid 0.05 sebelum masuk edit,
          // agar nilai dari NVS yang mengandung float-drift langsung bersih.
          tempPIDValue = roundf(tempPIDValue / 0.05f) * 0.05f;
          currentScreen = SCREEN_PID_EDIT;
        }
      }
      break;

    case SCREEN_PID_EDIT:
      {
        // Semua parameter (Kp, Ki, Kd) menggunakan step 0.05.
        // Setelah setiap operasi, nilai di-snap ke kelipatan 0.05
        // menggunakan roundf() untuk menghindari floating-point drift.
        const float PID_STEP = 0.05f;
        if (event == ENCODER_CW) {
          tempPIDValue += PID_STEP;
          tempPIDValue = roundf(tempPIDValue / PID_STEP) * PID_STEP;
          if (tempPIDValue > 99.95f) tempPIDValue = 99.95f;
        } else if (event == ENCODER_CCW) {
          tempPIDValue -= PID_STEP;
          tempPIDValue = roundf(tempPIDValue / PID_STEP) * PID_STEP;
          if (tempPIDValue < 0.0f) tempPIDValue = 0.0f;
        } else if (event == ENCODER_PRESS) {
          if (selectedPIDParam == 0) Kp = tempPIDValue;
          else if (selectedPIDParam == 1) Ki = tempPIDValue;
          else if (selectedPIDParam == 2) Kd = tempPIDValue;

          savePID();
          triggerBuzzer(BUZZER_SAVE);

          // Sinkronisasi PID & re-inisialisasi state seketika setelah perubahan parameter:
          // Pastikan output lama tidak terus digunakan dan hitung ulang output PID dengan parameter baru.
          if (systemRunning) {
            pidIntegralTerm   = 0.0f;
            pidIntegralActive = false;
            lastPidInput      = currentTemperature;
            lastPidComputeMs  = millis();
            pidFirstRun       = false;

            float err = setpoint - currentTemperature;
            pidPTerm = Kp * err;
            pidDTerm = 0.0f; // Reset D-Term agar tidak spike akibat perubahan Kd
            float total = pidPTerm + pidIntegralTerm + pidDTerm;

            if (total > 100.0f)      pidOutput = 100.0f;
            else if (total < 0.0f)   pidOutput = 0.0f;
            else                     pidOutput = total;

            heaterOutputPercent = pidOutput;
            ssrWindowStartTime  = millis();
            if (pidOutput <= 0.0f) {
              ssrWindowOnTimeMs = 0;
              setHeater(false);
            } else {
              ssrWindowOnTimeMs = (unsigned long)((pidOutput / 100.0f) * (float)SSR_WINDOW_MS);
            }
          }
          currentScreen = SCREEN_PID_MENU;
        }
      }
      break;

    case SCREEN_MANUAL_MENU:
      if (event == ENCODER_CW) {
        manualMenuCursor = (manualMenuCursor + 1) % MANUAL_MENU_COUNT;
      } else if (event == ENCODER_CCW) {
        manualMenuCursor = (manualMenuCursor - 1 + MANUAL_MENU_COUNT) % MANUAL_MENU_COUNT;
      } else if (event == ENCODER_PRESS) {
        if (manualMenuCursor == 0) {
          manualToggleOption = heaterState ? 1 : 0;
          currentScreen = SCREEN_MANUAL_HEATER;
        } else if (manualMenuCursor == 1) {
          manualToggleOption = fanState ? 1 : 0;
          currentScreen = SCREEN_MANUAL_FAN;
        } else if (manualMenuCursor == 2) {
          if (!systemRunning) {
            setHeater(false);
            setFan(false);
          }
          currentScreen = SCREEN_MAIN_MENU;
        }
      }
      break;

    case SCREEN_MANUAL_HEATER:
      if (event == ENCODER_CW || event == ENCODER_CCW) {
        manualToggleOption = 1 - manualToggleOption;
      } else if (event == ENCODER_PRESS) {
        setHeater(manualToggleOption == 1);
        currentScreen = SCREEN_MANUAL_MENU;
      }
      break;

    case SCREEN_MANUAL_FAN:
      if (event == ENCODER_CW || event == ENCODER_CCW) {
        manualToggleOption = 1 - manualToggleOption;
      } else if (event == ENCODER_PRESS) {
        setFan(manualToggleOption == 1);
        currentScreen = SCREEN_MANUAL_MENU;
      }
      break;

    case SCREEN_SYSTEM_MENU:
      if (event == ENCODER_CW) {
        systemMenuCursor = (systemMenuCursor + 1) % SYSTEM_MENU_COUNT;
      } else if (event == ENCODER_CCW) {
        systemMenuCursor = (systemMenuCursor - 1 + SYSTEM_MENU_COUNT) % SYSTEM_MENU_COUNT;
      } else if (event == ENCODER_PRESS) {
        if (systemMenuCursor == 0) currentScreen = SCREEN_SYSTEM_SENSOR;
        else if (systemMenuCursor == 1) currentScreen = SCREEN_SYSTEM_DEVICE;
        else if (systemMenuCursor == 2) currentScreen = SCREEN_SYSTEM_VERSION;
        else if (systemMenuCursor == 3) {
          resetConfirmOption = 1;
          isResetDoneMessage = false;
          currentScreen = SCREEN_SYSTEM_RESET_CONFIRM;
        } else if (systemMenuCursor == 4) currentScreen = SCREEN_MAIN_MENU;
      }
      break;

    case SCREEN_SYSTEM_RESET_CONFIRM:
      if (isResetDoneMessage) {
        if (event == ENCODER_PRESS) {
          isResetDoneMessage = false;
          currentScreen = SCREEN_SYSTEM_MENU;
        }
      } else {
        if (event == ENCODER_CW || event == ENCODER_CCW) {
          resetConfirmOption = 1 - resetConfirmOption;
        } else if (event == ENCODER_PRESS) {
          if (resetConfirmOption == 0) {
            resetSettings();
            triggerBuzzer(BUZZER_SAVE);
            isResetDoneMessage = true;
            resetDoneMsgStartTime = millis();
          } else {
            currentScreen = SCREEN_SYSTEM_MENU;
          }
        }
      }
      break;

    case SCREEN_SYSTEM_SENSOR:
    case SCREEN_SYSTEM_DEVICE:
    case SCREEN_SYSTEM_VERSION:
      if (event == ENCODER_PRESS) {
        currentScreen = SCREEN_SYSTEM_MENU;
      }
      break;

    default:
      break;
  }
}

// ================================================================
// BLOCK 12 - DISPLAY
// ================================================================

void updateDisplay() {
  if (displayNeedsUpdate) {
    renderScreen();
    displayNeedsUpdate = false;
  }
}

void renderScreen() {
  display.clearDisplay();

  switch (currentScreen) {
    case SCREEN_BOOT:           showBootScreen(); break;
    case SCREEN_HOME:           showHomeScreen(); break;
    case SCREEN_RUNNING:        showRunningScreen(); break;
    case SCREEN_MAIN_MENU:      showMainMenu(); break;
    case SCREEN_START_CONFIRM:  showStartConfirmScreen(); break;
    case SCREEN_SETPOINT_EDIT:  showSetpointScreen(); break;
    case SCREEN_TIMER:          showTimerScreen(); break;
    case SCREEN_PID_MENU:       showPIDMenu(); break;
    case SCREEN_PID_EDIT:       showPIDEditScreen(); break;
    case SCREEN_MANUAL_MENU:    showManualMenu(); break;
    case SCREEN_MANUAL_HEATER:  showManualToggleScreen("HEATER", manualToggleOption); break;
    case SCREEN_MANUAL_FAN:     showManualToggleScreen("FAN", manualToggleOption); break;
    case SCREEN_CALIBRATION:    showCalibrationScreen(); break;
    case SCREEN_SYSTEM_MENU:          showSystemMenu(); break;
    case SCREEN_SYSTEM_SENSOR:        showSystemSensorScreen(); break;
    case SCREEN_SYSTEM_DEVICE:        showSystemDeviceScreen(); break;
    case SCREEN_SYSTEM_VERSION:       showSystemVersionScreen(); break;
    case SCREEN_SYSTEM_RESET_CONFIRM: showSystemResetConfirmScreen(); break;
  }

  display.display();
}

void drawHeader(const char* title, const char* statusStr) {
  display.setCursor(0, 0);
  display.print(title);
  if (statusStr != NULL) {
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(statusStr, 0, 0, &x1, &y1, &w, &h);
    display.setCursor(128 - w, 0);
    display.print(statusStr);
  }
  display.drawFastHLine(0, 10, 128, SSD1306_WHITE);
}

void drawMenuWindow(const char* title, const char* items[], int itemCount, int cursorIndex) {
  drawHeader(title, NULL);

  int maxVisible = 4;
  int topIndex = 0;

  if (cursorIndex >= maxVisible) {
    topIndex = cursorIndex - maxVisible + 1;
  }

  for (int i = 0; i < maxVisible && (topIndex + i) < itemCount; i++) {
    int idx = topIndex + i;
    int yPos = 16 + (i * 12);
    display.setCursor(0, yPos);

    if (idx == cursorIndex) {
      display.print(F("> "));
    } else {
      display.print(F("  "));
    }
    display.print(items[idx]);
  }
}

void showBootScreen() {
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.setCursor(20, 10);
  display.print(F("OVEN CURING"));

  display.setCursor(26, 26);
  display.print(F("CONTROLLER"));

  display.setCursor(44, 38);
  display.print(F("v1.4.2"));

  display.setCursor(28, 50);
  display.print(F("STARTING..."));
}

void showHomeScreen() {
  drawHeader("OVEN CURING", systemRunning ? "HEATING" : "IDLE");

  display.setCursor(0, 18);
  display.print(F("TEMP      "));
  if (!sensorError) {
    display.print(currentTemperature, 1);
    display.print(F(" "));
    display.write(247);
    display.print(F("C"));
  } else {
    display.print(F("---"));
  }

  display.setCursor(0, 32);
  display.print(F("SETPOINT  "));
  display.print(setpoint, 1);
  display.print(F(" "));
  display.write(247);
  display.print(F("C"));

  display.setCursor(14, 52);
  if (systemRunning) {
    display.print(F("PRESS FOR MENU"));
  } else {
    display.print(F("PRESS TO START"));
  }
}

void showRunningScreen() {
  drawHeader("CURING", "HEATING");

  display.setCursor(0, 16);
  display.print(F("TEMP     "));
  if (currentSafetyState != SAFETY_SENSOR_ERROR) {
    display.print(currentTemperature, 1);
    display.print(F(" "));
    display.write(247);
    display.print(F("C"));
  } else {
    display.print(F("---"));
  }

  display.setCursor(0, 28);
  if (currentSafetyState == SAFETY_SENSOR_ERROR) {
    display.print(F("STATUS   SENSOR ERROR"));
  } else if (currentSafetyState == SAFETY_OVER_TEMP) {
    display.print(F("STATUS   OVER TEMP"));
  } else {
    display.print(F("SET      "));
    display.print(setpoint, 1);
    display.print(F(" "));
    display.write(247);
    display.print(F("C"));
  }

  display.setCursor(0, 40);
  display.print(F("HEATER   "));
  if (!heaterAllowed || currentSafetyState != SAFETY_OK) {
    display.print(F("OFF"));
  } else if (systemRunning) {
    display.print((int)pidOutput);
    display.print(F(" %"));
  } else {
    display.print(F("OFF"));
  }

  display.setCursor(0, 52);
  display.print(F("TIME     "));
  uint32_t mins = curingRemainingSeconds / 60;
  uint32_t secs = curingRemainingSeconds % 60;
  char timeBuf[12];
  snprintf(timeBuf, sizeof(timeBuf), "%02lu:%02lu", (unsigned long)mins, (unsigned long)secs);
  display.print(timeBuf);
}

void showMainMenu() {
  drawHeader("MAIN MENU", NULL);

  const char* activeLabel = (mainMenuCursor == 0 && systemRunning) ? "STOP" : mainMenuItems[mainMenuCursor];

  display.setCursor(2, 26);
  display.print(F("<"));
  display.setCursor(120, 26);
  display.print(F(">"));

  int len = strlen(activeLabel);
  int textSize = (len > 10) ? 1 : 2;
  display.setTextSize(textSize);

  int charWidth = 6 * textSize;
  int textWidth = len * charWidth;
  int xPos = (128 - textWidth) / 2;
  if (xPos < 12) xPos = 12;

  int yPos = (textSize == 1) ? 28 : 24;
  display.setCursor(xPos, yPos);
  display.print(activeLabel);
  display.setTextSize(1);

  if (mainMenuCursor == 4 && systemRunning) {
    display.setCursor(30, 44);
    display.print(F("(LOCKED WHILE RUNNING)"));
  }

  int dotCount = MAIN_MENU_COUNT;
  int dotRadius = 3;
  int dotSpacing = 12;
  int startX = (128 - ((dotCount - 1) * dotSpacing)) / 2;
  int dotY = 54;

  for (int i = 0; i < dotCount; i++) {
    int dotX = startX + (i * dotSpacing);
    if (i == mainMenuCursor) {
      display.fillCircle(dotX, dotY, dotRadius, SSD1306_WHITE);
    } else {
      display.drawCircle(dotX, dotY, dotRadius, SSD1306_WHITE);
    }
  }
}

void showTimerScreen() {
  drawHeader("TIMER SETTING", NULL);

  char timerBuf[16];
  if (tempTimerMinutes < 100) {
    snprintf(timerBuf, sizeof(timerBuf), "%02lu:00", (unsigned long)tempTimerMinutes);
  } else {
    snprintf(timerBuf, sizeof(timerBuf), "%lu:00", (unsigned long)tempTimerMinutes);
  }

  display.setTextSize(2);
  int len = strlen(timerBuf);
  int xPos = (128 - (len * 12)) / 2;
  if (xPos < 0) xPos = 0;
  display.setCursor(xPos, 22);
  display.print(timerBuf);
  display.setTextSize(1);

  display.setCursor(14, 44);
  display.print(F("ROTATE = ADJUST"));
  display.setCursor(24, 54);
  display.print(F("PRESS  = SAVE"));
}

void showCalibrationScreen() {
  drawHeader("CALIBRATION", NULL);

  if (isCalibSavedMessage) {
    display.setTextSize(1);
    display.setCursor(14, 26);
    display.print(F("CALIBRATION SAVED"));
    display.setCursor(24, 44);
    display.print(F("OFFSET: "));
    if (calibrationOffset >= 0) display.print(F("+"));
    display.print(calibrationOffset, 1);
    display.print(F(" "));
    display.write(247);
    display.print(F("C"));
  } else {
    display.setCursor(46, 18);
    display.print(F("OFFSET"));

    char offsetBuf[16];
    snprintf(offsetBuf, sizeof(offsetBuf), "%+.1f", tempCalibOffset);

    display.setTextSize(2);
    int len = strlen(offsetBuf) + 2;
    int xPos = (128 - (len * 12)) / 2;
    if (xPos < 0) xPos = 0;

    display.setCursor(xPos, 28);
    display.print(offsetBuf);
    display.setTextSize(1);
    display.print(F(" "));
    display.write(247);
    display.setTextSize(2);
    display.print(F("C"));
    display.setTextSize(1);

    display.setCursor(14, 45);
    display.print(F("ROTATE = ADJUST"));
    display.setCursor(24, 55);
    display.print(F("PRESS  = SAVE"));
  }
}

void showStartConfirmScreen() {
  const char* title = systemRunning ? "STOP OVEN" : "START OVEN";
  drawHeader(title, NULL);

  display.setCursor(4, 20);
  if (systemRunning) {
    display.print(F("Stop curing oven?"));
  } else {
    display.print(F("Start curing oven?"));
  }

  display.setCursor(20, 40);
  if (confirmOption == 0) display.print(F("> YES"));
  else                    display.print(F("  YES"));

  display.setCursor(70, 40);
  if (confirmOption == 1) display.print(F("> NO"));
  else                    display.print(F("  NO"));
}

void showSetpointScreen() {
  drawHeader("SETPOINT", NULL);

  display.setTextSize(2);
  display.setCursor(20, 20);
  display.print(tempSetpoint, 1);
  display.setTextSize(1);
  display.write(247);
  display.setTextSize(2);
  display.print(F("C"));
  display.setTextSize(1);

  display.setCursor(14, 44);
  display.print(F("ROTATE TO ADJUST"));
  display.setCursor(24, 54);
  display.print(F("PRESS TO SAVE"));
}

void showPIDMenu() {
  char bufKp[20], bufKi[20], bufKd[20];
  snprintf(bufKp, sizeof(bufKp), "Kp       %.2f", Kp);
  snprintf(bufKi, sizeof(bufKi), "Ki       %.2f", Ki);
  snprintf(bufKd, sizeof(bufKd), "Kd       %.2f", Kd);

  const char* items[] = {
    bufKp,
    bufKi,
    bufKd,
    "BACK"
  };
  drawMenuWindow("PID SETTINGS", items, PID_MENU_COUNT, pidMenuCursor);
}

void showPIDEditScreen() {
  const char* titles[] = {"EDIT Kp", "EDIT Ki", "EDIT Kd"};
  drawHeader(titles[selectedPIDParam], NULL);

  display.setTextSize(2);
  display.setCursor(35, 20);
  display.print(tempPIDValue, 2);
  display.setTextSize(1);

  display.setCursor(14, 44);
  display.print(F("ROTATE TO ADJUST"));
  display.setCursor(24, 54);
  display.print(F("PRESS TO SAVE"));
}

void showManualMenu() {
  char bufHeater[20], bufFan[20];
  snprintf(bufHeater, sizeof(bufHeater), "HEATER    [%s]", heaterManual ? "ON" : "OFF");
  snprintf(bufFan, sizeof(bufFan), "FAN       [%s]", fanManual ? "ON" : "OFF");

  const char* items[] = {
    bufHeater,
    bufFan,
    "BACK"
  };
  drawMenuWindow("MANUAL", items, MANUAL_MENU_COUNT, manualMenuCursor);
}

void showManualToggleScreen(const char* title, int currentVal) {
  drawHeader(title, NULL);

  display.setCursor(20, 24);
  if (currentVal == 0) display.print(F("> OFF"));
  else                 display.print(F("  OFF"));

  display.setCursor(70, 24);
  if (currentVal == 1) display.print(F("> ON"));
  else                 display.print(F("  ON"));

  display.setCursor(14, 50);
  display.print(F("PRESS TO CONFIRM"));
}

void showSystemMenu() {
  drawHeader("SYSTEM", NULL);

  const char* activeLabel = systemMenuItems[systemMenuCursor];

  display.setCursor(2, 26);
  display.print(F("<"));
  display.setCursor(120, 26);
  display.print(F(">"));

  int len = strlen(activeLabel);
  int textSize = (len > 10) ? 1 : 2;
  display.setTextSize(textSize);

  int charWidth = 6 * textSize;
  int textWidth = len * charWidth;
  int xPos = (128 - textWidth) / 2;
  if (xPos < 12) xPos = 12;

  int yPos = (textSize == 1) ? 28 : 24;
  display.setCursor(xPos, yPos);
  display.print(activeLabel);
  display.setTextSize(1);

  int dotCount = SYSTEM_MENU_COUNT;
  int dotRadius = 3;
  int dotSpacing = 14;
  int startX = (128 - ((dotCount - 1) * dotSpacing)) / 2;
  int dotY = 54;

  for (int i = 0; i < dotCount; i++) {
    int dotX = startX + (i * dotSpacing);
    if (i == systemMenuCursor) {
      display.fillCircle(dotX, dotY, dotRadius, SSD1306_WHITE);
    } else {
      display.drawCircle(dotX, dotY, dotRadius, SSD1306_WHITE);
    }
  }
}

void showSystemSensorScreen() {
  drawHeader("SENSOR", NULL);

  display.setCursor(0, 18);
  display.print(F("TEMP   : "));
  if (!sensorError) {
    display.print(currentTemperature, 2);
    display.print(F(" "));
    display.write(247);
    display.print(F("C"));
  } else {
    display.print(F("---"));
  }

  display.setCursor(0, 30);
  display.print(F("STATUS : "));
  display.print(sensorError ? F("ERROR") : F("OK"));

  display.setCursor(0, 46);
  display.print(F("MAX6675: "));
  display.print(sensorError ? F("ERROR") : F("CONNECTED"));
}

void showSystemDeviceScreen() {
  drawHeader("DEVICE", NULL);

  display.setCursor(0, 20);
  display.print(F("MCU: ESP32"));

  display.setCursor(0, 36);
  display.print(F("UI : OLED 128x64"));
}

void showSystemVersionScreen() {
  drawHeader("VERSION", NULL);

  display.setCursor(0, 20);
  display.print(F("OVEN CONTROLLER"));

  display.setCursor(0, 36);
  display.print(F("FW VERSION: 1.4.2"));
}

void showSystemResetConfirmScreen() {
  drawHeader("RISET", NULL);

  if (isResetDoneMessage) {
    display.setCursor(20, 24);
    display.print(F("SETTING RESET"));
    display.setCursor(26, 40);
    display.print(F("TO DEFAULT"));
  } else {
    display.setCursor(4, 20);
    display.print(F("Reset semua"));
    display.setCursor(4, 32);
    display.print(F("setting?"));

    display.setCursor(20, 48);
    if (resetConfirmOption == 0) display.print(F("> YES"));
    else                         display.print(F("  YES"));

    display.setCursor(70, 48);
    if (resetConfirmOption == 1) display.print(F("> NO"));
    else                         display.print(F("  NO"));
  }
}

// ================================================================
// BLOCK 13 - LOGGER
// ================================================================


void logCSVData() {
  static unsigned long lastLogTime = 0;
  static bool wasRunning = false;

  // Reset waktu ketika proses curing baru dimulai
  // Inisialisasi lastLogTime agar sampel t=0.0 langsung terkirim pada iterasi pertama
  if (systemRunning && !wasRunning) {
    wasRunning = true;
    lastLogTime = millis() - CSV_LOG_INTERVAL_MS;
  }

  // Berhenti logging ketika curing selesai
  if (!systemRunning) {
    wasRunning = false;
    return;
  }

  unsigned long now = millis();

  // Logging periodik setiap 1000 ms (1 detik)
  if (now - lastLogTime >= CSV_LOG_INTERVAL_MS) {
    lastLogTime = now;

    float error = setpoint - currentTemperature;
    float timeSeconds = (now - curingStartTime) / 1000.0f;

    // 1. Waktu pengujian (detik)
    Serial.print(timeSeconds, 1);
    Serial.print(",");

    // 2. Temperatur aktual
    Serial.print(currentTemperature, 2);
    Serial.print(",");

    // 3. Setpoint
    Serial.print(setpoint, 2);
    Serial.print(",");

    // 4. Error (setpoint - currentTemperature)
    Serial.print(error, 2);
    Serial.print(",");

    // 5. P-Term
    Serial.print(pidPTerm, 2);
    Serial.print(",");

    // 6. I-Term (akumulasi integral)
    Serial.print(pidIntegralTerm, 2);
    Serial.print(",");

    // 7. D-Term
    Serial.print(pidDTerm, 2);
    Serial.print(",");

    // 8. PID Output (0.00 - 100.00 %) — output SETELAH saturasi
    Serial.print(pidOutput, 2);
    Serial.print(",");

    // 9. Kp (nilai aktif, untuk verifikasi bahwa NVS sudah benar)
    Serial.print(Kp, 3);
    Serial.print(",");

    // 10. Ki
    Serial.print(Ki, 3);
    Serial.print(",");

    // 11. Kd
    Serial.print(Kd, 3);
    Serial.print(",");

    // 12. SSR ON-Time (ms) pada window saat ini
    Serial.print(ssrWindowOnTimeMs);
    Serial.print(",");

    // 13. Heater State (1 = ON, 0 = OFF) — status GPIO SSR aktual
    Serial.println(heaterState ? 1 : 0);
  }
}



// ================================================================
// BLOCK 14 - INITIALIZATION
// ================================================================

void setup() {
  emergencyPinsSafeState();

  Serial.begin(115200);

  esp_task_wdt_config_t wdtConfig = {
    .timeout_ms = WDT_TIMEOUT_SECONDS * 1000,
    .idle_core_mask = 0,
    .trigger_panic = true
  };
  esp_task_wdt_init(&wdtConfig);
  esp_task_wdt_add(NULL);

  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    for (;;) {
      esp_task_wdt_reset();
      yield();
    }
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  setupEncoder();
  setupActuators();
  setupMAX6675();
  setupBuzzer();
  loadSettings();

  checkSafetySystem();

  bootStartTime = millis();
  currentScreen = SCREEN_BOOT;
  resetUIInactivityTimer();
  displayNeedsUpdate = true;
}

// ================================================================
// BLOCK 15 - MAIN LOOP
// ================================================================

void loop() {
  esp_task_wdt_reset();

  handleInput();
  updateUI();
  updateDisplay();
  updateSystem();
}

void updateSystem() {
  readTemperatureSensor();
  checkSafetySystem();
  computePID();
  updateOutputs();
  updateBuzzer();
  logCSVData();
}
