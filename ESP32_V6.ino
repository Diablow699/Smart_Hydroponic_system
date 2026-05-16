/*
 * COMBINED ESP32 CODE — UPDATED v2.3
 * Receives packet format from Arduino Uno:
 *   <wl1,wl2,wl3,temp,hum,0,ec>
 *
 * Includes:
 *   - Ultrasonic tank level display/handling
 *   - DHT22 humidity control logic (pulse-based)
 *   - EC dosing pump logic (pulse-based state machine)
 *   - EC diaphragm pump logic (dilution + refill)
 *   - DC mixing motor control
 *   - RTC + LED grow light logic
 *   - Blynk IoT updates
 *   - Priority-based pump protection system
 *   - EC auto growth stage via RTC
 *   - TFT LCD display via TFT_Display.h
 *   - Offline-safe WiFi/Blynk (non-blocking, runs without network)  ← NEW in v2.3
 *   - LED grow light debounce fix (no more flicker at boundary)     ← NEW in v2.3
 *
 * PRIORITY SYSTEM:
 *   Priority 1 — Reservoir: gates all diaphragm operations
 *   Priority 2 — Mixing Tank: EC correction + submersible cycle
 *   Priority 3 — Nutrient Tank: gates dosing pump
 *
 * NEW PINS:
 *   GPIO 33 — Diaphragm pump relay
 *   GPIO 19 — DC Mixing motor relay
 *   TFT     — configured via TFT_eSPI User_Setup.h
 *
 * STAGE CONTROL BUTTONS (Blynk):
 *   V30 — Seedling inserted button
 *   V32 — Reset button (clears all stage state, unlocks V30 and V35)
 *   V35 — Grown stage button (only usable after V32 reset)
 *   V33 — Seedling phase LED indicator
 *   V34 — Grown phase LED indicator
 *   V29 — Elapsed days display
 */

// ================= BLYNK CREDENTIALS =================
#define BLYNK_TEMPLATE_ID   "TMPL6IkGktaE7"
#define BLYNK_TEMPLATE_NAME "Hydroponic"
#define BLYNK_AUTH_TOKEN    "zYQlrbS2K7Jp49GZbNJFqKjrvO9EtnCP"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <RTClib.h>
#include "TFT_Display.h"   // ← TFT: pulls in SPI, TFT_eSPI, and all display logic

// ================= WIFI =================
char ssid[] = "Diablow";
char pass[] = "12345678";

// ================= OFFLINE-SAFE WIFI/BLYNK =================
// WiFi connection attempt will time out after this many milliseconds.
// If the network is unavailable the system boots and runs fully offline.
// Blynk reconnects automatically in the background whenever WiFi returns.
#define WIFI_CONNECT_TIMEOUT_MS  10000UL   // 10 seconds max wait on boot

bool blynkOnline = false;   // true once WiFi + Blynk are confirmed connected

// ================= UART FROM ARDUINO UNO =================
#define PIN_RX_FROM_UNO  16
#define PIN_TX_TO_UNO    17

// ================= RTC I2C PINS =================
#define PIN_RTC_SDA 25
#define PIN_RTC_SCL 22

// ================= RELAYS — ORIGINAL =================
#define PIN_RELAY_EXHAUST      26
#define PIN_RELAY_MISTER       27
#define PIN_RELAY_DOSING       14
#define PIN_RELAY_SUBMERSIBLE  13
#define PIN_RELAY_LIGHTS       32

// ================= RELAYS — NEW =================
#define PIN_RELAY_DIAPHRAGM    33
#define PIN_RELAY_MIXING_MOTOR 19

#define RELAY_ON  LOW
#define RELAY_OFF HIGH

// ================= HUMIDITY THRESHOLDS =================
float HUMIDITY_MIN        = 67.26;
float HUMIDITY_MAX        = 69.72;
float HUMIDITY_HYSTERESIS = 0.25;

// ================= EC THRESHOLDS =================
float EC_HYSTERESIS = 0.05;

// ================= EC GROWTH STAGE TARGETS =================
float EC_SEEDLING_MIN = 0.5;
float EC_SEEDLING_MAX = 1.2;

float EC_GROWN_MIN = 1.2;
float EC_GROWN_MAX = 2.0;

// ================= EC GROWTH STAGE TRACKING =================
bool     seedlingModeActive = false;
bool     seedlingTimeSet    = false;
DateTime seedlingStartTime;

// ================= STAGE CONTROL VARIABLES =================
// grownModeActive  — true when user pressed grown button (V35)
// resetUnlocked    — true only after reset (V32) pressed,
//                    unlocks both V30 (seedling) and V35 (grown)
// GROWN_OFFSET_SEC — 12 days in seconds, subtracted from rtc.now()
//                    when grown button pressed so timer starts at day 12
bool grownModeActive = false;
bool resetUnlocked   = false;
#define GROWN_OFFSET_SEC  (12L * 86400L)  // 12 days in seconds
#define GROWN_MAX_DAYS    28              // grown stage ends at day 28

// ================= TANK HEIGHT CONSTANTS =================
#define TANK1_HEIGHT_CM  35.0
#define TANK2_HEIGHT_CM  27.0
#define TANK3_HEIGHT_CM  17.0

// ================= PRIORITY 1 — RESERVOIR THRESHOLDS =================
#define RESERVOIR_BLOCK_PCT   30.0
#define RESERVOIR_DILUTE_PCT  50.0

// ================= PRIORITY 2 — MIXING TANK THRESHOLDS =================
#define MIXING_LOW_PCT        25.0
#define MIXING_TARGET_PCT     80.0

// ================= PRIORITY 3 — NUTRIENT TANK THRESHOLDS =================
#define NUTRIENT_SHUTOFF_PCT  20.0
#define NUTRIENT_REENABLE_PCT 25.0

// ================= PULSE / TIMING CONSTANTS =================
#define PULSE_ON_MS              5000UL
#define MIXING_MOTOR_MS          5000UL
#define EC_SETTLE_MS             15000UL
#define MAX_PULSE_COUNT          10
#define SUBMERSIBLE_CYCLE_MS     900000UL

// ================= EC DEAD BAND =================
#define EC_DEAD_BAND             0.10f

// ================= HUMIDITY PULSE TIMING =================
#define HUMIDITY_PULSE_ON_MS        5000UL
#define HUMIDITY_SETTLE_MS          15000UL
#define HUMIDITY_MAX_PULSE          10
#define HUMIDITY_FAULT_DURATION_MS  300000UL
#define HUMIDITY_CRITICAL_LOW       50.0
#define HUMIDITY_CRITICAL_HIGH      80.0

// ================= TANK LOW LEVEL THRESHOLDS =================
float TANK1_LOW_LEVEL_CM = 10.5;
float TANK2_LOW_LEVEL_CM = 5.08;
float TANK3_LOW_LEVEL_CM = 4.25;

// ================= LIGHT SCHEDULE =================
const int LIGHTS_ON_HOUR  = 18;
const int LIGHTS_ON_MIN   = 0;
const int LIGHTS_OFF_HOUR = 6;
const int LIGHTS_OFF_MIN  = 0;

// ================= SENSOR VARIABLES =================
float currentWaterLevel1 = 0.0;
float currentWaterLevel2 = 0.0;
float currentWaterLevel3 = 0.0;
float currentTemperature = 0.0;
float currentHumidity    = 0.0;
float currentEC          = 0.0;

// ================= RELAY STATES — ORIGINAL =================
bool stateMister          = false;
bool stateExhaust         = false;
bool stateDosingPump      = false;
bool stateSubmersiblePump = false;
bool stateLED             = false;

// ================= RELAY STATES — NEW =================
bool stateDiaphragm       = false;
bool stateMixingMotor     = false;

// ================= TANK LOW FLAGS =================
bool tank1Low = false;
bool tank2Low = false;
bool tank3Low = false;

// ================= SERIAL PARSING =================
String serialBuffer = "";

// ================= RTC =================
RTC_DS3231 rtc;
char rtcTimeString[25] = "00:00:00 00/00/0000";
bool rtcAvailable = false;

// ================= RTC TIMING =================
unsigned long lastRTCRead = 0;
const unsigned long RTC_READ_INTERVAL_MS = 1000;

// ================= NOTIFICATION VARIABLES =================
#define NOTIFY_COOLDOWN_MS 300000UL

unsigned long lastNotifyTank1 = 0;
unsigned long lastNotifyTank2 = 0;
unsigned long lastNotifyTank3 = 0;

// ================= BLYNK TIMER =================
BlynkTimer timer;

// ================= STATE MACHINE =================
enum SystemState {
  STATE_IDLE,
  STATE_REFILLING_MIXING,
  STATE_EC_CORRECTION,
  STATE_SUBMERSIBLE_CYCLE,
  STATE_FAULT
};
SystemState systemState = STATE_IDLE;

// ================= EC CORRECTION TRACKING =================
int           pulseCount         = 0;
bool          pulsing            = false;
bool          motorRunning       = false;
bool          waitingSettle      = false;
bool          ecCorrectionIsHigh = false;
unsigned long pulseStartMs       = 0;
unsigned long motorStartMs       = 0;
unsigned long settleStartMs      = 0;

// ================= SUBMERSIBLE CYCLE TRACKING =================
unsigned long submersibleStartMs = 0;

// ================= REFILL TRACKING =================
bool refillActive = false;

// ================= NUTRIENT DOSING GATE =================
bool nutrientDosingAllowed = true;

// ================= HUMIDITY PULSE TRACKING =================
bool          misterPulsing       = false;
bool          exhaustPulsing      = false;
unsigned long misterPulseStartMs  = 0;
unsigned long exhaustPulseStartMs = 0;

bool          misterSettling       = false;
bool          exhaustSettling      = false;
unsigned long misterSettleStartMs  = 0;
unsigned long exhaustSettleStartMs = 0;

int           misterPulseCount    = 0;
int           exhaustPulseCount   = 0;

bool          misterFault         = false;
bool          exhaustFault        = false;

unsigned long misterCriticalStartMs  = 0;
unsigned long exhaustCriticalStartMs = 0;
bool          misterCriticalActive   = false;
bool          exhaustCriticalActive  = false;

unsigned long lastNotifyMisterFault  = 0;
unsigned long lastNotifyExhaustFault = 0;

// ================= NOTIFICATION TRACKING =================
unsigned long lastNotifyReservoirLow  = 0;
unsigned long lastNotifyNutrientLow   = 0;
unsigned long lastNotifyFault         = 0;
unsigned long lastNotifyCannotRefill  = 0;
unsigned long lastNotifyHarvest       = 0;  // harvest alert — repeats every 24h after day 28

// ================= PERCENTAGE HELPERS =================
float getReservoirPct() { return (currentWaterLevel1 / TANK1_HEIGHT_CM) * 100.0; }
float getNutrientPct()  { return (currentWaterLevel2 / TANK2_HEIGHT_CM) * 100.0; }
float getMixingPct()    { return (currentWaterLevel3 / TANK3_HEIGHT_CM) * 100.0; }

// ================= PRIORITY 1: DIAPHRAGM PERMISSION =================
int getDiaphragmPermission() {
  float pct = getReservoirPct();
  if (pct < RESERVOIR_BLOCK_PCT)  return 0;
  if (pct < RESERVOIR_DILUTE_PCT) return 1;
  return 2;
}

// ================= PRIORITY 3: NUTRIENT DOSING GATE =================
bool checkNutrientDosingAllowed() {
  float pct = getNutrientPct();
  if (pct < NUTRIENT_SHUTOFF_PCT)   nutrientDosingAllowed = false;
  if (pct >= NUTRIENT_REENABLE_PCT) nutrientDosingAllowed = true;
  return nutrientDosingAllowed;
}

// ================= STOP ALL PUMPS HELPER =================
void stopAllPumps() {
  digitalWrite(PIN_RELAY_DOSING,       RELAY_OFF); stateDosingPump      = false;
  digitalWrite(PIN_RELAY_SUBMERSIBLE,  RELAY_OFF); stateSubmersiblePump = false;
  digitalWrite(PIN_RELAY_DIAPHRAGM,    RELAY_OFF); stateDiaphragm       = false;
  digitalWrite(PIN_RELAY_MIXING_MOTOR, RELAY_OFF); stateMixingMotor     = false;
  pulsing       = false;
  motorRunning  = false;
  waitingSettle = false;
  refillActive  = false;
}

// ================= REMOTE PRINT =================
// Sends to both Serial Monitor and Blynk Terminal (V31)
void remotePrint(String msg) {
  Serial.println(msg);
  if (Blynk.connected()) {
    Blynk.virtualWrite(V31, msg + "\n");
  }
}

// ================= BLYNK: RESET BUTTON (V32) =================
// Clears all stage state, turns off both LEDs, unlocks V30 and V35
BLYNK_WRITE(V32)
{
  if (param.asInt() == 1) {
    seedlingModeActive = false;
    seedlingTimeSet    = false;
    grownModeActive    = false;
    resetUnlocked      = true;   // unlocks both seedling and grown buttons
    lastNotifyHarvest  = 0;      // clear harvest alert so it can fire again next cycle

    if (Blynk.connected()) {
      Blynk.virtualWrite(V33, 0);  // Seedling LED OFF
      Blynk.virtualWrite(V34, 0);  // Grown LED OFF
      Blynk.virtualWrite(V29, 0);  // Days counter reset to 0
      Blynk.virtualWrite(V30, 0);  // Seedling button reset to OFF
      Blynk.virtualWrite(V35, 0);  // Grown button reset to OFF
    }
    remotePrint("[STAGE] Reset pressed — all stage state cleared, buttons unlocked");
  }
}

// ================= BLYNK: SEEDLING BUTTON (V30) =================
// Works on first boot OR after reset is pressed
BLYNK_WRITE(V30)
{
  if (param.asInt() == 1) {
    // Block if a stage is already active and reset hasn't been pressed
    if ((seedlingModeActive || grownModeActive) && !resetUnlocked) {
      remotePrint("[STAGE] Seedling button blocked — press RESET first");
      if (Blynk.connected()) Blynk.virtualWrite(V30, 0);
      return;
    }

    if (!rtcAvailable) {
      Serial.println("[STAGE] Cannot start seedling mode - RTC not available");
      if (Blynk.connected()) Blynk.virtualWrite(V30, 0);
      return;
    }

    // Activate seedling mode — timer starts fresh at day 0
    seedlingModeActive = true;
    seedlingTimeSet    = true;
    grownModeActive    = false;
    seedlingStartTime  = rtc.now();
    resetUnlocked      = false;  // lock grown button again

    if (Blynk.connected()) {
      Blynk.virtualWrite(V33, 1);  // Seedling LED ON
      Blynk.virtualWrite(V34, 0);  // Grown LED OFF
      Blynk.virtualWrite(V35, 0);  // Grown button OFF
    }
    remotePrint("[STAGE] Seedling inserted — timer started at day 0, seedling EC range active");
    Serial.print("[STAGE] Start time: ");
    Serial.println(rtcTimeString);
  }
}

// ================= BLYNK: GROWN STAGE BUTTON (V35) =================
// ONLY works after reset is pressed — hard locked otherwise
BLYNK_WRITE(V35)
{
  if (param.asInt() == 1) {
    // Hard block — grown button does nothing unless reset was pressed
    if (!resetUnlocked) {
      remotePrint("[STAGE] Grown button blocked — press RESET first");
      if (Blynk.connected()) Blynk.virtualWrite(V35, 0);
      return;
    }

    if (!rtcAvailable) {
      Serial.println("[STAGE] Cannot start grown mode - RTC not available");
      if (Blynk.connected()) Blynk.virtualWrite(V35, 0);
      return;
    }

    // Activate grown mode
    // Fake start time = now minus 12 days so elapsedDays begins at 12
    grownModeActive    = true;
    seedlingModeActive = true;   // reuse seedling timer infrastructure
    seedlingTimeSet    = true;
    resetUnlocked      = false;  // lock both buttons again

    DateTime nowDt    = rtc.now();
    seedlingStartTime = DateTime(nowDt.unixtime() - GROWN_OFFSET_SEC);

    if (Blynk.connected()) {
      Blynk.virtualWrite(V33, 0);  // Seedling LED OFF
      Blynk.virtualWrite(V34, 1);  // Grown LED ON
      Blynk.virtualWrite(V30, 0);  // Seedling button OFF
    }
    remotePrint("[STAGE] Grown stage activated — timer at day 12, grown EC range active");
  }
}

// ================= EC STAGE FUNCTION =================
// Uses seedlingStartTime for both normal and grown-override mode.
// Normal:  day 0-11  = seedling EC (0.5-1.2), day 12+ = grown EC (1.2-2.0)
// Grown:   timer starts at day 12 already   = grown EC immediately
void getECTarget(float &ecMin, float &ecMax)
{
  if (!seedlingModeActive || !seedlingTimeSet || !rtcAvailable)
  {
    ecMin = EC_GROWN_MIN;
    ecMax = EC_GROWN_MAX;
    return;
  }

  DateTime now     = rtc.now();
  long elapsedDays = (long)(now.unixtime() - seedlingStartTime.unixtime()) / 86400L;

  Serial.print("[STAGE] Days elapsed: ");
  Serial.println(elapsedDays);

  if (elapsedDays < 12)
  {
    ecMin = EC_SEEDLING_MIN;
    ecMax = EC_SEEDLING_MAX;
    Serial.println("[STAGE] Seedling EC range (0.5 - 1.2 dS/m)");
  }
  else
  {
    ecMin = EC_GROWN_MIN;
    ecMax = EC_GROWN_MAX;
    Serial.println("[STAGE] Grown EC range (1.2 - 2.0 dS/m)");
  }
}

// ================= RECEIVE DATA =================
void receiveSerialData() {
  while (Serial2.available()) {
    char c = Serial2.read();

    if (c == '<') {
      serialBuffer = "";
    }
    else if (c == '>') {
      float wl1, wl2, wl3, temp, hum, tds, ec;

      int parsed = sscanf(serialBuffer.c_str(),
                          "%f,%f,%f,%f,%f,%f,%f",
                          &wl1, &wl2, &wl3,
                          &temp, &hum,
                          &tds, &ec);

      if (parsed == 7) {
        currentWaterLevel1 = wl1;
        currentWaterLevel2 = wl2;
        currentWaterLevel3 = wl3;
        currentTemperature = temp;
        currentHumidity    = hum;
        currentEC          = ec;

        tank1Low = (currentWaterLevel1 <= TANK1_LOW_LEVEL_CM);
        tank2Low = (currentWaterLevel2 <= TANK2_LOW_LEVEL_CM);
        tank3Low = (currentWaterLevel3 <= TANK3_LOW_LEVEL_CM);

        Serial.print("[OK] WL1: ");
        Serial.print(currentWaterLevel1, 2);
        Serial.print(" cm | WL2: ");
        Serial.print(currentWaterLevel2, 2);
        Serial.print(" cm | WL3: ");
        Serial.print(currentWaterLevel3, 2);
        Serial.print(" cm | Temp: ");
        Serial.print(currentTemperature, 2);
        Serial.print(" C | Hum: ");
        Serial.print(currentHumidity, 2);
        Serial.print(" % | EC: ");
        Serial.print(currentEC, 3);
        Serial.println(" dS/m");
      } else {
        Serial.print("[ERROR] Invalid packet: ");
        Serial.println(serialBuffer);
      }
    }
    else {
      serialBuffer += c;
    }
  }
}

// ================= HUMIDITY CONTROL (PULSE-BASED v2) =================
void controlHumidity() {
  unsigned long now = millis();

  float rhMisterActivate    = HUMIDITY_MIN - HUMIDITY_HYSTERESIS;
  float rhMisterDeactivate  = HUMIDITY_MIN + HUMIDITY_HYSTERESIS;
  float rhExhaustActivate   = HUMIDITY_MAX + HUMIDITY_HYSTERESIS;
  float rhExhaustDeactivate = HUMIDITY_MAX - HUMIDITY_HYSTERESIS;

  // --- AUTO-CLEAR FAULTS ---
  if (misterFault && currentHumidity >= HUMIDITY_CRITICAL_LOW) {
    misterFault          = false;
    misterPulseCount     = 0;
    misterCriticalActive = false;
    remotePrint("[MISTER] Fault cleared — humidity recovered");
  }
  if (exhaustFault && currentHumidity <= HUMIDITY_CRITICAL_HIGH) {
    exhaustFault          = false;
    exhaustPulseCount     = 0;
    exhaustCriticalActive = false;
    remotePrint("[EXHAUST] Fault cleared — humidity recovered");
  }

  // --- CRITICAL TRACKING + ALERT ---
  if (misterFault && currentHumidity < HUMIDITY_CRITICAL_LOW) {
    if (!misterCriticalActive) {
      misterCriticalActive  = true;
      misterCriticalStartMs = now;
    }
    else if (now - misterCriticalStartMs >= HUMIDITY_FAULT_DURATION_MS) {
      if (Blynk.connected() && now - lastNotifyMisterFault >= NOTIFY_COOLDOWN_MS) {
        Blynk.logEvent("system_fault",
                       "Humidity critically low! Possible mister failure. RH: "
                       + String(currentHumidity, 1) + "%");
        lastNotifyMisterFault = now;
      }
    }
  }
  else { misterCriticalActive = false; }

  if (exhaustFault && currentHumidity > HUMIDITY_CRITICAL_HIGH) {
    if (!exhaustCriticalActive) {
      exhaustCriticalActive  = true;
      exhaustCriticalStartMs = now;
    }
    else if (now - exhaustCriticalStartMs >= HUMIDITY_FAULT_DURATION_MS) {
      if (Blynk.connected() && now - lastNotifyExhaustFault >= NOTIFY_COOLDOWN_MS) {
        Blynk.logEvent("system_fault",
                       "Humidity critically high! Possible exhaust failure. RH: "
                       + String(currentHumidity, 1) + "%");
        lastNotifyExhaustFault = now;
      }
    }
  }
  else { exhaustCriticalActive = false; }

  // --- MISTER LOGIC ---
  if (!misterFault) {
    if (misterSettling) {
      if (now - misterSettleStartMs >= HUMIDITY_SETTLE_MS) {
        misterSettling = false;
        remotePrint("[MISTER] Settle complete — re-checking RH");
      }
    }
    else if (misterPulsing) {
      if (now - misterPulseStartMs >= HUMIDITY_PULSE_ON_MS) {
        digitalWrite(PIN_RELAY_MISTER, RELAY_OFF);
        stateMister         = false;
        misterPulsing       = false;
        misterSettling      = true;
        misterSettleStartMs = now;
        remotePrint("[MISTER] Pulse complete — settling 15s");
      }
    }
    else if (!stateMister
             && currentHumidity < rhMisterActivate
             && !stateExhaust
             && !exhaustPulsing
             && !exhaustSettling) {
      if (misterPulseCount >= HUMIDITY_MAX_PULSE) {
        misterFault      = true;
        misterPulseCount = 0;
        remotePrint("[MISTER] FAULT — 10 pulses exhausted, humidity unresolved");
      }
      else {
        digitalWrite(PIN_RELAY_MISTER, RELAY_ON);
        stateMister        = true;
        misterPulsing      = true;
        misterPulseStartMs = now;
        misterPulseCount++;
        Serial.print("[MISTER] Pulse ");
        Serial.print(misterPulseCount);
        Serial.println("/10 ON — RH below minimum");
      }
    }
    else if (stateMister && currentHumidity >= rhMisterDeactivate) {
      misterPulseCount = 0;
    }
  }

  // --- EXHAUST LOGIC ---
  if (!exhaustFault) {
    if (exhaustSettling) {
      if (now - exhaustSettleStartMs >= HUMIDITY_SETTLE_MS) {
        exhaustSettling = false;
        remotePrint("[EXHAUST] Settle complete — re-checking RH");
      }
    }
    else if (exhaustPulsing) {
      if (now - exhaustPulseStartMs >= HUMIDITY_PULSE_ON_MS) {
        digitalWrite(PIN_RELAY_EXHAUST, RELAY_OFF);
        stateExhaust         = false;
        exhaustPulsing       = false;
        exhaustSettling      = true;
        exhaustSettleStartMs = now;
        remotePrint("[EXHAUST] Pulse complete — settling 15s");
      }
    }
    else if (!stateExhaust
             && currentHumidity > rhExhaustActivate
             && !stateMister
             && !misterPulsing
             && !misterSettling) {
      if (exhaustPulseCount >= HUMIDITY_MAX_PULSE) {
        exhaustFault      = true;
        exhaustPulseCount = 0;
        remotePrint("[EXHAUST] FAULT — 10 pulses exhausted, humidity unresolved");
      }
      else {
        digitalWrite(PIN_RELAY_EXHAUST, RELAY_ON);
        stateExhaust        = true;
        exhaustPulsing      = true;
        exhaustPulseStartMs = now;
        exhaustPulseCount++;
        Serial.print("[EXHAUST] Pulse ");
        Serial.print(exhaustPulseCount);
        Serial.println("/10 ON — RH above maximum");
      }
    }
    else if (stateExhaust && currentHumidity <= rhExhaustDeactivate) {
      exhaustPulseCount = 0;
    }
  }

  // --- INTERLOCK ---
  if (stateMister && stateExhaust) {
    digitalWrite(PIN_RELAY_MISTER, RELAY_OFF);
    stateMister        = false;
    misterPulsing      = false;
    misterSettling     = false;
    remotePrint("[INTERLOCK] Mister OFF — Exhaust priority");
  }
}

// ================= MIXING TANK STATE MACHINE =================
void controlMixingTank() {
  unsigned long now = millis();

  float ecMin, ecMax;
  getECTarget(ecMin, ecMax);

  float mixingPct    = getMixingPct();
  float reservoirPct = getReservoirPct();
  int   diaphPerm    = getDiaphragmPermission();
  bool  dosingOK     = checkNutrientDosingAllowed();

  bool  ecInRange      = (currentEC >= ecMin && currentEC <= ecMax);
  float ecDoseThresh   = ecMin - EC_DEAD_BAND;
  float ecDiluteThresh = ecMax + EC_DEAD_BAND;

  // --- PRIORITY 1 ---
  if (diaphPerm == 0) {
    if (stateDiaphragm) {
      digitalWrite(PIN_RELAY_DIAPHRAGM, RELAY_OFF);
      stateDiaphragm = false;
      remotePrint("[P1] Diaphragm BLOCKED - Reservoir critically low");
    }
    if (systemState == STATE_REFILLING_MIXING ||
       (systemState == STATE_EC_CORRECTION && ecCorrectionIsHigh)) {
      stopAllPumps();
      systemState = STATE_IDLE;
      remotePrint("[P1] State forced to IDLE - Reservoir < 30%");
      if (Blynk.connected() && now - lastNotifyCannotRefill >= NOTIFY_COOLDOWN_MS) {
        Blynk.logEvent("operational_alert", "Cannot refill mixing tank — reservoir too low!");
        lastNotifyCannotRefill = now;
      }
    }
    if (Blynk.connected() && now - lastNotifyReservoirLow >= NOTIFY_COOLDOWN_MS) {
      Blynk.logEvent("operational_alert",
                     "Reservoir critically low! Level: "
                     + String(reservoirPct, 1) + "%");
      lastNotifyReservoirLow = now;
    }
    return;
  }

  // --- PRIORITY 3 ---
  if (!dosingOK) {
    if (stateDosingPump) {
      digitalWrite(PIN_RELAY_DOSING, RELAY_OFF);
      stateDosingPump = false;
      pulsing         = false;
      remotePrint("[P3] Dosing pump OFF - Nutrient tank low");
    }
    if (Blynk.connected() && now - lastNotifyNutrientLow >= NOTIFY_COOLDOWN_MS) {
      Blynk.logEvent("operational_alert",
                     "Nutrient tank low! Dosing disabled. Level: "
                     + String(getNutrientPct(), 1) + "%");
      lastNotifyNutrientLow = now;
    }
    if (systemState == STATE_EC_CORRECTION && !ecCorrectionIsHigh) {
      stopAllPumps();
      systemState = STATE_IDLE;
      remotePrint("[P3] EC correction aborted - Nutrient tank too low");
    }
  }

  // --- STATE MACHINE ---
  switch (systemState) {

    case STATE_IDLE: {
      pulseCount = 0;

      if (mixingPct < MIXING_LOW_PCT) {
        remotePrint("[STATE] Mixing tank low — entering REFILLING state");
        systemState  = STATE_REFILLING_MIXING;
        refillActive = true;
        break;
      }

      if (mixingPct >= MIXING_TARGET_PCT) {
        bool ecClearlyLow  = (currentEC < ecDoseThresh);
        bool ecClearlyHigh = (currentEC > ecDiluteThresh);

        if (ecClearlyLow || ecClearlyHigh) {
          remotePrint("[STATE] EC out of range — entering EC_CORRECTION state");
          ecCorrectionIsHigh = ecClearlyHigh;
          systemState        = STATE_EC_CORRECTION;
          pulseCount         = 0;
          pulsing            = false;
          motorRunning       = false;
          waitingSettle      = false;
        }
        else {
          remotePrint("[STATE] EC OK & mixing full — entering SUBMERSIBLE_CYCLE");
          digitalWrite(PIN_RELAY_SUBMERSIBLE, RELAY_ON);
          stateSubmersiblePump = true;
          submersibleStartMs   = now;
          systemState          = STATE_SUBMERSIBLE_CYCLE;
        }
      }
      break;
    }

    case STATE_REFILLING_MIXING: {
      if (diaphPerm < 2) {
        stopAllPumps();
        systemState = STATE_IDLE;
        remotePrint("[REFILL] Cannot refill — reservoir below 50%");
        if (Blynk.connected() && now - lastNotifyCannotRefill >= NOTIFY_COOLDOWN_MS) {
          Blynk.logEvent("operational_alert",
                         "Cannot refill mixing tank — reservoir below 50%!");
          lastNotifyCannotRefill = now;
        }
        break;
      }

      if (mixingPct < MIXING_TARGET_PCT) {
        if (!stateDiaphragm) {
          digitalWrite(PIN_RELAY_DIAPHRAGM, RELAY_ON);
          stateDiaphragm = true;
          remotePrint("[REFILL] Diaphragm ON — filling mixing tank");
        }
      }
      else {
        digitalWrite(PIN_RELAY_DIAPHRAGM, RELAY_OFF);
        stateDiaphragm = false;
        refillActive   = false;
        remotePrint("[REFILL] Mixing tank at 80% — moving to EC_CORRECTION");
        ecCorrectionIsHigh = (currentEC > ecDiluteThresh);
        systemState        = STATE_EC_CORRECTION;
        pulseCount         = 0;
        pulsing            = false;
        motorRunning       = false;
        waitingSettle      = false;
      }
      break;
    }

    case STATE_EC_CORRECTION: {
      if (pulseCount >= MAX_PULSE_COUNT) {
        stopAllPumps();
        systemState = STATE_FAULT;
        remotePrint("[FAULT] Max pulse count reached — manual check required");
        if (Blynk.connected() && now - lastNotifyFault >= NOTIFY_COOLDOWN_MS) {
          Blynk.logEvent("system_fault",
                         "EC correction failed after 10 pulses! Manual check required.");
          lastNotifyFault = now;
        }
        break;
      }

      if (ecInRange) {
        stopAllPumps();
        pulseCount = 0;
        remotePrint("[EC] EC in range — moving to SUBMERSIBLE_CYCLE");
        digitalWrite(PIN_RELAY_SUBMERSIBLE, RELAY_ON);
        stateSubmersiblePump = true;
        submersibleStartMs   = now;
        systemState          = STATE_SUBMERSIBLE_CYCLE;
        break;
      }

      if (waitingSettle) {
        if (now - settleStartMs >= EC_SETTLE_MS) {
          waitingSettle = false;
          remotePrint("[EC] Settle complete — re-evaluating EC direction");

          if (ecInRange) { break; }

          bool nowClearlyLow  = (currentEC < ecDoseThresh);
          bool nowClearlyHigh = (currentEC > ecDiluteThresh);
          bool wasDosingAndNowHigh  = (!ecCorrectionIsHigh && nowClearlyHigh);
          bool wasDilutingAndNowLow = ( ecCorrectionIsHigh && nowClearlyLow);

          if (wasDosingAndNowHigh || wasDilutingAndNowLow) {
            remotePrint("[EC] OVERSHOOT detected — switching direction, resetting to IDLE");
            Serial.print("[EC] EC value: ");
            Serial.print(currentEC, 3);
            Serial.println(" dS/m");
            stopAllPumps();
            pulseCount         = 0;
            ecCorrectionIsHigh = nowClearlyHigh;
            systemState        = STATE_IDLE;
            break;
          }
          remotePrint("[EC] Still out of range — continuing same direction");
        }
        break;
      }

      if (motorRunning) {
        if (now - motorStartMs >= MIXING_MOTOR_MS) {
          digitalWrite(PIN_RELAY_MIXING_MOTOR, RELAY_OFF);
          stateMixingMotor = false;
          motorRunning     = false;
          waitingSettle    = true;
          settleStartMs    = now;
          remotePrint("[EC] Mixing motor OFF — settling 15s before EC read");
        }
        break;
      }

      if (pulsing) {
        if (now - pulseStartMs >= PULSE_ON_MS) {
          if (ecCorrectionIsHigh) {
            digitalWrite(PIN_RELAY_DIAPHRAGM, RELAY_OFF);
            stateDiaphragm = false;
            remotePrint("[EC] Diaphragm pulse OFF — starting mixer");
          } else {
            if (dosingOK) {
              digitalWrite(PIN_RELAY_DOSING, RELAY_OFF);
              stateDosingPump = false;
              remotePrint("[EC] Dosing pulse OFF — starting mixer");
            }
          }
          pulsing = false;
          digitalWrite(PIN_RELAY_MIXING_MOTOR, RELAY_ON);
          stateMixingMotor = true;
          motorRunning     = true;
          motorStartMs     = now;
        }
        break;
      }

      if (ecCorrectionIsHigh) {
        if (currentEC <= ecMax) {
          stopAllPumps();
          pulseCount  = 0;
          systemState = STATE_IDLE;
          remotePrint("[EC] EC no longer high — returning to IDLE");
          break;
        }
        if (diaphPerm == 0) {
          stopAllPumps();
          systemState = STATE_IDLE;
          remotePrint("[EC] Dilution blocked — reservoir critically low");
          break;
        }
        digitalWrite(PIN_RELAY_DIAPHRAGM, RELAY_ON);
        stateDiaphragm = true;
        pulsing        = true;
        pulseStartMs   = now;
        pulseCount++;
        Serial.print("[EC] Dilution pulse ");
        Serial.print(pulseCount);
        Serial.println("/10 — Diaphragm ON 5s");
      }
      else {
        if (currentEC >= ecMin) {
          stopAllPumps();
          pulseCount  = 0;
          systemState = STATE_IDLE;
          remotePrint("[EC] EC no longer low — returning to IDLE");
          break;
        }
        if (!dosingOK) {
          stopAllPumps();
          systemState = STATE_IDLE;
          remotePrint("[EC] Dosing aborted — nutrient tank too low");
          break;
        }
        digitalWrite(PIN_RELAY_DOSING, RELAY_ON);
        stateDosingPump = true;
        pulsing         = true;
        pulseStartMs    = now;
        pulseCount++;
        Serial.print("[EC] Dosing pulse ");
        Serial.print(pulseCount);
        Serial.println("/10 — Dosing pump ON 5s");
      }
      break;
    }

    case STATE_SUBMERSIBLE_CYCLE: {
      if (mixingPct < MIXING_LOW_PCT) {
        digitalWrite(PIN_RELAY_SUBMERSIBLE, RELAY_OFF);
        stateSubmersiblePump = false;
        remotePrint("[CYCLE] Emergency stop — mixing tank < 25% mid-cycle");
        systemState  = STATE_REFILLING_MIXING;
        refillActive = true;
        break;
      }
      if (now - submersibleStartMs >= SUBMERSIBLE_CYCLE_MS) {
        digitalWrite(PIN_RELAY_SUBMERSIBLE, RELAY_OFF);
        stateSubmersiblePump = false;
        remotePrint("[CYCLE] 15-minute cycle complete — returning to IDLE");
        systemState = STATE_IDLE;
      }
      break;
    }

    case STATE_FAULT: {
      remotePrint("[FAULT] System halted — waiting for manual reset");
      break;
    }
  }
}

// ================= HARVEST ALERT =================
// Fires at day 28 and repeats every 24 hours until reset is pressed.
// Applies to both seedling mode (day 0 start) and grown mode (day 12 start)
// since both share the same seedlingStartTime infrastructure.
void checkHarvestAlert() {
  if (!Blynk.connected()) return;
  if (!seedlingModeActive || !seedlingTimeSet || !rtcAvailable) return;

  DateTime nowDt   = rtc.now();
  long elapsedDays = (long)(nowDt.unixtime() - seedlingStartTime.unixtime()) / 86400L;

  // Only fire at day 28 or beyond
  if (elapsedDays < GROWN_MAX_DAYS) return;

  unsigned long now = millis();

  // Fire once at day 28, then repeat every 24 hours
  if (lastNotifyHarvest == 0 || now - lastNotifyHarvest >= 86400000UL) {
    Blynk.logEvent("harvest_ready",
                   "Harvest time! Plants have reached day "
                   + String(elapsedDays)
                   + ". Please harvest and press RESET to start a new cycle.");
    lastNotifyHarvest = now;
    remotePrint("[HARVEST] Day " + String(elapsedDays) + " — Harvest alert sent");
  }
}

// ================= WATER LEVEL NOTIFICATIONS =================
// All three tank low conditions share event code "water_level_alert"
// Each condition has its own independent 5-minute cooldown timer
void checkWaterLevelAlerts() {
  if (!Blynk.connected()) return;

  unsigned long now = millis();

  if (tank1Low) {
    if (now - lastNotifyTank1 >= NOTIFY_COOLDOWN_MS) {
      Blynk.logEvent("water_level_alert", "Main Reservoir is low! Level: "
                     + String(currentWaterLevel1, 1) + " cm");
      lastNotifyTank1 = now;
    }
  } else { lastNotifyTank1 = 0; }

  if (tank2Low) {
    if (now - lastNotifyTank2 >= NOTIFY_COOLDOWN_MS) {
      Blynk.logEvent("water_level_alert", "Nutrient Tank is low! Level: "
                     + String(currentWaterLevel2, 1) + " cm");
      lastNotifyTank2 = now;
    }
  } else { lastNotifyTank2 = 0; }

  if (tank3Low) {
    if (now - lastNotifyTank3 >= NOTIFY_COOLDOWN_MS) {
      Blynk.logEvent("water_level_alert", "Mixing Tank is low! Level: "
                     + String(currentWaterLevel3, 1) + " cm");
      lastNotifyTank3 = now;
    }
  } else { lastNotifyTank3 = 0; }
}

// ================= RTC READ =================
void readRTC() {
  if (!rtcAvailable) return;

  DateTime now = rtc.now();

  snprintf(rtcTimeString, sizeof(rtcTimeString),
           "%02d:%02d:%02d %02d/%02d/%04d",
           now.hour(), now.minute(), now.second(),
           now.day(),  now.month(),  now.year());

  Serial.print("[RTC] ");
  Serial.println(rtcTimeString);
}

// ================= LED GROW LIGHT CONTROL =================
// Debounced to prevent flicker at schedule boundaries.
//
// ROOT CAUSE OF FLICKER:
//   controlGrowLights() is called every second alongside readRTC().
//   At the exact ON/OFF boundary minute, RTC I2C read latency and
//   millis() jitter can cause nowMin to straddle both sides of the
//   threshold across consecutive 1-second reads, toggling the relay
//   rapidly and producing visible flicker on the grow light.
//
// FIX — confirmation counter:
//   shouldBeOn must be stable and consistent for LED_CONFIRM_COUNT
//   consecutive reads (LED_CONFIRM_COUNT seconds) before the relay
//   is actually switched. A single transient reading cannot trigger
//   a state change, eliminating boundary flicker entirely.
//
#define LED_CONFIRM_COUNT  3   // must see same state 3 s in a row to switch

void controlGrowLights() {
  if (!rtcAvailable) return;

  // Static counters persist across calls — no extra global variables needed
  static int  confirmOnCount  = 0;
  static int  confirmOffCount = 0;

  DateTime now = rtc.now();

  int nowMin = now.hour() * 60 + now.minute();
  int onMin  = LIGHTS_ON_HOUR  * 60 + LIGHTS_ON_MIN;
  int offMin = LIGHTS_OFF_HOUR * 60 + LIGHTS_OFF_MIN;

  bool shouldBeOn = (onMin > offMin)
                    ? (nowMin >= onMin || nowMin < offMin)
                    : (nowMin >= onMin && nowMin < offMin);

  if (shouldBeOn) {
    confirmOffCount = 0;          // reset opposite counter immediately
    if (!stateLED) {
      confirmOnCount++;
      if (confirmOnCount >= LED_CONFIRM_COUNT) {
        confirmOnCount = 0;
        digitalWrite(PIN_RELAY_LIGHTS, RELAY_ON);
        stateLED = true;
        remotePrint("[LED] ON");
      }
    } else {
      confirmOnCount = 0;         // already ON — no action needed, reset counter
    }
  } else {
    confirmOnCount = 0;           // reset opposite counter immediately
    if (stateLED) {
      confirmOffCount++;
      if (confirmOffCount >= LED_CONFIRM_COUNT) {
        confirmOffCount = 0;
        digitalWrite(PIN_RELAY_LIGHTS, RELAY_OFF);
        stateLED = false;
        remotePrint("[LED] OFF");
      }
    } else {
      confirmOffCount = 0;        // already OFF — no action needed, reset counter
    }
  }
}


// ================= STATE NAME HELPER =================
String getStateName() {
  switch (systemState) {
    case STATE_IDLE:              return "IDLE";
    case STATE_REFILLING_MIXING:  return "REFILLING";
    case STATE_EC_CORRECTION:     return "EC CORRECTION";
    case STATE_SUBMERSIBLE_CYCLE: return "SUBM. CYCLE";
    case STATE_FAULT:             return "FAULT";
    default:                      return "UNKNOWN";
  }
}

// ================= SEND TO BLYNK =================
void sendToBlynk() {
  if (!Blynk.connected()) return;

  Blynk.virtualWrite(V0, currentTemperature);
  Blynk.virtualWrite(V1, currentHumidity);
  Blynk.virtualWrite(V2, currentEC);

  Blynk.virtualWrite(V3,  currentWaterLevel1);   // Reservoir cm
  Blynk.virtualWrite(V17, currentWaterLevel2);   // Nutrient tank cm
  Blynk.virtualWrite(V4,  currentWaterLevel3);   // Mixing tank cm

  Blynk.virtualWrite(V8,  stateLED ? 1 : 0);
  Blynk.virtualWrite(V9,  rtcTimeString);

  Blynk.virtualWrite(V6,  stateDosingPump      ? 1 : 0);
  Blynk.virtualWrite(V5,  stateSubmersiblePump ? 1 : 0);
  Blynk.virtualWrite(V15, stateMister          ? 1 : 0);
  Blynk.virtualWrite(V16, stateExhaust         ? 1 : 0);

  // Days elapsed + auto LED update for normal seedling timer
  if (seedlingModeActive && seedlingTimeSet && rtcAvailable) {
    DateTime nowDt   = rtc.now();
    long elapsedDays = (long)(nowDt.unixtime() - seedlingStartTime.unixtime()) / 86400L;
    Blynk.virtualWrite(V29, elapsedDays);

    // Auto-update LEDs only in normal seedling mode (not grown override)
    if (!grownModeActive) {
      if (elapsedDays < 12) {
        Blynk.virtualWrite(V33, 1);  // Seedling LED ON
        Blynk.virtualWrite(V34, 0);  // Grown LED OFF
      } else {
        Blynk.virtualWrite(V33, 0);  // Seedling LED OFF
        Blynk.virtualWrite(V34, 1);  // Grown LED ON (auto-switched at day 12)
      }
    }
  } else {
    // No stage active (after reset or on first boot) — clear all stage indicators
    Blynk.virtualWrite(V29, 0);   // Days counter OFF
    Blynk.virtualWrite(V33, 0);   // Seedling LED OFF
    Blynk.virtualWrite(V34, 0);   // Grown LED OFF
    Blynk.virtualWrite(V30, 0);   // Seedling button OFF
    Blynk.virtualWrite(V35, 0);   // Grown button OFF
  }

  Blynk.virtualWrite(V7,  stateDiaphragm    ? 1 : 0);
  Blynk.virtualWrite(V19, stateMixingMotor  ? 1 : 0);
  Blynk.virtualWrite(V20, getStateName());
  Blynk.virtualWrite(V21, pulseCount);
  Blynk.virtualWrite(V22, getReservoirPct());
  Blynk.virtualWrite(V23, getNutrientPct());
  Blynk.virtualWrite(V24, getMixingPct());
  Blynk.virtualWrite(V25, misterFault  ? 1 : 0);
  Blynk.virtualWrite(V26, exhaustFault ? 1 : 0);
  Blynk.virtualWrite(V27, misterPulseCount);
  Blynk.virtualWrite(V28, exhaustPulseCount);

  Serial.println("[BLYNK] Data sent");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  Serial2.setRxBufferSize(512);
  Serial2.begin(9600, SERIAL_8N1, PIN_RX_FROM_UNO, PIN_TX_TO_UNO);

  pinMode(PIN_RELAY_EXHAUST,      OUTPUT);
  pinMode(PIN_RELAY_MISTER,       OUTPUT);
  pinMode(PIN_RELAY_DOSING,       OUTPUT);
  pinMode(PIN_RELAY_SUBMERSIBLE,  OUTPUT);
  pinMode(PIN_RELAY_LIGHTS,       OUTPUT);
  pinMode(PIN_RELAY_DIAPHRAGM,    OUTPUT);
  pinMode(PIN_RELAY_MIXING_MOTOR, OUTPUT);

  digitalWrite(PIN_RELAY_EXHAUST,      RELAY_OFF);
  digitalWrite(PIN_RELAY_MISTER,       RELAY_OFF);
  digitalWrite(PIN_RELAY_DOSING,       RELAY_OFF);
  digitalWrite(PIN_RELAY_SUBMERSIBLE,  RELAY_OFF);
  digitalWrite(PIN_RELAY_LIGHTS,       RELAY_OFF);
  digitalWrite(PIN_RELAY_DIAPHRAGM,    RELAY_OFF);
  digitalWrite(PIN_RELAY_MIXING_MOTOR, RELAY_OFF);

  Wire.begin(PIN_RTC_SDA, PIN_RTC_SCL);

  if (!rtc.begin()) {
    Serial.println("[RTC] Not found - RTC features disabled, serial still works");
    rtcAvailable = false;
  } else {
    Serial.println("[RTC] Found successfully");
    rtcAvailable = true;
    // Uncomment ONCE to set time, then comment again:
    // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // ── Non-blocking WiFi + Blynk connect ──────────────────────────────────────
  // Uses Blynk.config() instead of Blynk.begin() so WiFi failure never hangs.
  Serial.print("[WIFI] Connecting to ");
  Serial.print(ssid);
  WiFi.begin(ssid, pass);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED &&
         millis() - wifiStart < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("[WIFI] Connected — IP: ");
    Serial.println(WiFi.localIP());

    // Configure Blynk with token only (WiFi already connected above)
    Blynk.config(BLYNK_AUTH_TOKEN);

    // Attempt one Blynk handshake (non-blocking, 3 s timeout)
    blynkOnline = Blynk.connect(3000);
    Serial.println(blynkOnline ? "[BLYNK] Connected" : "[BLYNK] Server unreachable — offline mode");
  } else {
    Serial.println();
    Serial.println("[WIFI] Not found — running offline. Blynk features disabled.");
    blynkOnline = false;
  }
  // ───────────────────────────────────────────────────────────────────────────

  timer.setInterval(2000L, sendToBlynk);

  tft_setup();   // ← TFT: initialise display and draw static meter faces

  Serial.println("[BOOT] ESP32 Hydroponic Controller v2.3 Started");
}

// ================= LOOP =================
void loop() {
  receiveSerialData();

  // Run Blynk and its timer only when connected.
  // Blynk.run() also handles automatic reconnection attempts in the background.
  if (Blynk.connected()) {
    blynkOnline = true;
    Blynk.run();
    timer.run();
  } else {
    blynkOnline = false;
  }

  controlHumidity();
  controlMixingTank();
  checkWaterLevelAlerts();
  checkHarvestAlert();  // harvest notification at day 28, repeats every 24h

  tft_update();   // ← TFT: refresh needles and pointers (non-blocking, 100 ms rate)

  unsigned long now = millis();

  if (now - lastRTCRead >= RTC_READ_INTERVAL_MS) {
    lastRTCRead = now;
    readRTC();
    controlGrowLights();
  }
}
