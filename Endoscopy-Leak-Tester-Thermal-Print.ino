#include "BluetoothSerial.h"
#include <LiquidCrystal.h>
#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include "time.h"
#include "soc/soc.h"             // ESP32 brownout detector control
#include "soc/rtc_cntl_reg.h"    // ESP32 brownout detector control

// =================================================================
// PROJECT: Endoscopy Leak Tester with Thermal Print & L293D Driver
// =================================================================
// IMPORTANT ARDUINO IDE COMPILATION SETTING:
//  Because this sketch uses both BluetoothSerial and WiFi stacks,
//  go to: Tools > Partition Scheme > "Huge APP (3MB No OTA/1MB SPIFFS)"
//  to prevent the "text section exceeds available space" error.
// =================================================================
// Actuators:
//  - 12V Air Pump via L293D (Input 1A / GPIO 26)
//  - 3.7V STSY033 Air Release Valve via L293D (Input 3A / GPIO 25)
//  - Piezo Speaker for Pass/Fail audible alerts (GPIO 32)
// Feedback:
//  - 16x2 LCD Display
//  - Bluetooth Thermal Receipt Printer (Deli DL-ES421)
// Sensors & Clock:
//  - MPX5500DP Pressure Sensor (GPIO 34 / ADC)
//  - DS3231 RTC Module (I2C: SDA=21, SCL=22) + WiFi NTP Sync
// =================================================================

// --- Pin Definitions ---
const int sensorPin = 34; // MPX5500DP Pressure Sensor Output (ADC1)
const int pumpPin   = 26; // L293D Input 1A (Controls 12V Air Pump)
const int valvePin  = 25; // L293D Input 3A (Controls 3.7V STSY033 Release Valve)
const int buzzerPin = 32; // Piezo Speaker / Buzzer
const int buttonPin = 27; // Start Button (Active LOW with internal pull-up)

// LCD Pins: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(4, 15, 16, 17, 18, 19);

// --- RTC Configuration ---
RTC_DS3231 rtc;

// --- WiFi & NTP Configuration ---
const char* ssid       = "RFID";
const char* password   = "rfid2025";
const char* ntpServer  = "pool.ntp.org";
const long  gmtOffset_sec = 21600; // +06:00 timezone (Bangladesh)
const int   daylightOffset_sec = 0;

// --- Bluetooth Printer Configuration ---
// MAC address of the Deli ES421 Printer
uint8_t printerMAC[6]  = {0x58, 0x22, 0x33, 0xD3, 0x0A, 0x60};
String printerName = "DL-ES421";
BluetoothSerial SerialBT;

// --- Calibration Values ---
int adcZero = 0;            // Auto-calibrated on startup
const int adcSpan300 = 280; // ADC span for 300 mmHg reference

// --- Test Parameters ---
const float targetPressure = 280.0;        // Target pressure to reach (mmHg)
const int testDuration = 30000;            // Leak test duration: 30 seconds (ms)
const float passPressureThreshold = 270.0; // Minimum pressure at end for PASS (mmHg)
const unsigned long valveReleaseDuration = 10000; // Venting duration: 10 seconds

// --- Two-Stage Inflation & Settle Control ---
// Prevents dynamic airflow backpressure drop when pump stops, ensuring true static 280 mmHg
const float pumpApproachPressure = 260.0; // Transition to gentle speed (mmHg)
const int   PUMP_FAST_PWM        = 255;   // Rapid initial inflation
const int   PUMP_SLOW_PWM        = 135;   // Gentle approach speed (drastically cuts backpressure)
const int   SETTLE_DELAY_MS      = 600;   // Post-cutoff stabilization delay (ms)
const int   TOPUP_PULSE_MS       = 140;   // Micro-pulse duration for static pressure top-up (ms)
const int   TOPUP_PWM            = 150;   // Moderate speed for top-up pulses
const int   MAX_TOPUP_ATTEMPTS   = 15;    // Safety limit for top-up micro-pulses

// --- STSY033 Valve Control (Normally Closed - Energize to Vent) ---
// Solenoid is Normally Closed (NC):
//  - At Rest / Testing (OFF / 0V): Closed by internal spring, seals chamber and holds air (0 power, 0 heat)
//  - Venting (ON / Energized): Solenoid pulls open exhaust gate to release air to atmosphere
const int VALVE_PULLIN_PWM = 255; // Full power pulse (80ms) to firmly snap gate open against pressure
const int VALVE_HOLD_PWM   = 180; // Holding PWM during 5-second vent (keeps resistor cool)

// State Tracking
bool isTesting = false;
unsigned long readyStartTime = 0;
unsigned long lastDisplayUpdate = 0;

// Function Prototypes
float readPressure();
void turnPumpOn();
void turnPumpOff();
bool inflateToTargetPressure();
void closeValveGate();
void openValveGate();
void startTest();
void releaseAir(unsigned long durationMs);
void printReport(float startP, float endP, float dropP, bool passed);
void playStartChirp();
void playTargetBeep();
void playPassSound();
void playFailSound();

void setup() {
  // Disable ESP32 brownout detector to prevent reboot from brief motor inrush surge
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);

  // Initialize Actuator & Sensor Pins
  pinMode(pumpPin, OUTPUT);
  turnPumpOff();              // Ensure Pump is OFF initially

  pinMode(valvePin, OUTPUT);
  closeValveGate();           // Ensure Valve is OFF (NC: Gate is CLOSED at rest, 0V)

  pinMode(buzzerPin, OUTPUT);
  digitalWrite(buzzerPin, LOW);

  pinMode(buttonPin, INPUT_PULLUP);

  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Initializing...");

  // Auto-zero calibration at startup (ensure sensor reads 0 mmHg at rest)
  long sumZero = 0;
  for (int i = 0; i < 50; i++) {
    sumZero += analogRead(sensorPin);
    delay(2);
  }
  adcZero = sumZero / 50;

  // Initialize I2C and RTC (DS3231)
  Wire.begin(21, 22); // SDA = GPIO 21, SCL = GPIO 22
  bool rtcFound = rtc.begin();
  if (!rtcFound) {
    Serial.println("Couldn't find RTC module!");
  }

  // Brief NTP Time Sync over WiFi
  lcd.clear();
  lcd.print("Syncing Time...");
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 10) { // Wait up to 5 seconds
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10000)) {
      Serial.println("Time synced from NTP!");
      if (rtcFound) {
        rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                            timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
        Serial.println("RTC updated with precise NTP time.");
      }
    } else {
      Serial.println("Failed to obtain time from NTP.");
    }
    // Disconnect WiFi to conserve power and keep 2.4GHz radio clear for Bluetooth
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  } else {
    Serial.println("WiFi connection failed. Proceeding with existing RTC time...");
    WiFi.mode(WIFI_OFF);
  }

  // Start Bluetooth as Master for Deli thermal printer
  SerialBT.begin("ESP32_Master", true);
  Serial.println("Bluetooth master started.");

  // Startup ready sound
  playStartChirp();

  lcd.clear();
  lcd.print("Ready to Test");
  readyStartTime = millis();
}

void loop() {
  if (!isTesting) {
    // Show live ambient/idle pressure reading after 10 seconds of idle time
    if (millis() - readyStartTime >= 10000) {
      if (millis() - lastDisplayUpdate >= 250) {
        lcd.setCursor(0, 1);
        lcd.print("P: ");
        lcd.print(readPressure(), 1);
        lcd.print(" mmHg  ");
        lastDisplayUpdate = millis();
      }
    }

    // Wait for the START button to be pressed (Active LOW)
    if (digitalRead(buttonPin) == LOW) {
      delay(50); // Debounce
      if (digitalRead(buttonPin) == LOW) {
        startTest();
      }
    }
  }
}

// -----------------------------------------------------------------
// Read & Filter Pressure Sensor (MPX5500DP)
// -----------------------------------------------------------------
float readPressure() {
  long sumADC = 0;
  const int numSamples = 50;

  for (int i = 0; i < numSamples; i++) {
    sumADC += analogRead(sensorPin);
    delay(2);
  }

  float avgADC = (float)sumADC / numSamples;
  float deltaADC = avgADC - adcZero;

  if (deltaADC <= 0) {
    return 0.0;
  }

  // Raw pressure calculation
  float raw_p = deltaADC * (180.0 / 235.0);
  float pressure = 0.0;

  // Multi-point calibration curve
  if (raw_p <= 0.0) {
    pressure = 0.0;
  } else if (raw_p <= 15.0) {
    pressure = raw_p * (50.0 / 15.0);
  } else if (raw_p <= 60.0) {
    pressure = 50.0 + (raw_p - 15.0) * ((120.0 - 50.0) / (60.0 - 15.0));
  } else if (raw_p <= 80.0) {
    pressure = 120.0 + (raw_p - 60.0) * ((160.0 - 120.0) / (80.0 - 60.0));
  } else if (raw_p <= 110.0) {
    pressure = 160.0 + (raw_p - 80.0) * ((200.0 - 160.0) / (110.0 - 80.0));
  } else if (raw_p <= 120.0) {
    pressure = 200.0 + (raw_p - 110.0) * ((220.0 - 200.0) / (120.0 - 110.0));
  } else if (raw_p <= 170.0) {
    pressure = 220.0 + (raw_p - 120.0) * ((300.0 - 220.0) / (170.0 - 120.0));
  } else {
    pressure = 300.0 + (raw_p - 170.0) * ((300.0 - 220.0) / (170.0 - 120.0));
  }

  return pressure;
}

// -----------------------------------------------------------------
// Pump Motor Control with PWM Soft-Start
// Prevents high inrush current spikes that cause ESP32 brownout
// and LCD blackout resets.
// -----------------------------------------------------------------
void turnPumpOn() {
  Serial.println("Starting pump with soft-start ramp...");
  // Ramp PWM duty from ~25% to 100% over 250ms to tame current spike
  for (int duty = 65; duty <= 255; duty += 15) {
    analogWrite(pumpPin, duty);
    delay(20);
  }
  analogWrite(pumpPin, 255); // Full power
}

void turnPumpOff() {
  analogWrite(pumpPin, 0);
  digitalWrite(pumpPin, LOW);
}

// -----------------------------------------------------------------
// Valve Gate Control (Normally Closed Solenoid Valve - STSY033)
//  - De-energized (0V / LOW): Gate is CLOSED by spring, holds pressure
//  - Energized (PWM / ON)  : Solenoid pulls OPEN to vent air
// -----------------------------------------------------------------
void closeValveGate() {
  // De-energize coil (0V) -> internal spring seals gate shut to hold pressure
  analogWrite(valvePin, 0);
  digitalWrite(valvePin, LOW);
}

void openValveGate() {
  // Energize coil -> pulls solenoid plunger to OPEN exhaust gate and vent air
  // Step 1: High pull-in pulse (80ms) to snap open against 280 mmHg backpressure
  analogWrite(valvePin, VALVE_PULLIN_PWM);
  delay(80);
  // Step 2: Hold open with safe PWM for the venting duration (keeps resistor cool)
  analogWrite(valvePin, VALVE_HOLD_PWM);
}

// -----------------------------------------------------------------
// Release Air: Opens gate (Valve ON) for 5 seconds to vent all pressure
// -----------------------------------------------------------------
void releaseAir(unsigned long durationMs) {
  Serial.println("Release Valve ON (Gate OPEN) - Venting air...");
  openValveGate(); // Valve ON -> energizes solenoid to open gate and release air

  unsigned long startTime = millis();
  while (millis() - startTime < durationMs) {
    // Keep LCD updated with decaying pressure while venting
    float currentP = readPressure();
    lcd.setCursor(0, 1);
    lcd.print("Venting: ");
    lcd.print(currentP, 1);
    lcd.print(" mmHg ");
    delay(100);
  }

  // De-energize valve after venting is complete (returns to closed rest state)
  closeValveGate();
  Serial.println("Venting complete. Scope is at 0 mmHg. Valve OFF.");
}

// -----------------------------------------------------------------
// Two-Stage Pressurization with Settling & Closed-Loop Top-Up
// Solves pneumatic dynamic backpressure drop and ensures the 30s
// leak test starts at true static target pressure (>= 280 mmHg).
// -----------------------------------------------------------------
bool inflateToTargetPressure() {
  unsigned long pumpStartTime = millis();
  const unsigned long maxPumpTimeout = 40000; // 40 second safety cutoff

  lcd.clear();
  lcd.print("Pumping (Fast)...");

  // Stage 1: Fast Inflation with soft-start ramp to 240 mmHg
  turnPumpOn(); // Ramps duty to PUMP_FAST_PWM (255)

  float currentPressure = 0.0;
  while (true) {
    currentPressure = readPressure();
    lcd.setCursor(0, 1);
    lcd.print("P: ");
    lcd.print(currentPressure, 1);
    lcd.print(" mmHg  ");

    if (millis() - pumpStartTime > maxPumpTimeout) {
      turnPumpOff();
      Serial.println("Error: Pump timeout reached during fast inflation!");
      return false;
    }

    if (currentPressure >= pumpApproachPressure) {
      break;
    }
  }

  // Stage 2: Gentle approach to target (240 -> 280 mmHg)
  // Low air velocity cuts dynamic flow resistance & backpressure
  lcd.clear();
  lcd.print("Fine Tuning...");
  analogWrite(pumpPin, PUMP_SLOW_PWM);

  while (true) {
    currentPressure = readPressure();
    lcd.setCursor(0, 1);
    lcd.print("P: ");
    lcd.print(currentPressure, 1);
    lcd.print(" mmHg  ");

    if (millis() - pumpStartTime > maxPumpTimeout) {
      turnPumpOff();
      Serial.println("Error: Pump timeout reached during fine tuning!");
      return false;
    }

    if (currentPressure >= targetPressure) {
      break;
    }
  }

  // Turn pump off immediately
  turnPumpOff();

  // Stage 3: Pneumatic & Electrical Settling Window
  // Allows airflow kinetic energy to dissipate and electrical rail/ground to stabilize
  lcd.clear();
  lcd.print("Stabilizing...");
  delay(SETTLE_DELAY_MS);

  // Stage 4: Closed-Loop Micro-Pulse Top-Up
  // Verify true static settled pressure with pump completely OFF
  float settledP = readPressure();
  int topUpAttempts = 0;

  while (settledP < targetPressure && topUpAttempts < MAX_TOPUP_ATTEMPTS) {
    lcd.setCursor(0, 1);
    lcd.print("Adj: ");
    lcd.print(settledP, 1);
    lcd.print(" mmHg ");

    // Deliver calibrated micro-pulse
    analogWrite(pumpPin, TOPUP_PWM);
    delay(TOPUP_PULSE_MS);
    turnPumpOff();

    // Allow static pressure to equilibrate
    delay(400);
    settledP = readPressure();
    topUpAttempts++;

    if (millis() - pumpStartTime > maxPumpTimeout) {
      turnPumpOff();
      Serial.println("Error: Pump timeout reached during top-up stabilization!");
      return false;
    }
  }

  Serial.print("Target settled pressure achieved: ");
  Serial.print(settledP);
  Serial.print(" mmHg (Top-ups: ");
  Serial.print(topUpAttempts);
  Serial.println(")");

  return true;
}

// -----------------------------------------------------------------
// Main Leak Test Workflow
// -----------------------------------------------------------------
void startTest() {
  isTesting = true;
  playStartChirp();

  // Step 1: Ensure Valve is closed (0V / de-energized holds pressure naturally)
  closeValveGate();
  delay(100); // 100ms settling time for solenoid seal

  // Step 2: Intelligent Two-Stage Inflation with Settling & Top-Up
  bool reachedTarget = inflateToTargetPressure();
  if (!reachedTarget) {
    lcd.clear();
    lcd.print("Inflate Failed!");
    playFailSound();
    delay(2000);
    releaseAir(valveReleaseDuration);
    isTesting = false;
    readyStartTime = millis();
    return;
  }

  playTargetBeep();

  // Baseline static start pressure after full stabilization (guaranteed >= 280 mmHg)
  float startPressure = readPressure();
  lcd.clear();
  lcd.print("Holding 30s...");

  // Step 3: Hold for testDuration (30 seconds) while Valve stays OFF (gate closed, holding pressure)
  unsigned long holdStartTime = millis();
  while (millis() - holdStartTime < testDuration) {
    float currentPressure = readPressure();
    lcd.setCursor(0, 1);
    lcd.print("P: ");
    lcd.print(currentPressure, 1);
    lcd.print(" mmHg  ");
    delay(100);
  }

  // Step 4: Measure drop and evaluate result
  float endPressure = readPressure();
  float pressureDrop = startPressure - endPressure;
  bool passed = (endPressure >= passPressureThreshold);

  // Step 5: Display Result & Sound Alert
  lcd.clear();
  if (passed) {
    lcd.print("Result: PASS");
    playPassSound(); // Pleasant success chime
  } else {
    lcd.print("Result: FAIL");
    playFailSound(); // Warning alert pulses
  }

  lcd.setCursor(0, 1);
  lcd.print("Drop: ");
  lcd.print(pressureDrop, 1);
  lcd.print(" mmHg");
  delay(2000);

  // Step 6: Release Air (Turn Valve ON -> Gate opens for 5 seconds to vent all air)
  lcd.clear();
  lcd.print(passed ? "PASS - Venting" : "FAIL - Venting");
  releaseAir(valveReleaseDuration); // 10000 ms venting

  // Step 7: Print Thermal Receipt via Bluetooth
  printReport(startPressure, endPressure, pressureDrop, passed);

  // Step 8: Ready for next test (Valve remains OFF / closed at idle)
  delay(3000);
  lcd.clear();
  lcd.print("Ready to Test");
  isTesting = false;
  readyStartTime = millis();
}

// -----------------------------------------------------------------
// Print Leak Test Report via Bluetooth Thermal Printer
// -----------------------------------------------------------------
void printReport(float startP, float endP, float dropP, bool passed) {
  lcd.clear();
  lcd.print("Connecting BT...");
  Serial.println("Connecting to printer...");

  if (SerialBT.connect(printerMAC)) {
    Serial.println("Connected to printer successfully!");
    lcd.clear();
    lcd.print("Printing...");

    DateTime now = rtc.now();

    // Print Receipt Format
    SerialBT.println("--------------------------------");
    SerialBT.println("        Endocare BD               ");
    SerialBT.println("      LEAK TEST REPORT          ");
    SerialBT.println("--------------------------------");

    SerialBT.print("Date: ");
    SerialBT.print(now.year(), DEC);
    SerialBT.print('/');
    if (now.month() < 10) SerialBT.print('0');
    SerialBT.print(now.month(), DEC);
    SerialBT.print('/');
    if (now.day() < 10) SerialBT.print('0');
    SerialBT.print(now.day(), DEC);

    SerialBT.print("  Time: ");
    if (now.hour() < 10) SerialBT.print('0');
    SerialBT.print(now.hour(), DEC);
    SerialBT.print(':');
    if (now.minute() < 10) SerialBT.print('0');
    SerialBT.print(now.minute(), DEC);
    SerialBT.println();

    SerialBT.println("--------------------------------");
    SerialBT.print("Target Pressure: ");
    SerialBT.print(targetPressure, 1);
    SerialBT.println(" mmHg");

    SerialBT.print("Start Pressure : ");
    SerialBT.print(startP, 1);
    SerialBT.println(" mmHg");

    SerialBT.print("End Pressure   : ");
    SerialBT.print(endP, 1);
    SerialBT.println(" mmHg");

    SerialBT.print("Pressure Drop  : ");
    SerialBT.print(dropP, 1);
    SerialBT.println(" mmHg");

    SerialBT.println("--------------------------------");
    SerialBT.print("RESULT: ");
    if (passed) {
      SerialBT.println("PASS");
    } else {
      SerialBT.println("FAIL");
    }

    SerialBT.println("- - - - - - - - - - - - - - - - "); // Tear line
    SerialBT.println("\n"); // Feed paper

    delay(3000);           // Allow printer buffer to complete
    SerialBT.disconnect(); // Disconnect to conserve battery
  } else {
    Serial.println("Failed to connect to printer!");
    lcd.clear();
    lcd.print("Printer Error!");
    delay(2000);
  }
}

// -----------------------------------------------------------------
// Audio Feedback Signatures (Piezo Speaker / Buzzer on GPIO 32)
// -----------------------------------------------------------------
void playTone(int frequency, int durationMs) {
  tone(buzzerPin, frequency, durationMs);
  delay(durationMs);
  noTone(buzzerPin);
}

void playStartChirp() {
  playTone(1500, 80);
}

void playTargetBeep() {
  playTone(2000, 80);
  delay(60);
  playTone(2000, 80);
}

void playPassSound() {
  // Pleasant ascending chime: C5 -> E5 -> G5 -> C6
  playTone(523, 100);
  delay(20);
  playTone(659, 100);
  delay(20);
  playTone(784, 100);
  delay(20);
  playTone(1046, 250);
}

void playFailSound() {
  // Low-frequency warning alarm: 3 repeating pulses
  for (int i = 0; i < 3; i++) {
    playTone(370, 220);
    delay(100);
  }
}
