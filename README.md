#include "BluetoothSerial.h"
#include <LiquidCrystal.h>
#include <Wire.h>
#include "RTClib.h"
#include <WiFi.h>
#include "time.h"

// --- RTC Configuration ---
RTC_DS3231 rtc;

// --- WiFi & NTP Configuration ---
const char* ssid       = "Nayem CSE";
const char* password   = "Nayem2026";
const char* ntpServer  = "pool.ntp.org";
const long  gmtOffset_sec = 21600; // +06:00 timezone (Bangladesh)
const int   daylightOffset_sec = 0;

// --- Bluetooth Printer Configuration ---
// MAC address of the Deli ES421 Printer
uint8_t printerMAC[6]  = {0x58, 0x22, 0x33, 0xD3, 0x0A, 0x60};
String printerName = "DL-ES421";
BluetoothSerial SerialBT;

// --- Pin Definitions ---
const int sensorPin = 34; // MPX5500DP Pressure sensor Output
const int relayPin = 26;  // Pump relay (BC547 base driver)
const int buttonPin = 27; // Start button

// LCD Pins: RS, EN, D4, D5, D6, D7
LiquidCrystal lcd(4, 15, 16, 17, 18, 19);

// --- Calibration Values ---
int adcZero = 0;            // Will be auto-calibrated on startup
const int adcSpan300 = 280; // ADC span for 300 mmHg (Calculated: 180 actual = 250 old display)

// --- Test Parameters ---
const float targetPressure = 280.0; // mmHg
const int testDuration = 30000;     // 30 seconds in milliseconds
const float passPressureThreshold = 260.0; // min pressure for PASS (mmHg)

bool isTesting = false;
unsigned long readyStartTime = 0;
unsigned long lastDisplayUpdate = 0;

void setup() {
  Serial.begin(115200);
  
  // Initialize Pins
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW); // Pump OFF initially
  pinMode(buttonPin, INPUT_PULLUP);
  
  // Auto-zero calibration at startup
  long sumZero = 0;
  for (int i = 0; i < 50; i++) {
    sumZero += analogRead(sensorPin);
    delay(2);
  }
  adcZero = sumZero / 50;
  
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  
  // Start RTC (I2C)
  Wire.begin(21, 22); // SDA = D21, SCL = D22
  bool rtcFound = rtc.begin();
  if (!rtcFound) {
    Serial.println("Couldn't find RTC");
  }

  // --- NTP Time Sync over WiFi ---
  // Always attempt a brief WiFi connection to ensure the time is perfectly synced
  lcd.print("Syncing Time...");
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 10) { // Wait 5 seconds max
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    
    // Wait up to 10 seconds for the NTP server to respond
    if (getLocalTime(&timeinfo, 10000)) { 
      Serial.println("Time synced from Internet!");
      if (rtcFound) {
        rtc.adjust(DateTime(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec));
        Serial.println("RTC updated with precise NTP time.");
      }
    } else {
      Serial.println("Failed to obtain time from NTP.");
    }
    
    // Disconnect WiFi to save resources for Bluetooth
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
  } else {
    Serial.println("WiFi connection failed. Proceeding with existing RTC time...");
    WiFi.mode(WIFI_OFF);
  }
  
  if (rtcFound) {
    DateTime now = rtc.now();
    Serial.print("Current RTC Time: ");
    Serial.print(now.year(), DEC); Serial.print('/');
    Serial.print(now.month(), DEC); Serial.print('/');
    Serial.print(now.day(), DEC); Serial.print(" ");
    Serial.print(now.hour(), DEC); Serial.print(':');
    Serial.print(now.minute(), DEC); Serial.print(':');
    Serial.println(now.second(), DEC);
  }
  
  lcd.clear();
  lcd.print("Ready to Test");
  readyStartTime = millis();

  // Start Bluetooth as Master
  SerialBT.begin("ESP32_Master", true); 
  Serial.println("Starting Bluetooth...");
}

void loop() {
  if (!isTesting) {
    // Show readings after 10 seconds of idle time
    if (millis() - readyStartTime >= 10000) {
      if (millis() - lastDisplayUpdate >= 250) {
        lcd.setCursor(0, 1);
        lcd.print("P: ");
        lcd.print(readPressure(), 1);
        lcd.print(" mmHg  ");
        lastDisplayUpdate = millis();
      }
    }

    // Wait for the START button to be pressed
    if (digitalRead(buttonPin) == LOW) {
      delay(50); // Simple debounce
      if (digitalRead(buttonPin) == LOW) {
        startTest();
      }
    }
  }
}

float readPressure() {
  long sumADC = 0;
  int numSamples = 50;
  
  for (int i = 0; i < numSamples; i++) {
    sumADC += analogRead(sensorPin);
    delay(2); // Short delay to let ADC settle
  }
  
  float avgADC = (float)sumADC / numSamples;
  float deltaADC = avgADC - adcZero;
  
  if (deltaADC <= 0) {
    return 0.0;
  }

  // Calculate raw pressure (this matches the old LCD readings)
  float raw_p = deltaADC * (180.0 / 235.0);
  float pressure = 0.0;
  
  // Multi-point calibration based on provided data mapping (LCD -> Analog)
  if (raw_p <= 0.0) {
    pressure = 0.0;
  } else if (raw_p <= 15.0) {
    // Map 0->0 and 15->50. Analog 20 at LCD 0 is ignored to ensure display returns to 0 when idle.
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
    // Extrapolate beyond 170 LCD (300 mmHg Analog)
    pressure = 300.0 + (raw_p - 170.0) * ((300.0 - 220.0) / (170.0 - 120.0));
  }
  
  return pressure;
}


void startTest() {
  isTesting = true;
  lcd.clear();
  lcd.print("Pumping...");
  
  digitalWrite(relayPin, HIGH); // Turn pump ON
  
  float currentPressure = 0;
  
  // Step 1: Pump until target pressure (300 mmHg) is reached
  while (currentPressure < targetPressure) {
    currentPressure = readPressure();
    lcd.setCursor(0, 1);
    lcd.print("P: ");
    lcd.print(currentPressure, 1);
    lcd.print(" mmHg  ");
    delay(100);
  }
  
  digitalWrite(relayPin, LOW); // Turn pump OFF
  
  float startPressure = readPressure();
  lcd.clear();
  lcd.print("Holding 30s...");
  
  // Step 2: Hold for 30 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < testDuration) {
    currentPressure = readPressure();
    lcd.setCursor(0, 1);
    lcd.print("P: ");
    lcd.print(currentPressure, 1);
    lcd.print(" mmHg  ");
    delay(100);
  }
  
  // Step 3: Measure drop and evaluate
  float endPressure = readPressure();
  float pressureDrop = startPressure - endPressure;
  
  bool passed = (endPressure >= passPressureThreshold);
  
  // Step 4: Display Result
  lcd.clear();
  if (passed) {
    lcd.print("Result: PASS");
  } else {
    lcd.print("Result: FAIL");
  }
  
  lcd.setCursor(0, 1);
  lcd.print("Drop: ");
  lcd.print(pressureDrop, 1);
  lcd.print(" mmHg");
  
  // Step 5: Print Report
  printReport(startPressure, endPressure, pressureDrop, passed);
  
  // Wait before allowing the next test
  delay(5000); 
  lcd.clear();
  lcd.print("Ready to Test");
  isTesting = false;
  readyStartTime = millis();
}

void printReport(float startP, float endP, float dropP, bool passed) {
  lcd.clear();
  lcd.print("Connecting BT...");
  Serial.println("Connecting to printer...");
  
  // Attempt to connect to the printer via MAC address
  if (SerialBT.connect(printerMAC)) {
    Serial.println("Connected to printer successfully!");
    lcd.clear();
    lcd.print("Printing...");
    
    DateTime now = rtc.now();
    
    // Print Document format
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
  
    SerialBT.println("- - - - - - - - - - - - - - - - "); // Cutting line
    
    // Feed paper out so the cutting line clears the print head
    SerialBT.println();
    
    delay(3000); // Give time for printer to finish printing
    SerialBT.disconnect(); // Disconnect to save battery/state
  } else {
    Serial.println("Failed to connect to printer!");
    lcd.clear();
    lcd.print("Printer Error!");
    delay(2000);
  }
}
