#include "BluetoothSerial.h"
#include <LiquidCrystal.h>

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
LiquidCrystal lcd(23, 22, 21, 19, 18, 5);

// --- Calibration Values ---
int adcZero = 0;            // Will be auto-calibrated on startup
const int adcSpan300 = 282; // ADC span for 300 mmHg (Recalibrated for 160 mmHg)

// --- Test Parameters ---
const float targetPressure = 300.0; // mmHg
const int testDuration = 30000;     // 30 seconds in milliseconds
const float maxDrop = 50.0;         // Max allowed pressure drop for PASS (mmHg)

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
  
  // Linear interpolation to convert ADC value to mmHg
  float pressure = ((avgADC - adcZero) * 300.0) / (float)adcSpan300;
  
  // Clamp tiny negative values to 0 to prevent slightly negative noise displays
  if (pressure < 0 && pressure > -3.0) {
    pressure = 0;
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
  
  bool passed = (pressureDrop <= maxDrop);
  
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
    
    // Print Document format
    SerialBT.println("--------------------------------");
    SerialBT.println("      LEAK TEST REPORT          ");
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
    SerialBT.println("--------------------------------");
    SerialBT.println("\n\n\n"); // Feed paper out
    
    delay(3000); // Give time for printer to finish printing
    SerialBT.disconnect(); // Disconnect to save battery/state
  } else {
    Serial.println("Failed to connect to printer!");
    lcd.clear();
    lcd.print("Printer Error!");
    delay(2000);
  }
}
