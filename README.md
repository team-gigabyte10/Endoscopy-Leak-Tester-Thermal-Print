# Endoscopy-Leak-Tester-Thermal-Print
Great. Since you are using:

ESP32 DevKit V1 (30-pin)

MPX5500DP

16×2 LCD (parallel, non-I2C)

BC547 + Relay for pump control

Start button on GPIO27

Deli ES421 Bluetooth Printer

Name: DL-ES421

MAC: 58:22:33:D3:0A:60

here is the complete ESP32 code.

Install Library First

In Arduino IDE:

BluetoothSerial (built into ESP32 core)

LiquidCrystal (standard Arduino library)

ESP32 Connections:

Device                             ESP32 Pin

Pressure Sensor Output             GPIO34

Relay (BC547 base driver)          GPIO26

Start Button                       GPIO27

LCD RS                             GPIO23

LCD EN                             GPIO22

LCD D4                             GPIO21

LCD D5                             GPIO19

LCD D6                             GPIO18

LCD D7                             GPIO5


Complete Code
Important Calibration Step

The code currently uses:

These are example values.

You must measure:

adcZero = ADC value at 0 mmHg

adcFull = ADC value at 300 mmHg

Then replace them in the code.

How It Works

Press START (GPIO27).

Pump turns ON through relay.

Pressure rises to 300 mmHg.

Pump OFF.

Hold for 30 seconds.

Measure pressure drop.

≤ 20 mmHg loss → PASS.

> 20 mmHg loss → FAIL.

Print report automatically on the Deli ES421 Bluetooth printer.
