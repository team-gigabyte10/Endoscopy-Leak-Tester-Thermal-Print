# Endoscopy Leak Tester with Thermal Print, L293D Driver & STSY033 Release Valve

Automated Endoscopy Leak Tester powered by an **ESP32**, featuring precision pressure measurement with the **MPX5500DP**, solid-state actuator control via the **L293D dual H-bridge motor driver**, automatic pressure venting using an **STSY033 3.7V micro release valve**, auditory feedback with a **Piezo speaker**, real-time date/time stamps from a **DS3231 RTC (synced via WiFi NTP)**, and Bluetooth thermal receipt printing (**Deli DL-ES421**).

---

## 🚀 Key Features

- **L293D Solid-State Actuator Driver**: Replaces mechanical relays, driving both the **12V Air Pump** and **3.7V STSY033 Air Release Valve** with built-in inductive flyback protection.
- **Normally Closed (NC) STSY033 Valve Control**:
  - **At Rest / Testing**: Valve is **OFF** (de-energized, 0V) $\rightarrow$ exhaust gate is **CLOSED** by internal spring, holding air pressure securely without consuming power or generating resistor heat.
  - **Air Release / Venting**: Valve turns **ON** (energized via peak-and-hold PWM) $\rightarrow$ solenoid pulls **OPEN** the exhaust gate, releasing pressurized air for 10 seconds down to 0 mmHg, then shuts back OFF.
- **Pass / Fail Auditory Feedback**: Distinct melodic chimes via a Piezo speaker (C-major ascending chime for **PASS**, repeating low-frequency warning pulses for **FAIL**).
- **High-Precision Pressure Monitoring**: Multi-point mapped and auto-zero calibrated readings from the MPX5500DP differential pressure sensor.
- **NTP-Synchronized RTC**: Syncs time via WiFi on boot, keeping the battery-backed DS3231 RTC accurate down to the second.
- **Bluetooth Thermal Printing**: Generates clean, formatted leak test receipts for clinical records.

---

## 📌 Pin Mapping Table

### ESP32 System Pin Connections
| Component | Pin / Signal | ESP32 GPIO | Notes |
| :--- | :--- | :--- | :--- |
| **MPX5500DP Sensor** | VOUT (Analog) | **GPIO 34** | ADC1_CH6 (Auto-zero calibrated at startup) |
| **Start Button** | Normally Open | **GPIO 27** | Connected to GND, uses internal `INPUT_PULLUP` |
| **Piezo Speaker** | (+) Signal | **GPIO 32** | Tone / alert sound generator |
| **L293D Input 1A** | Pin 2 | **GPIO 26** | Controls 12V Air Pump (`HIGH` = Run, `LOW` = Stop) |
| **L293D Input 3A** | Pin 10 | **GPIO 25** | Controls STSY033 Valve (`HIGH`/PWM = Open & Vent, `LOW` = Close & Hold) |
| **DS3231 RTC** | SDA | **GPIO 21** | I2C Data line |
| **DS3231 RTC** | SCL | **GPIO 22** | I2C Clock line |
| **LCD 16x2 RS** | RS (Pin 4) | **GPIO 4** | Register Select |
| **LCD 16x2 EN** | Enable (Pin 6)| **GPIO 15** | Enable strobe |
| **LCD 16x2 D4–D7**| Pins 11, 12, 13, 14 | **GPIO 16, 17, 18, 19** | 4-bit parallel data lines |

---

## ⚡ L293D Motor Driver & Actuator Pinout Guide

### L293D 16-Pin DIP IC Pinout
```text
                  ┌─────────∪─────────┐
       (5V) 1,2EN ┤ 1              16 ├ VCC1 (Logic +5V)
 (ESP32 GPIO 26) 1A ┤ 2              15 ├ 4A (GND)
   (Pump +) 1Y ┤ 3              14 ├ 4Y (NC)
        (GND) GND ┤ 4              13 ├ GND (Common GND)
        (GND) GND ┤ 5              12 ├ GND (Common GND)
             2Y ┤ 6              11 ├ 3Y (Valve + via Resistor)
        (GND) 2A ┤ 7              10 ├ 3A (ESP32 GPIO 25)
  (+12V DC) VCC2 ┤ 8               9 ├ 3,4EN (5V)
                  └───────────────────┘
```

### Complete L293D Connection Table
| L293D Pin | Pin Name | Connected To | Description / Signal |
| :--- | :--- | :--- | :--- |
| **Pin 1** | `1,2EN` | **+5V Rail** (or ESP32 5V/VIN) | Enables Channel 1 & 2 (Always active) |
| **Pin 2** | `1A` (Input 1) | **ESP32 GPIO 26** | Control signal for Air Pump (`HIGH` = ON, `LOW` = OFF) |
| **Pin 3** | `1Y` (Output 1) | **Air Pump (+)** | Switched +12V DC power to Air Pump |
| **Pin 4 & 5** | `GND` | **Common GND** | Ground and heatsink planes |
| **Pin 6** | `2Y` (Output 2) | *Not connected* | Unused |
| **Pin 7** | `2A` (Input 2) | **Common GND** | Pulled LOW to disable unused half-bridge |
| **Pin 8** | `VCC2` | **+12V Power Supply (+)** | High-voltage supply for motor/pump |
| **Pin 9** | `3,4EN` | **+5V Rail** (or ESP32 5V/VIN) | Enables Channel 3 & 4 (Always active) |
| **Pin 10** | `3A` (Input 3) | **ESP32 GPIO 25** | Control signal for Valve (`HIGH` = Seal/Hold, `LOW` = Vent) |
| **Pin 11** | `3Y` (Output 3) | **Valve (+) via 27Ω–33Ω 2W resistor** | Switched supply to 3.7V Valve coil (see safety note below) |
| **Pin 12 & 13**| `GND` | **Common GND** | Ground and heatsink planes |
| **Pin 14** | `4Y` (Output 4) | *Not connected* | Unused |
| **Pin 15** | `4A` (Input 4) | **Common GND** | Pulled LOW to disable unused half-bridge |
| **Pin 16** | `VCC1` | **+5V Rail** (or ESP32 5V/VIN) | Logic power supply for internal L293D circuitry |

---

### 🔌 Actuator Wiring & Voltage Safety

> [!IMPORTANT]
> - The **Air Pump** operates on **12V DC**, while the **STSY033 Solenoid Valve** is rated for **3.7V DC**.
> - Because L293D Pin 8 ($V_{CC2}$) is supplied with **12V**, Output `3Y` outputs ~10.5V–11V when `HIGH`.
> - Do **NOT** connect the 3.7V valve directly across Output 3Y and Ground without voltage dropping.

#### 1. 12V Air Pump Wiring:
- **Pump (+) [Red Wire]**: Connect to L293D **Pin 3 (Output 1Y)**.
- **Pump (-) [Black Wire]**: Connect to **Common GND**.

#### 2. STSY033 3.7V Valve Wiring:
- Place a **$27\Omega - 33\Omega$ (2W rating)** power resistor in series:
  ```text
  L293D Pin 11 (3Y) ────[ 27Ω - 33Ω, 2W Resistor ]──── Valve (+) [Red Wire]
  Common GND ────────────────────────────────────────── Valve (-) [Black Wire]
  ```
  *(The resistor safely drops ~7.5V, delivering the rated ~3.5V–3.8V across the coil without overheating).*

#### 3. Common Ground Rule (Star Topology):
All grounds must be tied together directly at the power supply terminal (Star Ground):
- **ESP32 GND**
- **12V DC Power Supply (-) / GND**
- **5V Logic Supply (-) / GND**
- **L293D Pins 4, 5, 12, 13**
- **Air Pump (-)** and **Valve (-)**
*Never daisy-chain the motor/pump ground through the ESP32 board or thin breadboard rails, which creates ground bounce.*

#### 4. Anti-Brownout & Noise Suppression (Critical):
When the 12V air pump motor starts up, it draws a massive inrush current spike and produces brush noise that can drop the voltage and reset the ESP32 (Brownout) or scramble the LCD. Add the following components:
1. **Bulk Electrolytic Capacitor ($470\mu\text{F} - 1000\mu\text{F}$, 25V or 35V)**: Connect directly across the 12V supply rails (`+12V` and `GND`) near L293D Pin 8 to buffer the inrush current surge.
2. **Noise Suppression Ceramic Capacitor ($0.1\mu\text{F} / 100\text{nF}$)**: Solder directly across the (+) and (-) terminals of the 12V Air Pump motor to absorb high-frequency commutator brush sparking.
3. **Logic Rail Decoupling ($100\mu\text{F} - 470\mu\text{F}$, 16V)**: Connect across the ESP32 `VIN/5V` and `GND` pin.
4. **Flyback Diode (1N4007 or 1N5819)**: Place in reverse-bias directly across the pump terminals (Cathode/band $\rightarrow$ `Pump +`, Anode $\rightarrow$ `Pump -`) to absorb back-EMF discharge.

---

### 🚦 Actuator Control Logic (Normally Closed Valve)

| Phase | ESP32 GPIO 26 (`pumpPin`) | ESP32 GPIO 25 (`valvePin`) | System State |
| :--- | :--- | :--- | :--- |
| **Idle / Rest** | `LOW` (Pump OFF) | `LOW` (Valve OFF / 0V) | Gate CLOSED by internal spring at rest (0 power) |
| **Pumping** | `HIGH`/PWM (Pump ON) | `LOW` (Valve OFF / 0V) | Gate CLOSED by spring, chamber seals & inflates |
| **Holding Test** | `LOW` (Pump OFF) | `LOW` (Valve OFF / 0V) | Gate CLOSED by spring, chamber sealed for 2 min (120s) test |
| **Exhaust / Vent** | `LOW` (Pump OFF) | `HIGH`/PWM (Valve ON) | Solenoid pulls gate OPEN to dump air (10s) |

---

## 🛡️ Multi-Layer Endoscope Tube & Sensor Protection System

Delicate endoscopes (Olympus, Pentax, Karl Storz, Fujifilm) can suffer catastrophic channel and bending rubber rupture if air is pumped while the pressure sensor is inactive or malfunctioning. The firmware enforces **5 independent hardware & software protection layers**:

1. **Boot-Time Sensor Health Check**:
   - At power-up, samples the MPX5500DP analog channel on GPIO 34.
   - Verifies raw ADC is within physical limits ($\le 4050$). Ambient resting pressure at 0 mmHg naturally yields low or zero ADC ($0 \le \text{ADC} \le 4050$). If saturated or shorted to VCC rail ($> 4050$), the system enters lockout mode:
     - LCD displays: `ERR: SENSOR FAIL` with actual ADC reading.
     - Audible alarm pulses.
     - **Pump is 100% blocked from activating**.
     - Auto-recovers once normal electrical connection is restored.

2. **Pre-Flight Sensor Readiness Validation**:
   - When the operator presses the Start button, a pre-flight sensor check verifies electrical sanity and confirms tube status before energizing the pump.
   - If residual trapped pressure is $> 60\text{ mmHg}$, the system automatically pre-vents for 3 seconds before pumping.

3. **Dynamic "Rate-of-Rise" Watchdog (Crucial Tube Protector)**:
   - If the pneumatic hose to the sensor pops off or the sensor ADC hangs, the sensor might stay at $0\text{ mmHg}$ while the pump forces air into the scope.
   - Within **$3.5\text{ seconds}$** of pump startup, pressure MUST rise by at least **$+8\text{ mmHg}$**.
   - If this threshold is not met within 3.5 seconds:
     - **Pump is killed instantly (`turnPumpOff()`)**.
     - **Release valve snaps open immediately (`openValveGate()`)** to dump trapped air.
     - System alarms with `ERR: NO AIR RISE / SENSOR CHECK`.

4. **Hard Overpressure Cutoff ($370\text{ mmHg}$ Ceiling with Noise Rejection)**:
   - Target pressure is $320\text{ mmHg}$. If verified pressure reaches $\ge 370\text{ mmHg}$ during pumping, settling, or holding, the pump is instantly disabled and the exhaust valve opens.

5. **Maximum Inflation Timeout (15 Seconds)**:
   - Total inflation timeout is set to **15 seconds**. If the system cannot reach target within 15 seconds, it immediately aborts, shuts off the pump, and vents to atmosphere.

---

## 🔄 Testing Workflow Sequence

1. **Boot & Auto-Zero**:
   - The STSY033 valve is **OFF** (0V / gate closed by spring at ambient atmospheric 0 mmHg baseline).
   - Sensor health check verifies valid raw ADC reading ($\text{ADC} \le 4050$).
   - The system samples ambient atmospheric pressure with 50 readings to calibrate auto-zero baseline.
   - Brief WiFi check fetches accurate NTP time and updates DS3231 RTC.
   - Ready chime plays on Piezo speaker.
2. **Start Button Pressed**:
   - Click chirp sounds.
   - Pre-flight sensor validation runs.
   - **Two-Stage Inflation with Watchdogs & Top-Up**:
     - **Fast Stage**: Air pump starts with soft-start PWM ramp and runs at 100% duty (`255`) up to **312 mmHg**.
     - **Rate-of-Rise Check**: Enforces $\ge 8\text{ mmHg}$ pressure rise within 3.5 seconds.
     - **Fine Tuning Stage**: Pump throttles to PWM (`195`) from 312 to **320 mmHg**, maintaining sufficient torque against backpressure while eliminating dynamic overshoot.
     - **Stabilization Settling**: Pump shuts off and system waits **600ms** for airflow kinetic energy to dissipate and electrical lines to stabilize.
     - **Closed-Loop Top-Up**: The true static settled pressure is sampled with the pump OFF. If it settles slightly below 320 mmHg, calibrated micro-pulses gently nudge static pressure to $\ge$ **320 mmHg**.
   - Safety cutoff automatically stops the pump and vents if pressure is not reached within 15 seconds.
3. **Holding Phase (2 minutes / 120 seconds)**:
   - Double-beep signals true static target pressure achieved ($\ge 320$ mmHg).
   - **STSY033 valve remains OFF (gate closed)**, sealing the air chamber with zero power and zero heat.
   - System monitors hold for 2 minutes with live countdown and continuous sensor integrity & overpressure monitoring.
4. **Evaluation & Result Tone**:
   - System evaluates final pressure and drop.
   - If `endPressure >= 310.0 mmHg` (drop $\le 10$ mmHg) $\rightarrow$ **PASS** (pleasant ascending chime).
   - If `endPressure < 310.0 mmHg` (drop $> 10$ mmHg) $\rightarrow$ **FAIL** (repeating low warning pulses).
5. **Air Exhaust (10 Seconds)**:
   - **STSY033 valve turns ON (gate opens)**: Energized via pull-in pulse followed by holding PWM to pull the solenoid open against backpressure and exhaust pressurized air to atmosphere.
   - The LCD displays live pressure drop during the 10-second venting period.
   - After 10 seconds, venting is complete down to 0 mmHg, and the valve turns **OFF** (gate closes back to rest).
6. **Thermal Receipt Print**:
   - Connects via Bluetooth to Deli DL-ES421 printer.
   - Prints date, time, target, start, end, drop, and PASS/FAIL result.

---

## 📦 Required Arduino Libraries

- `BluetoothSerial` (Included in ESP32 Arduino Core)
- `LiquidCrystal` (Built-in)
- `Wire` (Built-in)
- `RTClib` (by Adafruit)
- `WiFi` (Included in ESP32 Arduino Core)

---

## ⚙️ Arduino IDE Board & Partition Settings

To avoid the `text section exceeds available space in board` compilation error (caused by linking both Bluetooth and WiFi stacks):

1. **Board**: `ESP32 Dev Module`
2. **Partition Scheme**: **`Huge APP (3MB No OTA/1MB SPIFFS)`**
3. **Flash Frequency**: `80MHz`
4. **Upload Speed**: `921600` (or `115200`)

---

## 📄 License & Credits

Developed for **Endocare BD**. Designed for reliable endoscopy leak detection and maintenance reporting.
