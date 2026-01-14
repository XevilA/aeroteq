# 🚗 ESP32 Smart Mirror Folding System (Auto-Fold)

A smart side-mirror control system for vehicles, powered by **ESP32-S3**, **GPS**, and **Bluetooth Low Energy (BLE)**. This system automatically folds/unfolds mirrors based on vehicle speed, stationary time, or manual control via a physical button/mobile app.

![ESP32](https://img.shields.io/badge/Platform-ESP32--S3-blue)
![License](https://img.shields.io/badge/License-MIT-green)
![Status](https://img.shields.io/badge/Status-Stable-brightgreen)

## ✨ Key Features

* **⚡ Non-blocking Architecture:** Utilizes a state machine for servo control, ensuring GPS data and BLE commands are processed seamlessly while mirrors are moving.
* **📍 GPS-Based Automation:**
    * **Auto-Fold:** Automatically folds mirrors when parked (stationary for > 2 mins).
    * **Speed Lock:** Folds mirrors at high speeds (configurable threshold) and unfolds at low speeds.
* **📱 BLE Control:** Full control via Bluetooth Low Energy app (compatible with nRF Connect or custom apps).
* **🔘 Smart Button:** Physical button with "Smart Sync" logic—automatically detects current state to toggle between Open/Close correctly.
* **💾 EEPROM Settings:** Remembers last position, speed thresholds, and servo inversion settings even after power loss.
* **🛡️ Servo Protection:** Automatically detaches servos after movement to prevent overheating and save power.

---

## 🛠️ Hardware Requirements

| Component | Description |
| :--- | :--- |
| **MCU** | ESP32-S3 (or standard ESP32) |
| **GPS Module** | NEO-6M / NEO-8M (UART) |
| **Servos** | 2x High-Torque Servos (e.g., MG996R) |
| **Button** | 1x Momentary Push Button |
| **Power** | 5V - 6V DC Supply (Regulated for Servos) |

### Pin Configuration (Default)

| Device | ESP32 Pin | Note |
| :--- | :--- | :--- |
| **Servo Left (S1)** | GPIO 13 | PWM Output |
| **Servo Right (S2)** | GPIO 12 | PWM Output |
| **Button** | GPIO 10 | Input Pull-up |
| **GPS RX** | GPIO 16 | Connect to GPS TX |
| **GPS TX** | GPIO 17 | Connect to GPS RX |

> **Note:** Check your specific ESP32 board pinout. You can change these defines in the `CONFIGURATION` section of the code.

---

## 🚀 Installation & Setup

1.  **Dependencies:** Install the following libraries in Arduino IDE:
    * `ESP32Servo`
    * `TinyGPSPlus`
    * (Built-in) `BLEDevice`, `EEPROM`
2.  **Upload:**
    * Select your board (e.g., ESP32S3 Dev Module).
    * Compile and upload `SmartMirror.ino`.
3.  **Wiring:** Connect components according to the pin configuration table.

---

## 📡 BLE Commands

Connect to the device named **`ESP32S3_Mirror`**. Write strings to the characteristic UUID `beb5483e...`.

### Control Commands
| Command | Action |
| :--- | :--- |
| `OPEN` | Unfolds mirrors to position M1. |
| `CLOSE` | Folds mirrors to position M6. |
| `AUTO` | Enables automatic GPS control mode. |
| `M1` - `M6` | Move to specific angle preset (M1=Open, M6=Fold). |

### Configuration Commands
| Command | Action |
| :--- | :--- |
| `SET HIGH [val]` | Set high speed threshold (e.g., `SET HIGH 60`). |
| `SET LOW [val]` | Set low speed threshold (e.g., `SET LOW 10`). |
| `INVERT S1` | Toggle rotation direction for Servo 1. |
| `INVERT S2` | Toggle rotation direction for Servo 2. |
| `STATUS` | Returns current system status via Serial. |

---

## ⚙️ Logic Flow

1.  **Startup:** Loads settings from EEPROM. Moves mirrors to the last known position.
2.  **Loop:**
    * **GPS Tick:** Updates speed and location.
    * **Button Tick:** Checks for physical interaction.
    * **Servo Tick:** Manages non-blocking movement and power cut-off.
    * **Auto Control:** Evaluates speed rules to trigger Fold/Unfold events.

---

## 📝 License

This project is open-source and available under the [MIT License](LICENSE).
