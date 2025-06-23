| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- | -------- |

# Smartphone controlled Self-Levelling Caravan System

A smartphone-controlled, ESP-32-S3 based platform

Awards

- Nominated for the Dean’s Award of the Faculty of Electronics, Telecommunications and Informatics, Gdańsk University of Technology.

- Voted 3 rd place in the 2025 student popularity contest “My Favourite Group Project”.

## Project overview
SSPK (Pol. System Samopoziomowania Przyczepy Kempingowej) is a real-time platform that
automatically levels a caravan once it is parked.
The core is an ESP-32-S3 module running FreeRTOS, controlling:

| Sub - system	           | Purpose|
| ------ | ----------- |
| 4 × linear actuators	| lifting / lowering each corner|
| BMI323 & LSM6DSOX IMUs| pitch/roll feedback & redundancy|
| 4 × NA27 + HX711	| load cells – check if a leg touches ground|
| 4 × VL6180X & 4 × HC-SR04| laser / ultrasonic distance sensors |
| INA180 + ADS1115	| current monitoring of every actuator|
| HDC1080	| temperature & humidity inside the electronics box|

An onboard Wi-Fi access point (2.4 GHz, WPA2-PSK, IPv4) hosts a responsive web-app.
All levelling modes can be triggered from any phone / laptop browser – no internet required.

## Main features
- Three levelling modes
- Manual – jog every actuator independently.
- Simple auto – proportional controller (P-only) in X & Y, finishes in ≈ 8 s.
- Complex auto – detects floating legs, extends them first, then runs simple auto; typical time < 40 s.
- Real-time 3-D visualisation (HTML + CSS + JS) of caravan tilt.
- Safety – “STOP” button aborts any automatic cycle and powers down all actuators.
- Rich telemetry – live IMU, environmental, distance and load-cell read-outs.
- On-board current monitoring – detects stalled or overloaded actuators.
- Firmware over USB - C, micro USB and UART flashing.

## Hardware architecture
- PCB: 2-layer, 12 V bus, designed in Altium Designer.
- Power:
- - TPS62933 – buck 12 V → 5 V (ESP-32-S3, sensors)
- - AMS1117 – linear 5 V → 3 .3 V
- - AON6403 reverse-polarity MOSFET.
- Motor drivers: 4 × DRV8251DDAR (PWM → voltage, integrator-like behaviour).
- External: 3-D-printed adapters coupling load cells to actuators (modelled in Autodesk Inventor).

## Quick start 
1. Clone
````bash
git clone https://github.com/mik305/Smartphone_controlled_esp32_caravan
cd Smartphone_controlled_esp32_caravan
````
2. Configure & build (ESP-IDF ≥ 5.0)

````bash
idf.py menuconfig    # optional Wi-Fi SSID/PSK change
idf.py flash monitor
````
3. Connect

- Join the Wi-Fi AP ESP32_WIFI

- Open http://192.168.4.1 in a browser.

- Use AUTO LEVEL SIMPLE or COMPLEX – watch the caravan level itself!





## Example folder contents

The project **sample_project** contains one source file in C language [main.c](main/main.c). The file is located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt`
files that provide set of directives and instructions describing the project's source files and targets
(executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
src/
├─ main/
│   ├─ wifi_ap.c          # Wi-Fi AP initialisation
│   ├─ http_server.c      # REST-like handlers + SPA hosting
│   ├─ levelling.c        # control algorithms (modes, PID-P, EMA)
│   ├─ sensor drivers/
│   │   ├─ bmi323_sensor.c
│   │   ├─ lsm6dsox_sensor.c
│   │   ├─ hx711.c  ...
│   └─ ...
└─ webapp/                # HTML / CSS / JS
```
Additionally, the sample project contains Makefile and component.mk files, used for the legacy Make based build system. 
They are not used or needed when building with CMake and idf.py.
