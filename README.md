# UWAC-BP26-Firmware

## Overview

Firmware for the UWAC Beginners' Rocketry 2026 flight computer. Uses non-blocking state machine design.

## Technical Details

- **Delta-Time Calculation**: Time that passes between loop
- **EMA Sensor Filter**: EMA to smooth out sensor noise spikes
- **Continuity Checks**: Prevent premature ignition of pyros

## Diagnostic Status Codes

The flight computer uses the onboard LED (PC13) to signal system health using a (**Category**, **Code**) blink pattern.

| Category | Component | Code | Meaning |
| :--- | :--- | :--- | :--- |
| **1** | **System** | 1 | **Ready**: All systems initialized correctly. |
| **2** | **IMU** | 1 | **Failed**: MPU6050 not detected. |
| **3** | **Baro** | 1 | **Failed**: BMP280 not detected. |
| **4** | **Pyro** | 1 | **CH1**: Drogue continuity loop open. |
| | | 2 | **CH2**: Main continuity loop open. |
| | | 3 | **Both**: Both pyro loops open. |

*Note: Critical failures (IMU/Baro) will block the flight loop until resolved.*