# Charger VCU (Vehicle Control Unit)

[繁體中文](README_zh-TW.md) | English

This directory contains the Charger VCU firmware of the VR8-BMS project.

The Charger VCU is responsible for communicating with the On-Board Charger (OBC) and the BMS via CAN bus, monitoring battery status, and ensuring charging safety.

## Build and Upload

This project is configured as a separate PlatformIO environment. In VS Code, please ensure you switch to the `env:charger_vcu` environment:

1. Click the environment button on the bottom status bar (usually shows `env:vr8_bms` or `env:main`).
2. Select `env:charger_vcu`.
3. Click **Build** to compile, then click **Upload** to flash to the ESP32-C6.

Or using the CLI:
```bash
pio run -e charger_vcu
pio run -e charger_vcu -t upload
```

## Usage Instructions

The Charger VCU provides two interlocking methods to start charging:
1. **Physical Button Start**: Press the button connected to `START_BUTTON_PIN`.
2. **Serial Command Start**: Send the `start` command via Serial Monitor.

Charging will begin as long as the following conditions are met:
- A start charging request is received (button or `start` command).
- The Precharge signal is valid.
- (Hardware Relay protection is handled by the BMS).

**Stopping the charge**:
- Send the `stop` or `e` command via Serial.
- BMS hardware protection intervenes and disconnects the Relay.
- OBC output reaches the maximum configured voltage (`HV_BATT_U_LIM_VOLTS`).

## Status Monitoring
The system periodically outputs the current Charging State, battery voltage, OBC output parameters, and any fault warnings via the Serial port (Baud rate: `115200`).
