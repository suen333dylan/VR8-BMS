# VR8-BMS

[繁體中文](README_zh-TW.md) | English

VR8-BMS is a Battery Management System (BMS) firmware based on the Arduino Uno R4 Minima and LTC6813, designed for high-voltage multi-cell battery modules. This system connects multiple LTC6813 chips in a daisy chain via the isoSPI protocol and outputs battery status and alert information to the CAN Bus using an MCP2515 module.

## Core Features

* **Multi-cell Monitoring**: Supports daisy-chaining up to 6 LTC6813 ICs (totaling 102 cells).
* **Temperature Monitoring**: Supports 4 NTC thermistors per IC to ensure the battery pack operates within a safe temperature range.
* **Cell Balancing**: Features passive balancing to automatically discharge cells with high voltages.
* **Open Wire Detection**: Periodically checks if the cell voltage sensing wires are loose or disconnected.
* **Hardware Protection Signals**: Outputs a `BMS_OK` signal and a buzzer warning signal via dedicated pins.
* **CAN Bus Communication**: Supports standard and extended CAN frames. Includes a `vr8_bms.dbc` file for easy integration into the vehicle's communication network.
* **Detail Mode**: Provides real-time monitoring and debugging via the Serial port.

## Hardware Architecture

* **MCU**: Arduino Uno R4 Minima (Renesas RA4M1)
* **Battery Monitoring IC**: Analog Devices LTC6813-1 (Supports up to 18 cells; configured for 17 active channels in this project)
* **CAN Communication Module**: MCP2515 (SPI)
* **Communication Isolation**: isoSPI

### Pin Configuration

The main pin configurations are located in `src/BmsConfig.h`. The default settings are:

* **SPI (Default)**: MISO, MOSI, SCK
* **MCP2515 CS**: `Pin 9`
* **LTC6813 isoSPI CS**: `Pin 10`
* **BMS OK Output**: `Pin 5` (Outputs HIGH when normal, LOW when a fault occurs)
* **Buzzer (BEEP)**: `Pin 4`
* **Discharge Enable**: `Pin 7` (External switch or logic control to allow balancing discharge)

## Environment & Dependencies

This project uses [PlatformIO](https://platformio.org/) for development and environment management.

**Dependencies & References:**
* `coryjfowler/mcp_can@^1.5.1` (For the MCP2515 CAN module)
* Built-in or custom LTC681x/LTC6813 drivers, with implementations referencing the **[ADI Official Linduino Open Source Project (DC2350AB)](https://github.com/analogdevicesinc/Linduino/blob/master/LTSketchbook/Part%20Number/6000/6813/DC2350AB/DC2350AB.ino)**.

## How to Build and Upload

1. Install [Visual Studio Code](https://code.visualstudio.com/) and the **PlatformIO IDE** extension.
2. Open this project folder in VS Code.
3. PlatformIO will automatically download the toolchain and required libraries.
4. Click the **Build (✓)** button in the bottom status bar to compile the project.
5. Connect the Arduino Uno R4 Minima to your computer and click the **Upload (→)** button to flash the firmware.

## Monitoring & Debugging (Serial Monitor)

The system supports real-time debugging and status monitoring via the Serial port (Baud rate: `115200`).

In the terminal, you will see a "Startup Summary" upon booting and periodic "Cycle Reports" indicating the system status.

**Entering Detail Mode:**
* In the Serial Monitor, type the letter `d` or `D` and press Enter.
* The system will switch to Detail Mode, displaying a tabular view of **each cell's voltage**, **actively balancing cells (marked with `*`)**, and **the temperature of each NTC sensor**.
* Type `d` again to disable Detail Mode.

## CAN Bus Communication

For CAN Bus IDs and data parsing details, please refer to the `vr8_bms.dbc` file in the root directory.

The default CAN message Base IDs (in Hexadecimal) are:
* **Cell Voltages**: `0x1100` and up
* **Temperatures**: `0x1200` and up
* **System Status**: `0x1300` (Includes BMS OK, error codes, max/min voltages and temperatures)
* **Balance Status**: `0x1400` and up

All IDs are configured as Extended Frames, with a default transmission interval of `200ms`.

## Charger VCU

This project also includes the Charger VCU firmware (used to communicate with the On-Board Charger (OBC) and control charging), configured under `env:charger_vcu`.
For instructions on using and building the Charger VCU, please refer to the [Charger VCU Usage Instructions](src/charger_vcu/README.md).

---
*Developed for the VR8 Project.*
