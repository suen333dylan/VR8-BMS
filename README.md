# VR8-BMS 🔋

VR8-BMS 是一套基於 Arduino Uno R4 Minima 與 LTC6813 的電池管理系統 (Battery Management System) 韌體，專為高壓多串電池模組設計。本系統透過 isoSPI 協定串接多顆 LTC6813 晶片，並透過 MCP2515 模組將電池狀態與警報資訊輸出至 CAN Bus。

## 🌟 核心功能

* **多串電池監控**：支援串接 6 顆 LTC6813 (共 102 串電池)。
* **溫度監控**：每顆 IC 支援 4 組 NTC 溫度感測器，確保電池包溫度在安全範圍內。
* **電壓均衡 (Balancing)**：具備被動均衡功能，可自動將電壓過高的電池放電。
* **斷線偵測 (Open Wire Detection)**：定期檢查電池採樣線是否鬆脫或斷裂。
* **硬體保護訊號**：透過獨立腳位輸出 `BMS_OK` 與蜂鳴器警告訊號。
* **CAN Bus 輸出**：支援標準與擴展 CAN 幀，搭配 `vr8_bms.dbc` 檔案可輕鬆整合至整車通訊網路。
* **詳細診斷模式 (Detail Mode)**：可透過序列埠 (Serial) 進行即時監控與除錯。

## 🛠 硬體架構

* **主控板 (MCU)**: Arduino Uno R4 Minima (Renesas RA4M1)
* **電池監控 IC**: Analog Devices LTC6813-1 (最多支援 18 串，本專案設定為 17 串有效通道)
* **CAN 通訊模組**: MCP2515 (SPI)
* **通訊隔離**: isoSPI

### 📌 腳位定義 (Pin Configuration)

主要腳位設定位於 `src/BmsConfig.h`，預設配置如下：

* **SPI (預設)**: MISO, MOSI, SCK
* **MCP2515 CS**: `Pin 9`
* **LTC6813 isoSPI CS**: `Pin 10`
* **BMS OK 輸出**: `Pin 5` (正常時輸出 HIGH，異常時轉為 LOW)
* **蜂鳴器 (BEEP)**: `Pin 4`
* **放電啟用 (Discharge Enable)**: `Pin 7` (外部開關或邏輯控制是否允許進行均衡放電)

## 💻 開發環境與依賴

本專案使用 [PlatformIO](https://platformio.org/) 進行開發與環境管理。

**依賴函式庫與參考專案 (Dependencies & References):**
* `coryjfowler/mcp_can@^1.5.1` (用於 MCP2515 CAN 模組)
* 內建或自定義的 LTC681x/LTC6813 驅動，該部分實作參考自 **[ADI 官方 Linduino 開源專案 (DC2350AB)](https://github.com/analogdevicesinc/Linduino/blob/master/LTSketchbook/Part%20Number/6000/6813/DC2350AB/DC2350AB.ino)**。

## 🚀 如何編譯與上傳

1. 安裝 [Visual Studio Code](https://code.visualstudio.com/) 並安裝 **PlatformIO IDE** 擴充套件。
2. 將本專案資料夾用 VS Code 打開。
3. PlatformIO 會自動下載工具鏈與相依函式庫。
4. 點擊下方狀態列的 **Build (✓)** 編譯專案。
5. 連接 Arduino Uno R4 Minima 後，點擊 **Upload (→)** 將韌體燒錄至板子上。

## 📊 監控與除錯 (Serial Monitor)

本系統支援透過序列埠 (鮑率: `115200`) 進行即時除錯與狀態監控。

在終端機中，您會看到如「啟動摘要 (Startup Summary) 」與定期的「系統狀態報告 (Cycle Report)」。

**進入詳細模式 (Detail Mode):**
* 在序列埠監控視窗中，輸入字母 `d` 或 `D` 並送出。
* 系統將切換至詳細模式，表格化列出**每一顆電池的電壓**、**正在進行均衡放電的電池（標記為 `*`）**以及**每個 NTC 感測器的溫度**。
* 再次輸入 `d` 即可關閉詳細模式。

## 📡 CAN Bus 通訊

CAN Bus 通訊相關的 ID 與資料解析，請參考專案根目錄下的 `vr8_bms.dbc` 檔案。

預設的 CAN 訊息基底 (Base ID) 為十六進位：
* **Cell Voltages**: `0x1100` 起
* **Temperatures**: `0x1200` 起
* **System Status**: `0x1300` (包含 BMS OK、錯誤碼、最高/最低電壓與溫度等)
* **Balance Status**: `0x1400` 起

這些 ID 皆定義為擴展幀 (Extended Frame)，傳送頻率預設為 `200ms`。

---
*Developed for the VR8 Project.*
