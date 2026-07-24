# Charger VCU (充電控制單元)

[English](README.md) | 繁體中文

本目錄包含 VR8-BMS 專案中的充電控制單元 (Charger VCU) 韌體。

Charger VCU 主要負責與車載充電器 (OBC) 及 BMS 進行 CAN 通訊，監控電池狀態，並確保充電過程的安全性。

## 編譯與上傳

此專案設定為獨立的 PlatformIO 環境。在 VS Code 中，請確保切換到 `env:charger_vcu` 環境：

1. 點擊下方狀態列的環境按鈕 (通常顯示 `env:vr8_bms` 或 `env:main`)。
2. 選擇 `env:charger_vcu`。
3. 點擊 **Build** 進行編譯，點擊 **Upload** 燒錄至 ESP32-C6。

或使用指令：
```bash
pio run -e charger_vcu
pio run -e charger_vcu -t upload
```

## 使用說明

Charger VCU 提供兩種啟動充電的互鎖方式：
1. **實體按鈕啟動**：按下連接在 `START_BUTTON_PIN` 的按鈕。
2. **序列埠指令啟動**：透過 Serial Monitor 傳送 `start` 指令。

只要滿足以下條件，充電即會開始：
- 收到充電啟動要求 (按鈕或 `start` 指令)。
- Precharge 訊號有效。
- (硬體 Relay 保護由 BMS 負責)。

**停止充電**：
- 透過 Serial 傳送 `stop` 或 `e` 指令。
- BMS 硬體保護介入斷開 Relay。
- OBC 達到設定的最高電壓 (`HV_BATT_U_LIM_VOLTS`)。

## 狀態監控
系統會透過序列埠 (Baud rate: `115200`) 定期輸出目前的充電狀態 (Charging State)、電池電壓、OBC 輸出參數，以及是否有錯誤發生。
