#include <Arduino.h>
#include <SPI.h>
#include "HardwareSerial.h"
#include "mcp_can.h"
#include "BmsState.h"
#include "BmsConfig.h"
#include "LtcController.h"
#include "BmsLogic.h"
#include "BmsCan.h"

MCP_CAN can_bus(BmsConfig::CAN_CS_PIN);
BmsState bmsState;

uint32_t last_poll_ms = 0;
uint32_t last_balance_ms = 0;
uint32_t last_can_tx_ms = 0;
uint32_t last_open_wire_ms = 0;

void printStartupSummary() {
    Serial.println(F("========================================="));
    Serial.println(F("          VR8-BMS Startup Summary        "));
    Serial.println(F("========================================="));
    Serial.print(F(" - Total ICs:                ")); Serial.println(BmsConfig::TOTAL_IC);
    Serial.print(F(" - Active Cells per IC:      ")); Serial.println(BmsConfig::ACTIVE_CELLS_PER_IC);
    Serial.print(F(" - Physical Channels per IC: ")); Serial.println(BmsConfig::TOTAL_CELL_CHANNELS);
    Serial.print(F(" - NTC Thermistors per IC:   ")); Serial.println(BmsConfig::NTC_PER_IC);
    Serial.println(F("-----------------------------------------"));
    Serial.print(F(" - Cell OK Min Voltage:      ")); Serial.print(BmsConfig::CELL_OK_MIN_CODE * BmsConfig::CODE_TO_VOLT, 3); Serial.println(F(" V"));
    Serial.print(F(" - Cell OK Max Voltage:      ")); Serial.print(BmsConfig::CELL_OK_MAX_CODE * BmsConfig::CODE_TO_VOLT, 3); Serial.println(F(" V"));
    Serial.print(F(" - Balance Delta Threshold:  ")); Serial.print(BmsConfig::BALANCE_DELTA_CODE * BmsConfig::CODE_TO_VOLT, 3); Serial.println(F(" V"));
    Serial.print(F(" - Temp OK Range:            ")); Serial.print(BmsConfig::TEMP_MIN_DECI_C / 10.0f, 1); Serial.print(F(" ~ ")); Serial.print(BmsConfig::TEMP_MAX_DECI_C / 10.0f, 1); Serial.println(F(" C"));
    Serial.println(F("-----------------------------------------"));
    Serial.print(F(" - Open Wire Diagnostic:     ")); Serial.println(BmsConfig::OPEN_WIRE_ENABLED ? F("ENABLED") : F("DISABLED"));
    if (BmsConfig::OPEN_WIRE_ENABLED) {
        Serial.print(F("   * Interval:               ")); Serial.print(BmsConfig::OPEN_WIRE_INTERVAL_MS); Serial.println(F(" ms"));
        Serial.print(F("   * Blocks BMS_OK:          ")); Serial.println(BmsConfig::OPEN_WIRE_BLOCKS_BMS_OK ? F("YES") : F("NO"));
    }
    Serial.println(F("=========================================\n"));
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Initializing VR8-BMS..."));

    pinMode(BmsConfig::BMS_OK_PIN, OUTPUT);
    BmsLogic::setBmsOk(bmsState, false, false);

    pinMode(BmsConfig::ISO_SPI_PIN, OUTPUT);
    digitalWrite(BmsConfig::ISO_SPI_PIN, HIGH);

    pinMode(BmsConfig::CAN_CS_PIN, OUTPUT);
    digitalWrite(BmsConfig::CAN_CS_PIN, HIGH);

    pinMode(BmsConfig::BEEP_PIN, OUTPUT);
    digitalWrite(BmsConfig::BEEP_PIN, LOW);

    pinMode(BmsConfig::DISCHARGE_EN_PIN, INPUT_PULLUP);

    SPI.begin();
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    LtcController::initialize(bmsState);
    BmsLogic::resetOpenWireState(bmsState);
    BmsLogic::clearDischargeRequests(bmsState);
    BmsLogic::recomputeBalanceMask(bmsState);
    LtcController::writeConfiguration(bmsState);
    SPI.endTransaction();

    bmsState.can_ready = BmsCan::initialize();
    if (!bmsState.can_ready) {
        Serial.println(F("MCP2515 init failed"));
    }
    
    printStartupSummary();
}

void loop() {
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'd' || c == 'D') {
            bmsState.detail_mode = !bmsState.detail_mode;
            Serial.print(F("\n>>> Detail Mode: "));
            Serial.println(bmsState.detail_mode ? F("ON <<<") : F("OFF <<<"));
        }
    }

    const uint32_t now = millis();

    if (last_poll_ms == 0 || now - last_poll_ms >= BmsConfig::POLL_INTERVAL_MS) {
        last_poll_ms = now;
        bool cells_ok = LtcController::measureCells(bmsState);
        bool aux_ok = LtcController::measureAux(bmsState);
        
        if (!cells_ok || !aux_ok) {
            BmsLogic::clearDischargeRequests(bmsState);
            LtcController::writeConfiguration(bmsState);
            BmsLogic::setBmsOk(bmsState, false);
            Serial.print(F("Polling failed: cells="));
            Serial.print(cells_ok ? F("OK") : F("ERR"));
            Serial.print(F(", aux="));
            Serial.println(aux_ok ? F("OK") : F("ERR"));
        } else {
            BmsLogic::updateTemperatures(bmsState);
            BmsLogic::recomputeDischargeRequests(bmsState);
        }
    }

    if (BmsConfig::OPEN_WIRE_ENABLED && (last_open_wire_ms == 0 || now - last_open_wire_ms >= BmsConfig::OPEN_WIRE_INTERVAL_MS)) {
        last_open_wire_ms = now;
        BmsLogic::clearDischargeRequests(bmsState);
        LtcController::writeConfiguration(bmsState);
        LtcController::runOpenWireDiagnostic(bmsState);
    }

    bmsState.bms_ok = BmsLogic::computeFaultState(bmsState);
    BmsLogic::setBmsOk(bmsState, bmsState.bms_ok);
    bmsState.discharge_enabled = (digitalRead(BmsConfig::DISCHARGE_EN_PIN) == LOW);

    if (bmsState.bms_ok && bmsState.discharge_enabled) {
        if (last_balance_ms == 0 || now - last_balance_ms >= BmsConfig::BALANCE_INTERVAL_MS) {
            last_balance_ms = now;
            BmsLogic::recomputeBalanceMask(bmsState);
            LtcController::writeConfiguration(bmsState);
        }
    } else {
        BmsLogic::clearDischargeRequests(bmsState);
        LtcController::writeConfiguration(bmsState);
    }

    if (bmsState.can_ready && (!bmsState.bms_ok || last_can_tx_ms == 0 || now - last_can_tx_ms >= BmsConfig::CAN_TX_INTERVAL_MS)) {
        last_can_tx_ms = now;
        BmsCan::sendAllCan(bmsState);
        BmsLogic::printCycleReport(bmsState);
    }
}
