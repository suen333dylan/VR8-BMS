#pragma once
#include <Arduino.h>
#include "LTC681x.h"
#include "LTC6813.h"

namespace BmsConfig {
const uint8_t TOTAL_IC            = 6;
const uint8_t ACTIVE_CELLS_PER_IC = 17;
const uint8_t NTC_PER_IC          = 4;
const uint8_t TOTAL_CELL_CHANNELS = 18;

const uint8_t BMS_OK_PIN       = 5;
const uint8_t BEEP_PIN         = 4;
const uint8_t DISCHARGE_EN_PIN = 7;
const uint8_t CAN_CS_PIN       = 9;
const uint8_t ISO_SPI_PIN      = 10;

const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const uint8_t ADC_DCP             = DCP_DISABLED;

const uint16_t CELL_OK_MIN_CODE   = 30000;
const uint16_t CELL_OK_MAX_CODE   = 43500;
const uint16_t BALANCE_DELTA_CODE = 260;
const int16_t  TEMP_MAX_DECI_C    = 600;
const int16_t  TEMP_MIN_DECI_C    = 100;
const uint32_t REPORT_DELAY_MS    = 1000;

const uint8_t BALANCE_GROUP           = 1;
const uint8_t BALANCE_CELLS_PER_GROUP = 1;

const uint32_t POLL_INTERVAL_MS      = 200;
const uint32_t BALANCE_INTERVAL_MS   = 500;
const uint32_t CAN_TX_INTERVAL_MS    = 200;
const uint32_t OPEN_WIRE_INTERVAL_MS = 5000;

const uint16_t OPEN_WIRE_NONE          = 0xFFFF;
const bool     OPEN_WIRE_ENABLED       = true;
const bool     OPEN_WIRE_BLOCKS_BMS_OK = false;

const float CODE_TO_VOLT    = 0.0001f;
const float NTC_PULLUP_OHMS = 49900.0f;
const float NTC_R0_OHMS     = 47000.0f;
const float NTC_BETA        = 4050.0f;
const float NTC_T0_KELVIN   = 298.15f;

const uint16_t CAN_ID_CELL_BASE    = 0x1100;
const uint16_t CAN_ID_TEMP_BASE    = 0x1200;
const uint16_t CAN_ID_STATUS_BASE  = 0x1300;
const uint16_t CAN_ID_BALANCE_BASE = 0x1400;

const uint8_t NTC_AUX_INDEX[4] = {0, 1, 6, 7};
} // namespace BmsConfig

enum BmsFaultBits : uint8_t {
    FAULT_NONE         = 0x00,
    FAULT_BMS_NOT_OK   = 0x01,
    FAULT_UNDER_TEMP   = 0x02,
    FAULT_OVER_TEMP    = 0x04,
    FAULT_CELL_VOLTAGE = 0x08,
    FAULT_OPEN_WIRE    = 0x10
};
