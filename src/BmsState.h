#pragma once
#include <Arduino.h>
#include "BmsConfig.h"
#include "LTC681x.h"

struct BmsState {
    cell_asic ic[BmsConfig::TOTAL_IC];
    uint32_t  discharge_mask[BmsConfig::TOTAL_IC];
    uint32_t  balance_mask[BmsConfig::TOTAL_IC];
    int16_t   ntc_temperature_deci_c[BmsConfig::TOTAL_IC][BmsConfig::NTC_PER_IC];
    uint16_t  open_wire_channel[BmsConfig::TOTAL_IC];
    bool      open_wire_valid[BmsConfig::TOTAL_IC];

    bool     can_ready;
    bool     bms_ok;
    bool     discharge_enabled;
    uint32_t cycle_count;
    uint32_t first_error_state_ms;
    bool     detail_mode;

    BmsState() : can_ready(false),
                 bms_ok(false),
                 discharge_enabled(false),
                 cycle_count(0),
                 first_error_state_ms(0),
                 detail_mode(false) {
        for (uint8_t i = 0; i < BmsConfig::TOTAL_IC; ++i) {
            discharge_mask[i]    = 0;
            balance_mask[i]      = 0;
            open_wire_channel[i] = BmsConfig::OPEN_WIRE_NONE;
            open_wire_valid[i]   = false;
            for (uint8_t n = 0; n < BmsConfig::NTC_PER_IC; ++n) {
                ntc_temperature_deci_c[i][n] = INT16_MAX;
            }
        }
    }
};
