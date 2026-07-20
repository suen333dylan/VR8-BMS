#include "BmsLogic.h"
#include <math.h>

void BmsLogic::setBmsOk(BmsState& state, bool ok, bool delay) {
    digitalWrite(BmsConfig::BEEP_PIN, ok ? LOW : HIGH);
    uint32_t now = millis();
    if (!ok) {
        if (state.first_error_state_ms == 0) {
            state.first_error_state_ms = now;
        }
        if (delay) {
            digitalWrite(BmsConfig::BMS_OK_PIN, (now - state.first_error_state_ms) >= BmsConfig::REPORT_DELAY_MS ? LOW : HIGH);
        } else {
            digitalWrite(BmsConfig::BMS_OK_PIN, LOW);
        }
    } else {
        digitalWrite(BmsConfig::BMS_OK_PIN, HIGH);
        state.first_error_state_ms = 0;
    }
}

void BmsLogic::clearDischargeRequests(BmsState& state) {
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        state.discharge_mask[ic] = 0;
    }
}

void BmsLogic::recomputeDischargeRequests(BmsState& state) {
    clearDischargeRequests(state);
    uint16_t min_code = 0xFFFF;
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
            const uint16_t code = state.ic[ic].cells.c_codes[cell];
            if (code > 0 && code < min_code) {
                min_code = code;
            }
        }
    }
    if (min_code == 0xFFFF) return;
    
    const uint16_t balance_threshold = min_code + BmsConfig::BALANCE_DELTA_CODE;
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
            if (state.ic[ic].cells.c_codes[cell] >= balance_threshold) {
                state.discharge_mask[ic] |= (1UL << cell);
            }
        }
    }
}

void BmsLogic::recomputeBalanceMask(BmsState& state) {
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        state.balance_mask[ic] = 0;
        for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
            if ((state.cycle_count + cell) % BmsConfig::BALANCE_GROUP < BmsConfig::BALANCE_CELLS_PER_GROUP) {
                state.balance_mask[ic] |= (1UL << cell);
            }
        }
    }
    state.cycle_count++;
}

void BmsLogic::resetOpenWireState(BmsState& state) {
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        state.open_wire_channel[ic] = BmsConfig::OPEN_WIRE_NONE;
        state.open_wire_valid[ic] = false;
    }
}

bool BmsLogic::hasOpenWireFault(const BmsState& state, uint8_t ic) {
    return state.open_wire_valid[ic] && state.open_wire_channel[ic] != BmsConfig::OPEN_WIRE_NONE;
}

bool BmsLogic::computeFaultState(const BmsState& state) {
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
            const uint16_t code = state.ic[ic].cells.c_codes[cell];
            if (code < BmsConfig::CELL_OK_MIN_CODE || code > BmsConfig::CELL_OK_MAX_CODE) {
                return false;
            }
        }
        for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
            const int16_t temp_deci_c = state.ntc_temperature_deci_c[ic][ntc];
            if (temp_deci_c == INT16_MAX || temp_deci_c > BmsConfig::TEMP_MAX_DECI_C || temp_deci_c < BmsConfig::TEMP_MIN_DECI_C) {
                return false;
            }
        }
        if (BmsConfig::OPEN_WIRE_BLOCKS_BMS_OK && hasOpenWireFault(state, ic)) {
            return false;
        }
    }
    return true;
}

int16_t BmsLogic::temperatureFromAuxCode(uint16_t aux_code, uint16_t vref2_code) {
    if (aux_code == 0 || vref2_code == 0 || aux_code >= vref2_code) return INT16_MAX;
    const float aux_voltage = aux_code * BmsConfig::CODE_TO_VOLT;
    const float reference_voltage = vref2_code * BmsConfig::CODE_TO_VOLT;
    const float denominator = reference_voltage - aux_voltage;
    if (denominator <= 0.0f) return INT16_MAX;
    const float resistance = BmsConfig::NTC_PULLUP_OHMS * aux_voltage / denominator;
    if (resistance <= 0.0f) return INT16_MAX;
    const float inverse_kelvin = (1.0f / BmsConfig::NTC_T0_KELVIN) + (logf(resistance / BmsConfig::NTC_R0_OHMS) / BmsConfig::NTC_BETA);
    if (inverse_kelvin <= 0.0f) return INT16_MAX;
    const float temp_c = (1.0f / inverse_kelvin) - 273.15f;
    if (isnan(temp_c) || isinf(temp_c)) return INT16_MAX;
    const float scaled = temp_c * 10.0f;
    return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

void BmsLogic::updateTemperatures(BmsState& state) {
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        const uint16_t vref2_code = state.ic[ic].aux.a_codes[5];
        for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
            state.ntc_temperature_deci_c[ic][ntc] = temperatureFromAuxCode(state.ic[ic].aux.a_codes[BmsConfig::NTC_AUX_INDEX[ntc]], vref2_code);
        }
    }
}

void BmsLogic::printTemperatureValue(int16_t temp_deci_c) {
    if (temp_deci_c == INT16_MAX) { Serial.print(F("ERR")); return; }
    Serial.print(temp_deci_c / 10);
    Serial.print('.');
    Serial.print(abs(temp_deci_c % 10));
    Serial.print(F("C"));
}

void BmsLogic::printOpenWireStatus(const BmsState& state, uint8_t ic) {
    if (!BmsConfig::OPEN_WIRE_ENABLED) { Serial.print(F("disabled")); return; }
    if (!state.open_wire_valid[ic]) { Serial.print(F("pending")); return; }
    if (!hasOpenWireFault(state, ic)) { Serial.print(F("none")); return; }
    Serial.print(F("C"));
    Serial.print(state.open_wire_channel[ic]);
}

void BmsLogic::printBalanceCells(const BmsState& state, uint8_t ic) {
    bool first = true;
    for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
        if (!((state.discharge_mask[ic] >> cell) & 1)) continue;
        if (!first) Serial.print(',');
        Serial.print(cell + 1);
        first = false;
    }
    if (first) Serial.print(F("none"));
}

void BmsLogic::printIcReport(const BmsState& state, uint8_t ic) {
    uint16_t min_code = 0xFFFF;
    uint16_t max_code = 0;
    uint8_t min_cell = 0;
    uint8_t max_cell = 0;
    for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
        const uint16_t code = state.ic[ic].cells.c_codes[cell];
        if (code < min_code) { min_code = code; min_cell = cell; }
        if (code > max_code) { max_code = code; max_cell = cell; }
    }
    Serial.print(F("IC")); Serial.print(ic + 1); Serial.print(F(" Cells: "));
    for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
        if (cell > 0) Serial.print(F(", "));
        Serial.print(F("C")); Serial.print(cell + 1); Serial.print('=');
        Serial.print(state.ic[ic].cells.c_codes[cell] * BmsConfig::CODE_TO_VOLT, 4); Serial.print(F("V"));
    }
    Serial.println();
    Serial.print(F("IC")); Serial.print(ic + 1); Serial.print(F(" Temps: "));
    for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
        if (ntc > 0) Serial.print(F(", "));
        Serial.print(F("T")); Serial.print(ntc + 1); Serial.print('=');
        printTemperatureValue(state.ntc_temperature_deci_c[ic][ntc]);
    }
    Serial.println();
    Serial.print(F("IC")); Serial.print(ic + 1); Serial.print(F(" Range: min=C"));
    Serial.print(min_cell + 1); Serial.print('('); Serial.print(min_code * BmsConfig::CODE_TO_VOLT, 4);
    Serial.print(F("V), max=C")); Serial.print(max_cell + 1); Serial.print('(');
    Serial.print(max_code * BmsConfig::CODE_TO_VOLT, 4); Serial.print(F("V), balance="));
    printBalanceCells(state, ic); Serial.print(F(", open-wire="));
    printOpenWireStatus(state, ic); Serial.println();
}

void BmsLogic::printCycleReport(const BmsState& state) {
    Serial.println();
    Serial.print(F("=== BMS Time ")); Serial.print(millis()); Serial.println(F(" ms ==="));
    Serial.print(F("BMS_OK=")); Serial.print(state.bms_ok ? F("HIGH") : F("LOW"));
    Serial.print(F(", CAN=")); Serial.print(state.can_ready ? F("OK") : F("FAIL"));
    Serial.print(F(", Discharge=")); Serial.println(state.discharge_enabled ? F("ENABLED") : F("DISABLED"));
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        printIcReport(state, ic);
    }
}
