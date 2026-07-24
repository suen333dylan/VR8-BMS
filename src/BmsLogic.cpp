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



void BmsLogic::printCycleReport(const BmsState& state) {
    Serial.println(F("\n================================================================================"));
    Serial.print(F(" BMS Uptime: ")); Serial.print(millis() / 1000); Serial.print(F("s | "));
    Serial.print(F("BMS_OK: ")); Serial.print(state.bms_ok ? F("YES") : F("NO ")); Serial.print(F(" | "));
    Serial.print(F("CAN: ")); Serial.print(state.can_ready ? F("OK  ") : F("FAIL")); Serial.print(F(" | "));
    Serial.print(F("Discharge: ")); Serial.print(state.discharge_enabled ? F("ON ") : F("OFF"));
    Serial.println(F("\n------------------------------------------------------------------------------------------"));
    
    Serial.println(F(" IC | V_Min      | V_Max      | T_Min     |   T_Max   | Balance      | OW"));
    Serial.println(F("----+------------+------------+-----------+-----------+--------------+------"));
    
    uint16_t pack_min = 0xFFFF, pack_max = 0;
    uint8_t pack_min_ic = 0, pack_min_cell = 0;
    uint8_t pack_max_ic = 0, pack_max_cell = 0;
    
    int16_t pack_t_min = INT16_MAX, pack_t_max = INT16_MIN;
    uint8_t pack_t_min_ic = 0, pack_t_min_ntc = 0;
    uint8_t pack_t_max_ic = 0, pack_t_max_ntc = 0;

    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        uint16_t ic_min = 0xFFFF, ic_max = 0;
        uint8_t ic_min_c = 0, ic_max_c = 0;
        
        for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
            uint16_t code = state.ic[ic].cells.c_codes[cell];
            if (code < ic_min) { ic_min = code; ic_min_c = cell; }
            if (code > ic_max) { ic_max = code; ic_max_c = cell; }
        }
        if (ic_min < pack_min) { pack_min = ic_min; pack_min_ic = ic; pack_min_cell = ic_min_c; }
        if (ic_max > pack_max) { pack_max = ic_max; pack_max_ic = ic; pack_max_cell = ic_max_c; }
        
        int16_t ic_t_min = INT16_MAX, ic_t_max = INT16_MIN;
        uint8_t ic_t_min_n = 0, ic_t_max_n = 0;
        for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
            int16_t temp = state.ntc_temperature_deci_c[ic][ntc];
            if (temp != INT16_MAX) {
                if (temp < ic_t_min) { ic_t_min = temp; ic_t_min_n = ntc; }
                if (temp > ic_t_max) { ic_t_max = temp; ic_t_max_n = ntc; }
            }
        }
        if (ic_t_min != INT16_MAX && ic_t_min < pack_t_min) { pack_t_min = ic_t_min; pack_t_min_ic = ic; pack_t_min_ntc = ic_t_min_n; }
        if (ic_t_max != INT16_MIN && ic_t_max > pack_t_max) { pack_t_max = ic_t_max; pack_t_max_ic = ic; pack_t_max_ntc = ic_t_max_n; }

        Serial.print(F(" "));
        if (ic + 1 < 10) Serial.print(F(" "));
        Serial.print(ic + 1); Serial.print(F(" | "));
        
        Serial.print(F("C")); 
        if (ic_min_c + 1 < 10) Serial.print(F("0"));
        Serial.print(ic_min_c + 1);
        Serial.print(F("(")); Serial.print(ic_min * BmsConfig::CODE_TO_VOLT, 3); Serial.print(F(") | "));
        
        Serial.print(F("C")); 
        if (ic_max_c + 1 < 10) Serial.print(F("0"));
        Serial.print(ic_max_c + 1);
        Serial.print(F("(")); Serial.print(ic_max * BmsConfig::CODE_TO_VOLT, 3); Serial.print(F(") | "));

        if (ic_t_min != INT16_MAX) {
            Serial.print(F("T")); Serial.print(ic_t_min_n + 1); Serial.print(F("(")); 
            printTemperatureValue(ic_t_min); Serial.print(F(") | "));
        } else {
            Serial.print(F("ERR     | "));
        }
        
        if (ic_t_max != INT16_MIN) {
            Serial.print(F("T")); Serial.print(ic_t_max_n + 1); Serial.print(F("(")); 
            printTemperatureValue(ic_t_max); Serial.print(F(") | "));
        } else {
            Serial.print(F("ERR     | "));
        }
        
        uint8_t bal_count = 0;
        for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
            if ((state.discharge_mask[ic] >> cell) & 1) bal_count++;
        }
        if (bal_count == 0) {
            Serial.print(F("none         | "));
        } else {
            if (bal_count < 10) Serial.print(F(" "));
            Serial.print(bal_count); Serial.print(F(" cells      | "));
        }
        
        printOpenWireStatus(state, ic);
        Serial.println();
    }
    
    Serial.println(F("------------------------------------------------------------------------------------------"));
    Serial.print(F(" Pack V_Min: ")); Serial.print(pack_min * BmsConfig::CODE_TO_VOLT, 4); 
    Serial.print(F("V (IC")); Serial.print(pack_min_ic + 1); Serial.print(F(" C")); 
    if (pack_min_cell + 1 < 10) Serial.print(F("0"));
    Serial.print(pack_min_cell + 1);
    Serial.print(F(")   |   Pack V_Max: ")); Serial.print(pack_max * BmsConfig::CODE_TO_VOLT, 4);
    Serial.print(F("V (IC")); Serial.print(pack_max_ic + 1); Serial.print(F(" C")); 
    if (pack_max_cell + 1 < 10) Serial.print(F("0"));
    Serial.print(pack_max_cell + 1);
    Serial.print(F(")   |   Delta: ")); Serial.print((pack_max - pack_min) * BmsConfig::CODE_TO_VOLT, 4); Serial.println(F("V"));
    
    Serial.print(F(" Pack T_Min: ")); 
    if (pack_t_min != INT16_MAX) {
        printTemperatureValue(pack_t_min); 
        Serial.print(F(" (IC")); Serial.print(pack_t_min_ic + 1); Serial.print(F(" T")); Serial.print(pack_t_min_ntc + 1); Serial.print(F(")"));
    } else {
        Serial.print(F("ERR"));
    }
    Serial.print(F("         |   Pack T_Max: ")); 
    if (pack_t_max != INT16_MIN) {
        printTemperatureValue(pack_t_max);
        Serial.print(F(" (IC")); Serial.print(pack_t_max_ic + 1); Serial.print(F(" T")); Serial.print(pack_t_max_ntc + 1); Serial.print(F(")"));
    } else {
        Serial.print(F("ERR"));
    }
    Serial.println();
    Serial.println(F("=========================================================================================="));

    if (state.detail_mode) {
        Serial.println(F("\n[ Detailed Cell Voltages & Balance States ]"));
        Serial.print(F("IC | "));
        for (uint8_t c = 0; c < BmsConfig::ACTIVE_CELLS_PER_IC; ++c) {
            Serial.print(F("C"));
            if (c + 1 < 10) Serial.print(F("0"));
            Serial.print(c + 1);
            Serial.print(F("    | "));
        }
        for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
            Serial.print(F("T")); Serial.print(ntc + 1); Serial.print(F("    | "));
        }
        Serial.println();
        
        for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
            if (ic + 1 < 10) Serial.print(F(" "));
            Serial.print(ic + 1); Serial.print(F(" | "));
            for (uint8_t c = 0; c < BmsConfig::ACTIVE_CELLS_PER_IC; ++c) {
                float v = state.ic[ic].cells.c_codes[c] * BmsConfig::CODE_TO_VOLT;
                Serial.print(v, 3);
                bool balancing = (state.discharge_mask[ic] >> c) & 1;
                Serial.print(balancing ? F("* | ") : F("  | "));
            }
            for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
                int16_t temp = state.ntc_temperature_deci_c[ic][ntc];
                if (temp != INT16_MAX) {
                    if (temp >= 0 && temp < 100) Serial.print(F(" "));
                }
                printTemperatureValue(temp);
                Serial.print(F(" | "));
            }
            Serial.println();
        }
        Serial.println(F("Note: '*' indicates cell is actively balancing."));
    }
    Serial.println();
}
