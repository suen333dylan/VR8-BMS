#include "BmsCan.h"
#include "mcp_can.h"
#include "BmsLogic.h"

extern MCP_CAN can_bus;

bool BmsCan::initialize() {
    int result = can_bus.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
    if (result == CAN_OK) {
        can_bus.setMode(MCP_NORMAL);
        return true;
    }
    Serial.print(F("Error initializing CAN bus with error code:"));
    Serial.println(result);
    return false;
}

bool BmsCan::sendCanFrame(uint16_t id, const uint8_t *data, uint8_t length) {
    for (uint8_t attempt = 0; attempt < 3; ++attempt) {
        if (can_bus.sendMsgBuf(id, 1, length, const_cast<uint8_t *>(data)) == CAN_OK) {
            return true;
        }
        delay(2);
    }
    return false;
}

void BmsCan::sendCellFrames(const BmsState& state, uint8_t ic) {
    for (uint8_t frame = 0; frame < 5; ++frame) {
        uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
        for (uint8_t offset = 0; offset < 4; ++offset) {
            const uint8_t cell = (frame * 4) + offset;
            if (cell >= BmsConfig::ACTIVE_CELLS_PER_IC) break;
            const uint16_t code = state.ic[ic].cells.c_codes[cell];
            payload[offset * 2] = static_cast<uint8_t>(code & 0xFF);
            payload[(offset * 2) + 1] = static_cast<uint8_t>(code >> 8);
        }
        sendCanFrame(BmsConfig::CAN_ID_CELL_BASE + (ic * 0x10U) + frame, payload, sizeof(payload));
    }
}

void BmsCan::sendTemperatureFrame(const BmsState& state, uint8_t ic) {
    uint8_t payload[8];
    for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
        const int16_t temp = state.ntc_temperature_deci_c[ic][ntc];
        payload[ntc * 2] = static_cast<uint8_t>(temp & 0xFF);
        payload[(ntc * 2) + 1] = static_cast<uint8_t>((static_cast<uint16_t>(temp) >> 8) & 0xFF);
    }
    sendCanFrame(BmsConfig::CAN_ID_TEMP_BASE + ic, payload, sizeof(payload));
}

void BmsCan::sendStatusFrame(const BmsState& state) {
    uint16_t pack_min_code = 0xFFFF;
    uint16_t pack_max_code = 0;
    uint8_t pack_fault_bits = FAULT_NONE;
    uint8_t pack_balance_ic_mask = 0;

    if (!state.bms_ok) pack_fault_bits |= FAULT_BMS_NOT_OK;

    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        if (state.discharge_mask[ic] > 0) pack_balance_ic_mask |= (1 << ic);

        for (uint8_t ntc = 0; ntc < BmsConfig::NTC_PER_IC; ++ntc) {
            const int16_t temp = state.ntc_temperature_deci_c[ic][ntc];
            if (temp == INT16_MAX || temp < BmsConfig::TEMP_MIN_DECI_C) pack_fault_bits |= FAULT_UNDER_TEMP;
            if (temp > BmsConfig::TEMP_MAX_DECI_C) pack_fault_bits |= FAULT_OVER_TEMP;
        }

        for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
            const uint16_t code = state.ic[ic].cells.c_codes[cell];
            if (code < pack_min_code) pack_min_code = code;
            if (code > pack_max_code) pack_max_code = code;
            if (code < BmsConfig::CELL_OK_MIN_CODE || code > BmsConfig::CELL_OK_MAX_CODE) {
                pack_fault_bits |= FAULT_CELL_VOLTAGE;
            }
        }

        if (BmsLogic::hasOpenWireFault(state, ic)) {
            pack_fault_bits |= FAULT_OPEN_WIRE;
        }
    }

    uint8_t payload[8];
    payload[0] = pack_fault_bits;
    payload[1] = pack_balance_ic_mask;
    payload[2] = 0;
    payload[3] = 0;
    payload[4] = static_cast<uint8_t>(pack_min_code & 0xFF);
    payload[5] = static_cast<uint8_t>(pack_min_code >> 8);
    payload[6] = static_cast<uint8_t>(pack_max_code & 0xFF);
    payload[7] = static_cast<uint8_t>(pack_max_code >> 8);

    sendCanFrame(BmsConfig::CAN_ID_STATUS_BASE, payload, sizeof(payload));
}

void BmsCan::sendAllCan(const BmsState& state) {
    if (!state.can_ready) return;
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        sendCellFrames(state, ic);
        sendTemperatureFrame(state, ic);
    }
    sendStatusFrame(state);
}
