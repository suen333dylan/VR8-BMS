#include "LtcController.h"
#include <SPI.h>
#include "LTC6813.h"

void LtcController::initialize(BmsState& state) {
    LTC6813_init_cfg(BmsConfig::TOTAL_IC, state.ic);
    LTC6813_init_cfgb(BmsConfig::TOTAL_IC, state.ic);
    LTC6813_reset_crc_count(BmsConfig::TOTAL_IC, state.ic);
    LTC6813_init_reg_limits(BmsConfig::TOTAL_IC, state.ic);
}

void LtcController::configureIc(uint8_t ic_index, BmsState& state) {
    bool dcc_a[12] = {false};
    bool dcc_b[7]  = {false};

    for (uint8_t cell = 0; cell < BmsConfig::ACTIVE_CELLS_PER_IC; ++cell) {
        if (!((state.discharge_mask[ic_index] >> cell) & 1) || !((state.balance_mask[ic_index] >> cell) & 1)) {
            continue;
        }
        if (cell < 12) {
            dcc_a[cell] = true;
        } else {
            dcc_b[(cell - 12) + 1] = true;
        }
    }

    bool GPIOBITS_A[5] = {true, true, false, false, false};
    bool GPIOBITS_B[4] = {true, true, false, false};
    bool DCTOBITS[4]   = {false, false, false, false};
    bool PSBITS[2]     = {false, false};

    LTC6813_set_cfgr(ic_index, state.ic, true, false, GPIOBITS_A, dcc_a, DCTOBITS, BmsConfig::CELL_OK_MIN_CODE, BmsConfig::CELL_OK_MAX_CODE);
    LTC6813_set_cfgrb(ic_index, state.ic, false, true, PSBITS, GPIOBITS_B, dcc_b);
}

bool LtcController::writeConfiguration(BmsState& state) {
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        configureIc(ic, state);
    }
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    wakeup_sleep(BmsConfig::TOTAL_IC);
    LTC6813_wrcfg(BmsConfig::TOTAL_IC, state.ic);
    LTC6813_wrcfgb(BmsConfig::TOTAL_IC, state.ic);
    SPI.endTransaction();
    return true;
}

bool LtcController::measureCells(BmsState& state) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    wakeup_sleep(BmsConfig::TOTAL_IC);
    LTC6813_adcv(BmsConfig::ADC_CONVERSION_MODE, BmsConfig::ADC_DCP, CELL_CH_ALL);
    LTC6813_pollAdc();
    wakeup_idle(BmsConfig::TOTAL_IC);
    bool result = LTC6813_rdcv(REG_ALL, BmsConfig::TOTAL_IC, state.ic) == 0;
    SPI.endTransaction();
    return result;
}

bool LtcController::measureAux(BmsState& state) {
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    wakeup_sleep(BmsConfig::TOTAL_IC);
    LTC6813_adax(BmsConfig::ADC_CONVERSION_MODE, AUX_CH_ALL);
    LTC6813_pollAdc();
    wakeup_idle(BmsConfig::TOTAL_IC);
    bool result = LTC6813_rdaux(REG_ALL, BmsConfig::TOTAL_IC, state.ic) == 0;
    SPI.endTransaction();
    return result;
}

uint16_t LtcController::normalizeOpenWireChannel(long raw_channel) {
    if (raw_channel < 0 || raw_channel == BmsConfig::OPEN_WIRE_NONE || raw_channel > BmsConfig::ACTIVE_CELLS_PER_IC) {
        return BmsConfig::OPEN_WIRE_NONE;
    }
    return static_cast<uint16_t>(raw_channel);
}

void LtcController::runOpenWireDiagnostic(BmsState& state) {
    uint16_t saved_codes[BmsConfig::TOTAL_IC][BmsConfig::TOTAL_CELL_CHANNELS];
    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        for (uint8_t cell = 0; cell < BmsConfig::TOTAL_CELL_CHANNELS; ++cell) {
            saved_codes[ic][cell] = state.ic[ic].cells.c_codes[cell];
        }
    }

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
    LTC6813_run_openwire_single(BmsConfig::TOTAL_IC, state.ic);
    SPI.endTransaction();

    for (uint8_t ic = 0; ic < BmsConfig::TOTAL_IC; ++ic) {
        state.open_wire_channel[ic] = normalizeOpenWireChannel(state.ic[ic].system_open_wire);
        state.open_wire_valid[ic]   = true;
        for (uint8_t cell = 0; cell < BmsConfig::TOTAL_CELL_CHANNELS; ++cell) {
            state.ic[ic].cells.c_codes[cell] = saved_codes[ic][cell];
        }
    }
}
