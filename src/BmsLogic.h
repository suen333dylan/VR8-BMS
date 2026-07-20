#pragma once
#include "BmsState.h"

class BmsLogic {
public:
    static void setBmsOk(BmsState& state, bool ok, bool delay = true);
    static void clearDischargeRequests(BmsState& state);
    static void recomputeDischargeRequests(BmsState& state);
    static void recomputeBalanceMask(BmsState& state);
    static void resetOpenWireState(BmsState& state);
    static bool hasOpenWireFault(const BmsState& state, uint8_t ic);
    static bool computeFaultState(const BmsState& state);
    static void updateTemperatures(BmsState& state);
    static void printCycleReport(const BmsState& state);
    
private:
    static int16_t temperatureFromAuxCode(uint16_t aux_code, uint16_t vref2_code);
    static void printIcReport(const BmsState& state, uint8_t ic);
    static void printTemperatureValue(int16_t temp_deci_c);
    static void printOpenWireStatus(const BmsState& state, uint8_t ic);
    static void printBalanceCells(const BmsState& state, uint8_t ic);
};
