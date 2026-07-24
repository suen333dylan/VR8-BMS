#pragma once
#include "BmsState.h"

class LtcController {
  public:
    static void initialize(BmsState& state);
    static bool writeConfiguration(BmsState& state);
    static bool measureCells(BmsState& state);
    static bool measureAux(BmsState& state);
    static void runOpenWireDiagnostic(BmsState& state);

  private:
    static void     configureIc(uint8_t ic_index, BmsState& state);
    static uint16_t normalizeOpenWireChannel(long raw_channel);
};
