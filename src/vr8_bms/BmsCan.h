#pragma once
#include "BmsState.h"

class BmsCan {
  public:
    static bool initialize();
    static void sendAllCan(const BmsState& state);

  private:
    static bool sendCanFrame(uint16_t id, const uint8_t* data, uint8_t length);
    static void sendCellFrames(const BmsState& state, uint8_t ic);
    static void sendTemperatureFrame(const BmsState& state, uint8_t ic);
    static void sendStatusFrame(const BmsState& state);
    static void sendBalanceFrame(const BmsState& state, uint8_t ic);
};
