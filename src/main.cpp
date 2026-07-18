#include <Arduino.h>
#include <SPI.h>
#include <math.h>

#include "HardwareSerial.h"
#include "LTC6813.h"
#include "LTC681x.h"
#include "mcp_can.h"

namespace {

const uint8_t TOTAL_IC = 6;
const uint8_t ACTIVE_CELLS_PER_IC = 17;
const uint8_t NTC_PER_IC = 4;

const uint8_t BMS_OK_PIN = 5;
const uint8_t BEEP_PIN = 4;
const uint8_t DISCHARGE_EN_PIN = 7;
const uint8_t CAN_CS_PIN = 9;

const uint8_t ADC_CONVERSION_MODE = MD_7KHZ_3KHZ;
const uint8_t ADC_DCP = DCP_DISABLED;

const uint16_t CELL_OK_MIN_CODE = 33000;
const uint16_t CELL_OK_MAX_CODE = 43500;
// const uint16_t CELL_OK_MAX_CODE = 39000;
const uint16_t BALANCE_DELTA_CODE = 260;
const int16_t TEMP_MAX_DECI_C = 550;
const int16_t TEMP_MIN_DECI_C = 50;
const uint32_t REPORT_DELAY_MS = 1000;

const uint8_t BALANCE_GROUP = 1;
const uint8_t BALANCE_CELLS_PER_GROUP = 1;

const uint32_t POLL_INTERVAL_MS = 200;
const uint32_t BALANCE_INTERVAL_MS = 500;
const uint32_t CAN_TX_INTERVAL_MS = 1000;
const uint32_t OPEN_WIRE_INTERVAL_MS = 5000;

const uint8_t TOTAL_CELL_CHANNELS = 18;
const uint16_t OPEN_WIRE_NONE = 0xFFFF;
const bool OPEN_WIRE_ENABLED = true;
const bool OPEN_WIRE_BLOCKS_BMS_OK = false;

const float CODE_TO_VOLT = 0.0001f;
const float NTC_PULLUP_OHMS = 49900.0f;
const float NTC_R0_OHMS = 47000.0f;
const float NTC_BETA = 4050.0f;
const float NTC_T0_KELVIN = 298.15f;

const bool REFON = true;
const bool ADCOPT = false;
bool GPIOBITS_A[5] = {true, true, false, false, false};
bool GPIOBITS_B[4] = {true, true, false, false};
bool DCTOBITS[4] = {false, false, false, false};
const bool FDRF = false;
const bool DTMEN = true;
bool PSBITS[2] = {false, false};

const uint16_t CAN_ID_CELL_BASE = 0x1100;
const uint16_t CAN_ID_TEMP_BASE = 0x1200; // 0x201 is used by the Motor Controller
const uint16_t CAN_ID_STATUS_BASE = 0x1300;

// AUX order is GPIO 1, GPIO 2, GPIO 3, GPIO 4, GPIO 5, Vref2, GPIO 6, GPIO 7,
// GPIO 8, GPIO 9
const uint8_t NTC_AUX_INDEX[NTC_PER_IC] = {0, 1, 6, 7};

cell_asic BMS_IC[TOTAL_IC];
uint32_t discharge_mask[TOTAL_IC];
uint32_t balance_mask[TOTAL_IC];
int16_t ntc_temperature_deci_c[TOTAL_IC][NTC_PER_IC];
uint16_t open_wire_channel[TOTAL_IC];
bool open_wire_valid[TOTAL_IC];
MCP_CAN can_bus(CAN_CS_PIN);

bool can_ready = false;
bool bms_ok = false;
bool discharge_enabled = false;
uint32_t last_poll_ms = 0;
uint32_t last_balance_ms = 0;
uint32_t last_can_tx_ms = 0;
uint32_t last_open_wire_ms = 0;
uint32_t first_error_state_ms = 0;
uint32_t cycle_count = 0;

void setBmsOk(bool ok, bool delay = true) {
  digitalWrite(BEEP_PIN, ok ? LOW : HIGH);
  
  uint32_t now = millis();
  if (!ok) {
    if (first_error_state_ms == 0) {
      first_error_state_ms = now;
    }

    if (delay) {
      digitalWrite(BMS_OK_PIN, (now - first_error_state_ms) >= REPORT_DELAY_MS ? LOW : HIGH);
    } else {
      digitalWrite(BMS_OK_PIN, LOW);
    }
  } else {
    digitalWrite(BMS_OK_PIN, HIGH);
    first_error_state_ms = 0;
  }
}

void clearDischargeRequests() {
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    discharge_mask[ic] = 0;
  }
}

bool isDischargeRequested(uint8_t ic, uint8_t cell) {
  return (discharge_mask[ic] >> cell) & 1;
}

void configureIc(uint8_t ic_index) {
  bool dcc_a[12] = {false};
  bool dcc_b[7] = {false};

  for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
    if (!(isDischargeRequested(ic_index, cell) &&
          (balance_mask[ic_index] >> cell) & 1)) {
      continue;
    }

    if (cell < 12) {
      dcc_a[cell] = true;
      continue;
    }

    dcc_b[(cell - 12) + 1] = true;
  }

  LTC6813_set_cfgr(ic_index, BMS_IC, REFON, ADCOPT, GPIOBITS_A, dcc_a, DCTOBITS,
                   CELL_OK_MIN_CODE, CELL_OK_MAX_CODE);
  LTC6813_set_cfgrb(ic_index, BMS_IC, FDRF, DTMEN, PSBITS, GPIOBITS_B, dcc_b);
}

bool writeConfiguration() {
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    configureIc(ic);
  }

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  wakeup_sleep(TOTAL_IC);
  LTC6813_wrcfg(TOTAL_IC, BMS_IC);
  LTC6813_wrcfgb(TOTAL_IC, BMS_IC);
  SPI.endTransaction();
  return true;
}

bool initializeCan() {
  int result = can_bus.begin(MCP_ANY, CAN_500KBPS, MCP_16MHZ);
  if (result == CAN_OK) {
    can_bus.setMode(MCP_NORMAL);
    return true;
  }

  Serial.print(F("Error initializing CAN bus with error code:"));
  Serial.println(result);
  return false;
}

bool measureCells() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  wakeup_sleep(TOTAL_IC);
  LTC6813_adcv(ADC_CONVERSION_MODE, ADC_DCP, CELL_CH_ALL);
  LTC6813_pollAdc();
  wakeup_idle(TOTAL_IC);
  bool result = LTC6813_rdcv(REG_ALL, TOTAL_IC, BMS_IC) == 0;
  SPI.endTransaction();
  return result;
}

bool measureAux() {
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  wakeup_sleep(TOTAL_IC);
  LTC6813_adax(ADC_CONVERSION_MODE, AUX_CH_ALL);
  LTC6813_pollAdc();
  wakeup_idle(TOTAL_IC);
  bool result = LTC6813_rdaux(REG_ALL, TOTAL_IC, BMS_IC) == 0;
  SPI.endTransaction();
  return result;
}

int16_t temperatureFromAuxCode(uint16_t aux_code, uint16_t vref2_code) {
  if (aux_code == 0 || vref2_code == 0 || aux_code >= vref2_code) {
    return INT16_MAX;
  }

  const float aux_voltage = aux_code * CODE_TO_VOLT;
  const float reference_voltage = vref2_code * CODE_TO_VOLT;
  const float denominator = reference_voltage - aux_voltage;
  if (denominator <= 0.0f) {
    return INT16_MAX;
  }

  const float resistance = NTC_PULLUP_OHMS * aux_voltage / denominator;
  if (resistance <= 0.0f) {
    return INT16_MAX;
  }

  const float inverse_kelvin =
      (1.0f / NTC_T0_KELVIN) + (logf(resistance / NTC_R0_OHMS) / NTC_BETA);
  if (inverse_kelvin <= 0.0f) {
    return INT16_MAX;
  }

  const float temp_c = (1.0f / inverse_kelvin) - 273.15f;
  if (isnan(temp_c) || isinf(temp_c)) {
    return INT16_MAX;
  }

  const float scaled = temp_c * 10.0f;
  return static_cast<int16_t>(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

void updateTemperatures() {
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    const uint16_t vref2_code = BMS_IC[ic].aux.a_codes[5];
    for (uint8_t ntc = 0; ntc < NTC_PER_IC; ++ntc) {
      ntc_temperature_deci_c[ic][ntc] = temperatureFromAuxCode(
          BMS_IC[ic].aux.a_codes[NTC_AUX_INDEX[ntc]], vref2_code);
    }
  }
}

void resetOpenWireState() {
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    open_wire_channel[ic] = OPEN_WIRE_NONE;
    open_wire_valid[ic] = false;
  }

  last_open_wire_ms = 0;
}

uint16_t normalizeOpenWireChannel(long raw_channel) {
  if (raw_channel < 0) {
    return OPEN_WIRE_NONE;
  }

  const uint16_t channel = static_cast<uint16_t>(raw_channel);
  if (channel == OPEN_WIRE_NONE) {
    return OPEN_WIRE_NONE;
  }

  if (channel > ACTIVE_CELLS_PER_IC) {
    return OPEN_WIRE_NONE;
  }

  return channel;
}

bool hasOpenWireFault(uint8_t ic) {
  return open_wire_valid[ic] && open_wire_channel[ic] != OPEN_WIRE_NONE;
}

void runOpenWireDiagnostic() {
  uint16_t saved_codes[TOTAL_IC][TOTAL_CELL_CHANNELS];
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    for (uint8_t cell = 0; cell < TOTAL_CELL_CHANNELS; ++cell) {
      saved_codes[ic][cell] = BMS_IC[ic].cells.c_codes[cell];
    }
  }

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  LTC6813_run_openwire_single(TOTAL_IC, BMS_IC);
  SPI.endTransaction();

  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    open_wire_channel[ic] =
        normalizeOpenWireChannel(BMS_IC[ic].system_open_wire);
    open_wire_valid[ic] = true;

    for (uint8_t cell = 0; cell < TOTAL_CELL_CHANNELS; ++cell) {
      BMS_IC[ic].cells.c_codes[cell] = saved_codes[ic][cell];
    }
  }
}

bool computeFaultState() {
  bool ok = true;

  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
      const uint16_t code = BMS_IC[ic].cells.c_codes[cell];
      if (code < CELL_OK_MIN_CODE || code > CELL_OK_MAX_CODE) {
        ok = false;
      }
    }

    for (uint8_t ntc = 0; ntc < NTC_PER_IC; ++ntc) {
      const int16_t temp_deci_c = ntc_temperature_deci_c[ic][ntc];
      if (temp_deci_c == INT16_MAX || temp_deci_c > TEMP_MAX_DECI_C ||
          temp_deci_c < TEMP_MIN_DECI_C) {
        ok = false;
      }
    }

    if (OPEN_WIRE_BLOCKS_BMS_OK && hasOpenWireFault(ic)) {
      ok = false;
    }
  }

  return ok;
}

void recomputeDischargeRequests() {
  clearDischargeRequests();

  uint16_t min_code = 0xFFFF;
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
      const uint16_t code = BMS_IC[ic].cells.c_codes[cell];
      if (code > 0 && code < min_code) {
        min_code = code;
      }
    }
  }

  if (min_code == 0xFFFF) {
    return;
  }

  const uint16_t balance_threshold = min_code + BALANCE_DELTA_CODE;
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
      if (BMS_IC[ic].cells.c_codes[cell] >= balance_threshold) {
        discharge_mask[ic] |= (1UL << cell);
      }
    }
  }
}

void recomputeBalanceMask() {
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    balance_mask[ic] = 0;
    for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
      if ((cycle_count + cell) % BALANCE_GROUP < BALANCE_CELLS_PER_GROUP) {
        balance_mask[ic] |= (1UL << cell);
      }
    }
  }
  cycle_count++;
}

bool sendCanFrame(uint16_t id, const uint8_t *data, uint8_t length) {
  if (!can_ready) {
    return false;
  }

  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (can_bus.sendMsgBuf(id, 1, length, const_cast<uint8_t *>(data)) ==
        CAN_OK) {
      return true;
    }
    delay(2);
  }

  return false;
}

void printTemperatureValue(int16_t temp_deci_c) {
  if (temp_deci_c == INT16_MAX) {
    Serial.print(F("ERR"));
    return;
  }

  Serial.print(temp_deci_c / 10);
  Serial.print('.');
  Serial.print(abs(temp_deci_c % 10));
  Serial.print(F("C"));
}

void printOpenWireStatus(uint8_t ic) {
  if (!OPEN_WIRE_ENABLED) {
    Serial.print(F("disabled"));
    return;
  }

  if (!open_wire_valid[ic]) {
    Serial.print(F("pending"));
    return;
  }

  if (!hasOpenWireFault(ic)) {
    Serial.print(F("none"));
    return;
  }

  Serial.print(F("C"));
  Serial.print(open_wire_channel[ic]);
}

void printBalanceCells(uint8_t ic) {
  bool first = true;
  for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
    if (!isDischargeRequested(ic, cell)) {
      continue;
    }

    if (!first) {
      Serial.print(',');
    }
    Serial.print(cell + 1);
    first = false;
  }

  if (first) {
    Serial.print(F("none"));
  }
}

void printIcReport(uint8_t ic) {
  uint16_t min_code = 0xFFFF;
  uint16_t max_code = 0;
  uint8_t min_cell = 0;
  uint8_t max_cell = 0;

  for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
    const uint16_t code = BMS_IC[ic].cells.c_codes[cell];
    if (code < min_code) {
      min_code = code;
      min_cell = cell;
    }
    if (code > max_code) {
      max_code = code;
      max_cell = cell;
    }
  }

  Serial.print(F("IC"));
  Serial.print(ic + 1);
  Serial.print(F(" Cells: "));
  for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
    if (cell > 0) {
      Serial.print(F(", "));
    }
    Serial.print(F("C"));
    Serial.print(cell + 1);
    Serial.print('=');
    Serial.print(BMS_IC[ic].cells.c_codes[cell] * CODE_TO_VOLT, 4);
    Serial.print(F("V"));
  }
  Serial.println();

  Serial.print(F("IC"));
  Serial.print(ic + 1);
  Serial.print(F(" Temps: "));
  for (uint8_t ntc = 0; ntc < NTC_PER_IC; ++ntc) {
    if (ntc > 0) {
      Serial.print(F(", "));
    }
    Serial.print(F("T"));
    Serial.print(ntc + 1);
    Serial.print('=');
    printTemperatureValue(ntc_temperature_deci_c[ic][ntc]);
  }
  Serial.println();

  Serial.print(F("IC"));
  Serial.print(ic + 1);
  Serial.print(F(" Range: min=C"));
  Serial.print(min_cell + 1);
  Serial.print('(');
  Serial.print(min_code * CODE_TO_VOLT, 4);
  Serial.print(F("V), max=C"));
  Serial.print(max_cell + 1);
  Serial.print('(');
  Serial.print(max_code * CODE_TO_VOLT, 4);
  Serial.print(F("V), balance="));
  printBalanceCells(ic);
  Serial.print(F(", open-wire="));
  printOpenWireStatus(ic);
  Serial.println();
}

void printCycleReport(bool bms_ok, bool discharge_set) {
  Serial.println();
  Serial.print(F("=== BMS Time "));
  Serial.print(millis());
  Serial.println(F(" ms ==="));

  Serial.print(F("BMS_OK="));
  Serial.print(bms_ok ? F("HIGH") : F("LOW"));
  Serial.print(F(", CAN="));
  Serial.print(can_ready ? F("OK") : F("FAIL"));
  Serial.print(F(", Discharge="));
  Serial.println(discharge_set ? F("ENABLED") : F("DISABLED"));

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  wakeup_idle(TOTAL_IC);
  SPI.endTransaction();

  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    printIcReport(ic);
  }
}

void sendCellFrames(uint8_t ic) {
  for (uint8_t frame = 0; frame < 5; ++frame) {
    uint8_t payload[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    for (uint8_t offset = 0; offset < 4; ++offset) {
      const uint8_t cell = (frame * 4) + offset;
      if (cell >= ACTIVE_CELLS_PER_IC) {
        break;
      }

      const uint16_t code = BMS_IC[ic].cells.c_codes[cell];
      payload[offset * 2] = static_cast<uint8_t>(code & 0xFF);
      payload[(offset * 2) + 1] = static_cast<uint8_t>(code >> 8);
    }

    sendCanFrame(CAN_ID_CELL_BASE + (ic * 0x10U) + frame, payload,
                 sizeof(payload));
  }
}

void sendTemperatureFrame(uint8_t ic) {
  uint8_t payload[8];
  for (uint8_t ntc = 0; ntc < NTC_PER_IC; ++ntc) {
    const int16_t temp = ntc_temperature_deci_c[ic][ntc];
    payload[ntc * 2] = static_cast<uint8_t>(temp & 0xFF);
    payload[(ntc * 2) + 1] =
        static_cast<uint8_t>((static_cast<uint16_t>(temp) >> 8) & 0xFF);
  }
  sendCanFrame(CAN_ID_TEMP_BASE + ic, payload, sizeof(payload));
}

void sendStatusFrame(uint8_t ic, bool bms_ok) {
  uint16_t min_code = 0xFFFF;
  uint16_t max_code = 0;
  uint32_t balance_mask = discharge_mask[ic] & 0x1FFFFUL;

  for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
    const uint16_t code = BMS_IC[ic].cells.c_codes[cell];
    if (code < min_code) {
      min_code = code;
    }
    if (code > max_code) {
      max_code = code;
    }
  }

  uint8_t fault_bits = 0;
  if (!bms_ok) {
    fault_bits |= 0x01;
  }

  for (uint8_t ntc = 0; ntc < NTC_PER_IC; ++ntc) {
    const int16_t temp = ntc_temperature_deci_c[ic][ntc];
    if (temp == INT16_MAX || temp < TEMP_MIN_DECI_C) {
      fault_bits |= 0x02;
    }
    if (temp > TEMP_MAX_DECI_C) {
      fault_bits |= 0x04;
    }
  }

  for (uint8_t cell = 0; cell < ACTIVE_CELLS_PER_IC; ++cell) {
    const uint16_t code = BMS_IC[ic].cells.c_codes[cell];
    if (code < CELL_OK_MIN_CODE || code > CELL_OK_MAX_CODE) {
      fault_bits |= 0x08;
      break;
    }
  }

  if (hasOpenWireFault(ic)) {
    fault_bits |= 0x10;
  }

  uint8_t payload[8];
  payload[0] = fault_bits;
  payload[1] = static_cast<uint8_t>(balance_mask & 0xFF);
  payload[2] = static_cast<uint8_t>((balance_mask >> 8) & 0xFF);
  payload[3] = static_cast<uint8_t>((balance_mask >> 16) & 0x01);
  payload[4] = static_cast<uint8_t>(min_code & 0xFF);
  payload[5] = static_cast<uint8_t>(min_code >> 8);
  payload[6] = static_cast<uint8_t>(max_code & 0xFF);
  payload[7] = static_cast<uint8_t>(max_code >> 8);

  sendCanFrame(CAN_ID_STATUS_BASE + ic, payload, sizeof(payload));
}

void sendAllCan(bool bms_ok) {
  for (uint8_t ic = 0; ic < TOTAL_IC; ++ic) {
    sendCellFrames(ic);
    sendTemperatureFrame(ic);
    sendStatusFrame(ic, bms_ok);
  }
}

void initializeBms() {
  LTC6813_init_cfg(TOTAL_IC, BMS_IC);
  LTC6813_init_cfgb(TOTAL_IC, BMS_IC);
  LTC6813_reset_crc_count(TOTAL_IC, BMS_IC);
  LTC6813_init_reg_limits(TOTAL_IC, BMS_IC);
  resetOpenWireState();
  clearDischargeRequests();
  recomputeBalanceMask();
  writeConfiguration();
}

void printStartupSummary() {
  Serial.println(F("VR8-BMS Startup Summary"));
  Serial.print(F("Total ICs: "));
  Serial.println(TOTAL_IC);
  Serial.print(F("Active Cells per IC: "));
  Serial.println(ACTIVE_CELLS_PER_IC);
  Serial.print(F("Physical Cell Channels per IC: "));
  Serial.println(TOTAL_CELL_CHANNELS);
  Serial.print(F("NTC per IC: "));
  Serial.println(NTC_PER_IC);
  Serial.print(F("Cell OK Min Code: "));
  Serial.println(CELL_OK_MIN_CODE);
  Serial.print(F("Cell OK Max Code: "));
  Serial.println(CELL_OK_MAX_CODE);
  Serial.print(F("Open Wire Check: "));
  Serial.println(OPEN_WIRE_ENABLED ? F("ENABLED") : F("DISABLED"));
  Serial.print(F("Open Wire Blocks BMS_OK: "));
  Serial.println(OPEN_WIRE_BLOCKS_BMS_OK ? F("YES") : F("NO"));
  Serial.print(F("Open Wire Interval ms: "));
  Serial.println(OPEN_WIRE_INTERVAL_MS);
}

bool runPollingCycle(uint32_t now) {
  const bool cells_ok = measureCells();
  const bool aux_ok = measureAux();
  if (!cells_ok || !aux_ok) {
    clearDischargeRequests();
    writeConfiguration();
    setBmsOk(false);
    Serial.print(F("Polling failed: cells="));
    Serial.print(cells_ok ? F("OK") : F("ERR"));
    Serial.print(F(", aux="));
    Serial.println(aux_ok ? F("OK") : F("ERR"));
    return false;
  }

  updateTemperatures();
  recomputeDischargeRequests();
  return true;
}

} // namespace

void setup() {
  Serial.begin(115200);
  Serial.println(F("Initializing VR8-BMS..."));

  pinMode(BMS_OK_PIN, OUTPUT);
  setBmsOk(false);

  // 2. 關閉 LTC6820 (isoSPI) 的通訊 (Linduino 預設對應 Arduino 的 Pin 10)
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  pinMode(CAN_CS_PIN, OUTPUT);
  digitalWrite(CAN_CS_PIN, HIGH);

  pinMode(BEEP_PIN, OUTPUT);
  digitalWrite(BEEP_PIN, LOW);

  pinMode(DISCHARGE_EN_PIN, INPUT_PULLUP);

  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));

  initializeBms();
  SPI.endTransaction();
  can_ready = initializeCan();
  printStartupSummary();

  if (!can_ready) {
    Serial.println(F("MCP2515 init failed"));
  }
}

void loop() {
  const uint32_t now = millis();

  // Perform a polling cycle to read cell voltages and temperatures, and update
  if (last_poll_ms == 0 || now - last_poll_ms >= POLL_INTERVAL_MS) {
    last_poll_ms = now;
    const bool cycle_ok = runPollingCycle(now);
    if (!cycle_ok) {
      Serial.println(F("BMS polling error"));
    }
  }

  // Periodically run the open wire diagnostic
  if (OPEN_WIRE_ENABLED && (last_open_wire_ms == 0 ||
                            now - last_open_wire_ms >= OPEN_WIRE_INTERVAL_MS)) {
    last_open_wire_ms = now;
    clearDischargeRequests();
    writeConfiguration();
    runOpenWireDiagnostic();
  }

  // Compute the overall BMS fault state and determine if discharge should be
  // enabled.
  bms_ok = computeFaultState();
  setBmsOk(bms_ok);
  discharge_enabled = digitalRead(DISCHARGE_EN_PIN) == LOW;

  // If the BMS is OK and discharge is enabled, compute which cells need to be
  // discharged
  if (bms_ok && discharge_enabled) {
    if (last_balance_ms == 0 || now - last_balance_ms >= BALANCE_INTERVAL_MS) {
      last_balance_ms = now;
      recomputeBalanceMask();
      writeConfiguration();
    }
  } else {
    // If we're not OK to discharge, ensure all discharge requests are cleared.
    clearDischargeRequests();
    writeConfiguration();
  }

  if (can_ready && (!bms_ok || last_can_tx_ms == 0 || now - last_can_tx_ms >= CAN_TX_INTERVAL_MS)) {
    last_can_tx_ms = now;
    sendAllCan(bms_ok);
    printCycleReport(bms_ok, discharge_enabled);
  }
}
