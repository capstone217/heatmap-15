/*
  ESP32 + DW3000 TWR Anchor with light calibration/noise filtering.

  This sketch is written for Qorvo/Decawave-style DW3000 libraries that expose
  dwt_* APIs. Keep your existing SPI pin/reset/irq setup if your board package
  already has one; the filtering and calibration sections are intentionally
  small and isolated.
*/

#include <Arduino.h>
#include <SPI.h>

extern "C" {
  #include "deca_device_api.h"
  #include "deca_regs.h"
}

#ifndef dwt_setrxdelay
#define dwt_setrxdelay dwt_setrxantennadelay
#endif

#ifndef dwt_settxdelay
#define dwt_settxdelay dwt_settxantennadelay
#endif

// Tune this single value until a real 1.00 m separation prints about 1.00 m.
const uint16_t ANTENNA_DELAY = 16384;

// Reject weak reflected/multi-path measurements. Tune this threshold in one place.
const float RX_POWER_REJECT_DBM = -85.0f;

const uint8_t ANCHOR_ID = 0x01;
const uint8_t MAX_TAGS = 8;
const uint8_t AVG_WINDOW = 5;
const uint32_t RX_RESTART_INTERVAL_MS = 5;
const uint32_t RANGE_SESSION_TIMEOUT_MS = 120;
const uint32_t UUS_TO_DWT_TIME = 65536;
const uint16_t POLL_RX_TO_RESP_TX_DLY_UUS = 900;

// Replace these if your DW3000 board uses different pins.
const int PIN_RST = 27;
const int PIN_IRQ = 34;
const int PIN_SS = 5;

static dwt_config_t uwbConfig = {
  5,                // channel
  DWT_PLEN_128,     // preamble length
  DWT_PAC8,         // PAC size
  9,                // TX preamble code
  9,                // RX preamble code
  1,                // SFD type
  DWT_BR_6M8,       // data rate
  DWT_PHRMODE_STD,  // PHY header mode
  DWT_PHRRATE_STD,  // PHY header rate
  (129 + 8 - 8),    // SFD timeout
  DWT_STS_MODE_OFF,
  DWT_STS_LEN_64,
  DWT_PDOA_M0
};

enum MsgType : uint8_t {
  MSG_POLL = 0xE0,
  MSG_RESP = 0xE1,
  MSG_FINAL = 0xE2
};

struct __attribute__((packed)) TwrFrame {
  uint8_t type;
  uint8_t seq;
  uint8_t anchorId;
  uint8_t tagId;
  uint64_t pollRxTs;
  uint64_t respTxTs;
  uint64_t finalTxTs;
};

struct MovingAverage {
  float values[AVG_WINDOW];
  uint8_t count;
  uint8_t index;
};

struct TagFilter {
  uint8_t tagId;
  bool active;
  MovingAverage avg;
  uint32_t lastSeenMs;
};

struct RangeSession {
  uint8_t tagId;
  bool active;
  uint64_t pollRxTs;
  uint64_t respTxTs;
  uint32_t startedMs;
};

TagFilter tagFilters[MAX_TAGS];
RangeSession sessions[MAX_TAGS];
uint8_t rxBuffer[128];
uint8_t txBuffer[sizeof(TwrFrame)];
uint8_t sequenceNumber = 0;
uint32_t lastRxEnableMs = 0;

static void resetAverage(MovingAverage &avg) {
  avg.count = 0;
  avg.index = 0;
  for (uint8_t i = 0; i < AVG_WINDOW; i++) {
    avg.values[i] = 0.0f;
  }
}

static float pushAverage(MovingAverage &avg, float value) {
  avg.values[avg.index] = value;
  avg.index = (avg.index + 1) % AVG_WINDOW;
  if (avg.count < AVG_WINDOW) {
    avg.count++;
  }

  float sum = 0.0f;
  for (uint8_t i = 0; i < avg.count; i++) {
    sum += avg.values[i];
  }
  return sum / avg.count;
}

static TagFilter *filterForTag(uint8_t tagId) {
  TagFilter *oldest = &tagFilters[0];
  for (uint8_t i = 0; i < MAX_TAGS; i++) {
    if (tagFilters[i].active && tagFilters[i].tagId == tagId) {
      return &tagFilters[i];
    }
    if (!tagFilters[i].active) {
      tagFilters[i].active = true;
      tagFilters[i].tagId = tagId;
      tagFilters[i].lastSeenMs = millis();
      resetAverage(tagFilters[i].avg);
      return &tagFilters[i];
    }
    if (tagFilters[i].lastSeenMs < oldest->lastSeenMs) {
      oldest = &tagFilters[i];
    }
  }

  oldest->active = true;
  oldest->tagId = tagId;
  oldest->lastSeenMs = millis();
  resetAverage(oldest->avg);
  return oldest;
}

static RangeSession *sessionForTag(uint8_t tagId) {
  RangeSession *oldest = &sessions[0];
  for (uint8_t i = 0; i < MAX_TAGS; i++) {
    if (sessions[i].active && sessions[i].tagId == tagId) {
      return &sessions[i];
    }
    if (!sessions[i].active) {
      sessions[i].active = true;
      sessions[i].tagId = tagId;
      sessions[i].startedMs = millis();
      return &sessions[i];
    }
    if (sessions[i].startedMs < oldest->startedMs) {
      oldest = &sessions[i];
    }
  }

  oldest->active = true;
  oldest->tagId = tagId;
  oldest->startedMs = millis();
  return oldest;
}

static uint64_t readRxTimestampU64() {
  uint8_t ts[5];
  uint64_t value = 0;
  dwt_readrxtimestamp(ts);
  for (int i = 4; i >= 0; i--) {
    value = (value << 8) | ts[i];
  }
  return value;
}

static uint64_t readTxTimestampU64() {
  uint8_t ts[5];
  uint64_t value = 0;
  dwt_readtxtimestamp(ts);
  for (int i = 4; i >= 0; i--) {
    value = (value << 8) | ts[i];
  }
  return value;
}

static float ticksToMeters(double ticks) {
  const double DWT_TIME_UNITS = (1.0 / 499.2e6 / 128.0);
  const double SPEED_OF_LIGHT = 299702547.0;
  return (float)(ticks * DWT_TIME_UNITS * SPEED_OF_LIGHT);
}

/*
  Library adaptation point for multi-path filtering.

  Wall-reflected paths usually arrive weaker than the direct path. This function
  must return either total Rx Power or First Path Power in dBm. Different DW3000
  Arduino ports expose diagnostics with different names, so keep all library
  coupling in this one function.

  Common options to wire here:
    - dwt_readdiagnostics(&diag) then diag.rxPower / diag.firstPathPower
    - dwt_readstsquality(...) if your port converts STS quality to dBm
    - your library's getRxPower() / getFirstPathPower() helper
*/
static float readDw3000RxPowerDbm() {
#if defined(DWT_READDIAGNOSTICS_AVAILABLE)
  dwt_rxdiag_t diag;
  dwt_readdiagnostics(&diag);
  return diag.rxPower;
#else
  // Conservative default: accept until this one function is wired to your DW3000 library.
  // Do not change the filtering logic elsewhere.
  return -70.0f;
#endif
}

static bool signalLooksValid(float rxPowerDbm) {
  return rxPowerDbm > RX_POWER_REJECT_DBM;
}

static bool rejectWeakSignal(uint8_t tagId, float rxPowerDbm, const char *phase) {
  if (signalLooksValid(rxPowerDbm)) {
    return false;
  }

  Serial.printf(
    "REJECT tag=0x%02X phase=%s rxPower=%.1f dBm threshold=%.1f dBm\n",
    tagId,
    phase,
    rxPowerDbm,
    RX_POWER_REJECT_DBM
  );
  return true;
}

static void enableReceiver() {
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  lastRxEnableMs = millis();
}

static void sendResponse(const TwrFrame &pollFrame, uint64_t pollRxTs) {
  uint32_t respTxTime = (uint32_t)((pollRxTs + ((uint64_t)POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8);
  uint64_t respTxTs = (((uint64_t)(respTxTime & 0xFFFFFFFEUL)) << 8) + ANTENNA_DELAY;

  TwrFrame resp = {};
  resp.type = MSG_RESP;
  resp.seq = sequenceNumber++;
  resp.anchorId = ANCHOR_ID;
  resp.tagId = pollFrame.tagId;
  resp.pollRxTs = pollRxTs;
  resp.respTxTs = respTxTs;

  memcpy(txBuffer, &resp, sizeof(resp));
  dwt_setdelayedtrxtime(respTxTime);
  dwt_writetxdata(sizeof(resp), txBuffer, 0);
  dwt_writetxfctrl(sizeof(resp), 0, 1);
  if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) == DWT_ERROR) {
    dwt_forcetrxoff();
    enableReceiver();
    return;
  }

  RangeSession *session = sessionForTag(pollFrame.tagId);
  session->pollRxTs = pollRxTs;
  session->respTxTs = respTxTs;
  session->startedMs = millis();
}

static void handlePoll(uint16_t frameLen) {
  if (frameLen < sizeof(TwrFrame)) {
    enableReceiver();
    return;
  }

  TwrFrame poll = {};
  memcpy(&poll, rxBuffer, sizeof(poll));
  if (poll.type != MSG_POLL || poll.anchorId != ANCHOR_ID) {
    enableReceiver();
    return;
  }

  float rxPowerDbm = readDw3000RxPowerDbm();
  if (rejectWeakSignal(poll.tagId, rxPowerDbm, "poll")) {
    enableReceiver();
    return;
  }

  uint64_t pollRxTs = readRxTimestampU64();
  sendResponse(poll, pollRxTs);
}

static void handleFinal(uint16_t frameLen) {
  if (frameLen < sizeof(TwrFrame)) {
    enableReceiver();
    return;
  }

  TwrFrame finalFrame = {};
  memcpy(&finalFrame, rxBuffer, sizeof(finalFrame));
  if (finalFrame.type != MSG_FINAL || finalFrame.anchorId != ANCHOR_ID) {
    enableReceiver();
    return;
  }

  float rxPowerDbm = readDw3000RxPowerDbm();
  if (rejectWeakSignal(finalFrame.tagId, rxPowerDbm, "final")) {
    enableReceiver();
    return;
  }

  uint64_t finalRxTs = readRxTimestampU64();
  RangeSession *session = sessionForTag(finalFrame.tagId);
  if (!session->active || millis() - session->startedMs > RANGE_SESSION_TIMEOUT_MS) {
    Serial.printf("REJECT tag=0x%02X staleSession\n", finalFrame.tagId);
    enableReceiver();
    return;
  }

  // Symmetric double-sided TWR:
  // Ra = respRx - pollTx, Rb = finalRx - respTx, Da = finalTx - respRx, Db = respTx - pollRx.
  // In FINAL: pollRxTs field carries tag pollTxTs, respTxTs field carries tag respRxTs.
  double roundA = (double)(finalFrame.respTxTs - finalFrame.pollRxTs);
  double roundB = (double)(finalRxTs - session->respTxTs);
  double replyA = (double)(session->respTxTs - session->pollRxTs);
  double replyB = (double)(finalFrame.finalTxTs - finalFrame.respTxTs);
  double denominator = roundA + roundB + replyA + replyB;
  if (denominator == 0.0) {
    Serial.printf("REJECT tag=0x%02X zeroDenominator\n", finalFrame.tagId);
    enableReceiver();
    return;
  }
  double tofTicks = ((roundA * roundB) - (replyA * replyB)) / denominator;

  float distanceM = ticksToMeters(tofTicks);
  if (!isfinite(distanceM) || distanceM <= 0.0f || distanceM > 50.0f) {
    Serial.printf("REJECT tag=0x%02X badDistance=%.2f\n", finalFrame.tagId, distanceM);
    enableReceiver();
    return;
  }

  TagFilter *filter = filterForTag(finalFrame.tagId);
  filter->lastSeenMs = millis();
  float filteredM = pushAverage(filter->avg, distanceM);

  Serial.printf(
    "ANCHOR 0x%02X : %.2f m | tag=0x%02X raw=%.2f avgN=%u rxPower=%.1f dBm\n",
    ANCHOR_ID,
    filteredM,
    finalFrame.tagId,
    distanceM,
    filter->avg.count,
    rxPowerDbm
  );

  session->active = false;
  enableReceiver();
}

static void handleReceivedFrame() {
  uint32_t frameLen = dwt_read32bitreg(RX_FINFO_ID) & RXFLEN_MASK;
  if (frameLen > sizeof(rxBuffer)) {
    dwt_forcetrxoff();
    enableReceiver();
    return;
  }

  dwt_readrxdata(rxBuffer, frameLen, 0);
  TwrFrame header = {};
  if (frameLen >= 1) {
    memcpy(&header, rxBuffer, min((size_t)frameLen, sizeof(header)));
  }

  if (header.type == MSG_POLL) {
    handlePoll(frameLen);
  } else if (header.type == MSG_FINAL) {
    handleFinal(frameLen);
  } else {
    enableReceiver();
  }
}

static void initDw3000() {
  pinMode(PIN_RST, OUTPUT);
  digitalWrite(PIN_RST, LOW);
  delayMicroseconds(200);
  digitalWrite(PIN_RST, HIGH);

  SPI.begin();

  if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
    Serial.println("DW3000 init failed");
    while (true) {
      yield();
    }
  }

  dwt_configure(&uwbConfig);

  dwt_setrxdelay(ANTENNA_DELAY);
  dwt_settxdelay(ANTENNA_DELAY);

  dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);
  dwt_setrxtimeout(RANGE_SESSION_TIMEOUT_MS);
  enableReceiver();
}

void setup() {
  Serial.begin(115200);
  initDw3000();
  Serial.printf(
    "DW3000 Anchor 0x%02X ready, antennaDelay=%u, rejectBelow=%.1f dBm\n",
    ANCHOR_ID,
    ANTENNA_DELAY,
    RX_POWER_REJECT_DBM
  );
}

void loop() {
  uint32_t status = dwt_read32bitreg(SYS_STATUS_ID);

  if (status & SYS_STATUS_RXFCG_BIT_MASK) {
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK);
    handleReceivedFrame();
    return;
  }

  if (status & (SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK | SYS_STATUS_RXPHE_BIT_MASK |
                SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK)) {
    dwt_write32bitreg(
      SYS_STATUS_ID,
      SYS_STATUS_RXFTO_BIT_MASK | SYS_STATUS_RXPTO_BIT_MASK | SYS_STATUS_RXPHE_BIT_MASK |
      SYS_STATUS_RXFCE_BIT_MASK | SYS_STATUS_RXFSL_BIT_MASK
    );
    dwt_forcetrxoff();
    enableReceiver();
    return;
  }

  if (millis() - lastRxEnableMs >= RX_RESTART_INTERVAL_MS && status == 0) {
    enableReceiver();
  }
}
