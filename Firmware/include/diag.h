// ============================================================================
// diag.h — what the counter exposes to its USB diagnostic console
// ============================================================================
//
// The console (src/diag_console.cpp) reports on state that lives in main.cpp:
// the MCP23017 health records, the per-gate state machines, the totals and the
// poll timing. Like ble_link.h's getTelemetry(), these functions are declared
// here and implemented in main.cpp, so the console never touches main.cpp's
// statics directly and main.cpp never needs to know how anything is printed.
//
// Everything here is a READ of state the normal sampler already produced. None
// of it starts an I2C transaction or lights an emitter: watching a counter must
// not change what it counts.
//
// The view types are pure (no Arduino types) so the formatting helpers in
// diag_format.h can be host-tested against them.
// ============================================================================

#pragma once

#include <stdint.h>

#include "gate_logic.h"

namespace diag {

// How one sensor of one gate looks in the latest snapshot.
enum class SensorView : uint8_t {
    CLEAR,      // beam clear (line HIGH)
    BLOCKED,    // beam blocked / reflected (line LOW)
    BANK_OFF,   // the gate's emitter bank is switched off; not sampled
                // (not "DISABLED": esp32-hal-gpio.h #defines that)
    INVALID,    // its chip is unhealthy or failed its latest read
    STALE,      // no fresh sample: sensing paused (night mode / OTA)
};

struct GateView {
    const char* tag = "";        // "GATE_00"
    uint8_t     bank = 0;        // 1..3
    uint8_t     chip = 0;        // 0..2 = U2..U4
    SensorView  inner_raw = SensorView::STALE;
    SensorView  outer_raw = SensorView::STALE;
    bool        inner_debounced = false;   // true = blocked
    bool        outer_debounced = false;
    gatelogic::GateState state = gatelogic::GateState::IDLE;
};

struct ChipView {
    const char* name = "";       // "U2"
    uint8_t  addr = 0;
    bool     healthy = false;    // answering; its gates are being counted
    bool     last_read_ok = false;
    uint8_t  fail_streak = 0;
    uint16_t last_word = 0;      // last successfully read GPIOA | GPIOB << 8
    uint32_t last_ok_age_ms = UINT32_MAX;   // UINT32_MAX = never read
    uint32_t reads_ok = 0;       // all counters saturating
    uint32_t nacks = 0;          // address or register phase not acknowledged
    uint32_t short_reads = 0;    // fewer than two bytes returned
    uint32_t lost = 0;           // times declared unhealthy
    uint32_t recovered = 0;      // times re-probed back to healthy
    uint32_t max_read_us = 0;    // slowest successful two-byte read
};

struct StatusView {
    uint32_t uptime_ms = 0;
    uint32_t total_in = 0;
    uint32_t total_out = 0;
    uint32_t glitches = 0;
    uint8_t  status_flags = 0;
    uint8_t  bank_mask = 0;
    uint32_t idle_s = 0;         // night-mode seconds remaining
    bool     ota_active = false;
    bool     sensing = true;     // polling now (not idle, not OTA)
    uint32_t poll_interval_ms = 0;
    uint32_t late_after_us = 0;
    // Since boot.
    uint32_t polls = 0;
    uint32_t late = 0;
    uint32_t max_gap_us = 0;
    uint32_t max_poll_us = 0;
    // Current [STAT] window so far (reset every 30 s).
    uint32_t window_polls = 0;
    uint32_t window_late = 0;
    uint32_t window_max_gap_us = 0;
    uint32_t window_max_poll_us = 0;
};

constexpr uint8_t NUM_CHIPS = 3;

// Implemented in main.cpp.
uint8_t numGates();
void    gateView(uint8_t gate, GateView& out);
void    chipView(uint8_t chip, ChipView& out);
void    statusView(StatusView& out);

}  // namespace diag
