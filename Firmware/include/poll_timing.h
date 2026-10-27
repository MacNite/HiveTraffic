// ============================================================================
// poll_timing.h — is the gate sampler actually running every 5 ms?
// ============================================================================
//
// Pure logic, host-tested in test/test_poll_timing/, like gate_logic.h.
//
// The crossing detector's whole timing budget rests on one assumption: a poll
// every POLL_INTERVAL_MS. A bee dwells ~12 ms in a beam and the debounce needs
// two agreeing samples, so a gap between polls much longer than two intervals
// can swallow a crossing without leaving any trace in the totals or the glitch
// count. Nothing used to measure it. These records do, so that "does the USB
// log, the BLE stack or the diagnostic console stall counting?" is a number
// rather than a guess.
//
// Two figures per poll:
//   * the GAP — start of this poll to start of the previous one. Anything that
//     holds loop() (a blocking print, a long BLE callback on the same core)
//     shows up here;
//   * the DURATION — how long pollAllGates() itself took, which bounds the lit
//     emitter window plus the state-machine work.
//
// Deliberate pauses are not gaps. Night mode and OTA stop polling on purpose;
// the caller calls pause() while they last, so the next poll starts a fresh
// gap measurement instead of reporting a six-hour "stall".
//
// All arithmetic is unsigned on micros(), so its 71-minute rollover needs no
// special case.
// ============================================================================

#pragma once

#include <stdint.h>

namespace polltiming {

struct Figures {
    uint32_t polls = 0;            // polls measured (saturating)
    uint32_t late = 0;             // gaps longer than the late threshold (saturating)
    uint32_t max_gap_us = 0;       // longest gap between poll starts
    uint32_t max_duration_us = 0;  // longest single poll
};

struct State {
    bool     have_last = false;   // false until the first poll, and after pause()
    uint32_t last_start_us = 0;
    Figures  window;              // since the last takeWindow()
    Figures  boot;                // since boot, never reset
};

inline void saturatingInc(uint32_t& v) {
    if (v != UINT32_MAX) v++;
}

inline void fold(Figures& f, bool have_gap, uint32_t gap_us, bool late,
                 uint32_t duration_us) {
    saturatingInc(f.polls);
    if (late) saturatingInc(f.late);
    if (have_gap && gap_us > f.max_gap_us) f.max_gap_us = gap_us;
    if (duration_us > f.max_duration_us) f.max_duration_us = duration_us;
}

// Record one poll that started at `start_us` and took `duration_us`. A gap
// strictly longer than `late_after_us` counts as late.
inline void notePoll(State& s, uint32_t start_us, uint32_t duration_us,
                     uint32_t late_after_us) {
    const bool have_gap = s.have_last;
    const uint32_t gap_us = have_gap ? (uint32_t)(start_us - s.last_start_us) : 0;
    const bool late = have_gap && gap_us > late_after_us;
    fold(s.window, have_gap, gap_us, late, duration_us);
    fold(s.boot, have_gap, gap_us, late, duration_us);
    s.last_start_us = start_us;
    s.have_last = true;
}

// Polling stopped on purpose (night mode, OTA). The next poll measures no gap.
inline void pause(State& s) {
    s.have_last = false;
}

// Return the figures gathered since the previous call and start a new window.
inline Figures takeWindow(State& s) {
    const Figures w = s.window;
    s.window = Figures();
    return w;
}

}  // namespace polltiming
