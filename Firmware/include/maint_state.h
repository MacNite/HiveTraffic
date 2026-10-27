// ============================================================================
// maint_state.h — explicit maintenance mode for bench and field testing
// ============================================================================
//
// Pure logic, host-tested in test/test_maint_state/, like idle_state.h.
//
// The USB console's passive commands cannot change what the counter does. The
// maintenance commands can: force the emitters on or off, switch banks, scan
// the I2C bus, run register and dark/lit tests. Every one of those would
// corrupt the lifetime totals, or stall the sampler, if it could run during
// normal counting — so they only work inside an explicit maintenance session,
// and this header is the bookkeeping for one.
//
// While a session is active:
//
//   * crossings go into the session's TEST counters, never the lifetime totals
//     (and never raise the latched sensor-fault flag: a target held over a
//     sensor for half a minute is a test, not a fault);
//   * HiveHub's night-mode and bank requests are DEFERRED, not applied, and
//     replayed when the session ends — so a relay that arrives mid-test does
//     not dark the bench, and the counter still ends up where HiveHub wants it;
//   * the session's length is added to a lifetime maintenance-seconds figure
//     that is reported over BLE as "maint_s", so HiveHub can tell an interval
//     whose flat totals were a test from one where no bees flew.
//
// Like night mode, everything here fails open. A session ends by itself after
// TIMEOUT_MS without a console command, ends when an OTA starts, and is never
// persisted: a reset or a forgotten laptop always ends with the counter
// counting on its normal settings.
//
// All time comparisons are unsigned, so millis() rollover needs no special case.
// ============================================================================

#pragma once

#include <stdint.h>

namespace maintstate {

// A session with no console command for this long ends by itself. Long enough
// to work through all 24 gates by hand without the session expiring under the
// person doing it; short enough that a laptop left attached costs one upload
// interval or three, not a day of data.
constexpr uint32_t TIMEOUT_MS = 30UL * 60UL * 1000UL;

struct State {
    bool     active = false;
    uint32_t entered_ms = 0;
    uint32_t last_activity_ms = 0;

    // Lifetime seconds spent in finished sessions (saturating). The current
    // session is added by totalSeconds().
    uint32_t completed_s = 0;

    // Crossings seen during the current session, kept apart from the totals.
    uint32_t test_in = 0;
    uint32_t test_out = 0;
    uint32_t test_glitch = 0;

    // Mask in force when the session started, restored when it ends unless
    // HiveHub asked for another one in the meantime.
    uint8_t  banks_before = 0;

    // Requests that arrived (from HiveHub, or were in force at entry) while
    // the session ran, replayed when it ends.
    bool     idle_pending = false;
    uint32_t idle_requested_ms = 0;
    uint32_t idle_duration_s = 0;
    bool     banks_pending = false;
    uint8_t  banks_mask = 0;
};

inline void saturatingAdd(uint32_t& v, uint32_t add) {
    v = (UINT32_MAX - v < add) ? UINT32_MAX : v + add;
}

inline void saturatingInc(uint32_t& v) {
    if (v != UINT32_MAX) v++;
}

// Whole seconds in `ms` milliseconds, rounded UP: a session of any length must
// move "maint_s", or a short test between two HiveHub reads would leave no
// trace.
inline uint32_t ceilSeconds(uint32_t ms) {
    return ms / 1000 + (ms % 1000 ? 1 : 0);
}

// A session counts for at least one second, however short.
inline uint32_t sessionSeconds(uint32_t ms) {
    const uint32_t s = ceilSeconds(ms);
    return s ? s : 1;
}

// Start a session. `current_mask` is the bank mask in force; `idle_left_s` is
// the night-mode suspension still to run (0 if sensing), which the session
// takes over and replays on exit. Returns false if one is already running.
inline bool enter(State& s, uint32_t now_ms, uint8_t current_mask,
                  uint32_t idle_left_s) {
    if (s.active) return false;
    s.active = true;
    s.entered_ms = now_ms;
    s.last_activity_ms = now_ms;
    s.test_in = 0;
    s.test_out = 0;
    s.test_glitch = 0;
    s.banks_before = current_mask;
    s.banks_pending = false;
    s.banks_mask = 0;
    s.idle_pending = idle_left_s > 0;
    s.idle_requested_ms = now_ms;
    s.idle_duration_s = idle_left_s;
    return true;
}

// A console command arrived: the session is in use.
inline void touch(State& s, uint32_t now_ms) {
    if (s.active) s.last_activity_ms = now_ms;
}

inline bool timedOut(const State& s, uint32_t now_ms) {
    return s.active && now_ms - s.last_activity_ms >= TIMEOUT_MS;
}

// Seconds of inactivity left before the session ends by itself.
inline uint32_t timeoutRemainingSeconds(const State& s, uint32_t now_ms) {
    if (!s.active) return 0;
    const uint32_t idle = now_ms - s.last_activity_ms;
    return idle >= TIMEOUT_MS ? 0 : ceilSeconds(TIMEOUT_MS - idle);
}

// Lifetime maintenance seconds, including the running session.
inline uint32_t totalSeconds(const State& s, uint32_t now_ms) {
    uint32_t total = s.completed_s;
    if (s.active) saturatingAdd(total, sessionSeconds(now_ms - s.entered_ms));
    return total;
}

// HiveHub asked for a night-mode suspension of `duration_s` during the
// session. The latest request wins, as it would outside maintenance; 0 is a
// request to resume, which simply cancels any pending suspension.
inline void deferIdle(State& s, uint32_t now_ms, uint32_t duration_s) {
    s.idle_pending = duration_s > 0;
    s.idle_requested_ms = now_ms;
    s.idle_duration_s = duration_s;
}

// HiveHub asked for bank mask `mask` (already validated) during the session.
inline void deferBanks(State& s, uint8_t mask) {
    s.banks_pending = true;
    s.banks_mask = mask;
}

// What the counter must apply as the session ends.
struct Restore {
    uint8_t  bank_mask = 0;
    uint32_t idle_s = 0;      // 0: no suspension to resume
};

// End the session and say what to restore. The suspension resumes with only
// the time it has left — a request does not get longer by waiting.
inline Restore exit(State& s, uint32_t now_ms) {
    Restore r;
    if (!s.active) return r;
    r.bank_mask = s.banks_pending ? s.banks_mask : s.banks_before;
    if (s.idle_pending) {
        const uint32_t elapsed_ms = now_ms - s.idle_requested_ms;
        const uint32_t elapsed_s = elapsed_ms / 1000;
        if (elapsed_s < s.idle_duration_s) r.idle_s = s.idle_duration_s - elapsed_s;
    }
    saturatingAdd(s.completed_s, sessionSeconds(now_ms - s.entered_ms));
    s.active = false;
    s.idle_pending = false;
    s.banks_pending = false;
    return r;
}

}  // namespace maintstate
