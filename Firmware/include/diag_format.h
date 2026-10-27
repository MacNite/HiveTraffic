// ============================================================================
// diag_format.h — text renderings used by the USB diagnostic console
// ============================================================================
//
// Pure logic, host-tested in test/test_diag_format/. Keeping the wording here
// rather than inline in diag_console.cpp pins the output that a person (and,
// later, the bench gate-test script) reads, so it cannot drift silently.
// ============================================================================

#pragma once

#include <stdint.h>

#include "diag.h"
#include "gate_logic.h"

namespace diagfmt {

// One character per sensor for the compact `watch` line.
inline char glyph(diag::SensorView v) {
    switch (v) {
    case diag::SensorView::CLEAR:    return '.';
    case diag::SensorView::BLOCKED:  return '#';
    case diag::SensorView::BANK_OFF: return '-';
    case diag::SensorView::INVALID:  return '?';
    case diag::SensorView::STALE:    return '~';
    }
    return '?';
}

inline const char* word(diag::SensorView v) {
    switch (v) {
    case diag::SensorView::CLEAR:    return "clear";
    case diag::SensorView::BLOCKED:  return "BLOCK";
    case diag::SensorView::BANK_OFF: return "disabled";
    case diag::SensorView::INVALID:  return "INVALID";
    case diag::SensorView::STALE:    return "stale";
    }
    return "?";
}

inline const char* stateName(gatelogic::GateState s) {
    switch (s) {
    case gatelogic::GateState::IDLE:        return "IDLE";
    case gatelogic::GateState::OUTER_FIRST: return "OUTER_FIRST";
    case gatelogic::GateState::INNER_FIRST: return "INNER_FIRST";
    case gatelogic::GateState::PAIRED:      return "PAIRED";
    }
    return "?";
}

// Render one sensor row of `n` gates as glyphs, a '|' between chips (every 8
// gates), NUL-terminated. `out` needs n + (n-1)/8 + 1 bytes; returns false (and
// writes nothing useful) if `cap` is smaller.
//
//     ........|..#.....|--------
//
// `inner` selects which sensor of each gate is rendered.
inline bool sensorRow(char* out, unsigned cap, const diag::GateView* gates,
                      unsigned n, bool inner) {
    const unsigned need = n + (n > 0 ? (n - 1) / 8 : 0) + 1;
    if (out == 0 || cap < need) return false;
    unsigned k = 0;
    for (unsigned i = 0; i < n; i++) {
        if (i > 0 && i % 8 == 0) out[k++] = '|';
        out[k++] = glyph(inner ? gates[i].inner_raw : gates[i].outer_raw);
    }
    out[k] = '\0';
    return true;
}

}  // namespace diagfmt
