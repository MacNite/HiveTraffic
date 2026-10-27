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

inline const char* ledName(diag::LedSetting l) {
    switch (l) {
    case diag::LedSetting::AUTO: return "auto";
    case diag::LedSetting::ON:   return "on";
    case diag::LedSetting::OFF:  return "off";
    }
    return "?";
}

inline const char* resultText(diag::Result r) {
    switch (r) {
    case diag::Result::OK:           return "ok";
    case diag::Result::NOT_IN_MAINT: return "refused: not in maintenance - 'maint on' first";
    case diag::Result::OTA_ACTIVE:   return "refused: firmware update in progress";
    case diag::Result::ALREADY:      return "nothing to do";
    case diag::Result::REFUSED:      return "refused";
    }
    return "?";
}

// What one sensor's dark and lit readings, taken back to back with a target
// (or nothing) in front of it, say about that sensor.
//
//   lit BLOCK, dark clear  "reflects"  the emitter's own light comes back:
//                                      emitter, sensor and target all work
//   both clear             "none"      nothing reflected: no target, or a dead
//                                      emitter / sensor. Expected when empty
//   both BLOCK             "ambient"   LOW without the emitter: ambient IR (sun)
//                                      or an input stuck low
//   lit clear, dark BLOCK  "inverted"  should not happen; points at wiring
//
// Anything not a reading (bank off, chip unreadable) is reported as such.
inline const char* darkLitVerdict(diag::SensorView dark, diag::SensorView lit) {
    using diag::SensorView;
    const bool readable =
        (dark == SensorView::CLEAR || dark == SensorView::BLOCKED) &&
        (lit == SensorView::CLEAR || lit == SensorView::BLOCKED);
    if (!readable) {
        return word(lit != SensorView::CLEAR && lit != SensorView::BLOCKED ? lit : dark);
    }
    if (lit == SensorView::BLOCKED) {
        return dark == SensorView::CLEAR ? "reflects" : "ambient";
    }
    return dark == SensorView::CLEAR ? "none" : "inverted";
}

// MCP23017 register values the firmware relies on (IOCON.BANK = 0 addressing).
// All sixteen pins inputs, no polarity inversion, internal pull-ups off (the
// board has 100k), and IOCON with BANK = 0 (register map as addressed) and
// SEQOP = 0 (sequential reads, which the two-byte GPIOA+GPIOB read needs).
constexpr uint8_t IOCON_BANK  = 0x80;
constexpr uint8_t IOCON_SEQOP = 0x20;

inline bool mcpConfigOk(uint8_t iodir_a, uint8_t iodir_b, uint8_t ipol_a,
                        uint8_t ipol_b, uint8_t gppu_a, uint8_t gppu_b,
                        uint8_t iocon) {
    return iodir_a == 0xFF && iodir_b == 0xFF && ipol_a == 0x00 &&
           ipol_b == 0x00 && gppu_a == 0x00 && gppu_b == 0x00 &&
           (iocon & (IOCON_BANK | IOCON_SEQOP)) == 0;
}

}  // namespace diagfmt
