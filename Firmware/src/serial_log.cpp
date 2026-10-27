// Non-blocking USB serial log. See include/serial_log.h and include/log_line.h.
#include "serial_log.h"

#include <Arduino.h>
#include <stdarg.h>

#include "log_line.h"

namespace slog {
namespace {

// USB TX ring size. The driver's default is 256 bytes — two [STAT] lines —
// which a boot banner alone overflows once writes stop waiting for space.
// 4 KiB holds the whole boot log and several seconds of diagnostic output, so
// a host that is reading but briefly slower than the firmware loses nothing;
// only a host that has stopped reading altogether sees lines dropped. Heap
// cost is negligible next to the ~300 KiB this firmware leaves free.
constexpr size_t TX_RING_BYTES = 4096;

// Written from loop() and from the NimBLE host task.
uint32_t g_dropped = 0;

void noteDropped() {
    // Saturating, like every other counter this firmware reports.
    uint32_t seen = __atomic_load_n(&g_dropped, __ATOMIC_RELAXED);
    while (seen != UINT32_MAX &&
           !__atomic_compare_exchange_n(&g_dropped, &seen, seen + 1, false,
                                        __ATOMIC_RELAXED, __ATOMIC_RELAXED)) {
    }
}

}  // namespace

void begin(unsigned long baud) {
    // Must precede begin(): the driver only allocates its default ring if none
    // has been preset.
    Serial.setTxBufferSize(TX_RING_BYTES);
    Serial.begin(baud);
    // The load-bearing line. With a timeout of 0 the driver never waits for
    // ring space or for its TX lock — not here, and not on the ESP-IDF log
    // path (log_i/log_w at CORE_DEBUG_LEVEL=3), which writes through the same
    // driver one character at a time with the same timeout per character.
    Serial.setTxTimeoutMs(0);
}

void printf(const char* fmt, ...) {
    char line[logline::MAX_LINE];
    va_list args;
    va_start(args, fmt);
    const unsigned len = logline::formatLine(line, sizeof(line), fmt, args);
    va_end(args);
    if (len == 0) return;

    if (!logline::admits(len, Serial.availableForWrite())) {
        noteDropped();
        return;
    }
    // Another task can take the space between the check and the write; with a
    // zero timeout the driver then returns a short count rather than waiting.
    if (Serial.write(reinterpret_cast<const uint8_t*>(line), len) != len) {
        noteDropped();
    }
}

uint32_t droppedLines() {
    return __atomic_load_n(&g_dropped, __ATOMIC_RELAXED);
}

}  // namespace slog
