// ============================================================================
// log_line.h — formatting and admission of one USB serial log line
// ============================================================================
//
// Pure logic, free of Arduino dependencies like gate_logic.h, so the rules can
// be pinned by test/test_log_line/ on a host compiler. src/serial_log.cpp owns
// the actual USB CDC writes and calls in here for the decisions.
//
// Why the log has rules at all
// ----------------------------
// The counter samples its gates every 5 ms and a crossing bee dwells ~12 ms in
// a beam. Anything that holds loop() for a few tens of milliseconds can lose a
// bee, and a USB serial write is exactly such a thing: the Arduino HWCDC driver
// waits up to its TX timeout (100 ms by default) for ring-buffer space, and
// retries up to 20 times, whenever a host is attached but not reading — a
// terminal paused, a laptop asleep, HiveHub's USB port enumerated with nothing
// draining it. That is up to two seconds per print.
//
// So every log line is admitted whole or not at all:
//
//   * it is formatted into a bounded buffer first, and always ends in exactly
//     one newline — a truncated line is marked, never left without a newline
//     for the next line to run into;
//   * it is written only if the TX ring has room for ALL of it right now, and
//     otherwise dropped and counted. A dropped line costs the reader a line; a
//     waited-for line costs the hive a bee.
// ============================================================================

#pragma once

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>

namespace logline {

// Longest line the logger formats, including the newline and the NUL. The
// longest line the firmware prints today is the 30 s [STAT] dump at ~120
// characters; 192 leaves room for the diagnostic console without making every
// caller's stack frame large (BLE callbacks log from the NimBLE host task).
constexpr unsigned MAX_LINE = 192;

// Appended in place of the tail of a line that did not fit. Visible, so a
// reader never mistakes a cut-off number for a small one.
constexpr char TRUNCATION_MARK[] = "...\n";

// Format `fmt`/`args` into `out` as one newline-terminated line.
//
// The caller's format may or may not end in '\n' (both styles exist in the
// firmware); the result always ends in exactly one. Returns the length written
// excluding the NUL, or 0 if nothing usable could be produced (a NULL or empty
// format, an encoding error, or a buffer too small to hold even the mark).
inline unsigned formatLine(char* out, unsigned capacity, const char* fmt,
                           va_list args) {
    const unsigned mark_len = sizeof(TRUNCATION_MARK) - 1;
    if (out == 0 || fmt == 0 || capacity < mark_len + 1) return 0;

    const int n = vsnprintf(out, capacity, fmt, args);
    if (n < 0) {
        out[0] = '\0';
        return 0;
    }

    unsigned len = static_cast<unsigned>(n);
    if (len >= capacity) {
        // Truncated by vsnprintf: overwrite the tail with the mark, which
        // carries its own newline.
        len = capacity - 1;
        for (unsigned i = 0; i < mark_len; i++) {
            out[len - mark_len + i] = TRUNCATION_MARK[i];
        }
        out[len] = '\0';
        return len;
    }

    // Collapse any run of trailing newlines to exactly one.
    while (len > 0 && (out[len - 1] == '\n' || out[len - 1] == '\r')) len--;
    if (len == 0) {
        out[0] = '\0';
        return 0;
    }
    if (len + 1 >= capacity) {
        // A line that exactly filled the buffer has no room left for its
        // newline; mark it rather than emit an unterminated line.
        len = capacity - 1;
        for (unsigned i = 0; i < mark_len; i++) {
            out[len - mark_len + i] = TRUNCATION_MARK[i];
        }
        out[len] = '\0';
        return len;
    }
    out[len++] = '\n';
    out[len] = '\0';
    return len;
}

// Variadic convenience wrapper over formatLine().
inline unsigned formatLinef(char* out, unsigned capacity, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));
inline unsigned formatLinef(char* out, unsigned capacity, const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    const unsigned len = formatLine(out, capacity, fmt, args);
    va_end(args);
    return len;
}

// May a line of `len` bytes be written, given `available` bytes free in the
// TX ring? All or nothing: a partially written line would splice into the next
// one and corrupt both for anyone parsing the log.
inline bool admits(unsigned len, int available) {
    return len > 0 && available >= 0 && static_cast<unsigned>(available) >= len;
}

}  // namespace logline
