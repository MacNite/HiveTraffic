// ============================================================================
// serial_log.h — non-blocking USB serial log
// ============================================================================
//
// Every line the firmware prints goes through slog::printf(), never through
// Serial.print*() directly. The difference is that this one never waits: a
// line that does not fit the USB TX ring right now is dropped and counted
// instead of stalling the 5 ms gate sampler (see include/log_line.h for why
// that matters, and for the formatting rules).
//
// Safe to call from loop() and from the NimBLE host task alike. Each line is
// handed to the driver in one write() call, which the driver serialises.
// ============================================================================

#pragma once

#include <stdint.h>

namespace slog {

// Size the USB TX ring, start the CDC port and switch the driver to
// non-blocking writes. Call once, first thing in setup(), instead of
// Serial.begin().
void begin(unsigned long baud);

// Format and queue one line. A trailing '\n' in `fmt` is optional; exactly one
// is always sent. Dropped whole if the TX ring cannot take all of it.
void printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Bytes the USB TX ring can take right now without dropping anything. The
// console uses it to hold queued output back rather than lose it: a report or
// an event waits in its own queue until a whole line fits.
int writable();

// Lines dropped since boot because the TX ring was full (no host reading, or
// output faster than USB drains it). Saturating.
uint32_t droppedLines();

}  // namespace slog
