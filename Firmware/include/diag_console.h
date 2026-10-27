// ============================================================================
// diag_console.h — the USB diagnostic console, built into every image
// ============================================================================
//
// Type `help` + Enter in any serial terminal at 115200 baud. The console is
// PASSIVE: every command reports state the normal 5 ms sampler already
// produced (see diag.h). It never reads a sensor itself, never lights an
// emitter and never touches the totals, so attaching a laptop to a counter on
// a hive cannot change what it counts.
//
// Output is paced, never blocking: multi-line reports, the `watch` view and
// the `events` stream each wait in their own queue and go out a few lines per
// loop pass, only when the USB buffer has room for a whole line. A host that
// stops reading costs the console its output, never the sampler its timing.
// ============================================================================

#pragma once

#include <stdint.h>

#include "event_ring.h"

namespace console {

// Print the one-line hint at boot.
void begin();

// Read pending input, act on complete commands, and emit queued output.
// Call once per loop() pass, after that pass's sampling.
void poll(uint32_t now_ms);

// Whether the sampler should bother offering events (`events on`).
bool eventsEnabled();

// Offer one event from the sampler. O(1): it only copies into a ring.
void noteEvent(uint32_t t_ms, uint8_t gate, eventring::Kind kind);

}  // namespace console
