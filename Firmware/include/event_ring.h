// ============================================================================
// event_ring.h — bounded queue of sensor and crossing events
// ============================================================================
//
// Pure logic, host-tested in test/test_event_ring/.
//
// The console's `events on` stream reports every debounced sensor edge and
// every counted crossing, timestamped at the poll that saw it. Those happen at
// the 5 ms sampling rate, in bursts, while USB drains at whatever pace the host
// reads. So the sampler only ever PUSHES into this ring — an O(1) copy, never
// a print — and the console drains it a few lines per loop pass when the USB
// buffer has room.
//
// When the ring is full the NEW event is dropped, not the oldest: a reader
// gets an unbroken prefix of what happened, followed by a visible gap. Every
// event is numbered whether it was kept or not, so a jump in `seq` on the
// reader's side says exactly how many were lost and where.
// ============================================================================

#pragma once

#include <stdint.h>

namespace eventring {

enum class Kind : uint8_t {
    INNER_BLOCK,   // inner sensor's debounced state became blocked
    INNER_CLEAR,
    OUTER_BLOCK,
    OUTER_CLEAR,
    BEE_IN,        // counted crossing, outer-then-inner
    BEE_OUT,       // counted crossing, inner-then-outer
    GLITCH,        // ambiguous or aborted pairing
};

struct Event {
    uint32_t seq = 0;
    uint32_t t_ms = 0;
    uint8_t  gate = 0;   // dense index into gates::TABLE
    Kind     kind = Kind::INNER_BLOCK;
};

// 256 events is ~1.3 s of every gate on a 24-gate entrance changing state at
// the full poll rate — far more than real traffic — and 3 KiB of RAM.
template <unsigned N>
struct Ring {
    static_assert(N > 0 && (N & (N - 1)) == 0, "N must be a power of two");

    Event    slots[N];
    uint32_t head = 0;       // next slot to read (monotonic, wraps via mask)
    uint32_t tail = 0;       // next slot to write
    uint32_t next_seq = 1;   // seq of the next event offered
    uint32_t dropped = 0;    // offered while full (saturating)

    unsigned size() const { return (unsigned)(tail - head); }
    bool empty() const { return head == tail; }

    // Offer one event. Returns false (and counts it) if the ring is full.
    bool push(uint32_t t_ms, uint8_t gate, Kind kind) {
        const uint32_t seq = next_seq++;
        if (size() >= N) {
            if (dropped != UINT32_MAX) dropped++;
            return false;
        }
        Event& e = slots[tail & (N - 1)];
        e.seq = seq;
        e.t_ms = t_ms;
        e.gate = gate;
        e.kind = kind;
        tail++;
        return true;
    }

    bool pop(Event& out) {
        if (empty()) return false;
        out = slots[head & (N - 1)];
        head++;
        return true;
    }

    // Discard queued events (the sequence numbering carries on).
    void clear() { head = tail; }
};

inline const char* kindName(Kind k) {
    switch (k) {
    case Kind::INNER_BLOCK: return "inner_block";
    case Kind::INNER_CLEAR: return "inner_clear";
    case Kind::OUTER_BLOCK: return "outer_block";
    case Kind::OUTER_CLEAR: return "outer_clear";
    case Kind::BEE_IN:      return "bee_in";
    case Kind::BEE_OUT:     return "bee_out";
    case Kind::GLITCH:      return "glitch";
    }
    return "?";
}

}  // namespace eventring
