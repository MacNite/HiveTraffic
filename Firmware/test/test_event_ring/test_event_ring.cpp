// ============================================================================
// Host-side tests for include/event_ring.h
// ============================================================================
//
// The event stream's promise is "an unbroken prefix, then a visible gap": the
// sampler never waits for the reader, and the reader can always tell exactly
// how many events it missed. Both halves are pinned here.
//
//     c++ -std=c++11 -I include
//         test/test_event_ring/test_event_ring.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

// Arduino's macro soup, reproduced before the include on purpose (see
// test_idle_state.cpp).
#define DISABLED 0x00
#define INPUT 0x01
#define OUTPUT 0x03
#define HIGH 0x1
#define LOW 0x0

#include "event_ring.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace eventring;

static int g_failures = 0;
static const char* g_case = "";

#define CHECK(cond)                                                          \
    do {                                                                     \
        if (!(cond)) {                                                       \
            std::printf("  FAIL %s:%d  [%s]  %s\n", __FILE__, __LINE__,      \
                        g_case, #cond);                                      \
            ++g_failures;                                                    \
        }                                                                    \
    } while (0)

// --------------------------------------------------------------------------

static void test_fifo_order() {
    g_case = "events come out in order with consecutive seq";
    Ring<8> r;
    CHECK(r.empty());
    CHECK(r.push(10, 0, Kind::OUTER_BLOCK));
    CHECK(r.push(15, 0, Kind::INNER_BLOCK));
    CHECK(r.push(15, 0, Kind::BEE_IN));
    CHECK(r.size() == 3);
    Event e;
    CHECK(r.pop(e) && e.seq == 1 && e.t_ms == 10 && e.kind == Kind::OUTER_BLOCK);
    CHECK(r.pop(e) && e.seq == 2 && e.kind == Kind::INNER_BLOCK);
    CHECK(r.pop(e) && e.seq == 3 && e.kind == Kind::BEE_IN && e.gate == 0);
    CHECK(!r.pop(e));
    CHECK(r.empty());
}

static void test_full_drops_newest_and_leaves_a_gap() {
    g_case = "a full ring drops the newest, counts it, and seq shows the gap";
    Ring<4> r;
    for (uint32_t i = 0; i < 6; i++) r.push(i, 1, Kind::GLITCH);
    CHECK(r.size() == 4);
    CHECK(r.dropped == 2);
    Event e;
    for (uint32_t i = 1; i <= 4; i++) CHECK(r.pop(e) && e.seq == i);
    // The next kept event carries seq 7: events 5 and 6 are the visible gap.
    CHECK(r.push(100, 2, Kind::BEE_OUT));
    CHECK(r.pop(e) && e.seq == 7 && e.gate == 2);
}

static void test_wraparound() {
    g_case = "the ring keeps working across many wraps of its slots";
    Ring<4> r;
    Event e;
    uint32_t expect = 1;
    for (uint32_t round = 0; round < 1000; round++) {
        CHECK(r.push(round, (uint8_t)(round % 24), Kind::INNER_CLEAR));
        CHECK(r.push(round, (uint8_t)(round % 24), Kind::OUTER_CLEAR));
        CHECK(r.pop(e) && e.seq == expect++);
        CHECK(r.pop(e) && e.seq == expect++);
    }
    CHECK(r.empty());
    CHECK(r.dropped == 0);
}

static void test_clear_keeps_numbering() {
    g_case = "clear() empties the ring but seq carries on";
    Ring<8> r;
    r.push(1, 0, Kind::BEE_IN);
    r.push(2, 0, Kind::BEE_IN);
    r.clear();
    CHECK(r.empty());
    Event e;
    CHECK(!r.pop(e));
    r.push(3, 0, Kind::BEE_IN);
    CHECK(r.pop(e) && e.seq == 3);
}

static void test_dropped_saturates() {
    g_case = "the dropped count saturates";
    Ring<1> r;
    r.dropped = UINT32_MAX - 1;
    r.push(0, 0, Kind::GLITCH);   // fills the ring
    r.push(0, 0, Kind::GLITCH);
    r.push(0, 0, Kind::GLITCH);
    CHECK(r.dropped == UINT32_MAX);
}

static void test_kind_names() {
    g_case = "every kind has a distinct printable name";
    const Kind kinds[] = {Kind::INNER_BLOCK, Kind::INNER_CLEAR, Kind::OUTER_BLOCK,
                          Kind::OUTER_CLEAR, Kind::BEE_IN,      Kind::BEE_OUT,
                          Kind::GLITCH};
    const unsigned n = sizeof(kinds) / sizeof(kinds[0]);
    for (unsigned i = 0; i < n; i++) {
        CHECK(std::strcmp(kindName(kinds[i]), "?") != 0);
        for (unsigned j = i + 1; j < n; j++) {
            CHECK(std::strcmp(kindName(kinds[i]), kindName(kinds[j])) != 0);
        }
    }
}

int main() {
    std::printf("event_ring tests\n");
    test_fifo_order();
    test_full_drops_newest_and_leaves_a_gap();
    test_wraparound();
    test_clear_keeps_numbering();
    test_dropped_saturates();
    test_kind_names();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
