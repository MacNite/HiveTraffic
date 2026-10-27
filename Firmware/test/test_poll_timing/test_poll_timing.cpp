// ============================================================================
// Host-side tests for include/poll_timing.h
// ============================================================================
//
// The poll-gap record is how a stall that would swallow a crossing becomes
// visible at all, so its two easy mistakes are pinned here: counting a
// deliberate pause (night mode, OTA) as a stall, and mis-measuring across the
// micros() rollover, which happens every ~71 minutes on a live counter.
//
//     c++ -std=c++11 -I include
//         test/test_poll_timing/test_poll_timing.cpp -o /tmp/t && /tmp/t
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
#define DEC 10
#define HEX 16

#include "poll_timing.h"

#include <cstdio>
#include <cstdlib>

using namespace polltiming;

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

static const uint32_t LATE = 10000;   // 2 x 5 ms, as in main.cpp

// --------------------------------------------------------------------------

static void test_first_poll_has_no_gap() {
    g_case = "the first poll measures no gap";
    State s;
    notePoll(s, 123456, 900, LATE);
    CHECK(s.window.polls == 1);
    CHECK(s.window.late == 0);
    CHECK(s.window.max_gap_us == 0);
    CHECK(s.window.max_duration_us == 900);
}

static void test_regular_polls_are_not_late() {
    g_case = "polls every 5 ms are never late";
    State s;
    for (uint32_t t = 0; t < 100; t++) notePoll(s, t * 5000, 800, LATE);
    CHECK(s.window.polls == 100);
    CHECK(s.window.late == 0);
    CHECK(s.window.max_gap_us == 5000);
}

static void test_threshold_is_strict() {
    g_case = "a gap equal to the threshold is not late, one longer is";
    State s;
    notePoll(s, 0, 1, LATE);
    notePoll(s, 10000, 1, LATE);
    CHECK(s.window.late == 0);
    notePoll(s, 20001, 1, LATE);
    CHECK(s.window.late == 1);
    CHECK(s.window.max_gap_us == 10001);
}

static void test_pause_is_not_a_stall() {
    // Six hours of night mode must not read as a six-hour stall.
    g_case = "a deliberate pause measures no gap";
    State s;
    notePoll(s, 0, 1, LATE);
    pause(s);
    notePoll(s, 3600000000UL, 1, LATE);
    CHECK(s.window.late == 0);
    CHECK(s.window.max_gap_us == 0);
    notePoll(s, 3600005000UL, 1, LATE);
    CHECK(s.window.max_gap_us == 5000);
}

static void test_micros_rollover() {
    g_case = "a gap across the micros() rollover is measured correctly";
    State s;
    notePoll(s, 0xFFFFF000UL, 1, LATE);   // 4096 us before the wrap
    notePoll(s, 0x00000388UL, 1, LATE);   // 904 us after it: 5000 us gap
    CHECK(s.window.max_gap_us == 5000);
    CHECK(s.window.late == 0);
}

static void test_window_resets_boot_does_not() {
    g_case = "takeWindow() resets the window but not the boot record";
    State s;
    notePoll(s, 0, 700, LATE);
    notePoll(s, 50000, 2500, LATE);       // one late poll, 50 ms gap
    const Figures w = takeWindow(s);
    CHECK(w.polls == 2);
    CHECK(w.late == 1);
    CHECK(w.max_gap_us == 50000);
    CHECK(w.max_duration_us == 2500);
    CHECK(s.window.polls == 0);
    CHECK(s.window.late == 0);
    CHECK(s.window.max_gap_us == 0);
    CHECK(s.window.max_duration_us == 0);
    CHECK(s.boot.late == 1);
    CHECK(s.boot.max_gap_us == 50000);

    // The gap chain continues across a window boundary: the next poll's gap is
    // measured from the last poll of the previous window.
    notePoll(s, 55000, 700, LATE);
    CHECK(s.window.max_gap_us == 5000);
}

static void test_counters_saturate() {
    g_case = "poll and late counts saturate rather than wrap";
    State s;
    s.window.polls = UINT32_MAX;
    s.window.late = UINT32_MAX;
    notePoll(s, 0, 1, LATE);
    notePoll(s, 99999, 1, LATE);
    CHECK(s.window.polls == UINT32_MAX);
    CHECK(s.window.late == UINT32_MAX);
}

int main() {
    std::printf("poll_timing tests\n");
    test_first_poll_has_no_gap();
    test_regular_polls_are_not_late();
    test_threshold_is_strict();
    test_pause_is_not_a_stall();
    test_micros_rollover();
    test_window_resets_boot_does_not();
    test_counters_saturate();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
