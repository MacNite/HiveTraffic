// ============================================================================
// Host-side tests for include/maint_state.h
// ============================================================================
//
// A maintenance session freezes the totals and holds HiveHub's requests back.
// Its failure modes are therefore the same silent kind night mode has — a
// counter left frozen, or one that ends a test with the wrong banks or no
// night mode — and they are pinned here: the inactivity timeout, the replay
// of deferred requests (with a suspension's elapsed time deducted), and the
// "maint_s" figure HiveHub uses to tell a test from a quiet hive.
//
//     c++ -std=c++11 -I include
//         test/test_maint_state/test_maint_state.cpp -o /tmp/t && /tmp/t
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

#include "maint_state.h"

#include <cstdio>
#include <cstdlib>

using namespace maintstate;

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

static void test_default_is_inactive() {
    g_case = "a fresh state is not in maintenance";
    State s;
    CHECK(!s.active);
    CHECK(!timedOut(s, 0xFFFFFFFFUL));
    CHECK(totalSeconds(s, 123456) == 0);
    CHECK(timeoutRemainingSeconds(s, 0) == 0);
}

static void test_enter_once() {
    g_case = "enter() starts one session and refuses a second";
    State s;
    CHECK(enter(s, 1000, 0x07, 0));
    CHECK(s.active);
    CHECK(!enter(s, 2000, 0x01, 0));
    CHECK(s.entered_ms == 1000);
    CHECK(s.banks_before == 0x07);
}

static void test_enter_resets_test_counters() {
    g_case = "each session starts its test counters at zero";
    State s;
    enter(s, 0, 0x07, 0);
    s.test_in = 5;
    s.test_out = 4;
    s.test_glitch = 3;
    exit(s, 1000);
    enter(s, 2000, 0x07, 0);
    CHECK(s.test_in == 0 && s.test_out == 0 && s.test_glitch == 0);
}

static void test_timeout_and_touch() {
    g_case = "inactivity ends a session; any command restarts the clock";
    State s;
    enter(s, 0, 0x07, 0);
    CHECK(!timedOut(s, TIMEOUT_MS - 1));
    CHECK(timedOut(s, TIMEOUT_MS));
    touch(s, TIMEOUT_MS - 1000);
    CHECK(!timedOut(s, TIMEOUT_MS));
    CHECK(timeoutRemainingSeconds(s, TIMEOUT_MS) == TIMEOUT_MS / 1000 - 1);
    CHECK(timedOut(s, 2 * TIMEOUT_MS - 1000));
}

static void test_timeout_across_millis_rollover() {
    g_case = "the timeout survives the millis() rollover";
    State s;
    const uint32_t near_wrap = 0xFFFFFFFFUL - 1000;
    enter(s, near_wrap, 0x07, 0);
    uint32_t later = near_wrap;
    later += 60000;                      // wraps
    CHECK(!timedOut(s, later));
    later = near_wrap;
    later += TIMEOUT_MS;
    CHECK(timedOut(s, later));
}

static void test_exit_restores_mask_at_entry() {
    g_case = "with no HiveHub request, the mask at entry comes back";
    State s;
    enter(s, 0, 0x05, 0);
    const Restore r = exit(s, 5000);
    CHECK(!s.active);
    CHECK(r.bank_mask == 0x05);
    CHECK(r.idle_s == 0);
}

static void test_exit_applies_latest_hivehub_mask() {
    g_case = "HiveHub's latest bank request wins over the mask at entry";
    State s;
    enter(s, 0, 0x07, 0);
    deferBanks(s, 0x01);
    deferBanks(s, 0x03);
    const Restore r = exit(s, 5000);
    CHECK(r.bank_mask == 0x03);
}

static void test_deferred_idle_loses_elapsed_time() {
    // A request does not get longer by waiting: HiveHub asked for "the next
    // ten minutes", not "ten minutes from whenever the bench is done".
    g_case = "a deferred suspension resumes with only its remaining time";
    State s;
    enter(s, 0, 0x07, 0);
    deferIdle(s, 10000, 600);            // at t=10 s, 600 s
    Restore r = exit(s, 250000);          // 240 s later
    CHECK(r.idle_s == 360);

    enter(s, 0, 0x07, 0);
    deferIdle(s, 0, 600);
    r = exit(s, 700000);                  // it has run out
    CHECK(r.idle_s == 0);
}

static void test_resume_request_cancels_deferred_idle() {
    g_case = "a deferred resume (0 s) cancels a pending suspension";
    State s;
    enter(s, 0, 0x07, 0);
    deferIdle(s, 0, 600);
    deferIdle(s, 1000, 0);
    CHECK(exit(s, 2000).idle_s == 0);
}

static void test_suspension_in_force_at_entry_is_taken_over() {
    g_case = "night mode running at entry is replayed with its remaining time";
    State s;
    enter(s, 100000, 0x07, 900);         // 900 s of night mode left
    const Restore r = exit(s, 400000);   // 300 s later
    CHECK(r.idle_s == 600);
}

static void test_total_seconds() {
    g_case = "maint_s counts finished and running sessions, rounding up";
    State s;
    enter(s, 0, 0x07, 0);
    CHECK(totalSeconds(s, 0) == 1);       // a session always counts
    CHECK(totalSeconds(s, 1500) == 2);    // 1.5 s rounds up
    exit(s, 1500);
    CHECK(s.completed_s == 2);
    CHECK(totalSeconds(s, 999999) == 2);  // not running: frozen
    enter(s, 10000, 0x07, 0);
    CHECK(totalSeconds(s, 70000) == 62);  // 2 + 60
    exit(s, 70000);
    CHECK(totalSeconds(s, 80000) == 62);
}

static void test_total_seconds_is_monotonic_across_exit() {
    // HiveHub differences consecutive reads; the figure must never step back
    // when a session ends between two of them.
    g_case = "maint_s never decreases at the end of a session";
    State s;
    enter(s, 0, 0x07, 0);
    const uint32_t during = totalSeconds(s, 12345);
    exit(s, 12345);
    CHECK(totalSeconds(s, 12345) >= during);
}

static void test_exit_when_inactive_is_harmless() {
    g_case = "exit() without a session changes nothing";
    State s;
    s.completed_s = 42;
    const Restore r = exit(s, 1000);
    CHECK(r.bank_mask == 0 && r.idle_s == 0);
    CHECK(s.completed_s == 42);
}

static void test_saturation() {
    g_case = "the lifetime figure saturates rather than wraps";
    State s;
    s.completed_s = UINT32_MAX - 1;
    enter(s, 0, 0x07, 0);
    CHECK(totalSeconds(s, 10000) == UINT32_MAX);
    exit(s, 10000);
    CHECK(s.completed_s == UINT32_MAX);
}

int main() {
    std::printf("maint_state tests\n");
    test_default_is_inactive();
    test_enter_once();
    test_enter_resets_test_counters();
    test_timeout_and_touch();
    test_timeout_across_millis_rollover();
    test_exit_restores_mask_at_entry();
    test_exit_applies_latest_hivehub_mask();
    test_deferred_idle_loses_elapsed_time();
    test_resume_request_cancels_deferred_idle();
    test_suspension_in_force_at_entry_is_taken_over();
    test_total_seconds();
    test_total_seconds_is_monotonic_across_exit();
    test_exit_when_inactive_is_harmless();
    test_saturation();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
