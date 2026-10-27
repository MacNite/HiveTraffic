// ============================================================================
// Host-side tests for include/log_line.h
// ============================================================================
//
// The serial log's contract is "whole lines or nothing, never wait". These
// checks pin the "whole lines" half: every formatted line ends in exactly one
// newline, a line too long for the buffer is visibly marked rather than
// silently cut, and the admission check never lets a partial line through.
// The "never wait" half is the driver's zero TX timeout and can only be
// verified on hardware — see the README's serial-output section.
//
//     c++ -std=c++11 -I include
//         test/test_log_line/test_log_line.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

// Arduino's macro soup, reproduced before the include on purpose (see
// test_idle_state.cpp). The number-base macros are the ones a formatting
// header is most likely to meet.
#define DISABLED 0x00
#define INPUT 0x01
#define OUTPUT 0x03
#define HIGH 0x1
#define LOW 0x0
#define DEC 10
#define HEX 16
#define OCT 8
#define BIN 2

#include "log_line.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace logline;

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

static void test_newline_is_added_when_missing() {
    g_case = "a format without a newline gets exactly one";
    char buf[MAX_LINE];
    const unsigned n = formatLinef(buf, sizeof(buf), "[BLE] up %d", 7);
    CHECK(n == std::strlen("[BLE] up 7\n"));
    CHECK(std::strcmp(buf, "[BLE] up 7\n") == 0);
}

static void test_existing_newline_is_kept_single() {
    g_case = "a format ending in newlines keeps exactly one";
    char buf[MAX_LINE];
    CHECK(formatLinef(buf, sizeof(buf), "x=%u\n", 3u) == 4);
    CHECK(std::strcmp(buf, "x=3\n") == 0);
    CHECK(formatLinef(buf, sizeof(buf), "y\r\n\n") == 2);
    CHECK(std::strcmp(buf, "y\n") == 0);
}

static void test_empty_line_is_nothing() {
    // An empty line carries nothing; returning 0 means the caller writes
    // nothing and counts nothing as dropped.
    g_case = "an empty or newline-only line produces nothing";
    char buf[MAX_LINE];
    CHECK(formatLinef(buf, sizeof(buf), "%s", "") == 0);
    CHECK(formatLinef(buf, sizeof(buf), "\n") == 0);
    CHECK(formatLine(buf, sizeof(buf), 0, 0) == 0);
}

static void test_overlong_line_is_marked() {
    g_case = "a line longer than the buffer ends in the truncation mark";
    char buf[16];
    const unsigned n = formatLinef(buf, sizeof(buf), "%s",
                                   "0123456789abcdefghijklmnopqrstuvwxyz");
    CHECK(n == sizeof(buf) - 1);
    CHECK(std::strlen(buf) == n);
    CHECK(std::strcmp(buf + n - 4, "...\n") == 0);
    CHECK(std::strncmp(buf, "0123456789a", 11) == 0);
}

static void test_exact_fit_has_room_for_its_newline() {
    // 15 visible characters in a 16-byte buffer: vsnprintf fits them exactly,
    // leaving no room for the newline the line must end in.
    g_case = "a line that exactly fills the buffer is marked, not left open";
    char buf[16];
    const unsigned n = formatLinef(buf, sizeof(buf), "%s", "0123456789abcde");
    CHECK(n == 15);
    CHECK(buf[n - 1] == '\n');
    CHECK(std::strcmp(buf + n - 4, "...\n") == 0);

    // One shorter fits with its newline and is not marked.
    const unsigned m = formatLinef(buf, sizeof(buf), "%s", "0123456789abcd");
    CHECK(m == 15);
    CHECK(std::strcmp(buf, "0123456789abcd\n") == 0);
}

static void test_tiny_buffer_produces_nothing() {
    g_case = "a buffer too small for the mark produces nothing";
    char buf[4];
    CHECK(formatLinef(buf, sizeof(buf), "abc") == 0);
}

static void test_stat_line_fits() {
    // The longest line the firmware prints in normal operation, with every
    // field saturated. If this ever gets marked, MAX_LINE needs to grow.
    g_case = "the saturated [STAT] and [TIMING] lines fit unmarked";
    char buf[MAX_LINE];
    unsigned n = formatLinef(
        buf, sizeof(buf),
        "[STAT] uptime=%lus total_in=%lu total_out=%lu "
        "glitches=%lu status=0x%02X idle=%lus banks=0x%02X\n",
        4294967UL, 4294967295UL, 4294967295UL, 4294967295UL, 255u,
        4294967295UL, 255u);
    std::printf("  saturated [STAT] line: %u bytes of %u\n", n, MAX_LINE);
    CHECK(n > 0 && std::strstr(buf, "...") == 0);
    n = formatLinef(
        buf, sizeof(buf),
        "[TIMING] polls=%lu late=%lu max_gap=%luus max_poll=%luus "
        "log_dropped=%lu (boot: late=%lu max_gap=%luus)\n",
        4294967295UL, 4294967295UL, 4294967295UL, 4294967295UL,
        4294967295UL, 4294967295UL, 4294967295UL);
    std::printf("  saturated [TIMING] line: %u bytes of %u\n", n, MAX_LINE);
    CHECK(n > 0 && std::strstr(buf, "...") == 0);
}

static void test_admission_is_all_or_nothing() {
    g_case = "a line is admitted only if all of it fits";
    CHECK(admits(10, 10));
    CHECK(admits(10, 4096));
    CHECK(!admits(10, 9));
    CHECK(!admits(10, 0));
    CHECK(!admits(10, -1));     // driver could not even take its lock
    CHECK(!admits(0, 4096));    // nothing to write is never "admitted"
}

int main() {
    std::printf("log_line tests\n");
    test_newline_is_added_when_missing();
    test_existing_newline_is_kept_single();
    test_empty_line_is_nothing();
    test_overlong_line_is_marked();
    test_exact_fit_has_room_for_its_newline();
    test_tiny_buffer_produces_nothing();
    test_stat_line_fits();
    test_admission_is_all_or_nothing();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
