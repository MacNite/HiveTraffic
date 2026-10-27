// ============================================================================
// Host-side tests for include/diag_format.h
// ============================================================================
//
// Pins the console's compact renderings: one distinct glyph per sensor view
// (so "chip unreadable" can never look like "clear"), and the `watch` row
// layout with a separator between chips.
//
//     c++ -std=c++11 -I include
//         test/test_diag_format/test_diag_format.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

// Arduino's macro soup, reproduced before the include on purpose (see
// test_idle_state.cpp). DISABLED is the one that matters here: a sensor view
// for "bank switched off" would naturally be called that, and a scoped enum
// does not protect an enumerator from a macro.
#define DISABLED 0x00
#define INPUT 0x01
#define OUTPUT 0x03
#define HIGH 0x1
#define LOW 0x0
#define DEC 10
#define HEX 16

#include "diag_format.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using diag::GateView;
using diag::SensorView;

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

static void test_glyphs_are_distinct() {
    g_case = "each sensor view renders as a distinct glyph and word";
    const SensorView views[] = {SensorView::CLEAR, SensorView::BLOCKED,
                                SensorView::BANK_OFF, SensorView::INVALID,
                                SensorView::STALE};
    const unsigned n = sizeof(views) / sizeof(views[0]);
    for (unsigned i = 0; i < n; i++) {
        for (unsigned j = i + 1; j < n; j++) {
            CHECK(diagfmt::glyph(views[i]) != diagfmt::glyph(views[j]));
            CHECK(std::strcmp(diagfmt::word(views[i]), diagfmt::word(views[j])) != 0);
        }
    }
    CHECK(diagfmt::glyph(SensorView::CLEAR) == '.');
    CHECK(diagfmt::glyph(SensorView::BLOCKED) == '#');
}

static void test_state_names() {
    g_case = "every gate state has a name";
    CHECK(std::strcmp(diagfmt::stateName(gatelogic::GateState::IDLE), "IDLE") == 0);
    CHECK(std::strcmp(diagfmt::stateName(gatelogic::GateState::PAIRED), "PAIRED") == 0);
    CHECK(std::strcmp(diagfmt::stateName(gatelogic::GateState::OUTER_FIRST),
                      "OUTER_FIRST") == 0);
    CHECK(std::strcmp(diagfmt::stateName(gatelogic::GateState::INNER_FIRST),
                      "INNER_FIRST") == 0);
}

static void test_sensor_row_layout() {
    g_case = "a 24-gate row has a separator between each chip's eight gates";
    GateView g[24];
    for (unsigned i = 0; i < 24; i++) {
        g[i].inner_raw = SensorView::CLEAR;
        g[i].outer_raw = SensorView::CLEAR;
    }
    g[2].inner_raw = SensorView::BLOCKED;
    g[9].outer_raw = SensorView::BLOCKED;
    for (unsigned i = 16; i < 24; i++) {
        g[i].inner_raw = SensorView::BANK_OFF;
        g[i].outer_raw = SensorView::BANK_OFF;
    }

    char row[24 + 2 + 1];
    CHECK(diagfmt::sensorRow(row, sizeof(row), g, 24, true));
    CHECK(std::strcmp(row, "..#.....|........|--------") == 0);
    CHECK(diagfmt::sensorRow(row, sizeof(row), g, 24, false));
    CHECK(std::strcmp(row, "........|.#......|--------") == 0);
}

static void test_sensor_row_refuses_small_buffer() {
    g_case = "a buffer one byte short is refused";
    GateView g[24];
    char row[24 + 2];
    CHECK(!diagfmt::sensorRow(row, sizeof(row), g, 24, true));
    CHECK(!diagfmt::sensorRow(0, 100, g, 24, true));
}

static void test_short_rows() {
    g_case = "rows shorter than one chip need no separator";
    GateView g[8];
    char row[9];
    CHECK(diagfmt::sensorRow(row, sizeof(row), g, 8, true));
    CHECK(std::strlen(row) == 8);
    char empty[1];
    CHECK(diagfmt::sensorRow(empty, sizeof(empty), g, 0, true));
    CHECK(empty[0] == '\0');
}

int main() {
    std::printf("diag_format tests\n");
    test_glyphs_are_distinct();
    test_state_names();
    test_sensor_row_layout();
    test_sensor_row_refuses_small_buffer();
    test_short_rows();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
