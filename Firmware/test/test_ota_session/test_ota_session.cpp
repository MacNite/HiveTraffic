// ============================================================================
// Host-side tests for include/ota_session.h
// ============================================================================
//
// Both rules in that header guard states that are hard to reach on a bench and
// costly in the field: a relay that hangs mid-image stops the counter counting,
// and a frame that arrives after a verified END used to cancel the reboot while
// the new image stayed selected for the next one.
//
//     c++ -std=c++11 -I include
//         test/test_ota_session/test_ota_session.cpp -o /tmp/t && /tmp/t
//
// or via test/run_tests.sh, which builds every host test.
// ============================================================================

// Arduino's macro soup, before the include: see test_idle_state.cpp for why.
#define DISABLED 0x00
#define INPUT 0x01
#define OUTPUT 0x03
#define PULLUP 0x04
#define PULLDOWN 0x08
#define HIGH 0x1
#define LOW 0x0
#define ANALOG 0xC0
#define OPEN_DRAIN 0x10

#include "ota_session.h"

#include <cstdio>
#include <cstdlib>

using namespace otasession;
using namespace beecounter_proto;

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

static void test_opcodes_before_done() {
    g_case = "control opcodes map to their actions outside DONE";
    const uint8_t states[] = {OTA_STATE_IDLE, OTA_STATE_RECEIVING,
                              OTA_STATE_ERR_SEQ, OTA_STATE_ERR_CRC};
    for (uint8_t s : states) {
        CHECK(control(s, OP_BEGIN) == Control::BEGIN);
        CHECK(control(s, OP_END) == Control::END);
        CHECK(control(s, OP_ABORT) == Control::ABORT);
        CHECK(control(s, 0x02) == Control::FAIL_SEQ);
        CHECK(control(s, 0xFF) == Control::FAIL_SEQ);
    }
}

static void test_done_is_terminal_for_control() {
    // The boot slot is already switched when DONE is published. A repeated
    // END (a relay retrying a write whose response it lost), an ABORT, a fresh
    // BEGIN or garbage must all leave DONE and the pending reboot alone.
    g_case = "every control frame after DONE keeps DONE";
    CHECK(control(OTA_STATE_DONE, OP_END) == Control::KEEP_DONE);
    CHECK(control(OTA_STATE_DONE, OP_ABORT) == Control::KEEP_DONE);
    CHECK(control(OTA_STATE_DONE, OP_BEGIN) == Control::KEEP_DONE);
    CHECK(control(OTA_STATE_DONE, 0x7E) == Control::KEEP_DONE);
    CHECK(!mayFail(OTA_STATE_DONE));
}

static void test_failure_allowed_everywhere_else() {
    g_case = "failures may end any session that is not DONE";
    CHECK(mayFail(OTA_STATE_IDLE));
    CHECK(mayFail(OTA_STATE_RECEIVING));
    CHECK(mayFail(OTA_STATE_ERR_WRITE));
}

static void test_data_rules() {
    g_case = "DATA is accepted only inside the declared size";
    CHECK(data(OTA_STATE_RECEIVING, 0, 1000, 244) == Data::ACCEPT);
    CHECK(data(OTA_STATE_RECEIVING, 756, 1000, 244) == Data::ACCEPT);   // exactly fills
    CHECK(data(OTA_STATE_RECEIVING, 757, 1000, 244) == Data::FAIL_SEQ); // one byte over
    CHECK(data(OTA_STATE_RECEIVING, 1001, 1000, 1) == Data::FAIL_SEQ);  // already past
    CHECK(data(OTA_STATE_IDLE, 0, 0, 20) == Data::FAIL_SEQ);
    CHECK(data(OTA_STATE_ERR_CRC, 0, 1000, 20) == Data::FAIL_SEQ);

    g_case = "empty DATA writes are ignored";
    CHECK(data(OTA_STATE_RECEIVING, 0, 1000, 0) == Data::IGNORE);
    CHECK(data(OTA_STATE_IDLE, 0, 0, 0) == Data::IGNORE);

    g_case = "late DATA after DONE is ignored, not an error";
    CHECK(data(OTA_STATE_DONE, 1000, 1000, 20) == Data::IGNORE);
}

static void test_stall_timeout() {
    g_case = "a receiving transfer stalls after PROGRESS_TIMEOUT_MS";
    const uint32_t t0 = 5000;
    CHECK(!stalled(OTA_STATE_RECEIVING, t0, t0));
    CHECK(!stalled(OTA_STATE_RECEIVING, t0, t0 + PROGRESS_TIMEOUT_MS - 1));
    CHECK(stalled(OTA_STATE_RECEIVING, t0, (uint32_t)(t0 + PROGRESS_TIMEOUT_MS)));
    CHECK(stalled(OTA_STATE_RECEIVING, t0, t0 + 3600000UL));   // the hour-long hang

    g_case = "only RECEIVING can stall";
    CHECK(!stalled(OTA_STATE_IDLE, t0, t0 + 3600000UL));
    CHECK(!stalled(OTA_STATE_DONE, t0, t0 + 3600000UL));
    CHECK(!stalled(OTA_STATE_ERR_SEQ, t0, t0 + 3600000UL));
}

static void test_stall_across_millis_rollover() {
    g_case = "the stall timer survives the millis() rollover";
    const uint32_t t0 = 0xFFFFFFFFUL - 1000;   // 1 s before the wrap
    CHECK(!stalled(OTA_STATE_RECEIVING, t0, (uint32_t)(t0 + 30000UL)));   // wrapped, 30 s in
    CHECK(stalled(OTA_STATE_RECEIVING, t0, (uint32_t)(t0 + PROGRESS_TIMEOUT_MS)));
}

static void test_timeout_is_sane() {
    // Long enough for a slow HTTPS source, short enough that a hung relay
    // costs minutes of counting rather than a day.
    g_case = "the progress timeout is between 10 s and 10 min";
    CHECK(PROGRESS_TIMEOUT_MS >= 10000UL);
    CHECK(PROGRESS_TIMEOUT_MS <= 600000UL);
}

int main() {
    std::printf("ota_session tests\n");
    test_opcodes_before_done();
    test_done_is_terminal_for_control();
    test_failure_allowed_everywhere_else();
    test_data_rules();
    test_stall_timeout();
    test_stall_across_millis_rollover();
    test_timeout_is_sane();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
