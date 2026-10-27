// ============================================================================
// Host-side tests for include/console_line.h
// ============================================================================
//
// The console ships in every image and listens on a port that anything can be
// plugged into, so its parser is pinned to be strict: a line acts only when it
// is exactly a known command with a valid argument, line endings from every
// common terminal work, and an overlong line is discarded whole rather than
// truncated into some other command.
//
//     c++ -std=c++11 -I include
//         test/test_console_line/test_console_line.cpp -o /tmp/t && /tmp/t
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

#include "console_line.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace consoleline;

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

// Feed a whole string; return the last non-NONE result (NONE if there was none).
static Feed feedAll(Reader& r, const char* s) {
    Feed last = Feed::NONE;
    for (; *s; s++) {
        const Feed f = feed(r, *s);
        if (f != Feed::NONE) last = f;
    }
    return last;
}

// --------------------------------------------------------------------------

static void test_line_endings() {
    g_case = "CR, LF and CRLF all end a line, and CRLF yields one line";
    Reader r;
    CHECK(feedAll(r, "status\r") == Feed::LINE);
    CHECK(std::strcmp(r.line, "status") == 0);
    CHECK(feedAll(r, "mcp\n") == Feed::LINE);
    CHECK(std::strcmp(r.line, "mcp") == 0);

    int lines = 0;
    for (const char* s = "help\r\n"; *s; s++) {
        if (feed(r, *s) == Feed::LINE) lines++;
    }
    CHECK(lines == 1);
}

static void test_blank_lines_are_ignored() {
    g_case = "blank lines produce nothing";
    Reader r;
    CHECK(feedAll(r, "\r\n\n\r") == Feed::NONE);
}

static void test_backspace_edits() {
    g_case = "backspace and DEL edit the line being typed";
    Reader r;
    CHECK(feedAll(r, "statsu\b\bus\n") == Feed::LINE);
    CHECK(std::strcmp(r.line, "status") == 0);
    CHECK(feedAll(r, "mcx\x7f" "p\n") == Feed::LINE);
    CHECK(std::strcmp(r.line, "mcp") == 0);
    // Backspace on an empty line is harmless.
    CHECK(feedAll(r, "\b\b\bhelp\n") == Feed::LINE);
    CHECK(std::strcmp(r.line, "help") == 0);
}

static void test_control_characters_are_dropped() {
    g_case = "escape sequences and other control bytes are not stored";
    Reader r;
    CHECK(feedAll(r, "\x1b[Ast\x01" "atus\n") == Feed::LINE);
    // ESC is dropped; '[' and 'A' are printable and remain, which the parser
    // then rejects as an unknown command — never as "status".
    CHECK(std::strcmp(r.line, "[Astatus") == 0);
    CHECK(parse(r.line).verb == Verb::UNKNOWN);
}

static void test_overflow_discards_whole_line() {
    g_case = "an overlong line is discarded whole, then input recovers";
    Reader r;
    char longline[MAX_LINE + 10];
    std::memset(longline, 'x', sizeof(longline) - 1);
    longline[sizeof(longline) - 1] = '\0';
    CHECK(feedAll(r, longline) == Feed::NONE);
    CHECK(feed(r, '\n') == Feed::OVERFLOW);
    CHECK(feedAll(r, "status\n") == Feed::LINE);
    CHECK(std::strcmp(r.line, "status") == 0);

    // Exactly MAX_LINE characters still fits.
    char exact[MAX_LINE + 1];
    std::memset(exact, 'y', MAX_LINE);
    exact[MAX_LINE] = '\0';
    CHECK(feedAll(r, exact) == Feed::NONE);
    CHECK(feed(r, '\n') == Feed::LINE);
    CHECK(std::strlen(r.line) == MAX_LINE);
}

static void test_simple_verbs() {
    g_case = "argument-less verbs parse, case-insensitively, with spacing";
    CHECK(parse("help").verb == Verb::HELP);
    CHECK(parse("?").verb == Verb::HELP);
    CHECK(parse("h").verb == Verb::HELP);
    CHECK(parse("status").verb == Verb::STATUS);
    CHECK(parse("STATUS").verb == Verb::STATUS);
    CHECK(parse("  sensors  ").verb == Verb::SENSORS);
    CHECK(parse("mcp").verb == Verb::MCP);
    CHECK(parse("").verb == Verb::NONE);
    CHECK(parse("   ").verb == Verb::NONE);
}

static void test_simple_verbs_reject_arguments() {
    g_case = "argument-less verbs refuse an argument";
    CHECK(parse("status now").verb == Verb::BAD_ARG);
    CHECK(parse("mcp 1").verb == Verb::BAD_ARG);
    CHECK(parse("help me please").verb == Verb::BAD_ARG);
}

static void test_watch() {
    g_case = "watch takes 1..10 Hz, 'off', or defaults";
    Command c = parse("watch");
    CHECK(c.verb == Verb::WATCH && c.arg == WATCH_DEFAULT_HZ);
    c = parse("watch 1");
    CHECK(c.verb == Verb::WATCH && c.arg == 1);
    c = parse("watch 10");
    CHECK(c.verb == Verb::WATCH && c.arg == 10);
    c = parse("Watch OFF");
    CHECK(c.verb == Verb::WATCH && c.arg == 0);
    c = parse("watch 0");
    CHECK(c.verb == Verb::WATCH && c.arg == 0);   // 0 Hz is "off"
    CHECK(parse("watch 11").verb == Verb::BAD_ARG);
    CHECK(parse("watch -1").verb == Verb::BAD_ARG);
    CHECK(parse("watch 5x").verb == Verb::BAD_ARG);
    CHECK(parse("watch 99999999999999").verb == Verb::BAD_ARG);
    CHECK(parse("watch 5 6").verb == Verb::BAD_ARG);
}

static void test_events() {
    g_case = "events needs exactly on or off";
    Command c = parse("events on");
    CHECK(c.verb == Verb::EVENTS && c.arg == 1);
    c = parse("events OFF");
    CHECK(c.verb == Verb::EVENTS && c.arg == 0);
    CHECK(parse("events").verb == Verb::BAD_ARG);
    CHECK(parse("events maybe").verb == Verb::BAD_ARG);
    CHECK(parse("events on off").verb == Verb::BAD_ARG);
}

static void test_maintenance_commands() {
    g_case = "maintenance commands parse exactly";
    Command c = parse("maint on");
    CHECK(c.verb == Verb::MAINT && c.arg == MAINT_ON);
    c = parse("MAINT off");
    CHECK(c.verb == Verb::MAINT && c.arg == MAINT_OFF);
    c = parse("maint");
    CHECK(c.verb == Verb::MAINT && c.arg == MAINT_SHOW);
    CHECK(parse("maint please").verb == Verb::BAD_ARG);

    c = parse("leds auto");
    CHECK(c.verb == Verb::LEDS && c.arg == LEDS_AUTO);
    c = parse("leds on");
    CHECK(c.verb == Verb::LEDS && c.arg == LEDS_ON);
    c = parse("leds off");
    CHECK(c.verb == Verb::LEDS && c.arg == LEDS_OFF);
    CHECK(parse("leds").verb == Verb::BAD_ARG);
    CHECK(parse("leds dim").verb == Verb::BAD_ARG);

    c = parse("bank 2 off");
    CHECK(c.verb == Verb::BANK && c.arg == 2 && c.arg2 == 0);
    c = parse("bank 3 ON");
    CHECK(c.verb == Verb::BANK && c.arg == 3 && c.arg2 == 1);
    CHECK(parse("bank 0 on").verb == Verb::BAD_ARG);
    CHECK(parse("bank 4 on").verb == Verb::BAD_ARG);
    CHECK(parse("bank 1").verb == Verb::BAD_ARG);
    CHECK(parse("bank on 1").verb == Verb::BAD_ARG);
    CHECK(parse("bank 1 on now").verb == Verb::BAD_ARG);

    CHECK(parse("i2c scan").verb == Verb::I2C_SCAN);
    CHECK(parse("i2c").verb == Verb::BAD_ARG);
    CHECK(parse("i2c probe").verb == Verb::BAD_ARG);
    CHECK(parse("mcp test").verb == Verb::MCP_TEST);
    CHECK(parse("mcp").verb == Verb::MCP);
    CHECK(parse("mcp exam").verb == Verb::BAD_ARG);
    CHECK(parse("darklit").verb == Verb::DARKLIT);
    CHECK(parse("darklit 1").verb == Verb::BAD_ARG);
}

static void test_unknown() {
    g_case = "anything else is unknown, including over-long words";
    CHECK(parse("reboot").verb == Verb::UNKNOWN);
    CHECK(parse("statusstatusstatus").verb == Verb::UNKNOWN);
    CHECK(parse("1").verb == Verb::UNKNOWN);
}

int main() {
    std::printf("console_line tests\n");
    test_line_endings();
    test_blank_lines_are_ignored();
    test_backspace_edits();
    test_control_characters_are_dropped();
    test_overflow_discards_whole_line();
    test_simple_verbs();
    test_simple_verbs_reject_arguments();
    test_watch();
    test_events();
    test_maintenance_commands();
    test_unknown();

    if (g_failures == 0) {
        std::printf("all tests passed\n");
        return EXIT_SUCCESS;
    }
    std::printf("%d check(s) failed\n", g_failures);
    return EXIT_FAILURE;
}
