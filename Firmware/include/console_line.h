// ============================================================================
// console_line.h — line input and command parsing for the USB console
// ============================================================================
//
// Pure logic, host-tested in test/test_console_line/. src/diag_console.cpp
// feeds it bytes from the USB CDC port and acts on the parsed commands.
//
// The console is line-based ("watch 5", "events on") rather than the old
// single-key IR_DEBUG keys, because it now ships in every image: a stray
// keypress in a terminal attached to a counter on a hive should not be able to
// do anything. A line is only acted on once Enter arrives, and an unknown or
// malformed line is rejected outright rather than guessed at.
//
// Terminals do not agree on line endings. CR, LF and CRLF all end a line, and
// an empty line (the LF of a CRLF pair) is ignored. Backspace and DEL edit the
// line being typed. A line longer than MAX_LINE is discarded whole, never
// truncated into some other valid command.
// ============================================================================

#pragma once

#include <stdint.h>
#include <string.h>

namespace consoleline {

// Longest accepted command, excluding the terminator. The longest command is
// well under 20 characters; this is room for typos, not for payloads.
constexpr uint8_t MAX_LINE = 48;

// What one byte of input completed, if anything.
enum class Feed : uint8_t {
    NONE,       // byte absorbed, no line yet
    LINE,       // a complete line is in reader.line
    OVERFLOW,   // a line ended that was too long; it was discarded
};

struct Reader {
    char    line[MAX_LINE + 1] = {0};
    uint8_t len = 0;
    bool    overflowed = false;   // current line has exceeded MAX_LINE
};

inline Feed feed(Reader& r, char c) {
    if (c == '\r' || c == '\n') {
        if (r.overflowed) {
            r.overflowed = false;
            r.len = 0;
            r.line[0] = '\0';
            return Feed::OVERFLOW;
        }
        if (r.len == 0) return Feed::NONE;   // blank line / LF of CRLF
        r.line[r.len] = '\0';
        r.len = 0;
        return Feed::LINE;
    }
    if (c == '\b' || c == 0x7F) {
        if (!r.overflowed && r.len > 0) r.len--;
        return Feed::NONE;
    }
    if (r.overflowed) return Feed::NONE;
    // Control characters other than the ones above are noise (a terminal's
    // escape sequences, a stray Ctrl-key); drop them rather than store them.
    if ((unsigned char)c < 0x20) return Feed::NONE;
    if (r.len >= MAX_LINE) {
        r.overflowed = true;
        return Feed::NONE;
    }
    r.line[r.len++] = c;
    return Feed::NONE;
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
enum class Verb : uint8_t {
    NONE,       // empty line
    HELP,
    STATUS,
    SENSORS,
    MCP,
    WATCH,      // arg = rate in Hz, 0 = off
    EVENTS,     // arg = 1 on, 0 off
    // Maintenance (see maint_state.h). MAINT itself is always accepted; the
    // rest are refused by the console unless a session is active.
    MAINT,      // arg = MAINT_OFF / MAINT_ON / MAINT_SHOW
    LEDS,       // arg = LEDS_AUTO / LEDS_ON / LEDS_OFF
    BANK,       // arg = bank 1..3, arg2 = 1 on / 0 off
    I2C_SCAN,
    MCP_TEST,
    DARKLIT,
    UNKNOWN,    // first word not recognised
    BAD_ARG,    // recognised verb, unusable argument
};

constexpr uint32_t MAINT_OFF  = 0;
constexpr uint32_t MAINT_ON   = 1;
constexpr uint32_t MAINT_SHOW = 2;

constexpr uint32_t LEDS_AUTO = 0;
constexpr uint32_t LEDS_ON   = 1;
constexpr uint32_t LEDS_OFF  = 2;

// Highest rate `watch` accepts. Each watch line is ~100 bytes, so 10 Hz is
// ~1 KB/s — trivial for USB, and already faster than anyone can read.
constexpr uint32_t WATCH_MAX_HZ = 10;
// Rate used by a bare `watch`.
constexpr uint32_t WATCH_DEFAULT_HZ = 5;

// Banks a `bank` command may name.
constexpr uint32_t BANK_MAX = 3;

struct Command {
    Verb     verb = Verb::NONE;
    uint32_t arg = 0;
    uint32_t arg2 = 0;
};

// Split off the next space-separated word of `*p`, lower-casing it into `out`.
// Returns false if there is no further word, or it does not fit `cap`.
inline bool nextWord(const char*& p, char* out, unsigned cap) {
    while (*p == ' ' || *p == '\t') p++;
    if (*p == '\0') return false;
    unsigned n = 0;
    while (*p != '\0' && *p != ' ' && *p != '\t') {
        if (n + 1 >= cap) return false;
        char c = *p++;
        if (c >= 'A' && c <= 'Z') c = (char)(c - 'A' + 'a');
        out[n++] = c;
    }
    out[n] = '\0';
    return true;
}

// Parse a small decimal number. Rejects empty strings, signs, non-digits and
// anything past `max`.
inline bool parseUint(const char* s, uint32_t max, uint32_t& out) {
    if (*s == '\0') return false;
    uint32_t v = 0;
    for (; *s; s++) {
        if (*s < '0' || *s > '9') return false;
        v = v * 10 + (uint32_t)(*s - '0');
        if (v > max) return false;
    }
    out = v;
    return true;
}

// on/off -> 1/0.
inline bool parseOnOff(const char* s, uint32_t& out) {
    if (strcmp(s, "on") == 0)  { out = 1; return true; }
    if (strcmp(s, "off") == 0) { out = 0; return true; }
    return false;
}

constexpr unsigned MAX_WORDS = 3;
constexpr unsigned WORD_CAP = 12;

inline Command parse(const char* line) {
    Command c;
    char w[MAX_WORDS][WORD_CAP];
    unsigned n = 0;
    const char* p = line;
    bool too_much = false;
    while (true) {
        char word[WORD_CAP];
        if (!nextWord(p, word, sizeof(word))) {
            // Out of words, or one too long for any verb or argument (which
            // leaves input behind).
            while (*p == ' ' || *p == '\t') p++;
            if (*p != '\0') too_much = true;
            break;
        }
        if (n == MAX_WORDS) {
            too_much = true;
            break;
        }
        memcpy(w[n++], word, sizeof(word));
    }
    if (n == 0) {
        c.verb = too_much ? Verb::UNKNOWN : Verb::NONE;
        return c;
    }
    const char* verb = w[0];
    const unsigned nargs = n - 1;

    // Verb known, argument shape wrong.
    auto bad = [&]() { c.verb = Verb::BAD_ARG; return c; };

    if (strcmp(verb, "help") == 0 || strcmp(verb, "?") == 0 ||
        strcmp(verb, "h") == 0) {
        if (too_much || nargs) return bad();
        c.verb = Verb::HELP;
    } else if (strcmp(verb, "status") == 0) {
        if (too_much || nargs) return bad();
        c.verb = Verb::STATUS;
    } else if (strcmp(verb, "sensors") == 0) {
        if (too_much || nargs) return bad();
        c.verb = Verb::SENSORS;
    } else if (strcmp(verb, "mcp") == 0) {
        if (too_much || nargs > 1) return bad();
        if (nargs == 0) {
            c.verb = Verb::MCP;
        } else if (strcmp(w[1], "test") == 0) {
            c.verb = Verb::MCP_TEST;
        } else {
            return bad();
        }
    } else if (strcmp(verb, "watch") == 0) {
        if (too_much || nargs > 1) return bad();
        if (nargs == 0) {
            c.verb = Verb::WATCH;
            c.arg = WATCH_DEFAULT_HZ;
        } else if (strcmp(w[1], "off") == 0) {
            c.verb = Verb::WATCH;
            c.arg = 0;
        } else if (parseUint(w[1], WATCH_MAX_HZ, c.arg)) {
            c.verb = Verb::WATCH;
        } else {
            return bad();
        }
    } else if (strcmp(verb, "events") == 0) {
        if (too_much || nargs != 1 || !parseOnOff(w[1], c.arg)) return bad();
        c.verb = Verb::EVENTS;
    } else if (strcmp(verb, "maint") == 0) {
        if (too_much || nargs > 1) return bad();
        if (nargs == 0) {
            c.arg = MAINT_SHOW;
        } else if (!parseOnOff(w[1], c.arg)) {
            return bad();
        }
        c.verb = Verb::MAINT;
    } else if (strcmp(verb, "leds") == 0) {
        if (too_much || nargs != 1) return bad();
        if (strcmp(w[1], "auto") == 0) {
            c.arg = LEDS_AUTO;
        } else if (strcmp(w[1], "on") == 0) {
            c.arg = LEDS_ON;
        } else if (strcmp(w[1], "off") == 0) {
            c.arg = LEDS_OFF;
        } else {
            return bad();
        }
        c.verb = Verb::LEDS;
    } else if (strcmp(verb, "bank") == 0) {
        if (too_much || nargs != 2 || !parseUint(w[1], BANK_MAX, c.arg) ||
            c.arg == 0 || !parseOnOff(w[2], c.arg2)) {
            return bad();
        }
        c.verb = Verb::BANK;
    } else if (strcmp(verb, "i2c") == 0) {
        if (too_much || nargs != 1 || strcmp(w[1], "scan") != 0) return bad();
        c.verb = Verb::I2C_SCAN;
    } else if (strcmp(verb, "darklit") == 0) {
        if (too_much || nargs) return bad();
        c.verb = Verb::DARKLIT;
    } else {
        c.verb = Verb::UNKNOWN;
    }
    return c;
}

}  // namespace consoleline
