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
    UNKNOWN,    // first word not recognised
    BAD_ARG,    // recognised verb, unusable argument
};

// Highest rate `watch` accepts. Each watch line is ~100 bytes, so 10 Hz is
// ~1 KB/s — trivial for USB, and already faster than anyone can read.
constexpr uint32_t WATCH_MAX_HZ = 10;
// Rate used by a bare `watch`.
constexpr uint32_t WATCH_DEFAULT_HZ = 5;

struct Command {
    Verb     verb = Verb::NONE;
    uint32_t arg = 0;
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

inline Command parse(const char* line) {
    Command c;
    const char* p = line;
    char verb[12];
    char arg[12];
    if (!nextWord(p, verb, sizeof(verb))) {
        // Either blank, or a first word too long to be any verb.
        while (*line == ' ' || *line == '\t') line++;
        c.verb = (*line == '\0') ? Verb::NONE : Verb::UNKNOWN;
        return c;
    }
    const bool has_arg = nextWord(p, arg, sizeof(arg));
    char extra[2];
    const bool has_extra = has_arg && nextWord(p, extra, sizeof(extra));
    // A word too long for `arg` also lands here: nextWord() returned false
    // with input left over.
    while (*p == ' ' || *p == '\t') p++;
    const bool leftover = has_extra || *p != '\0';

    auto noArgs = [&](Verb v) {
        c.verb = (has_arg || leftover) ? Verb::BAD_ARG : v;
    };

    if (strcmp(verb, "help") == 0 || strcmp(verb, "?") == 0 ||
        strcmp(verb, "h") == 0) {
        noArgs(Verb::HELP);
    } else if (strcmp(verb, "status") == 0) {
        noArgs(Verb::STATUS);
    } else if (strcmp(verb, "sensors") == 0) {
        noArgs(Verb::SENSORS);
    } else if (strcmp(verb, "mcp") == 0) {
        noArgs(Verb::MCP);
    } else if (strcmp(verb, "watch") == 0) {
        if (leftover) {
            c.verb = Verb::BAD_ARG;
        } else if (!has_arg) {
            c.verb = Verb::WATCH;
            c.arg = WATCH_DEFAULT_HZ;
        } else if (strcmp(arg, "off") == 0) {
            c.verb = Verb::WATCH;
            c.arg = 0;
        } else if (parseUint(arg, WATCH_MAX_HZ, c.arg)) {
            c.verb = Verb::WATCH;
        } else {
            c.verb = Verb::BAD_ARG;
        }
    } else if (strcmp(verb, "events") == 0) {
        if (!has_arg || leftover) {
            c.verb = Verb::BAD_ARG;
        } else if (strcmp(arg, "on") == 0) {
            c.verb = Verb::EVENTS;
            c.arg = 1;
        } else if (strcmp(arg, "off") == 0) {
            c.verb = Verb::EVENTS;
            c.arg = 0;
        } else {
            c.verb = Verb::BAD_ARG;
        }
    } else {
        c.verb = Verb::UNKNOWN;
    }
    return c;
}

}  // namespace consoleline
