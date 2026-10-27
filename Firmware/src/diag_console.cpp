// USB diagnostic console. See include/diag_console.h for what it promises, and
// include/console_line.h for the command grammar.
#include "diag_console.h"

#include <Arduino.h>

#include "console_line.h"
#include "diag.h"
#include "diag_format.h"
#include "log_line.h"
#include "serial_log.h"
#include "version.h"
#include "counter_protocol.h"

namespace console {
namespace {

// Most console lines emitted per loop() pass. Each is a short snprintf plus a
// copy into the USB ring, a few tens of microseconds; four keeps a pass well
// inside the 5 ms poll interval even with a report, watch and events all
// active at once.
constexpr unsigned MAX_LINES_PER_PASS = 4;

// Most input bytes consumed per pass. A pasted command is a few dozen bytes;
// anything beyond this simply waits for the next pass.
constexpr unsigned MAX_INPUT_PER_PASS = 64;

constexpr uint8_t MAX_GATES = 24;

consoleline::Reader g_reader;

// Multi-line reports are snapshotted when the command arrives and then paced
// out line by line, so a report describes one instant even though it takes
// several loop passes to print.
enum class Report : uint8_t {
    NONE, HELP, STATUS, SENSORS, MCP, MAINT, I2C_SCAN, MCP_TEST, DARKLIT,
};
Report   g_report = Report::NONE;
unsigned g_report_line = 0;
uint32_t g_report_t_ms = 0;
diag::StatusView g_status;
diag::GateView   g_gates[MAX_GATES];
diag::ChipView   g_chips[diag::NUM_CHIPS];
uint8_t  g_num_gates = 0;
diag::MaintView   g_maint;
diag::I2cScanView g_scan;
diag::McpTestView g_mcp_test[diag::NUM_CHIPS];
diag::DarkLitView g_darklit[MAX_GATES];

uint32_t g_watch_period_ms = 0;   // 0 = off
uint32_t g_watch_next_ms = 0;

bool g_events_on = false;
eventring::Ring<256> g_events;
uint32_t g_events_dropped_reported = 0;

// Room for one more whole line right now?
bool roomForLine() {
    return slog::writable() >= (int)logline::MAX_LINE;
}

const char* const HELP_LINES[] = {
    "[HELP] commands (type one, then Enter):",
    "[HELP]   status          firmware, totals, banks, night/OTA state, poll timing",
    "[HELP]   sensors         latest reading of every gate: raw/debounced, gate state",
    "[HELP]   mcp             MCP23017 health and read statistics",
    "[HELP]   watch [1-10]    compact live view at N Hz (default 5); 'watch off' stops",
    "[HELP]   events on|off   stream sensor edges and counted crossings as they happen",
    "[HELP]   help            this list",
    "[HELP] glyphs: . clear  # blocked  - bank off  ? chip unreadable  ~ not sampling",
    "[HELP] The commands above only report what the sampler already read; they",
    "[HELP] never read a sensor, light an emitter or touch the totals.",
    "[HELP] maintenance (freezes the totals; crossings go to test counters):",
    "[HELP]   maint on|off    start / end a session (ends itself after 30 min idle)",
    "[HELP]   maint           session state, test counters, what it will restore",
    "[HELP]   leds auto|on|off   emitters pulsed (normal), steady on, or dark",
    "[HELP]   bank 1-3 on|off    switch one emitter bank",
    "[HELP]   darklit         read every gate once dark, once lit: reflection test",
    "[HELP]   mcp test        MCP23017 register check + 100-read burst per chip",
    "[HELP]   i2c scan        list every address that answers on the bus",
};
constexpr unsigned NUM_HELP_LINES = sizeof(HELP_LINES) / sizeof(HELP_LINES[0]);

void startReport(Report r, uint32_t now_ms) {
    g_report = r;
    g_report_line = 0;
    g_report_t_ms = now_ms;
    switch (r) {
    case Report::STATUS:
        diag::statusView(g_status);
        break;
    case Report::SENSORS:
        g_num_gates = diag::numGates();
        if (g_num_gates > MAX_GATES) g_num_gates = MAX_GATES;
        for (uint8_t i = 0; i < g_num_gates; i++) diag::gateView(i, g_gates[i]);
        break;
    case Report::MCP:
        for (uint8_t i = 0; i < diag::NUM_CHIPS; i++) diag::chipView(i, g_chips[i]);
        break;
    case Report::MAINT:
        diag::maintView(g_maint);
        break;
    default:
        break;
    }
}

// Emit line `n` of the active report. Returns false once the report is done.
bool emitReportLine(unsigned n) {
    switch (g_report) {
    case Report::HELP:
        if (n >= NUM_HELP_LINES) return false;
        slog::printf("%s", HELP_LINES[n]);
        return true;

    case Report::STATUS: {
        const diag::StatusView& s = g_status;
        switch (n) {
        case 0:
            slog::printf(
                "[STATUS] fw=%s proto=%u uptime=%lus in=%lu out=%lu "
                "glitches=%lu status=0x%02X banks=0x%02X",
                HIVETRAFFIC_FW_VERSION,
                (unsigned)beecounter_proto::PROTOCOL_VERSION,
                (unsigned long)s.uptime_s, (unsigned long)s.total_in,
                (unsigned long)s.total_out, (unsigned long)s.glitches,
                (unsigned)s.status_flags, (unsigned)s.bank_mask);
            return true;
        case 1:
            slog::printf("[STATUS] sensing=%s night_idle=%lus ota=%s maint=%s",
                         s.sensing ? "yes" : "no", (unsigned long)s.idle_s,
                         s.ota_active ? "active" : "no", s.maint ? "on" : "off");
            return true;
        case 2:
            slog::printf(
                "[STATUS] boot: polls=%lu late=%lu max_gap=%luus max_poll=%luus "
                "(interval %lums, late > %luus)",
                (unsigned long)s.polls, (unsigned long)s.late,
                (unsigned long)s.max_gap_us, (unsigned long)s.max_poll_us,
                (unsigned long)s.poll_interval_ms,
                (unsigned long)s.late_after_us);
            return true;
        case 3:
            slog::printf(
                "[STATUS] window: polls=%lu late=%lu max_gap=%luus max_poll=%luus",
                (unsigned long)s.window_polls, (unsigned long)s.window_late,
                (unsigned long)s.window_max_gap_us,
                (unsigned long)s.window_max_poll_us);
            return true;
        case 4:
            slog::printf(
                "[STATUS] log_dropped=%lu events_dropped=%lu watch=%luHz events=%s",
                (unsigned long)slog::droppedLines(),
                (unsigned long)g_events.dropped,
                (unsigned long)(g_watch_period_ms ? 1000 / g_watch_period_ms : 0),
                g_events_on ? "on" : "off");
            return true;
        default:
            return false;
        }
    }

    case Report::SENSORS: {
        if (n == 0) {
            slog::printf("[SENSORS] t=%lums  inner/outer = raw/debounced",
                         (unsigned long)g_report_t_ms);
            return true;
        }
        const unsigned i = n - 1;
        if (i >= g_num_gates) return false;
        const diag::GateView& g = g_gates[i];
        slog::printf(
            "[GATE] %s bank=%u inner=%s/%s outer=%s/%s state=%s", g.tag,
            (unsigned)g.bank, diagfmt::word(g.inner_raw),
            g.inner_debounced ? "BLOCK" : "clear", diagfmt::word(g.outer_raw),
            g.outer_debounced ? "BLOCK" : "clear", diagfmt::stateName(g.state));
        return true;
    }

    case Report::MCP: {
        if (n >= diag::NUM_CHIPS) return false;
        const diag::ChipView& c = g_chips[n];
        char age[16];
        if (c.last_ok_age_ms == UINT32_MAX) {
            snprintf(age, sizeof(age), "never");
        } else {
            snprintf(age, sizeof(age), "%lums", (unsigned long)c.last_ok_age_ms);
        }
        slog::printf(
            "[MCP] %s addr=0x%02X healthy=%u last_read=%s streak=%u "
            "word=0x%04X last_ok=%s reads=%lu nack=%lu short=%lu lost=%lu "
            "recovered=%lu max_read=%luus",
            c.name, (unsigned)c.addr, c.healthy ? 1u : 0u,
            c.last_read_ok ? "ok" : "FAIL", (unsigned)c.fail_streak,
            (unsigned)c.last_word, age, (unsigned long)c.reads_ok,
            (unsigned long)c.nacks, (unsigned long)c.short_reads,
            (unsigned long)c.lost, (unsigned long)c.recovered,
            (unsigned long)c.max_read_us);
        return true;
    }

    case Report::MAINT: {
        const diag::MaintView& m = g_maint;
        switch (n) {
        case 0:
            if (!m.active) {
                slog::printf("[MAINT] off; lifetime maintenance %lus",
                             (unsigned long)m.total_s);
                return true;
            }
            slog::printf("[MAINT] on for %lus, ends itself in %lus without a command",
                         (unsigned long)m.session_s, (unsigned long)m.timeout_s);
            return true;
        case 1:
            if (!m.active) return false;
            slog::printf("[MAINT] test in=%lu out=%lu glitches=%lu (totals frozen)",
                         (unsigned long)m.test_in, (unsigned long)m.test_out,
                         (unsigned long)m.test_glitch);
            return true;
        case 2:
            if (!m.active) return false;
            slog::printf("[MAINT] leds=%s banks=0x%02X; on exit: banks 0x%02X, "
                         "night mode %lus",
                         diagfmt::ledName(m.leds), (unsigned)m.bank_mask,
                         (unsigned)m.restore_mask,
                         (unsigned long)m.restore_idle_s);
            return true;
        default:
            return false;
        }
    }

    case Report::I2C_SCAN: {
        if (n == 0) {
            slog::printf("[I2C] %u device(s) answered (expected 0x20 0x21 0x22)",
                         (unsigned)g_scan.count);
            return true;
        }
        const unsigned i = n - 1;
        const unsigned listed =
            g_scan.count < sizeof(g_scan.addrs) ? g_scan.count : sizeof(g_scan.addrs);
        if (i >= listed) return false;
        slog::printf("[I2C] ack at 0x%02X", (unsigned)g_scan.addrs[i]);
        return true;
    }

    case Report::MCP_TEST: {
        // Two lines per chip: registers, then the read burst.
        const unsigned chip = n / 2;
        if (chip >= diag::NUM_CHIPS) return false;
        const diag::McpTestView& t = g_mcp_test[chip];
        if (n % 2 == 0) {
            if (!t.present) {
                slog::printf("[MCPTEST] %s 0x%02X: FAIL no ACK on its address",
                             t.name, (unsigned)t.addr);
            } else if (!t.regs_read) {
                slog::printf("[MCPTEST] %s 0x%02X: FAIL register read failed",
                             t.name, (unsigned)t.addr);
            } else {
                slog::printf(
                    "[MCPTEST] %s 0x%02X: config %s iodir=%02X/%02X ipol=%02X/%02X "
                    "gppu=%02X/%02X iocon=%02X",
                    t.name, (unsigned)t.addr, t.config_ok ? "ok" : "FAIL",
                    (unsigned)t.iodir_a, (unsigned)t.iodir_b, (unsigned)t.ipol_a,
                    (unsigned)t.ipol_b, (unsigned)t.gppu_a, (unsigned)t.gppu_b,
                    (unsigned)t.iocon);
            }
            return true;
        }
        if (!t.present) return true;   // nothing to say; keep the pairing
        slog::printf("[MCPTEST] %s burst %s: %u/%u ok nack=%u short=%u max=%luus",
                     t.name, t.burst_ok == t.burst ? "ok" : "FAIL",
                     (unsigned)t.burst_ok, (unsigned)t.burst, (unsigned)t.burst_nack,
                     (unsigned)t.burst_short, (unsigned long)t.burst_max_us);
        return true;
    }

    case Report::DARKLIT: {
        if (n == 0) {
            slog::printf("[DARKLIT] inner/outer = dark/lit -> verdict "
                         "(reflects / none / ambient / inverted)");
            return true;
        }
        const unsigned i = n - 1;
        if (i >= g_num_gates) return false;
        const diag::DarkLitView& d = g_darklit[i];
        slog::printf(
            "[DARKLIT] %s bank=%u inner=%s/%s->%s outer=%s/%s->%s", d.tag,
            (unsigned)d.bank, diagfmt::word(d.inner_dark),
            diagfmt::word(d.inner_lit),
            diagfmt::darkLitVerdict(d.inner_dark, d.inner_lit),
            diagfmt::word(d.outer_dark), diagfmt::word(d.outer_lit),
            diagfmt::darkLitVerdict(d.outer_dark, d.outer_lit));
        return true;
    }

    case Report::NONE:
    default:
        return false;
    }
}

// Word a maintenance command's outcome. Returns true if it succeeded.
bool reportResult(const char* what, diag::Result r) {
    if (r == diag::Result::OK) return true;
    slog::printf("[CON] %s: %s", what, diagfmt::resultText(r));
    return false;
}

void emitWatchLine(uint32_t now_ms) {
    diag::GateView gates[MAX_GATES];
    uint8_t n = diag::numGates();
    if (n > MAX_GATES) n = MAX_GATES;
    for (uint8_t i = 0; i < n; i++) diag::gateView(i, gates[i]);
    diag::StatusView s;
    diag::statusView(s);

    char inner[MAX_GATES + MAX_GATES / 8 + 1];
    char outer[MAX_GATES + MAX_GATES / 8 + 1];
    diagfmt::sensorRow(inner, sizeof(inner), gates, n, true);
    diagfmt::sensorRow(outer, sizeof(outer), gates, n, false);
    slog::printf("[W] t=%lu.%03lu in=%lu out=%lu inner=%s outer=%s",
                 (unsigned long)(now_ms / 1000), (unsigned long)(now_ms % 1000),
                 (unsigned long)s.total_in, (unsigned long)s.total_out, inner,
                 outer);
}

void handleLine(const char* line, uint32_t now_ms) {
    const consoleline::Command c = consoleline::parse(line);
    // Echo what was understood, so a terminal without local echo still shows
    // which command produced the output that follows.
    slog::printf("[CON] > %s", line);
    // Any line, valid or not, is someone at the terminal: a running
    // maintenance session is still in use.
    diag::maintTouch();

    switch (c.verb) {
    case consoleline::Verb::NONE:
        break;
    case consoleline::Verb::HELP:
        startReport(Report::HELP, now_ms);
        break;
    case consoleline::Verb::STATUS:
        startReport(Report::STATUS, now_ms);
        break;
    case consoleline::Verb::SENSORS:
        startReport(Report::SENSORS, now_ms);
        break;
    case consoleline::Verb::MCP:
        startReport(Report::MCP, now_ms);
        break;
    case consoleline::Verb::WATCH:
        if (c.arg == 0) {
            g_watch_period_ms = 0;
            slog::printf("[CON] watch off");
        } else {
            g_watch_period_ms = 1000 / c.arg;
            g_watch_next_ms = now_ms;
            slog::printf("[CON] watch %luHz", (unsigned long)c.arg);
        }
        break;
    case consoleline::Verb::EVENTS:
        g_events_on = c.arg != 0;
        // Either way, start clean: nothing queued before `events on` belongs
        // to this stream, and nothing queued should trickle out after `off`.
        g_events.clear();
        g_events_dropped_reported = g_events.dropped;
        slog::printf("[CON] events %s (next seq=%lu)", g_events_on ? "on" : "off",
                     (unsigned long)g_events.next_seq);
        break;
    case consoleline::Verb::MAINT:
        if (c.arg == consoleline::MAINT_ON) {
            if (reportResult("maint on", diag::maintEnter())) {
                startReport(Report::MAINT, now_ms);
            }
        } else if (c.arg == consoleline::MAINT_OFF) {
            reportResult("maint off", diag::maintExit());
        } else {
            startReport(Report::MAINT, now_ms);
        }
        break;
    case consoleline::Verb::LEDS: {
        const diag::LedSetting l =
            c.arg == consoleline::LEDS_ON  ? diag::LedSetting::ON
          : c.arg == consoleline::LEDS_OFF ? diag::LedSetting::OFF
                                           : diag::LedSetting::AUTO;
        if (reportResult("leds", diag::setLeds(l))) {
            slog::printf("[CON] leds %s", diagfmt::ledName(l));
        }
        break;
    }
    case consoleline::Verb::BANK: {
        const diag::Result r = diag::setBank((uint8_t)c.arg, c.arg2 != 0);
        if (r == diag::Result::REFUSED) {
            slog::printf("[CON] bank: refused - at least one bank must stay on");
        } else if (reportResult("bank", r)) {
            slog::printf("[CON] bank %lu %s", (unsigned long)c.arg,
                         c.arg2 ? "on" : "off");
        }
        break;
    }
    case consoleline::Verb::I2C_SCAN:
        if (reportResult("i2c scan", diag::i2cScan(g_scan))) {
            startReport(Report::I2C_SCAN, now_ms);
        }
        break;
    case consoleline::Verb::MCP_TEST:
        if (reportResult("mcp test", diag::mcpTest(g_mcp_test))) {
            startReport(Report::MCP_TEST, now_ms);
        }
        break;
    case consoleline::Verb::DARKLIT:
        g_num_gates = diag::numGates();
        if (g_num_gates > MAX_GATES) g_num_gates = MAX_GATES;
        if (reportResult("darklit", diag::darkLit(g_darklit, g_num_gates))) {
            startReport(Report::DARKLIT, now_ms);
        }
        break;
    case consoleline::Verb::BAD_ARG:
        slog::printf("[CON] error: bad argument - type 'help'");
        break;
    case consoleline::Verb::UNKNOWN:
    default:
        slog::printf("[CON] error: unknown command - type 'help'");
        break;
    }
}

}  // namespace

void begin() {
    slog::printf("[CON] USB console ready - type 'help' and Enter");
}

bool eventsEnabled() {
    return g_events_on;
}

void noteEvent(uint32_t t_ms, uint8_t gate, eventring::Kind kind) {
    if (!g_events_on) return;
    g_events.push(t_ms, gate, kind);
}

void poll(uint32_t now_ms) {
    // ---- input ----
    for (unsigned i = 0; i < MAX_INPUT_PER_PASS && Serial.available() > 0; i++) {
        const int b = Serial.read();
        if (b < 0) break;
        switch (consoleline::feed(g_reader, (char)b)) {
        case consoleline::Feed::LINE:
            handleLine(g_reader.line, now_ms);
            break;
        case consoleline::Feed::OVERFLOW:
            slog::printf("[CON] error: line too long - ignored");
            break;
        case consoleline::Feed::NONE:
        default:
            break;
        }
    }

    // ---- output, paced ----
    unsigned budget = MAX_LINES_PER_PASS;

    while (budget > 0 && g_report != Report::NONE && roomForLine()) {
        if (emitReportLine(g_report_line)) {
            g_report_line++;
            budget--;
        } else {
            g_report = Report::NONE;
        }
    }

    if (budget > 0 && g_watch_period_ms != 0 &&
        (int32_t)(now_ms - g_watch_next_ms) >= 0 && roomForLine()) {
        emitWatchLine(now_ms);
        budget--;
        g_watch_next_ms += g_watch_period_ms;
        // After a stall (host not reading), resynchronise rather than emit a
        // burst of catch-up lines describing the same instant.
        if ((int32_t)(now_ms - g_watch_next_ms) >= 0) {
            g_watch_next_ms = now_ms + g_watch_period_ms;
        }
    }

    eventring::Event e;
    while (g_events_on && budget > 0 && roomForLine() && g_events.pop(e)) {
        diag::GateView g;
        diag::gateView(e.gate, g);
        slog::printf("[EV] seq=%lu t=%lu gate=%s ev=%s", (unsigned long)e.seq,
                     (unsigned long)e.t_ms, g.tag, eventring::kindName(e.kind));
        budget--;
    }

    // Losses are summarised once the queue has drained. The exact place they
    // happened is the jump in seq, which the reader sees regardless.
    if (g_events_on && budget > 0 && g_events.empty() && roomForLine() &&
        g_events.dropped != g_events_dropped_reported) {
        slog::printf("[EV] dropped=%lu (queue full; see the seq gap)",
                     (unsigned long)(g_events.dropped - g_events_dropped_reported));
        g_events_dropped_reported = g_events.dropped;
    }
}

}  // namespace console
