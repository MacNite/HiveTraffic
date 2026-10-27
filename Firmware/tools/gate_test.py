#!/usr/bin/env python3
"""Guided bench test of a HiveTraffic counter's 24 gates over USB.

Drives the firmware's USB diagnostic console (see "USB diagnostic console" and
"Maintenance mode" in Firmware/README.md) and walks a person through every
gate, one check at a time, then prints a PASS / FAIL / NOT TESTED report.

    pip install pyserial
    python3 Firmware/tools/gate_test.py --port /dev/ttyACM0
    python3 Firmware/tools/gate_test.py --port COM5 --gates 00-07 --csv u2.csv

What it checks
--------------
Once, before the gates:

  * ``i2c scan``  -- exactly the three MCP23017s answer (0x20, 0x21, 0x22);
  * ``mcp test``  -- each chip's configuration registers read back as the
                     firmware set them, and 100 back-to-back reads all succeed;
  * ``darklit`` with every tunnel EMPTY -- every sensor reads clear both dark
    and lit ("none"). "ambient" means sunlight or an input stuck low;
    "reflects" in an empty tunnel means something in it reflects, or
    crosstalk from a neighbour.

Then for each gate, with a target (a strip of white paper is fine):

  1. inner sensor: target over it, then away  -> inner_block, inner_clear
  2. outer sensor: the same                    -> outer_block, outer_clear
  3. dark/lit with the target over both        -> "reflects" on both: the
     response comes from the gate's own emitter, not from ambient light
  4. target passed outside -> inside           -> bee_in on this gate
  5. target passed inside -> outside           -> bee_out on this gate

Any check can be skipped (NOT TESTED). Events seen on OTHER gates during a
check are reported as crosstalk notes. A direction check passes only on exactly
one crossing, and a check whose event stream had a gap (dropped events) fails:
what it did not see could have been the wrong answer.

Everything runs inside a maintenance session (``maint on``), so the counter's
lifetime totals are untouched -- test crossings go to its test counters -- and
the session is ended (``maint off``) on exit, including on Ctrl-C. The firmware
ends a session by itself after 30 minutes without a console command, and a
person reading a prompt sends none, so the session is checked (and if need be
restarted) after every prompt, before the action; a check during which the
session ended anyway fails, because its crossings may have reached the
lifetime totals. Run it with the housing fitted: reflections and neighbour
crosstalk depend on it.

The exit status is 0 only if nothing FAILed -- the chip checks included.

The script only types console commands; it cannot do anything the console
does not allow. A firmware without the maintenance commands (before 0.5.0)
is refused at the start.
"""

import argparse
import csv
import queue
import re
import sys
import threading
import time

# ---------------------------------------------------------------------------
# Gate topology (mirrors gates::TABLE in Firmware/include/pins.h)
# ---------------------------------------------------------------------------

GATE_NUMBERS = [n for chip in (0, 10, 20) for n in range(chip, chip + 8)]
GATE_TAGS = ["GATE_%02d" % n for n in GATE_NUMBERS]
EXPECTED_I2C = {0x20, 0x21, 0x22}

PASS = "PASS"
FAIL = "FAIL"
NOT_TESTED = "NOT TESTED"

CHECKS = ("inner", "outer", "darklit", "in", "out")


def parse_gates(spec):
    """'00,03,10-17' -> ['GATE_00', 'GATE_03', 'GATE_10', ..., 'GATE_17'].

    Numbers are gate numbers as printed on the PCB (00-07, 10-17, 20-27).
    Raises ValueError on anything else.
    """
    if spec is None or spec.strip().lower() in ("", "all"):
        return list(GATE_TAGS)
    wanted = set()
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = part.split("-", 1)
            lo, hi = int(lo), int(hi)
            if lo > hi:
                raise ValueError("bad range %r" % part)
            nums = range(lo, hi + 1)
        else:
            nums = [int(part)]
        for n in nums:
            if n in GATE_NUMBERS:
                wanted.add(n)
            elif "-" not in part:
                raise ValueError("no gate %02d (gates are 00-07, 10-17, 20-27)" % n)
    if not wanted:
        raise ValueError("no gates selected by %r" % spec)
    return ["GATE_%02d" % n for n in GATE_NUMBERS if n in wanted]


# ---------------------------------------------------------------------------
# Console line parsing
# ---------------------------------------------------------------------------

_EV = re.compile(r"^\[EV\] seq=(\d+) t=(\d+) gate=(GATE_\d\d) ev=(\w+)")
_EV_DROPPED = re.compile(r"^\[EV\] dropped=(\d+)")
# Lines that mean the maintenance session is over: ended (timeout, OTA, request)
# or gone with a reset. Crossings after either count toward the lifetime totals.
_MAINT_ENDED = re.compile(r"^\[MAINT\] ended |firmware booting")
_DARKLIT = re.compile(
    r"^\[DARKLIT\] (GATE_\d\d) bank=(\d) "
    r"inner=(\S+)/(\S+)->(\S+) outer=(\S+)/(\S+)->(\S+)")
_MCPTEST_CFG = re.compile(r"^\[MCPTEST\] (U\d) 0x([0-9A-Fa-f]{2}): (.*)$")
_MCPTEST_BURST = re.compile(
    r"^\[MCPTEST\] (U\d) burst (ok|FAIL): (\d+)/(\d+) ok nack=(\d+) short=(\d+) max=(\d+)us")
_I2C_COUNT = re.compile(r"^\[I2C\] (\d+) device\(s\) answered")
_I2C_ACK = re.compile(r"^\[I2C\] ack at 0x([0-9A-Fa-f]{2})")


class Event(object):
    __slots__ = ("seq", "t_ms", "gate", "kind")

    def __init__(self, seq, t_ms, gate, kind):
        self.seq, self.t_ms, self.gate, self.kind = seq, t_ms, gate, kind

    def __repr__(self):
        return "Event(%d, %d, %s, %s)" % (self.seq, self.t_ms, self.gate, self.kind)


def parse_event(line):
    m = _EV.match(line)
    if not m:
        return None
    return Event(int(m.group(1)), int(m.group(2)), m.group(3), m.group(4))


def parse_darklit(line):
    """-> (gate, {'bank', 'inner_dark', 'inner_lit', 'inner', 'outer_dark', ...})"""
    m = _DARKLIT.match(line)
    if not m:
        return None
    g = m.groups()
    return g[0], {
        "bank": int(g[1]),
        "inner_dark": g[2], "inner_lit": g[3], "inner": g[4],
        "outer_dark": g[5], "outer_lit": g[6], "outer": g[7],
    }


# ---------------------------------------------------------------------------
# Check evaluation (pure: events / verdicts in, result + note out)
# ---------------------------------------------------------------------------

def others(events, gate):
    """Short description of events on gates other than `gate` (crosstalk)."""
    seen = sorted({"%s %s" % (e.gate, e.kind) for e in events if e.gate != gate})
    return ("also saw: " + ", ".join(seen)) if seen else ""


def eval_sensor(events, gate, sensor):
    """A sensor check passes on a block followed by a clear on that sensor."""
    mine = [e.kind for e in events if e.gate == gate]
    block, clear = sensor + "_block", sensor + "_clear"
    note = others(events, gate)
    if block not in mine:
        return FAIL, _join("no %s seen" % block, note)
    if clear not in mine[mine.index(block):]:
        return FAIL, _join("%s seen but it never cleared" % block, note)
    return PASS, note


def eval_direction(events, gate, expected):
    """expected is 'bee_in' or 'bee_out'. One pass must count exactly once."""
    mine = [e.kind for e in events if e.gate == gate]
    wrong = "bee_out" if expected == "bee_in" else "bee_in"
    note = others(events, gate)
    n = mine.count(expected)
    if n > 1 and wrong not in mine:
        return FAIL, _join("counted %d %s for one pass" % (n, expected), note)
    if n == 1 and wrong not in mine:
        return PASS, note
    if wrong in mine and expected not in mine:
        return FAIL, _join("counted %s: direction reversed" % wrong, note)
    if expected in mine and wrong in mine:
        return FAIL, _join("counted both directions", note)
    if "glitch" in mine:
        return FAIL, _join("only a glitch (both sensors at once, or a false start)", note)
    return FAIL, _join("no crossing counted", note)


def eval_baseline(verdicts):
    """Empty-tunnel darklit: every readable sensor must say 'none'.

    -> {gate: (result, note)}; gates whose bank is off are NOT TESTED.
    """
    out = {}
    for gate, v in verdicts.items():
        if v["inner"] == "disabled" or v["outer"] == "disabled":
            out[gate] = (NOT_TESTED, "bank off")
            continue
        bad = ["%s=%s" % (s, v[s]) for s in ("inner", "outer") if v[s] != "none"]
        out[gate] = (FAIL, "empty tunnel: " + ", ".join(bad)) if bad else (PASS, "")
    return out


def eval_darklit_target(v):
    """Target over both sensors: both must 'reflect'."""
    bad = ["%s=%s" % (s, v[s]) for s in ("inner", "outer") if v[s] != "reflects"]
    if not bad:
        return PASS, ""
    return FAIL, "with target: " + ", ".join(bad)


def judge(result, problems):
    """A check whose observation was compromised fails, whatever it saw.

    `result` is (verdict, note) from an eval_* function; `problems` lists what
    went wrong while watching (dropped events, the session ending).
    """
    if not problems:
        return result
    return FAIL, _join("; ".join(problems), result[1])


def overall(results):
    """Gate result from its individual checks."""
    vals = [r for r, _ in results.values()]
    if any(r == FAIL for r in vals):
        return FAIL
    if vals and all(r == PASS for r in vals):
        return PASS
    return NOT_TESTED


def _join(*parts):
    return "; ".join(p for p in parts if p)


# ---------------------------------------------------------------------------
# Serial link
# ---------------------------------------------------------------------------

class Link(object):
    """A line-oriented connection to the counter's console.

    A reader thread queues every received line; `send` writes one command.
    Constructed around any object with pyserial's readline()/write().
    """

    def __init__(self, port, echo=None):
        self._port = port
        self._lines = queue.Queue()
        self._echo = echo
        self._stop = False
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def _reader(self):
        while not self._stop:
            try:
                raw = self._port.readline()
            except Exception:  # port gone; let the caller time out
                time.sleep(0.1)
                continue
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").rstrip("\r\n")
            if self._echo:
                self._echo(line)
            self._lines.put(line)

    def close(self):
        self._stop = True

    def send(self, cmd):
        self._port.write((cmd + "\n").encode("ascii"))

    def drain(self):
        """Discard everything received so far."""
        while True:
            try:
                self._lines.get_nowait()
            except queue.Empty:
                return

    def next_line(self, deadline):
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return None
        try:
            return self._lines.get(timeout=remaining)
        except queue.Empty:
            return None

    def collect(self, seconds, until=None):
        """Lines received within `seconds`, stopping early once until(lines)."""
        deadline = time.monotonic() + seconds
        lines = []
        while True:
            line = self.next_line(deadline)
            if line is None:
                return lines
            lines.append(line)
            if until and until(lines):
                return lines


class ConsoleError(Exception):
    pass


# ---------------------------------------------------------------------------
# The guided test
# ---------------------------------------------------------------------------

class GateTest(object):
    # After the last event of an action, how long to keep listening before
    # moving on. Long enough for a slow hand; short enough not to drag.
    quiet_s = 2.0

    def __init__(self, link, gates, timeout_s=20.0, ask=input, say=print,
                 skip_direction=False):
        self.link = link
        self.gates = gates
        self.timeout_s = timeout_s
        self.ask = ask
        self.say = say
        self.skip_direction = skip_direction
        self._maint_started = False
        self._pending = []                      # lines seen by the last prompt()
        self.chip_results = []                  # (name, result, note)
        self.baseline = {}                      # gate -> (result, note)
        self.results = {g: {} for g in gates}   # gate -> check -> (result, note)

    # -- console helpers --------------------------------------------------

    def command(self, cmd, until, seconds=5.0):
        """Send `cmd`, return the lines after its echo up to `until`."""
        self.link.drain()
        self.link.send(cmd)
        echo = "[CON] > " + cmd
        seen_echo = [False]

        def done(lines):
            if not seen_echo[0]:
                if lines[-1] == echo:
                    seen_echo[0] = True
                    del lines[:]
                return False
            last = lines[-1]
            if last.startswith("[CON]") and ("error" in last or "refused" in last):
                raise ConsoleError("%s: %s" % (cmd, last))
            return until(lines)

        lines = self.link.collect(seconds, done)
        if not seen_echo[0]:
            raise ConsoleError("no reply to %r -- wrong port, or not a HiveTraffic counter?" % cmd)
        # The echo only says the line was read. Running out of time before the
        # reply was complete is a failure, not an answer: callers act on these
        # lines as if they were the whole reply.
        if not lines or not until(lines):
            raise ConsoleError("incomplete reply to %r (timed out after %gs)" % (cmd, seconds))
        return lines

    def ensure_maint(self):
        """Make sure a maintenance session is running.

        -> (lines, restarted): every console line seen while checking, so the
        caller can keep events that arrived meanwhile, and whether a session
        had to be started although this run already had one -- i.e. it ended
        (30 min without a command, an OTA, a reset) since the last check.

        Any command also restarts the firmware's 30-minute timer, which is why
        this runs around every prompt rather than once per gate.
        """
        seen = list(self.command("maint", lambda ls: ls[-1].startswith("[MAINT] o")))
        if seen[-1].startswith("[MAINT] on"):
            return seen, False
        seen += self.command("maint on", lambda ls: ls[-1].startswith("[MAINT] on"))
        restarted = self._maint_started
        self._maint_started = True
        if restarted:
            self.say("    (the maintenance session had ended; started a new one)")
        return seen, restarted

    def darklit(self):
        """-> {gate: verdicts} for all 24 gates."""
        found = {}

        def until(lines):
            p = parse_darklit(lines[-1])
            if p:
                found[p[0]] = p[1]
            return len(found) == len(GATE_TAGS)

        self.command("darklit", until)
        if len(found) != len(GATE_TAGS):
            raise ConsoleError("darklit returned %d of %d gates" % (len(found), len(GATE_TAGS)))
        return found

    def prompt(self, text):
        """Ask the person something, inside a maintenance session.

        -> (answer, ended): `ended` is True if the session had ended while the
        person was at the prompt. The session is refreshed before asking, and
        checked -- and restarted if need be -- right after the answer, before
        the person acts on it.
        """
        self.ensure_maint()
        answer = self.ask(text).strip().lower()
        if answer == "q":
            raise KeyboardInterrupt
        if answer == "s":
            return answer, False
        self._pending, ended = self.ensure_maint()
        return answer, ended

    def events_during(self, prompt):
        """Ask for an action; collect events for up to timeout_s afterwards.

        -> None if the person skipped, else (events, problems). `problems`
        lists what makes the window untrustworthy: dropped events (a seq gap
        or the firmware's dropped= line) or the maintenance session ending
        while watching. Stops early two seconds after the last event, so a
        quick action is not kept waiting for the full timeout.
        """
        answer, _ = self.prompt(prompt + "  [Enter = go, s = skip] ")
        if answer == "s":
            return None
        events = []
        problems = []
        last_seq = [None]

        def take(line):
            if _EV_DROPPED.match(line):
                self.say("    (warning: the counter dropped events: %s)" % line)
                _add(problems, "the counter dropped events")
            elif _MAINT_ENDED.search(line):
                _add(problems, "maintenance ended during the check, so its "
                               "crossings may have reached the lifetime totals")
            e = parse_event(line)
            if not e:
                return False
            if last_seq[0] is not None and e.seq != last_seq[0] + 1:
                _add(problems, "event stream gap (seq %d -> %d)" % (last_seq[0], e.seq))
            last_seq[0] = e.seq
            events.append(e)
            return True

        # Events can already arrive while ensure_maint() is talking to the
        # console; they belong to this action.
        for line in self._pending:
            take(line)
        self._pending = []
        deadline = time.monotonic() + self.timeout_s
        quiet_until = time.monotonic() + self.quiet_s if events else None
        self.say("    ...watching for %ds" % self.timeout_s)
        while True:
            end = deadline if quiet_until is None else min(deadline, quiet_until)
            line = self.link.next_line(end)
            if line is None:
                return events, problems
            if take(line):
                quiet_until = time.monotonic() + self.quiet_s

    # -- phases -----------------------------------------------------------

    def check_bus(self):
        self.say("\n== I2C bus and MCP23017s ==")
        lines = self.command("i2c scan", lambda ls: _i2c_done(ls))
        found = {int(m.group(1), 16) for m in (_I2C_ACK.match(l) for l in lines) if m}
        missing = sorted(EXPECTED_I2C - found)
        extra = sorted(found - EXPECTED_I2C)
        note = _join("missing " + " ".join("0x%02X" % a for a in missing) if missing else "",
                     "unexpected " + " ".join("0x%02X" % a for a in extra) if extra else "")
        self.chip_results.append(("i2c scan", FAIL if missing or extra else PASS, note))

        lines = self.command("mcp test", _mcptest_done)
        for chip in ("U2", "U3", "U4"):
            cfg = [l for l in lines if _MCPTEST_CFG.match(l) and l.split()[1] == chip]
            burst = [_MCPTEST_BURST.match(l) for l in lines]
            burst = [m for m in burst if m and m.group(1) == chip]
            if not cfg:
                self.chip_results.append((chip, FAIL, "no mcp test result"))
                continue
            text = _MCPTEST_CFG.match(cfg[0]).group(3)
            if "FAIL" in text:
                self.chip_results.append((chip, FAIL, text))
            elif not burst or burst[0].group(2) != "ok":
                detail = burst[0].group(0) if burst else "no burst result"
                self.chip_results.append((chip, FAIL, detail))
            else:
                self.chip_results.append((chip, PASS, "max read %sus" % burst[0].group(7)))
        for name, result, note in self.chip_results:
            self.say("  %-9s %-10s %s" % (name, result, note))

    def check_baseline(self):
        self.say("\n== Empty-tunnel baseline ==")
        self.prompt("Make sure every tunnel is EMPTY (no target, no bees), then press Enter ")
        self.baseline = eval_baseline({g: v for g, v in self.darklit().items()
                                       if g in self.results})
        bad = {g: r for g, r in self.baseline.items() if r[0] != PASS}
        if not bad:
            self.say("  all selected gates read clear, dark and lit")
        for g, (result, note) in sorted(bad.items()):
            self.say("  %s %s %s" % (g, result, note))

    def test_gate(self, gate):
        r = self.results[gate]
        if self.baseline.get(gate, (None, ""))[1] == "bank off":
            self.say("\n== %s: bank switched off, skipped ==" % gate)
            return
        self.say("\n== %s ==" % gate)

        for sensor in ("inner", "outer"):
            w = self.events_during(
                "%s: hold the target over the %s sensor (%s side), then take it away."
                % (gate, sensor.upper(), "hive" if sensor == "inner" else "entrance"))
            r[sensor] = ((NOT_TESTED, "skipped") if w is None
                         else judge(eval_sensor(w[0], gate, sensor), w[1]))
            self.say("    %s: %s %s" % (sensor, r[sensor][0], r[sensor][1]))

        # Here the person acts BEFORE pressing Enter, so a session that ended
        # at the prompt was already gone while the target went in or out.
        answer, ended = self.prompt("%s: hold the target over BOTH sensors and keep it "
                                    "there.  [Enter = go, s = skip] " % gate)
        if answer == "s":
            r["darklit"] = (NOT_TESTED, "skipped")
        else:
            r["darklit"] = eval_darklit_target(self.darklit()[gate])
            _, ended_after = self.prompt("%s: take the target away, then press Enter " % gate)
            ended = ended or ended_after
        if ended:
            r["darklit"] = judge(r["darklit"], [
                "maintenance had ended while the target was moved, so the "
                "movement may have reached the lifetime totals"])
        self.say("    dark/lit: %s %s" % r["darklit"])

        if self.skip_direction:
            r["in"] = r["out"] = (NOT_TESTED, "--skip-direction")
            return
        for key, prompt, expected in (
                ("in", "pass the target through from OUTSIDE to INSIDE (outer first)", "bee_in"),
                ("out", "pass the target through from INSIDE to OUTSIDE (inner first)", "bee_out")):
            w = self.events_during("%s: %s." % (gate, prompt))
            r[key] = ((NOT_TESTED, "skipped") if w is None
                      else judge(eval_direction(w[0], gate, expected), w[1]))
            self.say("    %s: %s %s" % (expected, r[key][0], r[key][1]))

    def run(self):
        self.ensure_maint()
        self.command("events on", lambda ls: ls[-1].startswith("[CON] events on"))
        self.check_bus()
        self.check_baseline()
        for gate in self.gates:
            self.test_gate(gate)

    def finish(self):
        """Leave the counter as it was. Best effort: the port may be gone."""
        # "nothing to do" is a fine answer to maint off: the session had
        # already ended on its own.
        for cmd, ok in (("events off", ("[CON] events off",)),
                        ("maint off", ("[MAINT] ended", "[CON] maint off: nothing"))):
            try:
                self.command(cmd, lambda ls, ok=ok: ls[-1].startswith(ok), seconds=3.0)
            except Exception as e:
                # Not fatal -- a session ends by itself after 30 minutes -- but
                # the person should know the counter may still be in it.
                self.say("warning: %r was not confirmed (%s)" % (cmd, e))

    # -- report -----------------------------------------------------------

    def rows(self):
        out = []
        for gate in self.gates:
            base = self.baseline.get(gate, (NOT_TESTED, "not run"))
            checks = dict(self.results[gate])
            checks["baseline"] = base
            for c in CHECKS:
                checks.setdefault(c, (NOT_TESTED, ""))
            # Every note, including a PASS's: crosstalk seen during a passing
            # check is still evidence about the neighbouring gate.
            notes = _join(*["%s: %s" % (c, checks[c][1])
                            for c in ("baseline",) + CHECKS
                            if checks[c][1]])
            out.append({
                "gate": gate,
                "baseline": base[0],
                "inner": checks["inner"][0],
                "outer": checks["outer"][0],
                "darklit": checks["darklit"][0],
                "bee_in": checks["in"][0],
                "bee_out": checks["out"][0],
                "result": overall(checks),
                "notes": notes,
            })
        return out

    def report(self):
        self.say("\n== Report ==")
        for name, result, note in self.chip_results:
            self.say("%-9s %-10s %s" % (name, result, note))
        cols = ("gate", "baseline", "inner", "outer", "darklit", "bee_in", "bee_out", "result")
        self.say("")
        self.say("  ".join("%-10s" % c for c in cols) + "  notes")
        for row in self.rows():
            self.say("  ".join("%-10s" % row[c] for c in cols) + "  " + row["notes"])
        counts = {}
        for row in self.rows():
            counts[row["result"]] = counts.get(row["result"], 0) + 1
        self.say("\n%d PASS, %d FAIL, %d NOT TESTED" % (
            counts.get(PASS, 0), counts.get(FAIL, 0), counts.get(NOT_TESTED, 0)))
        chip_fails = sum(1 for _, r, _ in self.chip_results if r == FAIL)
        if chip_fails:
            self.say("%d bus/chip check(s) FAILED" % chip_fails)

    def failed(self):
        """Did anything FAIL -- a gate or a bus/chip check?"""
        return (any(r == FAIL for _, r, _ in self.chip_results) or
                any(row["result"] == FAIL for row in self.rows()))

    def write_csv(self, path):
        """Gate rows, then one row per bus/chip check (its name in `gate`)."""
        rows = self.rows()
        fields = list(rows[0].keys()) if rows else ["gate", "result", "notes"]
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=fields, restval="")
            w.writeheader()
            for row in rows:
                w.writerow(row)
            for name, result, note in self.chip_results:
                w.writerow({"gate": name, "result": result, "notes": note})


def _add(items, item):
    if item not in items:
        items.append(item)


def _mcptest_done(lines):
    """Two lines per chip, except a chip with no ACK, which gets one."""
    done = set()
    for l in lines:
        m = _MCPTEST_CFG.match(l)
        if m and "no ACK" in m.group(3):
            done.add(m.group(1))
        m = _MCPTEST_BURST.match(l)
        if m:
            done.add(m.group(1))
    return done >= {"U2", "U3", "U4"}


def _i2c_done(lines):
    count = None
    acks = 0
    for l in lines:
        m = _I2C_COUNT.match(l)
        if m:
            count = int(m.group(1))
        elif _I2C_ACK.match(l):
            acks += 1
    return count is not None and acks >= min(count, 16)


# ---------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Guided gate test for a HiveTraffic counter over USB.")
    ap.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyACM0 or COM5")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--gates", default="all",
                    help="gates to test, e.g. '00-07,12' (default: all 24)")
    ap.add_argument("--timeout", type=float, default=20.0,
                    help="seconds to wait for each action (default 20)")
    ap.add_argument("--skip-direction", action="store_true",
                    help="skip the bee_in / bee_out pass-through checks")
    ap.add_argument("--csv", help="also write the report to this CSV file")
    ap.add_argument("--verbose", action="store_true", help="print every console line")
    args = ap.parse_args(argv)

    try:
        gates = parse_gates(args.gates)
    except ValueError as e:
        ap.error(str(e))

    try:
        import serial  # pyserial
    except ImportError:
        sys.exit("pyserial is required: pip install pyserial")

    port = serial.Serial(args.port, args.baud, timeout=0.2)
    link = Link(port, echo=(lambda l: print("      | " + l)) if args.verbose else None)
    test = GateTest(link, gates, timeout_s=args.timeout,
                    skip_direction=args.skip_direction)
    code = 0
    try:
        test.run()
    except KeyboardInterrupt:
        print("\nstopped -- reporting what was tested")
        code = 1
    except ConsoleError as e:
        print("\nconsole error: %s" % e)
        code = 2
    finally:
        test.finish()
        link.close()
    test.report()
    if args.csv:
        test.write_csv(args.csv)
        print("report written to %s" % args.csv)
    if code == 0 and test.failed():
        code = 3
    return code


if __name__ == "__main__":
    sys.exit(main())
