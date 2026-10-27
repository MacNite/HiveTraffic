#!/usr/bin/env python3
"""Tests for tools/gate_test.py, against a simulated counter.

    python3 -m unittest discover -s Firmware/tools -p 'test_*.py'

or via Firmware/test/run_tests.sh. Needs no pyserial and no hardware: the fake
port below answers console commands with the exact line formats the firmware
prints (src/diag_console.cpp), so a change to either side that breaks the
other fails here first.
"""

import os
import re
import sys
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import gate_test as gt  # noqa: E402


# ---------------------------------------------------------------------------
# A simulated counter console
# ---------------------------------------------------------------------------

class FakeCounter(object):
    """Implements readline()/write() like a pyserial port.

    `gates` maps a gate tag to how its hardware behaves:
        'ok'        everything works
        'dead_inner' the inner sensor never responds
        'reversed'   inner and outer are swapped (direction counts backwards)
        'ambient'    the inner sensor reads LOW even dark (sunlight)
        'double'     one pass through the gate counts twice
        'gap'        the event stream skips a sequence number mid-check
        'crosstalk'  the neighbouring gate also blocks on the inner check
        'break'      the person leaves at the OUTSIDE -> INSIDE prompt for so
                     long that the maintenance session times out
        'ota'        an OTA ends the session in the middle of the bee_in check

    `chips` maps a chip name to 'absent' or 'bad_config'. `mute` lists
    commands the counter echoes but never completes.

    Crossings read while no session is running are what the real firmware
    would add to its lifetime totals; they are counted in `lifetime`.
    """

    def __init__(self, behaviour=None, maint_supported=True, chips=None, mute=()):
        self.behaviour = behaviour or {}
        self.maint_supported = maint_supported
        self.chips = chips or {}
        self.mute = set(mute)
        self.lifetime = 0
        self.maint = False
        self.events_on = False
        self.target = None          # gate the person is holding a target over
        self.seq = 1
        self._out = []
        self._lock = threading.Lock()
        self.sent = []

    # -- port interface --
    def readline(self):
        deadline = time.monotonic() + 0.05
        while time.monotonic() < deadline:
            with self._lock:
                now = time.monotonic()
                for i, (t, line) in enumerate(self._out):
                    if t <= now:
                        del self._out[i]
                        if not self.maint and re.search(r"ev=bee_(in|out)", line):
                            self.lifetime += 1
                        return (line + "\n").encode()
            time.sleep(0.002)
        return b""

    def write(self, data):
        for cmd in data.decode().splitlines():
            self.sent.append(cmd)
            self._command(cmd)

    # -- helpers --
    def _emit(self, line, delay=0.0):
        with self._lock:
            self._out.append((time.monotonic() + delay, line))

    def _ev(self, gate, kind, delay, skip=0):
        self.seq += skip
        if self.events_on:
            self._emit("[EV] seq=%d t=%d gate=%s ev=%s" % (self.seq, self.seq * 10, gate, kind),
                       delay)
            self.seq += 1

    def kind_of(self, gate):
        return self.behaviour.get(gate, "ok")

    def _command(self, cmd):
        self._emit("[CON] > " + cmd)
        if cmd in self.mute:
            return
        if cmd == "maint":
            self._emit("[MAINT] on for 3s, ends itself in 1797s without a command"
                       if self.maint else "[MAINT] off; lifetime maintenance 0s")
        elif cmd == "maint on":
            if not self.maint_supported:
                self._emit("[CON] error: unknown command - type 'help'")
                return
            self.maint = True
            self._emit("[MAINT] started: totals frozen, crossings go to test counters, "
                       "ends after 1800s without a command")
            self._emit("[MAINT] on for 0s, ends itself in 1800s without a command")
        elif cmd == "maint off":
            if not self.maint:
                self._emit("[CON] maint off: nothing to do")
                return
            self.maint = False
            self._emit("[MAINT] ended on request after 9s: test in=0 out=0 glitches=0; "
                       "banks 0x07, night mode 0s")
        elif cmd == "events on":
            self.events_on = True
            self._emit("[CON] events on (next seq=%d)" % self.seq)
        elif cmd == "events off":
            self.events_on = False
            self._emit("[CON] events off (next seq=%d)" % self.seq)
        elif cmd == "i2c scan":
            self._emit("[I2C] 3 device(s) answered (expected 0x20 0x21 0x22)")
            for a in (0x20, 0x21, 0x22):
                self._emit("[I2C] ack at 0x%02X" % a)
        elif cmd == "mcp test":
            for name, addr in (("U2", 0x20), ("U3", 0x21), ("U4", 0x22)):
                chip = self.chips.get(name)
                if chip == "absent":
                    self._emit("[MCPTEST] %s 0x%02X: FAIL no ACK on its address" % (name, addr))
                    continue
                self._emit("[MCPTEST] %s 0x%02X: config %s iodir=FF/FF ipol=00/00 "
                           "gppu=%s/00 iocon=00"
                           % (name, addr, "FAIL" if chip == "bad_config" else "ok",
                              "FF" if chip == "bad_config" else "00"))
                self._emit("[MCPTEST] %s burst ok: 100/100 ok nack=0 short=0 max=180us" % name)
        elif cmd == "darklit":
            self._emit("[DARKLIT] inner/outer = dark/lit -> verdict "
                       "(reflects / none / ambient / inverted)")
            for g in gt.GATE_TAGS:
                inner = outer = ("clear", "clear", "none")
                if g == self.target:
                    inner = outer = ("clear", "BLOCK", "reflects")
                    if self.kind_of(g) == "dead_inner":
                        inner = ("clear", "clear", "none")
                if self.kind_of(g) == "ambient":
                    inner = ("BLOCK", "BLOCK", "ambient")
                self._emit("[DARKLIT] %s bank=1 inner=%s/%s->%s outer=%s/%s->%s"
                           % ((g,) + inner + outer))
        else:
            self._emit("[CON] error: unknown command - type 'help'")

    # -- the "person" --
    def act(self, prompt):
        """React to a prompt the way a person following it would."""
        gate = next((g for g in gt.GATE_TAGS if g in prompt), None)
        if gate is None:
            return ""
        kind = self.kind_of(gate)
        d = 0.05
        if "over the INNER" in prompt:
            if kind != "dead_inner":
                s = "outer" if kind == "reversed" else "inner"
                self._ev(gate, s + "_block", d)
                if kind == "crosstalk":
                    nxt = gt.GATE_TAGS[gt.GATE_TAGS.index(gate) + 1]
                    self._ev(nxt, "inner_block", d)
                self._ev(gate, s + "_clear", 2 * d)
        elif "over the OUTER" in prompt:
            s = "inner" if kind == "reversed" else "outer"
            self._ev(gate, s + "_block", d)
            self._ev(gate, s + "_clear", 2 * d)
        elif "over BOTH" in prompt:
            self.target = gate
        elif "take the target away" in prompt:
            self.target = None
        elif "OUTSIDE to INSIDE" in prompt:
            if kind == "break":
                # Thirty minutes at the prompt: the firmware ends the session.
                self.maint = False
                self._emit("[MAINT] ended after inactivity timeout after 1800s: "
                           "test in=0 out=0 glitches=0; banks 0x07, night mode 0s")
                self._ev(gate, "bee_in", d)
            elif kind == "ota":
                self._ev(gate, "bee_in", d)
                self._emit("[MAINT] ended for a firmware update after 5s: test in=1 "
                           "out=0 glitches=0; banks 0x07, night mode 0s", 1.5 * d)

                def end():
                    self.maint = False
                threading.Timer(1.5 * d, end).start()
            elif kind == "double":
                self._ev(gate, "bee_in", d)
                self._ev(gate, "bee_in", 2 * d)
            elif kind == "gap":
                self._ev(gate, "outer_block", d)
                self._ev(gate, "bee_in", 2 * d, skip=1)   # one event lost between
            elif kind == "dead_inner":
                self._ev(gate, "outer_block", d)
                self._ev(gate, "outer_clear", 2 * d)
                self._ev(gate, "glitch", 2 * d)
            else:
                self._ev(gate, "bee_out" if kind == "reversed" else "bee_in", d)
        elif "INSIDE to OUTSIDE" in prompt:
            if kind != "dead_inner":
                self._ev(gate, "bee_in" if kind == "reversed" else "bee_out", d)
        return ""


def make_test(counter, gates, answers=None, **kw):
    """A GateTest wired to `counter`, with a scripted person."""
    link = gt.Link(counter)
    answers = dict(answers or {})
    said = []

    def ask(prompt):
        for key, reply in answers.items():
            if key in prompt:
                return reply
        return counter.act(prompt)

    t = gt.GateTest(link, gates, timeout_s=1.0, ask=ask, say=said.append, **kw)
    t.quiet_s = 0.2
    return t, link, said


# ---------------------------------------------------------------------------

class ParseTests(unittest.TestCase):
    def test_parse_gates(self):
        self.assertEqual(len(gt.parse_gates("all")), 24)
        self.assertEqual(gt.parse_gates("03"), ["GATE_03"])
        self.assertEqual(gt.parse_gates("10-12,00"), ["GATE_00", "GATE_10", "GATE_11", "GATE_12"])
        # Ranges skip the numbering gaps (08, 09, 18, 19).
        self.assertEqual(len(gt.parse_gates("00-27")), 24)
        with self.assertRaises(ValueError):
            gt.parse_gates("08")
        with self.assertRaises(ValueError):
            gt.parse_gates("12-10")
        with self.assertRaises(ValueError):
            gt.parse_gates("x")

    def test_parse_event(self):
        e = gt.parse_event("[EV] seq=12 t=34567 gate=GATE_14 ev=outer_block")
        self.assertEqual((e.seq, e.t_ms, e.gate, e.kind), (12, 34567, "GATE_14", "outer_block"))
        self.assertIsNone(gt.parse_event("[EV] dropped=3 (queue full; see the seq gap)"))
        self.assertIsNone(gt.parse_event("[STAT] uptime=1s"))

    def test_parse_darklit(self):
        g, v = gt.parse_darklit(
            "[DARKLIT] GATE_21 bank=3 inner=clear/BLOCK->reflects outer=disabled/disabled->disabled")
        self.assertEqual(g, "GATE_21")
        self.assertEqual(v["bank"], 3)
        self.assertEqual(v["inner"], "reflects")
        self.assertEqual(v["inner_lit"], "BLOCK")
        self.assertEqual(v["outer"], "disabled")
        self.assertIsNone(gt.parse_darklit("[DARKLIT] inner/outer = dark/lit -> verdict"))


class EvalTests(unittest.TestCase):
    E = gt.Event

    def test_sensor(self):
        ev = [self.E(1, 0, "GATE_01", "inner_block"), self.E(2, 9, "GATE_01", "inner_clear")]
        self.assertEqual(gt.eval_sensor(ev, "GATE_01", "inner")[0], gt.PASS)
        self.assertEqual(gt.eval_sensor(ev, "GATE_01", "outer")[0], gt.FAIL)
        self.assertEqual(gt.eval_sensor(ev[:1], "GATE_01", "inner")[0], gt.FAIL)
        # Clear before block does not count as a response.
        self.assertEqual(gt.eval_sensor(ev[::-1], "GATE_01", "inner")[0], gt.FAIL)

    def test_crosstalk_is_noted(self):
        ev = [self.E(1, 0, "GATE_01", "inner_block"), self.E(2, 0, "GATE_02", "inner_block"),
              self.E(3, 9, "GATE_01", "inner_clear")]
        result, note = gt.eval_sensor(ev, "GATE_01", "inner")
        self.assertEqual(result, gt.PASS)
        self.assertIn("GATE_02 inner_block", note)

    def test_direction(self):
        E = self.E
        self.assertEqual(gt.eval_direction([E(1, 0, "GATE_05", "bee_in")], "GATE_05", "bee_in")[0], gt.PASS)
        r, note = gt.eval_direction([E(1, 0, "GATE_05", "bee_out")], "GATE_05", "bee_in")
        self.assertEqual(r, gt.FAIL)
        self.assertIn("reversed", note)
        r, note = gt.eval_direction([E(1, 0, "GATE_05", "glitch")], "GATE_05", "bee_in")
        self.assertIn("glitch", note)
        self.assertEqual(gt.eval_direction([], "GATE_05", "bee_out")[0], gt.FAIL)
        # A crossing on another gate is not this gate's.
        self.assertEqual(gt.eval_direction([E(1, 0, "GATE_06", "bee_in")], "GATE_05", "bee_in")[0], gt.FAIL)

    def test_direction_counts_once(self):
        E = self.E
        r, note = gt.eval_direction([E(1, 0, "GATE_05", "bee_in"), E(2, 9, "GATE_05", "bee_in")],
                                    "GATE_05", "bee_in")
        self.assertEqual(r, gt.FAIL)
        self.assertIn("counted 2 bee_in", note)

    def test_judge(self):
        self.assertEqual(gt.judge((gt.PASS, "n"), []), (gt.PASS, "n"))
        r, note = gt.judge((gt.PASS, "n"), ["the counter dropped events"])
        self.assertEqual(r, gt.FAIL)
        self.assertIn("dropped", note)
        self.assertIn("n", note)

    def test_baseline(self):
        v = lambda i, o: {"inner": i, "outer": o}
        out = gt.eval_baseline({"GATE_00": v("none", "none"), "GATE_01": v("ambient", "none"),
                                "GATE_02": v("disabled", "disabled"), "GATE_03": v("none", "reflects")})
        self.assertEqual(out["GATE_00"][0], gt.PASS)
        self.assertEqual(out["GATE_01"][0], gt.FAIL)
        self.assertEqual(out["GATE_02"], (gt.NOT_TESTED, "bank off"))
        self.assertIn("outer=reflects", out["GATE_03"][1])

    def test_overall(self):
        P, F, N = gt.PASS, gt.FAIL, gt.NOT_TESTED
        self.assertEqual(gt.overall({"a": (P, ""), "b": (P, "")}), P)
        self.assertEqual(gt.overall({"a": (P, ""), "b": (F, "")}), F)
        self.assertEqual(gt.overall({"a": (P, ""), "b": (N, "")}), N)
        self.assertEqual(gt.overall({"a": (N, ""), "b": (F, "")}), F)


class EndToEndTests(unittest.TestCase):
    def run_test(self, behaviour, gates, **kw):
        counter = FakeCounter(behaviour)
        t, link, said = make_test(counter, gates, **kw)
        try:
            t.run()
        finally:
            t.finish()
            link.close()
        return t, counter, said

    def test_healthy_gates_pass(self):
        t, counter, _ = self.run_test({}, ["GATE_00", "GATE_17"])
        rows = {r["gate"]: r for r in t.rows()}
        for g in ("GATE_00", "GATE_17"):
            self.assertEqual(rows[g]["result"], gt.PASS, rows[g])
        self.assertTrue(all(r == gt.PASS for _, r, _ in t.chip_results), t.chip_results)
        # It left the counter as it found it.
        self.assertFalse(counter.maint)
        self.assertFalse(counter.events_on)
        self.assertIn("maint off", counter.sent)

    def test_faults_are_reported(self):
        t, _, _ = self.run_test(
            {"GATE_01": "dead_inner", "GATE_02": "reversed", "GATE_03": "ambient"},
            ["GATE_01", "GATE_02", "GATE_03"])
        rows = {r["gate"]: r for r in t.rows()}
        self.assertEqual(rows["GATE_01"]["inner"], gt.FAIL)
        self.assertEqual(rows["GATE_01"]["darklit"], gt.FAIL)
        self.assertEqual(rows["GATE_01"]["result"], gt.FAIL)
        self.assertEqual(rows["GATE_02"]["bee_in"], gt.FAIL)
        self.assertIn("reversed", rows["GATE_02"]["notes"])
        self.assertEqual(rows["GATE_03"]["baseline"], gt.FAIL)

    def test_skipping_gives_not_tested(self):
        t, _, _ = self.run_test({}, ["GATE_20"], answers={"OUTER sensor": "s"},
                                skip_direction=True)
        row = t.rows()[0]
        self.assertEqual(row["outer"], gt.NOT_TESTED)
        self.assertEqual(row["inner"], gt.PASS)
        self.assertEqual(row["bee_in"], gt.NOT_TESTED)
        self.assertEqual(row["result"], gt.NOT_TESTED)

    def test_old_firmware_is_refused(self):
        counter = FakeCounter(maint_supported=False)
        t, link, _ = make_test(counter, ["GATE_00"])
        try:
            with self.assertRaises(gt.ConsoleError):
                t.run()
        finally:
            link.close()

    def test_maintenance_timeout_at_a_prompt_is_recovered(self):
        # The person walks away at a prompt for longer than the session lasts.
        # The session is restarted before they act, so the crossing is a test
        # stimulus, not traffic -- and the check stays valid.
        t, counter, said = self.run_test({"GATE_04": "break"}, ["GATE_04"])
        row = t.rows()[0]
        self.assertEqual(counter.lifetime, 0)
        self.assertEqual(row["bee_in"], gt.PASS, row)
        self.assertTrue(any("had ended" in s for s in said), said)

    def test_session_ending_during_a_check_fails_it(self):
        t, counter, _ = self.run_test({"GATE_05": "ota"}, ["GATE_05"])
        row = t.rows()[0]
        self.assertEqual(row["bee_in"], gt.FAIL, row)
        self.assertIn("maintenance ended during the check", row["notes"])
        self.assertTrue(t.failed())

    def test_double_count_fails(self):
        t, _, _ = self.run_test({"GATE_06": "double"}, ["GATE_06"])
        row = t.rows()[0]
        self.assertEqual(row["bee_in"], gt.FAIL, row)
        self.assertIn("counted 2 bee_in", row["notes"])

    def test_sequence_gap_fails(self):
        t, _, _ = self.run_test({"GATE_07": "gap"}, ["GATE_07"])
        row = t.rows()[0]
        self.assertEqual(row["bee_in"], gt.FAIL, row)
        self.assertIn("gap", row["notes"])

    def test_crosstalk_note_survives_a_pass(self):
        t, _, _ = self.run_test({"GATE_10": "crosstalk"}, ["GATE_10"])
        row = t.rows()[0]
        self.assertEqual(row["inner"], gt.PASS, row)
        self.assertIn("GATE_11 inner_block", row["notes"])

    def test_chip_failure_fails_the_run(self):
        counter = FakeCounter(chips={"U3": "bad_config", "U4": "absent"})
        t, link, _ = make_test(counter, ["GATE_00"])
        try:
            t.run()
        finally:
            t.finish()
            link.close()
        # Every gate passed, and the run still fails on the chips.
        self.assertEqual(t.rows()[0]["result"], gt.PASS)
        chips = {name: result for name, result, _ in t.chip_results}
        self.assertEqual(chips["U2"], gt.PASS)
        self.assertEqual(chips["U3"], gt.FAIL)
        self.assertEqual(chips["U4"], gt.FAIL)
        self.assertTrue(t.failed())

    def test_incomplete_reply_is_an_error(self):
        counter = FakeCounter(mute=("events on",))
        t, link, _ = make_test(counter, ["GATE_00"])
        try:
            with self.assertRaises(gt.ConsoleError) as cm:
                t.run()
            self.assertIn("incomplete reply", str(cm.exception))
        finally:
            link.close()

    def test_csv(self):
        import tempfile
        t, _, _ = self.run_test({}, ["GATE_00"], skip_direction=True)
        with tempfile.NamedTemporaryFile("r", suffix=".csv", delete=False) as f:
            path = f.name
        try:
            t.write_csv(path)
            with open(path) as f:
                text = f.read()
            self.assertIn("gate,baseline,inner,outer,darklit,bee_in,bee_out,result,notes", text)
            self.assertIn("GATE_00,PASS,PASS,PASS,PASS,NOT TESTED,NOT TESTED,NOT TESTED", text)
            # The bus and chip checks are part of the record too.
            self.assertIn("i2c scan,,,,,,,PASS", text)
            self.assertIn("U3,,,,,,,PASS", text)
        finally:
            os.unlink(path)


if __name__ == "__main__":
    unittest.main()
