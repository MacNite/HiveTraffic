#!/bin/sh
# Build and run the host-side logic tests.
#
# These cover the pure headers, none of which needs an ESP32, an
# MCP23017 or a bee:
#
#   * include/gate_logic.h      — sensor debounce, the per-gate crossing state
#                                 machine, and the saturating counters;
#   * include/measurement_json.h — the exact bytes of the BLE measurement
#                                 characteristic, i.e. HiveHub's half of the
#                                 wire contract;
#   * include/idle_state.h      — the night-mode suspension deadline, including
#                                 the millis() rollover and the "HiveHub stopped
#                                 re-arming" case;
#   * include/bank_state.h      — the emitter-bank enable mask, whose mistakes
#                                 are eight gates that silently stop counting;
#   * include/device_name.h     — the advertised BLE name, whose address suffix
#                                 is how a person tells two counters apart;
#   * include/log_line.h        — the USB log's "whole lines or nothing" rule,
#                                 which keeps a stalled terminal from stalling
#                                 the gate sampler;
#   * include/poll_timing.h     — the poll-gap record that makes such a stall
#                                 visible, across pauses and micros() rollover;
#   * include/console_line.h    — the USB console's line input and its strict
#                                 command grammar;
#   * include/event_ring.h      — the queue between the sampler and the
#                                 console's event stream, and its visible gaps;
#   * include/diag_format.h     — the console's glyphs and row layout, the
#                                 dark/lit verdicts and the MCP23017 register
#                                 check;
#   * include/maint_state.h     — maintenance sessions: the inactivity timeout,
#                                 the replay of HiveHub's deferred requests and
#                                 the maint_s figure;
#   * tools/gate_test.py        — the guided bench gate test, run against a
#                                 simulated console (python3, no hardware).
#
# Everything hardware-facing stays in src/main.cpp and is still verified on the
# bench (see the USB diagnostic console in the README).
#
#     ./test/run_tests.sh
#
# Exits non-zero if any check fails.
set -eu

cd "$(dirname "$0")/.."

CXX=${CXX:-c++}
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

for t in gate_logic measurement_json idle_state bank_state device_name \
         log_line poll_timing console_line event_ring diag_format maint_state; do
    "$CXX" -std=c++11 -Wall -Wextra -Werror \
        -I include \
        "test/test_$t/test_$t.cpp" \
        -o "$OUT/test_$t"
    "$OUT/test_$t"
done

# The bench gate-test script (tools/gate_test.py), against a simulated
# counter that prints the console's exact line formats. Needs only python3.
python3 -m unittest discover -s tools -p 'test_*.py'
