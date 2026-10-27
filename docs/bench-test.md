# Bench test manual

How to check a HiveTraffic counter end to end, chips, sensors, emitters and
direction counting, with the guided test script
[`Firmware/tools/gate_test.py`](../Firmware/tools/gate_test.py).

Use it on a newly assembled board, after a repair, or whenever a counter's
numbers look wrong. A full run of all 24 gates takes about 20–30 minutes.

> The script only types commands into the counter's USB console. Everything
> it does, you could also do by hand; the commands are described in
> [`Firmware/README.md`](../Firmware/README.md#usb-diagnostic-console).

---

## 1. What you need

| Item | Notes |
| --- | --- |
| The counter, running firmware **0.5.0 or later** | Older firmware has no maintenance mode; the script refuses it at the start. `status` in the console shows the version (`fw=...`). |
| A **USB-C data cable** | Some cables only carry power. If no serial port appears when you plug it in, try another cable. |
| A computer with **Python 3** and **pyserial** | `pip install pyserial` (or `python3 -m pip install pyserial`). |
| A **target** | A strip of white paper or card, about the width of one tunnel. It must reflect IR; matt black does not work. |
| The **housing fitted** | Reflections and crosstalk between neighbouring gates depend on it. A bare PCB passes or fails differently from the finished counter. |
| **Shade** | Direct sunlight into the entrance reads like a blocked sensor. Test indoors or in shade. |

Powering the counter from the USB cable is fine. The emitters draw up to
~0.3 A, which any computer USB port supplies.

---

## 2. Find the serial port

Plug the counter in, then:

| System | How | Typical name |
| --- | --- | --- |
| Linux | `ls /dev/ttyACM*` | `/dev/ttyACM0` |
| macOS | `ls /dev/cu.usbmodem*` | `/dev/cu.usbmodem1101` |
| Windows | Device Manager → *Ports (COM & LPT)* | `COM5` |

On Linux, if you get *Permission denied*, add yourself to the serial group
once and log in again: `sudo usermod -aG dialout $USER` (on Arch:
`uucp`).

**Only one program can hold the port.** Close `pio device monitor`, the
Arduino serial monitor or any other terminal before starting the script.

---

## 3. Optional: a first look by hand

Before the guided test, it can help to look at the counter yourself. With
`pio device monitor --echo` (or any terminal at 115200 baud), type:

```
status
mcp
watch 5
```

* `status` shows the firmware version, totals and bank mask. It also shows
  `late=` polls, which should be `0`.
* `mcp` should show `healthy=1` for U2, U3 and U4.
* `watch 5` prints a live line per sensor row. Wave the paper over a gate and
  its `.` should turn into `#`. Stop it with `watch off`.

Then close the terminal again before you start the script.

---

## 4. Run the test

```bash
python3 Firmware/tools/gate_test.py --port /dev/ttyACM0
```

Useful options:

| Option | Use |
| --- | --- |
| `--gates 00-07` | Test only some gates, e.g. one chip: `00-07` (U2), `10-17` (U3), `20-27` (U4). Single gates and lists work too: `--gates 03,12`. |
| `--csv result.csv` | Also save the report as a CSV file. |
| `--skip-direction` | Leave out the two pass-through checks (faster, e.g. for a quick re-check of one sensor). |
| `--timeout 30` | Seconds each step waits for you (default 20). Raise it if you need longer to get into position. |
| `--verbose` | Print every line the counter sends, including each sensor event. Use it when a step fails and you want to see why. |

At every prompt:

* **Enter** starts the step. Do the action *after* pressing Enter. The script
  watches for up to `--timeout` seconds, and moves on 2 seconds after the last
  event it sees.
* **s** + Enter skips the step. It is reported as `NOT TESTED`.
* **q** + Enter stops the run. **Ctrl-C** does the same. The report covers
  what was tested so far.

The script first starts a **maintenance session** on the counter. While it
runs, the counter's real totals are frozen, so test crossings never reach
HiveHub as bee traffic. The session is always ended when the script stops,
even on Ctrl-C or an error.

---

## 5. What the script does, step by step

### 5.1 Bus check (automatic)

```
== I2C bus and MCP23017s ==
  i2c scan  PASS
  U2        PASS       max read 180us
  U3        PASS       max read 180us
  U4        PASS       max read 180us
```

* `i2c scan` must find exactly the three port expanders, at 0x20, 0x21 and
  0x22.
* For each chip, `mcp test` reads back the configuration registers (all inputs,
  no inversion, no internal pull-ups) and does 100 reads in a row. All must
  succeed.

### 5.2 Empty-tunnel baseline

```
== Empty-tunnel baseline ==
Make sure every tunnel is EMPTY (no target, no bees), then press Enter
  all selected gates read clear, dark and lit
```

Every gate is read once with the emitters off and once with them on. With
nothing in the tunnels, every sensor must read clear both times (`none`).

### 5.3 Each gate

For every gate, the script asks for five actions. "Inner" is the hive side,
"outer" the entrance side.

```
== GATE_00 ==
GATE_00: hold the target over the INNER sensor (hive side), then take it away.  [Enter = go, s = skip]
    ...watching for 20s
    inner: PASS
GATE_00: hold the target over the OUTER sensor (entrance side), then take it away.  [Enter = go, s = skip]
    outer: PASS
GATE_00: hold the target over BOTH sensors and keep it there.  [Enter = go, s = skip]
GATE_00: take the target away, then press Enter
    dark/lit: PASS
GATE_00: pass the target through from OUTSIDE to INSIDE (outer first).  [Enter = go, s = skip]
    bee_in: PASS
GATE_00: pass the target through from INSIDE to OUTSIDE (inner first).  [Enter = go, s = skip]
    bee_out: PASS
```

| Step | Do this | Passes when |
| --- | --- | --- |
| inner | Press Enter, hold the paper over the inner sensor for about a second, take it away | This gate reports `inner_block`, then `inner_clear` |
| outer | The same over the outer sensor | `outer_block`, then `outer_clear` |
| dark/lit | Hold the paper over **both** sensors, press Enter, keep holding until asked to take it away | Both sensors read clear with the emitters off and blocked with them on (`reflects`). The response comes from this gate's own emitter. |
| bee_in | Press Enter, then slide the paper through the tunnel from the entrance into the hive, in one smooth movement | This gate counts `bee_in` |
| bee_out | The same from the hive out to the entrance | This gate counts `bee_out` |

Move at roughly the speed of a walking bee: a sweep over one to two seconds.
A very fast flick can be shorter than the two samples the firmware needs to
accept a change.

If anything happens on *another* gate during a step, the result shows it as a
note (`also saw: GATE_01 inner_block`). That is crosstalk; see section 6.

---

## 6. Reading the report

```
== Report ==
i2c scan  PASS
U2        PASS       max read 180us
U3        PASS       max read 180us
U4        PASS       max read 180us

gate        baseline    inner       outer       darklit     bee_in      bee_out     result      notes
GATE_00     PASS        PASS        PASS        PASS        PASS        PASS        PASS
GATE_01     PASS        FAIL        FAIL        PASS        FAIL        FAIL        FAIL        inner: no inner_block seen; outer: no outer_block seen; in: counted bee_out: direction reversed; out: counted bee_in: direction reversed

1 PASS, 1 FAIL, 0 NOT TESTED
```

A gate is `PASS` only when every check passed, `FAIL` when any check failed,
and `NOT TESTED` when a check was skipped and none failed. The script's exit
code is `0` when nothing failed, `3` when something did, `1` when you stopped
it and `2` when the console did not answer as expected.

### What a failure means

| Result | Likely cause | Check |
| --- | --- | --- |
| `i2c scan` missing an address | That MCP23017 is not answering | Solder joints, its 3.3 V (pin 9) and GND (pin 10), the A0/A1/A2 straps (U2 all GND, U3 A0→3V3, U4 A1→3V3), and the R4/R5 pull-ups |
| `i2c scan` unexpected address | A wrong strap, or a short on SDA/SCL | The A0/A1/A2 straps of the chip at the extra address |
| `U2/U3/U4` config `FAIL` | The chip answered but holds the wrong configuration | Power-cycle; if it repeats, the chip is suspect |
| `U2/U3/U4` burst `FAIL` (nack / short reads) | Marginal I²C signal | Pull-ups, cable or trace length, solder joints; the `max read` time is shown too |
| baseline `ambient` | The sensor reads blocked even with its emitter off | Sunlight or a strong lamp on the entrance; otherwise that input is stuck low (sensor or solder bridge) |
| baseline `reflects` | Something in the empty tunnel reflects the emitter's light | Debris, a shiny tunnel wall, or the housing not fitted |
| inner or outer: `no ..._block seen` | That sensor does not respond | The sensor's orientation and solder joints, and its 100 k pull-up |
| inner **and** outer fail, bee_in and bee_out "direction reversed" | Inner and outer sensors swapped | Assembly or wiring of that gate (run with `--verbose`: you will see `outer_block` when you covered the inner sensor) |
| `..._block` seen but it never cleared | The sensor stays blocked | Something left in the tunnel, or a stuck input |
| darklit `none` with the target present | Nothing reflected: the emitter is dead or the sensor is blind | That gate's IR LED pair and its 22 Ω resistor. **Every gate on one chip `none`** → that bank's MOSFET (Q1/Q2/Q3) or its supply |
| darklit `ambient` with the target present | Ambient light, not the emitter, is doing the work | Shade the entrance and repeat |
| bee_in / bee_out `no crossing counted` | The sensors work but no crossing was recognised | Move more smoothly. `only a glitch` means both sensors went blocked in the same sample: the paper covered both at once. Use a narrower strip and lead with its edge |
| `also saw: GATE_xx ...` notes | Crosstalk to a neighbouring gate | Light leaking between tunnels: the housing walls and their fit |
| `bank off`, gate skipped | That emitter bank is switched off (by HiveHub) | Expected if you run fewer than 24 gates; otherwise check the bank mask in `status` |

### Re-testing

After a repair, re-run only the affected gates:

```bash
python3 Firmware/tools/gate_test.py --port /dev/ttyACM0 --gates 01
```

---

## 7. Afterwards

* The script ends the maintenance session itself. To be sure, open a terminal
  and type `status`: the second line must say `maint=off`. If you forget and
  the session is still running, it ends by itself after 30 minutes without a
  command.
* On exit, the counter puts back the LED mode (`auto`) and its bank mask. If
  HiveHub sent a night-mode or bank command during the test, that command is
  applied now.
* The real totals were frozen during the test. HiveHub sees the test as an
  increase of `maint_s` in the counter's measurement, which tells it that
  those minutes' flat totals were a test and not a hive without bees.
* Keep the CSV (`--csv`) with the board's records, so a later fault can be
  compared with how the counter tested when it was built.

---

## Troubleshooting the script itself

| Message | Meaning |
| --- | --- |
| `pyserial is required` | `pip install pyserial` |
| A traceback ending in `could not open port` | Wrong port name, port held by another program, or (Linux) `Permission denied` from a missing `dialout` group, see section 2 |
| `console error: no reply to 'maint' -- wrong port, or not a HiveTraffic counter?` | The port is not the counter, the counter is still booting (wait a few seconds and run again), or its firmware is older than 0.4.0 and has no console at all |
| `console error: maint: [CON] error: unknown command - type 'help'` | Firmware 0.4.x: it has the console but no maintenance mode. Update the counter first (OTA via HiveHub, or `pio run -t upload` over USB). |
| `console error: ... refused: firmware update in progress` | HiveHub is updating the counter over BLE right now. Wait until it reboots. |
| `(warning: the counter dropped events ...)` | More events than the counter could send. Rare; repeat the step. |
