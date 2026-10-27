# HiveTraffic bee counter — ESP32-C6 mini firmware

Firmware for the ESP32-C6 mini bee counter PCB. Continuously polls the 24
entrance gates, detects directional bee crossings, and serves lifetime totals
to [MacNite/HiveHub](https://github.com/MacNite/HiveHub) over BLE/GATT.

There is one build, and it is wireless. Flash it with:

```sh
pio run -t upload
```

The firmware also exposes HiveInside-style connectable BLE OTA characteristics,
which HiveHub drives with its `update_beecounter` command. Image size and CRC-32
are verified before the inactive app slot is selected; see
[`docs/ble-mode.md`](../docs/ble-mode.md) for the framing and the relay.

The image version lives in [`include/version.h`](include/version.h) and is
reported over BLE as the measurement JSON's `ver` field. **Bump it for every
released image** — HiveHub gates an OTA relay on it being newer than what the
counter reports, and uses it afterwards to confirm the update took.

> **The wired HiveScale link has been removed.** Earlier firmware ran the C6's
> second I2C controller as a permanent slave at 0x30, serving a register map
> (totals, `CMD_LATCH` interval counters, per-gate arrays, OTA-over-I2C) to a
> HiveScale polling over J1. HiveHub reads counters over BLE only and dropped
> its wired client, so nothing spoke that protocol anymore. The connector and
> traces are still on the PCB; the firmware simply never brings that controller
> up.

---

## How it works

### Sensing

Each of the 24 gates has two QRE1113 reflective IR sensors:

- **Inner** — toward the hive interior
- **Outer** — toward the outside world

The 48 sensor lines are read through 3 MCP23017 I2C port expanders:

| MCP   | I2C addr | Gates       | Inner sensors  | Outer sensors  |
| ----- | -------- | ----------- | -------------- | -------------- |
| U2    | 0x20     | GATE_00..07 | GPA0..GPA7     | GPB0..GPB7     |
| U3    | 0x21     | GATE_10..17 | GPA0..GPA7     | GPB0..GPB7     |
| U4    | 0x22     | GATE_20..27 | GPA0..GPA7     | GPB0..GPB7     |

The IR emitters are split into three banks driven by IRLB8721 N-channel MOSFETs — one FET per MCP23017 since the 2026-08 hardware revision:

| Bank | Gates | FET | Enable GPIO | XIAO silk | Schematic net |
| ---- | ----- | --- | ----------- | --------- | ------------- |
| LED_BANK_1 | GATE_00..07 (U2) | Q1 | GPIO19 | D8  | /GPIO4 |
| LED_BANK_2 | GATE_10..17 (U3) | Q2 | GPIO20 | D9  | /GPIO5 |
| LED_BANK_3 | GATE_20..27 (U4) | Q3 | GPIO18 | D10 | /GPIO6 |

The net names `/GPIO4`, `/GPIO5` and `/GPIO6` are labels only — they are **not** physical GPIO4/5/6. Use the Enable GPIO column.

The previous 2-FET build split U3's gates across banks 1 and 2 (00..13 / 14..27); banks now line up 1:1 with the expanders. `pins::IR_LED_BANK_EN[]` maps bank number − 1 to its GPIO, and `gates::TABLE[i].led_bank` gives each gate's bank.

Driving the GPIO HIGH turns the bank's emitters on. In the default `LedMode::AUTO` the emitters are **pulsed**: all three banks are lit together only for the settle + MCP-read window of each poll (~1.75 ms at 100 kHz), then switched off until the next poll. This drops the emitter duty cycle from 100% to roughly 35% at  the default 5 ms poll interval, cutting average emitter current proportionally, with no change to detection behaviour.

Each bank can also be **switched off entirely**, which is a separate control from the LED mode: HiveHub writes an enable bitmask to the control characteristic (`SET_BANKS`, protocol v5) and `include/bank_state.h` decides what is applied. One bank draws ~0.14 A at 3.3 V, two ~0.22 A, three ~0.30 A, so this is the coarsest and most effective power knob on the board — for an entrance narrower than 24 gates, or a supply that will not carry the full one.

All three banks are enabled by default and after any reset; the mask is never persisted, and HiveHub re-asserts it every upload cycle. A mask of `0` is refused rather than applied. Gates on a dark bank are **skipped**, not read as "clear": an unpowered QRE1113 is a bare phototransistor under a 100 kΩ pull-up, and direct sun into the entrance can pull one low. The expander itself is still read and health-checked, so `mcps_healthy` keeps its meaning. The USB console's `status` command shows the mask in force, and `watch` shows a dark bank's gates as `-`.

### Counting

Each gate is a small state machine:

```
   IDLE ──Inner blocked──▶ INNER_FIRST ──Outer blocked──▶  count OUT
        ──Outer blocked──▶ OUTER_FIRST ──Inner blocked──▶  count IN
                          (within 2 s, else timeout)
```

After a count, the gate waits in `PAIRED` until both sensors are clear,
then returns to `IDLE`. Glitches (both blocked simultaneously, or only one
ever blocks before timeout) are counted and reported as the JSON `glitches` field for diagnostics.

### One I2C bus

The MCP23017s sit on `Wire` (GPIO22 / silk D4 = SDA, GPIO23 / silk D5 = SCL), with the on-board 4.7 kΩ
pull-ups R4/R5. The loop polls them on a ~5 ms cadence and nothing else shares
the bus, so there is no arbitration to think about.

The PCB also routes a second bus to J1 on silk D2/D3 (GPIO2/GPIO21) for the retired HiveScale
link. The firmware never initialises that controller, so those pins stay inputs.

### Reporting

Counting produces **lifetime totals only**. HiveHub connects once per upload
cycle, reads one JSON characteristic and disconnects; it derives each interval by
differencing consecutive reads on its server. That is why there is no latch, no
reset and no per-interval state on the device — a missed connection cannot lose
traffic, because nothing is ever consumed by being read.

See [`docs/ble-mode.md`](../docs/ble-mode.md) for the GATT contract, the JSON
fields and the OTA framing, and `include/counter_protocol.h` for the status
bitfield and OTA state codes shared between `main.cpp` and `ble_link.cpp`.

---

## Building & flashing

```bash
pio run               # build
pio run -t upload     # flash via USB
pio device monitor    # 115200 baud serial output
```

PlatformIO target is `seeed_xiao_esp32c6`. U5 on this PCB is a **Seeed XIAO
ESP32C6** (Seeed part 113991054), not a bare ESP32-C6-MINI-1, so this is the
board profile that matches the hardware: it declares the module's 4 MB flash
(the earlier `esp32-c6-devkitc-1` profile declared 8 MB) and pulls in the
`XIAO_ESP32C6` variant. That variant defines the `D0..D10` aliases but does
*not* enable Arduino pin remapping, so the raw GPIO numbers in `pins.h` are
still correct as written. Do not read the board's silk numbers as GPIO numbers —
they differ (silk D4 is GPIO22). USB CDC is enabled in `platformio.ini`, so
`Serial` output comes out over USB without an external UART bridge.

### USB serial output never waits

Every log line goes through `slog::printf()` (`src/serial_log.cpp`), never
through `Serial.print*()` directly, and it never blocks. A line that does not
fit the 4 KiB USB TX buffer right now is **dropped and counted**. The Arduino
USB driver normally waits up to 100 ms per write (and retries up to 20 times)
whenever a host is attached but not reading: a paused terminal, a sleeping
laptop. That is long enough to miss crossings, because the gates are sampled
every 5 ms and a bee spends only ~12 ms in a beam. So `slog::begin()` sets the
driver's TX timeout to zero, which also covers the ESP-IDF `log_i/log_w` output
that shares the port. Lines are written whole or not at all, so a dropped line
never splices into the next one (`include/log_line.h`).

The firmware also measures whether the sampler really runs every 5 ms
(`include/poll_timing.h`): the gap between consecutive polls and how long each
poll takes. Deliberate pauses (night mode, OTA) are not counted as gaps. A
`[TIMING]` line follows the 30 s `[STAT]` line **only when something was
wrong** in that window. A healthy counter's log looks exactly as before.

```
[TIMING] polls=5821 late=3 max_gap=41234us max_poll=1690us log_dropped=0 (boot: late=3 max_gap=41234us)
```

* `late`: polls that started more than two intervals (10 ms) after the
  previous one. These are what can cost a crossing.
* `max_gap` / `max_poll`: longest gap between poll starts, and longest single
  poll (which bounds the lit emitter window), in this window.
* `log_dropped`: lines dropped since the last `[TIMING]` line because the
  USB host was not reading.

To check a build on the bench, let it run with BLE active in each of these
three states for a few minutes, then read the log:

1. no USB host at all (powered from a charger);
2. `pio device monitor` attached and reading;
3. `pio device monitor` attached but suspended with `Ctrl-Z` for a minute
   or two (it then stops reading the port), then resumed with `fg`.

Case 3 should produce `log_dropped>0` after the resume and **no** `late`
polls. Any `late` polls point at something other than the log.

The build uses `partitions_4mb_ota_no_fs.csv` — two 2 MB app slots so BLE OTA
can write the inactive one. The table sums to exactly 4 MB, which is the flash
the XIAO board profile declares. A counter still running a pre-BLE image was
flashed with a single-app layout and **must be updated once over USB**; OTA
cannot migrate a partition table.

### Build artifacts

[`rename_firmware.py`](rename_firmware.py) (wired in as a `pre:` extra script)
names the products after the board and the version from `include/version.h`
instead of the generic `firmware.bin`:

| Environment          | Artifact in `.pio/build/<env>/`      |
| -------------------- | ------------------------------------ |
| `seeed_xiao_esp32c6` | `hivetraffic_esp32-c6_<version>.bin` |

That name is what makes an image self-describing to HiveHub. Drop the `.bin`
into the dashboard's **Upload firmware** form and it reads the filename to
pre-select target *HiveTraffic counter* and pre-fill the Version field — the
board is fixed at `esp32-c6`, and the backend refuses to publish a release whose
board disagrees with its filename, so a cross-architecture image can never be
relayed to a counter. Upload this file **as-is**: it is the application-only
image, which is what the counter's OTA writes into its inactive app slot, not a
merged factory image.

Because the naming is a contract with HiveHub, the board token and layout have
to stay in sync with HiveHub's `rename_firmware.py`, `server/firmware.py`
(`board_from_filename`, `BEECOUNTER_BOARDS`) and the dashboard's filename
parsing. The header comment in `rename_firmware.py` lists them.

Should a variant build ever be needed again, stamp it after the version rather
than into the board token: `0.1.0-something` is what shows up in the pre-filled
Version field, so it is hard to publish by mistake, and HiveHub compares
versions component-wise with non-digits stripped — such a build ties with the
production image of the same version and can never be relayed over it.
`rename_firmware.py` already falls back to `-<env name>` for any environment it
does not know.

---

## USB diagnostic console

Every image has a diagnostic console on the USB serial port. Nothing has to be
rebuilt or reflashed to troubleshoot a counter. Attach any terminal at
115200 baud, type a command and press Enter:

```bash
pio device monitor --echo    # --echo shows what you type
```

The console is **passive**. Every command reports state the normal 5 ms
sampler has already produced. None of them reads a sensor, lights an emitter
or changes the totals, so watching a counter on a hive cannot change what it
counts. Its output is paced (a few lines per loop pass, and only when the USB
buffer has room for a whole line), so a terminal that stops reading loses
console output, never sampler time.

| Command | Shows |
| --- | --- |
| `help` | The command list and the glyph legend |
| `status` | Firmware and protocol version, uptime, totals, glitches, status bits, bank mask, night-mode / OTA state, poll timing since boot and in the current 30 s window, dropped log lines and events |
| `sensors` | One line per gate from the latest poll: raw / debounced state of both sensors, and the gate's state machine (`IDLE`, `OUTER_FIRST`, `INNER_FIRST`, `PAIRED`) |
| `mcp` | Per MCP23017: healthy, last read, failure streak, last port word, age of the last good read, successful reads, NACKs, short reads, times lost / recovered, slowest read |
| `watch [1-10]` | One compact line per period at N Hz (default 5). `watch off` stops it |
| `events on` / `events off` | Every debounced sensor edge and every counted crossing or glitch, timestamped at the poll that saw it |

Every line starts with a tag (`[CON]`, `[STATUS]`, `[GATE]`, `[MCP]`, `[W]`,
`[EV]`), so it can be told apart from the ordinary log and parsed by a script.

A `watch` line packs one glyph per sensor, with `|` between the three chips
(gates 00–07, 10–17, 20–27):

```
[W] t=812.400 in=1043 out=1029 inner=........|..#.....|-------- outer=........|.##.....|--------
```

| Glyph | Meaning |
| --- | --- |
| `.` | clear (line HIGH) |
| `#` | blocked or reflecting (line LOW) |
| `-` | that gate's emitter bank is switched off, so it is not sampled |
| `?` | its chip is unhealthy or failed its latest read |
| `~` | not sampling right now (night mode or OTA), or no fresh read |

`watch` shows raw readings at a few Hz and will miss a quick bee. `events`
misses nothing. The sampler queues every edge at the full 5 ms rate into a
256-entry buffer, and the console drains it as USB allows:

```
[EV] seq=118 t=815230 gate=GATE_12 ev=outer_block
[EV] seq=119 t=815240 gate=GATE_12 ev=inner_block
[EV] seq=120 t=815240 gate=GATE_12 ev=bee_in
```

If the buffer ever fills, the newest events are dropped, never the oldest. The
gap shows as a jump in `seq`, followed by an `[EV] dropped=N` line.

---

## Tuning

All the knobs are at the top of `src/main.cpp`:

| Constant                  | Default | Effect                                                        |
| ------------------------- | ------- | ------------------------------------------------------------- |
| `POLL_INTERVAL_MS`        | 5       | How often the MCP23017s are polled for crossings              |
| `SENSOR_DEBOUNCE_MS`      | 5       | Minimum time a new sensor value must persist before it is accepted |
| `SENSOR_DEBOUNCE_SAMPLES` | 2       | Consecutive polls that must agree before a change is accepted |
| `GATE_PAIRING_WINDOW_MS`  | 2000    | Max time between inner/outer trip for a directional count     |
| `SENSOR_STUCK_MS`         | 30000   | After this many ms continuously blocked, fault flag is raised |
| `MCP_FAIL_THRESHOLD`      | 3       | Consecutive failed reads before a chip is declared unhealthy  |
| `MCP_RETRY_INTERVAL_MS`   | 5000    | How often an unhealthy chip is re-probed                      |
| `I2C_MASTER_HZ`           | 400000  | MCP bus clock; reduces every pulsed sample's IR-on and CPU-wait time |
| `LED_SETTLE_US`           | 250     | IR emitter settle time before each pulsed read. Lower = less power but risks reading stale "clear" levels if shorter than the real phototransistor settle time. |

If your hive has unusually long entrance tunnels or slow-moving bees, raise
`GATE_PAIRING_WINDOW_MS`.

### The debounce rule

A sensor change is accepted only once the new value has been seen on
`SENSOR_DEBOUNCE_SAMPLES` **consecutive** polls *and* has persisted for at least
`SENSOR_DEBOUNCE_MS`. Both halves matter, and they interact with
`POLL_INTERVAL_MS`:

- A `SENSOR_DEBOUNCE_MS` shorter than one poll period cannot reject anything —
  the first differing sample already satisfies it. At the 5 ms poll rate the
  sample count is the half that actually does the work.
- `SENSOR_DEBOUNCE_SAMPLES = 2` is the practical ceiling. A bee crossing at
  ~25 cm/s dwells ~12 ms in a 3 mm beam, i.e. 2-3 samples; requiring three
  agreeing samples (~10 ms) would start dropping real crossings. Two rejects a
  single corrupted or noisy word while accepting a genuine block ~5 ms after it
  appears.

So if you see noise on `glitches`, do **not** simply raise
`SENSOR_DEBOUNCE_SAMPLES` — poll faster first (lower `POLL_INTERVAL_MS`, at the
cost of emitter duty cycle) so a longer debounce still fits inside the dwell.

The rules live in `include/gate_logic.h` and are covered by the host-side tests
described under [Tests](#tests).

### Expander health

Each MCP23017 is tracked at runtime rather than only at boot:

- A chip that fails `MCP_FAIL_THRESHOLD` consecutive reads is marked unhealthy,
  its `STATUS_MCP_U*_OK` status bit is cleared, and its eight gates are reset
  and skipped. Gates are skipped from the *first* failed read — a failed I2C
  transaction reads as `0x0000`, which is indistinguishable from "all 16 beams
  blocked", so it must never reach the state machine.
- An unhealthy chip (including one missing at boot) is re-probed every
  `MCP_RETRY_INTERVAL_MS`, with the emitters dark so a probe never lengthens the
  LED pulse. On recovery its gates are reset and its status bit is set again.
- `mcps_healthy` in the BLE telemetry reflects this live state — it counts
  expanders (0..3), not gates. It was called `gates_healthy` before protocol v3.

---

## Tests

Several pure headers carry logic that is worth testing without hardware, so each
is deliberately free of Arduino, I2C and NimBLE dependencies and all of them run
on a host compiler:

```
./test/run_tests.sh
```

No ESP32, no MCP23017 and no bee required.

* **`include/gate_logic.h`** — sensor debounce, the per-gate direction state
  machine and the saturating counters. The suite covers single-sample spikes,
  alternating noise, simultaneous transitions, minimum-dwell and repeated
  crossings, pairing-window expiry, stuck sensors, `millis()` rollover and
  counter saturation. Run it before pushing a change to the timing rules: the
  failure it was written to catch (a debounce that accepted the first differing
  sample) is invisible in a code read and effectively untestable on a live hive.
* **`include/measurement_json.h`** — the exact bytes of the BLE measurement
  characteristic, which is HiveHub's half of a two-repo contract. The suite
  pins the field names, checks that `uptime_s` and `glitches` really do carry
  values past their old 16-bit ceilings, and measures the saturated worst-case
  document against the buffer it has to fit in. Run it before changing any
  reported field — and bump `PROTOCOL_VERSION` and update HiveHub's parser in
  the same revision when you do (see `docs/ble-mode.md`).
* **`include/idle_state.h`** — the night-mode suspension deadline. The suite
  covers clamping an over-long request, the `millis()` rollover, and the case
  that matters most in the field: HiveHub stopping re-arming, after which the
  counter has to free itself. A counter stuck suspended emits perfectly
  well-formed documents full of zeros, which is indistinguishable from a spell
  of bad weather until someone reads a week of totals — so none of this is
  something a bench session would catch.
* **`include/bank_state.h`** — the emitter-bank enable mask. Every mistake it
  can make is silent: a bank that should be on but is off produces a
  permanently flat third of the totals, which reads exactly like a dead FET,
  and a bank that should be off but is on quietly costs ~80 mA on a supply
  sized without it. The suite pins the refusal of an all-off mask, the
  one-based bank numbering that `gates::TABLE[].led_bank` depends on, and the
  masking of bits above the last physical bank.
* **`include/device_name.h`** — the advertised BLE name, `HiveTraffic-AB:12`.
  The suffix is the last two bytes of the counter's own address, and every way
  of getting it wrong produces a name that still looks like a name while
  pointing at the wrong device: the suite pins the byte order (NimBLE stores
  addresses little-endian, so the two bytes are `val[1]`, `val[0]`), the
  two-digit uppercase rendering a scanner matches against, the fallback when no
  address can be read, and the refusal to truncate into a buffer that cannot
  hold the whole name.

Everything hardware-facing stays in `src/main.cpp` and is still verified on the
bench with the USB diagnostic console above.

---

## Wiring sanity check

When you power the board for the first time you should see, on the serial
monitor:

```
==============================================
Easy Bee Counter 2026 — firmware booting (BLE/GATT link)
==============================================
[MCP] U2 (gates 00..07) @ 0x20: OK
[MCP] U3 (gates 10..17) @ 0x21: OK
[MCP] U4 (gates 20..27) @ 0x22: OK
[BLE] HiveTraffic-AB:12 0.4.0 advertising for HiveHub
[SETUP] Entering normal counting loop (pulsed IR)
[CON] USB console ready - type 'help' and Enter
```

A chip that shows `NOT FOUND` is not written off: the firmware re-probes it
every `MCP_RETRY_INTERVAL_MS` and logs `recovered` if it starts answering, so a
marginal connection can come back without a power cycle. Its eight gates are
skipped (not counted as blocked) while it is down. If it never recovers, check:

- That the A0/A1/A2 strap pins of that chip are connected as the schematic
  says (U2=0x20: all GND; U3=0x21: A0→3V3; U4=0x22: A1→3V3).
- That the 4.7 k pull-ups R4/R5 are populated.
- That the chip's VDD pin 9 is at 3.3 V and VSS pin 10 is at GND.

After boot, type `events on` and pass something reflective (a piece of
paper) through a gate, outer side first: the gate's `outer_block`, then
`inner_block`, then a `bee_in` should appear at once, and the totals on the
next 30 s `[STAT]` line go up by one. `mcp` shows whether all three chips are
answering.
