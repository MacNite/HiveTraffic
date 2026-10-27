// ============================================================================
// HiveTraffic bee counter — ESP32-C6 mini firmware
// ----------------------------------------------------------------------------
// Author: rewritten 2026 for the ESP32-C6 mini board.
//
// 2026-08 hardware revision: the IR emitters are split across THREE MOSFET
// banks instead of two — one IRLB8721 per MCP23017, driven from silk D8/D9/D10
// (GPIO19/GPIO20/GPIO18). See pins.h for the full map.
//
// 2026-06 power revision: the IR emitters are PULSED instead of left on
// continuously. Each poll turns the emitter banks on only for the brief window
// needed to settle the phototransistors and read the MCP23017s, then turns
// them off again. At the default POLL_INTERVAL_MS=5 / LED_SETTLE_US=250 with a
// ~1.5 ms three-chip read, the emitter duty cycle drops from 100% to roughly
// 35%, and raising POLL_INTERVAL_MS lowers it proportionally. See the
// "Pulsed-LED sampling" section below.
//
// What this firmware does
// -----------------------
//   1. Continuously polls 24 entrance gates (each gate = Inner IR sensor +
//      Outer IR sensor) through three MCP23017 I2C port expanders on Wire.
//   2. Detects directional bee crossings:
//        Outer-then-Inner = bee entered the hive   -> "in" counter++
//        Inner-then-Outer = bee left the hive      -> "out" counter++
//   3. Maintains LIFETIME totals, and serves them over BLE/GATT to HiveHub,
//      which differences consecutive reads into per-interval counts.
//   4. Accepts a firmware image over the same GATT service (see ble_link.cpp),
//      verifying size and CRC-32 before swapping app slots.
//   5. Suspends 1-3 on request (night mode), so the emitters — which dominate
//      this board's power draw by an order of magnitude — are dark through the
//      hours European honey bees do not fly. See the section below.
//   6. Runs on a subset of its three emitter banks on request, so an entrance
//      narrower than 24 gates — or a supply that will not carry 24 — costs only
//      the banks it uses. See the section below.
//   7. Answers a passive diagnostic console on USB (src/diag_console.cpp):
//      status, per-gate readings, MCP23017 statistics, a live view and an
//      event stream, all read from what the sampler already produced.
//
// Emitter bank enables (protocol v5)
// ----------------------------------
// The 2026-08 revision put one IRLB8721 behind each MCP23017, which makes each
// bank independently switchable: bank 1 = U2 = gates 00..07, bank 2 = U3 =
// 10..17, bank 3 = U4 = 20..27. Measured on the 3.3 V rail with the pulsed
// sampler at its defaults, one bank draws ~0.14 A, two ~0.22 A and three
// ~0.30 A, so dropping a bank is worth roughly 80 mA continuously — about what
// a quarter of a night of night mode saves, except it applies all day.
//
// This is a configuration rather than a deadline, so unlike night mode it has
// no expiry; like night mode it is deliberately NOT persisted, and HiveHub
// re-asserts it every upload cycle. A reset therefore comes back counting on
// all 24 gates. The mask arithmetic lives in include/bank_state.h.
//
// The gates of a dark bank are SKIPPED, not merely unlit. An unpowered QRE1113
// is a bare phototransistor under a 100k pull-up, and direct sun into a hive
// entrance can pull one low; feeding those readings to the state machine would
// invent crossings on gates the operator switched off.
//
// Night mode (protocol v4)
// ------------------------
// Honey bees are diurnal: flight needs light, and stops below roughly 10 C
// regardless. Sampling 24 gates through the night therefore burns the largest
// item in the power budget to count nothing. HiveHub — which knows the time,
// and this board does not — writes a suspension DURATION to the control
// characteristic once per upload cycle for as long as its configured night
// window lasts; idle_state.h turns that into a deadline and this file stops
// pulsing and polling until it expires.
//
// It is deliberately NOT deep sleep. The emitters are >90% of the draw, so
// parking them captures essentially the whole saving, while staying awake keeps
// the counter readable, updatable and cancellable all night — none of which
// survives deep sleep — and keeps the lifetime totals in RAM where they belong.
// See docs/ble-mode.md for the reasoning and the numbers.
//
// The wired HiveScale link is gone
// --------------------------------
// Earlier revisions ran the C6's second I2C controller as a permanent slave at
// 0x30, serving a register map (lifetime totals, per-interval counters reset by
// CMD_LATCH, per-gate arrays, and an OTA-over-I2C block) to a HiveScale polling
// over J1. All of it has been removed: HiveHub reads counters over BLE only and
// dropped its wired client, so nothing spoke that protocol anymore.
//
// What that removal simplifies, and why the code no longer mentions it:
//   - No slave callbacks, no register pointer, no ISR-context response buffer.
//   - No interval or per-gate counters. The wire format is totals-only, so a
//     missed connection cannot lose traffic — there is nothing to latch and
//     nothing to reset.
//   - One I2C bus, used only as a master for the MCP23017s.
// ============================================================================

#include <Arduino.h>
#include <Wire.h>
#include <esp_timer.h>

#include "pins.h"
#include "counter_protocol.h"
#include "gate_logic.h"
#include "idle_state.h"
#include "bank_state.h"
#include "poll_timing.h"
#include "serial_log.h"
#include "diag.h"
#include "diag_console.h"
#include "diag_format.h"
#include "maint_state.h"

// The BLE/GATT transport: a connectable NimBLE GATT server serving the
// measurement characteristic and the OTA characteristics. See ble_link.h.
#include "ble_link.h"

// ============================================================================
// Compile-time tuning knobs — keep these together for easy field adjustment
// ============================================================================

// How often (ms) we poll the MCP23017s for crossing detection. With a
// dedicated master bus we can poll continuously; 5 ms keeps CPU use modest
// while still catching the ~12 ms beam dwell of a crossing bee.
//
// With pulsed LEDs, this is also the emitter pulse PERIOD: the LEDs are lit
// for (LED_SETTLE_US + the three-chip read time) once every POLL_INTERVAL_MS.
// Raising it lowers both CPU use and emitter duty cycle, at the cost of coarser
// timing resolution on a crossing. 5 ms still resolves a ~12 ms beam dwell into
// ~2-3 samples, which is enough for the inner/outer ordering to be reliable.
static constexpr uint32_t POLL_INTERVAL_MS = 5;

// Minimum time a sensor must hold a NEW value before that value is accepted as
// the debounced state. See gate_logic.h for how the debounce is implemented and
// why the previous version did not actually enforce this.
//
// This has to be read together with SENSOR_DEBOUNCE_SAMPLES below and with
// POLL_INTERVAL_MS: a threshold shorter than one poll period cannot reject
// anything, because the first differing sample already satisfies it. The old
// 3 ms value was exactly that case (3 < 5), which is why the sample count is
// the load-bearing half of the rule at the current poll rate.
static constexpr uint32_t SENSOR_DEBOUNCE_MS = 5;

// How many consecutive polls must agree before a sensor change is accepted.
//
// Two is the most the beam physics allow here. A bee crossing at ~25 cm/s
// dwells ~12 ms in a 3 mm beam, i.e. 2-3 samples at POLL_INTERVAL_MS=5, so
// requiring three consecutive agreeing samples (~10 ms) would start dropping
// real crossings. Two rejects the single corrupted or noisy word — the failure
// mode the old code was blind to — while accepting a genuine block ~5 ms after
// it first appears, comfortably inside the dwell.
//
// If you ever raise POLL_INTERVAL_MS, this must stay at 2 (or the debounce
// window grows past the dwell and crossings are silently lost); if you lower it
// well below 5 ms, raise this instead of SENSOR_DEBOUNCE_MS.
static constexpr uint8_t SENSOR_DEBOUNCE_SAMPLES = 2;

// Maximum elapsed time between the two sensors of one gate triggering for
// the pair to count as a directional crossing. Anything longer is just a
// bee sitting in the tunnel and is discarded.
static constexpr uint32_t GATE_PAIRING_WINDOW_MS = 2000;

// If a sensor is continuously "blocked" for longer than this, we flag the
// sensor-fault status bit. A bee cannot physically block a beam for 30 s.
static constexpr uint32_t SENSOR_STUCK_MS = 30000;

// A gap between two poll starts longer than this counts as a LATE poll (see
// poll_timing.h). Two full intervals: one late poll at that size still leaves
// the debounce its two samples inside a ~12 ms beam dwell, anything longer
// starts to eat into it. Used only for reporting — it changes no behaviour.
static constexpr uint32_t LATE_POLL_GAP_US = 2 * POLL_INTERVAL_MS * 1000;

// A gate's raw reading is reported as STALE on the console once its chip's last
// good read is older than this. Twenty poll intervals: comfortably longer than
// any normal jitter, short enough that a paused sampler is never mistaken for
// a live one.
static constexpr uint32_t SNAPSHOT_STALE_MS = 100;

// How often the periodic [STAT] line (and, when there is something to say, the
// [TIMING] line) is printed.
static constexpr uint32_t STAT_INTERVAL_MS = 30000;

// Run the short, on-board MCP23017 bus at its 400 kHz fast-mode limit. The IR
// emitters stay lit for the complete three-chip read, so this reduces their
// on-time and CPU blocking roughly 3-4x versus the former 100 kHz default. Fall
// back to 100 kHz if an assembled board cannot meet fast-mode signal integrity.
static constexpr uint32_t I2C_MASTER_HZ = 400000;

// ---------------------------------------------------------------------------
// Pulsed-LED sampling
// ---------------------------------------------------------------------------
// The QRE1113 reflective sensor's phototransistor needs a short settle time
// after its IR emitter switches on before the collector voltage is valid. The
// device's optical rise/fall times are on the order of ~10 us; 250 us is a
// comfortable, conservative margin that also covers the RC settling of the
// 100k MCP pull-up against the phototransistor + any board capacitance.
//
// The sampling sequence each poll is:
//     1. turn ALL emitter banks ON
//     2. busy-wait LED_SETTLE_US for the phototransistors to settle
//     3. read all three MCP23017s (emitters stay on across the whole read)
//     4. turn ALL emitter banks OFF
//
// All banks are pulsed together (not per-bank) so that a single readGPIOAB()
// sweep of all three chips sees every gate correctly lit. Since the 2026-08
// revision each bank feeds exactly one MCP23017, so a per-bank read is now
// possible in principle — but it would triple the number of settle windows
// (each ~250 us) for no benefit, since the three chips are read back to back
// anyway. Average emitter current ≈ peak * (settle + read) /
// (POLL_INTERVAL_MS * 1000). With the defaults that is ~35% of the old
// always-on draw; raise POLL_INTERVAL_MS and/or I2C_MASTER_HZ to push lower.
//
// FORCE_ON keeps the old continuous behaviour (useful for bench/oscilloscope
// work); FORCE_OFF blacks the emitters out entirely (counts will read clear);
// AUTO is the new pulsed mode and the default.
static constexpr uint32_t LED_SETTLE_US = 250;

// ============================================================================
// Per-gate state machine
// ============================================================================
//
// We model each gate as a tiny state machine. The two sensors (Inner, Outer)
// each can be in one of two debounced states: CLEAR (HIGH, beam clear) or
// BLOCKED (LOW, body in beam). Transitions to BLOCKED are timestamped. When
// the second sensor of a pair becomes BLOCKED within GATE_PAIRING_WINDOW_MS
// of the first, we emit a directional event:
//
//     first BLOCKED = Outer -> then Inner BLOCKED  =>  INCOMING bee
//     first BLOCKED = Inner -> then Outer BLOCKED  =>  OUTGOING bee
//
// After an event the gate resets to IDLE only once BOTH sensors have
// returned to CLEAR (each having passed the debounce rule above).
//
// The debounce and the state machine themselves live in include/gate_logic.h,
// free of Arduino dependencies, so the timing rules can be exercised by the
// host-side tests in test/test_gate_logic/ instead of only on a live hive.
// ============================================================================

static gatelogic::GateRuntime g_gate_rt[gates::NUM_GATES];

static constexpr gatelogic::Tuning GATE_TUNING = {
    SENSOR_DEBOUNCE_MS,
    SENSOR_DEBOUNCE_SAMPLES,
    GATE_PAIRING_WINDOW_MS,
    SENSOR_STUCK_MS,
};

// Aggregate counters. LIFETIME totals only: the BLE contract is totals-only and
// HiveHub derives each interval by differencing consecutive reads, so a missed
// connection can never lose traffic. (The wired path additionally kept
// per-interval and per-gate counters, zeroed by CMD_LATCH; both went with it.)
static volatile uint32_t g_total_in  = 0;
static volatile uint32_t g_total_out = 0;

// 32-bit since protocol v3. It saturates rather than wrapping either way, but
// as a uint16_t a noisy device pinned at 65535 was reporting a bounded number
// that said nothing about how bad things had got.
static volatile uint32_t g_glitch_count  = 0;
static volatile uint8_t  g_status_flags  = 0;

// Night-mode suspension. Owned here because this is where the emitters and the
// poll loop live; armed from the BLE control characteristic through
// ble::applyIdleRequest() below. Never persisted: a reset resumes counting.
static idlestate::State g_idle;

// Which emitter banks are allowed to light. Owned here for the same reason
// g_idle is — this file drives the FET gates — and armed from the BLE control
// characteristic through ble::applyBankMask() below. Never persisted: a reset
// comes back with all three banks counting, and HiveHub re-asserts the mask on
// its next upload cycle. See bank_state.h.
static bankstate::State g_banks;

// Maintenance session (USB console `maint on`). Owned here for the same reason
// g_idle and g_banks are: while it runs, this file routes crossings into test
// counters and holds HiveHub's night-mode and bank requests back. Never
// persisted: a reset always comes back counting. See maint_state.h.
static maintstate::State g_maint;

// ============================================================================
// MCP23017 channels + runtime health  (all on Wire / bus 0)
// ============================================================================
//
// Health used to be decided once, in setup(), and never revisited. That made
// two failure modes invisible:
//
//   * a chip absent at boot could never come back without a power cycle, and
//   * a chip that died AFTER boot kept being reported healthy, while its port
//     word read as all-zeros — which this firmware interprets as "every beam
//     blocked" — so its eight gates silently pinned into PAIRED, inflated the
//     glitch tally and eventually latched the sensor-fault flag.
//
// So each chip now carries a small health record: consecutive failures (one
// transient NAK is not a dead chip), a validity flag for the current poll's
// snapshot, and a retry deadline. Gates are only fed from a snapshot marked
// valid, and the STATUS_MCP_U*_OK bits plus the mcps_healthy telemetry now track
// the live state instead of boot-time discovery.
static constexpr uint8_t NUM_MCP = 3;

// Consecutive failed reads before a chip is declared unhealthy. Its gates are
// skipped from the first failure regardless; this only controls when we stop
// trusting the chip altogether, tear its gates down and start re-probing.
static constexpr uint8_t MCP_FAIL_THRESHOLD = 3;

// How often an unhealthy chip is re-probed. Each attempt is one addressed
// transaction that a missing chip NAKs immediately, so this is cheap; it runs
// before the emitters are lit so a failed probe never extends the LED pulse.
static constexpr uint32_t MCP_RETRY_INTERVAL_MS = 5000;

// MCP23017 register addresses with IOCON.BANK = 0, the power-on default and the
// value initMcp() writes. GPIOB follows GPIOA at 0x13 and the chip's
// sequential-read mode returns both from one 2-byte read, issued directly so
// the I2C transaction result is visible: a driver call that returns a bare
// uint16_t reports a failed bus transaction as 0x0000, i.e. as "all sensors
// blocked". IODIRA..GPPUB are 0x00..0x0D, so one 14-byte read covers every
// configuration register.
//
// The chips are driven with plain Wire transactions throughout. (The Adafruit
// driver used to be called once per chip at boot and again on every recovery,
// and its begin_I2C() allocates a new bus object each time without freeing the
// last — a chip that kept dropping out leaked heap at every return.)
static constexpr uint8_t MCP23X17_REG_GPIOA = 0x12;
static constexpr uint8_t MCP_REG_IODIRA = 0x00;
static constexpr uint8_t MCP_REG_IODIRB = 0x01;
static constexpr uint8_t MCP_REG_IPOLA  = 0x02;
static constexpr uint8_t MCP_REG_IPOLB  = 0x03;
static constexpr uint8_t MCP_REG_IOCON  = 0x0A;
static constexpr uint8_t MCP_REG_GPPUA  = 0x0C;
static constexpr uint8_t MCP_REG_GPPUB  = 0x0D;
static constexpr uint8_t MCP_CONFIG_BYTES = 14;

// Read statistics for the diagnostic console's `mcp` command. Counted by the
// normal sampler as it goes; nothing extra is read to produce them. All
// saturating.
struct McpStats {
    uint32_t reads_ok = 0;
    uint32_t nacks = 0;          // address or register phase not acknowledged
    uint32_t short_reads = 0;    // fewer than the two requested bytes
    uint32_t lost = 0;           // times declared unhealthy
    uint32_t recovered = 0;      // times re-probed back to healthy
    uint32_t max_read_us = 0;    // slowest successful read
    uint16_t last_word = 0;      // last successfully read port word
    bool     have_ok = false;    // has any read ever succeeded?
    uint32_t last_ok_ms = 0;     // millis() of that read
};

struct McpHealth {
    uint8_t     addr;
    uint8_t     status_bit;
    const char* tag;
    const char* name;           // "U2", for compact console output
    bool        ok;             // chip is believed present and answering
    uint8_t     fail_streak;    // consecutive failed reads
    uint32_t    next_retry_ms;  // when to re-probe while !ok
    uint16_t    value;          // this poll's port word (GPIOA | GPIOB << 8)
    bool        valid;          // is `value` from a successful read this poll?
    // Set when the chip was declared lost inside the lit sampling window; the
    // message is printed by pollAllGates() once the emitters are dark again,
    // so logging never lengthens an LED pulse.
    bool        report_lost;
    McpStats    stats;
};

static McpHealth g_mcp[NUM_MCP] = {
    { i2c_addr::MCP_GATES_00_07, beecounter_proto::STATUS_MCP_U2_OK,
      "U2 (gates 00..07)", "U2", false, 0, 0, 0, false, false, McpStats() },
    { i2c_addr::MCP_GATES_10_17, beecounter_proto::STATUS_MCP_U3_OK,
      "U3 (gates 10..17)", "U3", false, 0, 0, 0, false, false, McpStats() },
    { i2c_addr::MCP_GATES_20_27, beecounter_proto::STATUS_MCP_U4_OK,
      "U4 (gates 20..27)", "U4", false, 0, 0, 0, false, false, McpStats() },
};

static void saturatingInc(uint32_t& v) {
    if (v != UINT32_MAX) v++;
}

// gates::TABLE identifies a chip by I2C address; the health records are indexed.
static int8_t mcpIndexForAddress(uint8_t addr) {
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        if (g_mcp[i].addr == addr) return (int8_t)i;
    }
    return -1;
}

// LED-bank control.
// AUTO now PULSES the LEDs: they are lit only for the settle+read window of
// each poll (see pollAllGates / sampleGatesPulsed). FORCE_ON restores the old
// always-on behaviour; FORCE_OFF keeps them dark.
// The IRLB8721 N-FET is logic-level and just needs a digital HIGH on its gate.
enum class LedMode : uint8_t { AUTO, FORCE_ON, FORCE_OFF };
static volatile LedMode g_led_mode = LedMode::AUTO;

// ============================================================================
// Low-level helpers
// ============================================================================

// Seconds since boot, from the 64-bit microsecond timer. millis() is 32-bit and
// wraps after ~49.7 days, which made uptime restart on a counter that never
// rebooted — and uptime restarting is exactly what is supposed to mean a
// reboot. The 32-bit second count this returns lasts ~136 years.
static uint32_t uptimeSeconds() {
    return (uint32_t)(esp_timer_get_time() / 1000000LL);
}

// Drive every emitter-bank MOSFET to the same state and update the status bit.
// This is the raw control used by FORCE_ON/FORCE_OFF and by the pulsed sampler.
// It does NOT consult g_led_mode (the caller decides), so the pulsed sampler
// can momentarily turn the LEDs on/off within AUTO mode without fighting the
// mode gate.
static void driveIrLeds(bool on) {
    // A disabled bank's FET gate is held LOW whatever `on` says: this is the
    // one place that decides whether an emitter rail is ever energised, so
    // enforcing the mask here means no other path — the pulsed sampler, the
    // FORCE_ON debug mode, the night-mode backstop — can light a bank the
    // operator switched off.
    for (uint8_t b = 0; b < pins::NUM_LED_BANKS; b++) {
        const bool live = on && bankstate::enabled(g_banks, (uint8_t)(b + 1));
        digitalWrite(pins::IR_LED_BANK_EN[b], live ? HIGH : LOW);
    }
    if (on) g_status_flags |=  beecounter_proto::STATUS_IR_LEDS_ON;
    else    g_status_flags &= ~beecounter_proto::STATUS_IR_LEDS_ON;
}

// Apply a steady LED state honouring the mode overrides. Used when the mode
// changes and at boot. In AUTO mode the steady state is
// OFF — the emitters are only lit transiently by the pulsed sampler — so a
// call here with on=true while in AUTO is treated as "leave pulsing to the
// sampler" and forces the steady level OFF.
static void setIrLeds(bool on) {
    if (g_led_mode == LedMode::FORCE_OFF) on = false;
    else if (g_led_mode == LedMode::FORCE_ON) on = true;
    else /* AUTO */ on = false;   // pulsed: steady level is OFF between samples
    driveIrLeds(on);
}


// ============================================================================
// MCP23017 acquisition + health
// ============================================================================

static bool writeMcpReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// Read IODIRA..GPPUB in one sequential read.
static bool readMcpConfig(uint8_t addr, uint8_t (&r)[MCP_CONFIG_BYTES]) {
    Wire.beginTransmission(addr);
    Wire.write(MCP_REG_IODIRA);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)addr, (int)MCP_CONFIG_BYTES) != MCP_CONFIG_BYTES) {
        return false;
    }
    for (uint8_t k = 0; k < MCP_CONFIG_BYTES; k++) r[k] = (uint8_t)Wire.read();
    return true;
}

static bool mcpConfigMatches(const uint8_t (&r)[MCP_CONFIG_BYTES]) {
    return diagfmt::mcpConfigOk(r[MCP_REG_IODIRA], r[MCP_REG_IODIRB],
                                r[MCP_REG_IPOLA], r[MCP_REG_IPOLB],
                                r[MCP_REG_GPPUA], r[MCP_REG_GPPUB],
                                r[MCP_REG_IOCON]);
}

// Put a chip into the configuration the sampler relies on, then read it back.
// An ACK only proves that something answers at the address; a chip is trusted
// only once its registers say what was written.
//
// All 16 pins are sensor inputs. The board provides 100k pull-ups, so the
// MCP's weak internal pull-ups stay off. IOCON goes first: BANK = 0 fixes the
// register map everything here addresses, SEQOP = 0 the sequential reads the
// sampler and the read-back depend on. Each register is written on its own so
// none of this assumes SEQOP before it is set.
static bool initMcp(uint8_t addr, const char* tag, bool quiet = false) {
    const bool written =
        writeMcpReg(addr, MCP_REG_IOCON, 0x00) &&
        writeMcpReg(addr, MCP_REG_IODIRA, 0xFF) &&
        writeMcpReg(addr, MCP_REG_IODIRB, 0xFF) &&
        writeMcpReg(addr, MCP_REG_IPOLA, 0x00) &&
        writeMcpReg(addr, MCP_REG_IPOLB, 0x00) &&
        writeMcpReg(addr, MCP_REG_GPPUA, 0x00) &&
        writeMcpReg(addr, MCP_REG_GPPUB, 0x00);
    uint8_t r[MCP_CONFIG_BYTES] = {0};
    if (!written) {
        if (!quiet) slog::printf("[MCP] %s @ 0x%02X: NOT FOUND\n", tag, addr);
        return false;
    }
    if (!readMcpConfig(addr, r) || !mcpConfigMatches(r)) {
        if (!quiet) {
            slog::printf("[MCP] %s @ 0x%02X: configuration not confirmed\n", tag, addr);
        }
        return false;
    }
    if (!quiet) slog::printf("[MCP] %s @ 0x%02X: OK\n", tag, addr);
    return true;
}

// Drop every gate on this chip back to a clean IDLE. Called whenever a chip
// changes health in either direction, so that a half-finished pairing from
// before an outage cannot combine with a sensor reading from after it and
// fabricate a crossing. The counters are lifetime totals and are untouched.
static void resetGatesForMcp(uint8_t addr) {
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        if (gates::TABLE[i].mcp_address == addr) {
            g_gate_rt[i] = gatelogic::GateRuntime();
        }
    }
}

// Drop every gate on one emitter bank back to IDLE. Called when a bank is
// switched off (its gates stop being sampled, so a half-finished pairing would
// otherwise sit there indefinitely) and when one is switched back on (the
// pairing predates however long the bank was dark). Same reasoning as
// resetGatesForMcp() across a chip outage; the lifetime totals are untouched.
static void resetGatesForBank(uint8_t bank) {
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        if (gates::TABLE[i].led_bank == bank) {
            g_gate_rt[i] = gatelogic::GateRuntime();
        }
    }
}

// Drop every gate back to IDLE. Used on both edges of a night-mode suspension:
// a gate that was half-way through a pairing when sensing stopped must not
// combine that stale half with the first sample taken hours later and fabricate
// a crossing. Same reasoning as resetGatesForMcp() across a chip outage — the
// lifetime totals are untouched either way.
static void resetAllGates() {
    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        g_gate_rt[i] = gatelogic::GateRuntime();
    }
}

// One checked 2-byte read of GPIOA/GPIOB. Unlike readGPIOAB() this reports bus
// failures instead of silently returning 0x0000 ("all blocked").
//
// Also keeps the chip's read statistics (McpStats) — the failure kind and the
// transaction time are only visible here, and recording them costs a couple of
// increments and a micros() call.
static bool readMcpChecked(uint8_t idx, uint16_t& out) {
    McpHealth& m = g_mcp[idx];
    const uint32_t t0 = micros();
    Wire.beginTransmission(m.addr);
    Wire.write(MCP23X17_REG_GPIOA);
    if (Wire.endTransmission(false) != 0) {   // repeated start
        saturatingInc(m.stats.nacks);
        return false;
    }
    if (Wire.requestFrom((int)m.addr, (int)2) != 2) {
        saturatingInc(m.stats.short_reads);
        return false;
    }
    const uint8_t a = (uint8_t)Wire.read();   // GPIOA -> inner sensors
    const uint8_t b = (uint8_t)Wire.read();   // GPIOB -> outer sensors
    out = (uint16_t)a | ((uint16_t)b << 8);
    const uint32_t dt = (uint32_t)(micros() - t0);
    saturatingInc(m.stats.reads_ok);
    if (dt > m.stats.max_read_us) m.stats.max_read_us = dt;
    m.stats.last_word = out;
    m.stats.have_ok = true;
    m.stats.last_ok_ms = millis();
    return true;
}

// Fold one poll's read result into a chip's health record.
static void noteMcpResult(uint8_t idx, bool read_ok, uint32_t now_ms) {
    McpHealth& m = g_mcp[idx];
    if (read_ok) {
        m.fail_streak = 0;
        return;
    }
    if (m.fail_streak < 0xFF) m.fail_streak++;
    if (m.ok && m.fail_streak >= MCP_FAIL_THRESHOLD) {
        m.ok = false;
        m.valid = false;
        g_status_flags &= (uint8_t)~m.status_bit;
        m.next_retry_ms = now_ms + MCP_RETRY_INTERVAL_MS;
        resetGatesForMcp(m.addr);
        saturatingInc(m.stats.lost);
        // Called with the emitters lit: defer the message until they are dark.
        m.report_lost = true;
    }
}

// Does anything ACK at this address? One addressed byte, no data phase.
static bool mcpPresent(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// The standard I2C bus clear, for a slave left holding SDA low part-way through
// a byte: clock SCL until SDA is released (nine pulses finish any byte), send a
// STOP, and hand the pins back to the controller. The datasheet's warning about
// GPA7/GPB7 inputs (see pins.h) is exactly this kind of SDA disruption, and a
// wedged bus would otherwise keep all 24 gates dark until a power cycle.
// Takes ~150 us with the emitters dark.
static void recoverI2cBus() {
    Wire.end();
    pinMode(pins::I2C_SDA, INPUT_PULLUP);
    pinMode(pins::I2C_SCL, OUTPUT_OPEN_DRAIN);
    digitalWrite(pins::I2C_SCL, HIGH);
    delayMicroseconds(5);
    for (uint8_t i = 0; i < 9 && digitalRead(pins::I2C_SDA) == LOW; i++) {
        digitalWrite(pins::I2C_SCL, LOW);
        delayMicroseconds(5);
        digitalWrite(pins::I2C_SCL, HIGH);
        delayMicroseconds(5);
    }
    // STOP: SDA low while SCL is low, then release SCL, then SDA.
    digitalWrite(pins::I2C_SCL, LOW);
    delayMicroseconds(5);
    pinMode(pins::I2C_SDA, OUTPUT_OPEN_DRAIN);
    digitalWrite(pins::I2C_SDA, LOW);
    delayMicroseconds(5);
    digitalWrite(pins::I2C_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(pins::I2C_SDA, HIGH);
    delayMicroseconds(5);
    Wire.begin(pins::I2C_SDA, pins::I2C_SCL, (uint32_t)I2C_MASTER_HZ);
}

// When every expander is lost at once, the likely fault is the shared bus, not
// three chips. Clear it then, and again once per MCP_RETRY_INTERVAL_MS for as
// long as the outage lasts, so each round of re-probes runs on a freed bus.
// A single chip that still answers proves the bus works, and nothing happens.
static bool     g_bus_outage = false;
static uint32_t g_bus_recover_at_ms = 0;

static void serviceBusRecovery(uint32_t now_ms) {
    bool all_lost = true;
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        if (g_mcp[i].ok) all_lost = false;
    }
    if (!all_lost) {
        g_bus_outage = false;
        return;
    }
    if (!g_bus_outage) {
        g_bus_outage = true;
        g_bus_recover_at_ms = now_ms;
        slog::printf("[MCP] no expander answering; bus recovery before each re-probe");
    }
    if ((int32_t)(now_ms - g_bus_recover_at_ms) < 0) return;
    g_bus_recover_at_ms = now_ms + MCP_RETRY_INTERVAL_MS;
    recoverI2cBus();
}

// Re-probe an unhealthy chip, at most once per MCP_RETRY_INTERVAL_MS. Runs with
// the emitters dark, before the sampling window.
static void retryMcp(uint8_t idx, uint32_t now_ms) {
    McpHealth& m = g_mcp[idx];
    if (m.ok) return;
    if ((int32_t)(now_ms - m.next_retry_ms) < 0) return;
    m.next_retry_ms = now_ms + MCP_RETRY_INTERVAL_MS;

    // Cheap ACK probe first, so the configure-and-verify only runs when the
    // chip is genuinely back.
    if (!mcpPresent(m.addr)) return;

    // quiet: a chip that ACKs but fails to configure would otherwise log on
    // every retry. It stays unhealthy until its registers read back right.
    if (!initMcp(m.addr, m.tag, /*quiet=*/true)) return;

    m.ok = true;
    m.fail_streak = 0;
    g_status_flags |= m.status_bit;
    resetGatesForMcp(m.addr);
    saturatingInc(m.stats.recovered);
    slog::printf("[MCP] %s @ 0x%02X: recovered\n", m.tag, m.addr);
}

// ============================================================================
// Crossing detection
// ============================================================================

// Saturating lifetime totals: see gatelogic::countSaturating. The flag is
// raised as soon as a counter reaches its maximum, and it stays pinned there
// rather than wrapping to zero behind HiveHub's back.
static void countCrossing(volatile uint32_t& total) {
    if (gatelogic::countSaturating(total)) {
        g_status_flags |= beecounter_proto::STATUS_OVERFLOW_FLAG;
    }
}

// Read every healthy MCP23017 into its health record. The emitters must
// already be lit and settled before this is called.
//
// Each record's `valid` flag says whether THIS poll's word can be trusted. A
// chip that is unhealthy, or that failed its read this cycle, is left invalid
// and its gates are skipped entirely — the old code left the word at zero and
// fed it to the state machine anyway, which reads as "all 16 beams blocked".
//
// Returns false if any chip did not produce a usable word.
static bool readAllMcp(uint32_t now_ms) {
    bool all_ok = true;
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        McpHealth& m = g_mcp[i];
        m.valid = false;
        m.value = 0;
        if (!m.ok) {
            all_ok = false;
            continue;
        }
        uint16_t v = 0;
        const bool read_ok = readMcpChecked(i, v);
        if (read_ok) {
            m.value = v;
            m.valid = true;
        } else {
            all_ok = false;
        }
        noteMcpResult(i, read_ok, now_ms);
    }
    return all_ok;
}

// Acquire one fresh set of sensor readings with the emitters pulsed on only for
// the settle + read window. In FORCE_ON the emitters are already steady-on, so
// we skip the extra toggling. In FORCE_OFF we never light them (readings will
// look "clear"), preserving the diagnostic meaning of that mode.
static bool sampleGates(uint32_t now_ms) {
    switch (g_led_mode) {
    case LedMode::FORCE_OFF:
        // Emitters stay dark; read whatever the sensors show (nominally clear).
        return readAllMcp(now_ms);

    case LedMode::FORCE_ON:
        // Emitters are already steady-on (set when the mode was entered); just
        // read. No per-sample toggling so a scope trace shows a clean DC level.
        return readAllMcp(now_ms);

    case LedMode::AUTO:
    default:
        // Pulsed path: light all banks, let the phototransistors settle, read
        // across the lit window, then black the emitters out again.
        driveIrLeds(true);
        delayMicroseconds(LED_SETTLE_US);
        {
            bool ok = readAllMcp(now_ms);
            driveIrLeds(false);
            return ok;
        }
    }
}

// Poll all three MCP23017s and update every gate whose snapshot is trustworthy.
// Returns false if any chip did not produce a usable word this cycle; gates on
// such a chip are skipped rather than fed a fabricated all-blocked reading.
static bool pollAllGates() {
    const uint32_t now_ms = millis();

    // Re-probe anything currently marked unhealthy. Done first, with the
    // emitters still dark, so a probe never lengthens the LED pulse.
    serviceBusRecovery(now_ms);
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        retryMcp(i, now_ms);
    }

    // Acquire a fresh sensor snapshot with the emitters pulsed (AUTO) or steady
    // (FORCE_ON) per the current LED mode. Each chip's word is GPIOA in the low
    // byte and GPIOB in the high byte.
    const bool ok = sampleGates(now_ms);

    // The emitters are dark again; now it is safe to spend time printing.
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        McpHealth& m = g_mcp[i];
        if (!m.report_lost) continue;
        m.report_lost = false;
        slog::printf("[MCP] %s @ 0x%02X: lost after %u failed reads\n",
                     m.tag, m.addr, (unsigned)m.fail_streak);
    }

    // Named getBit to avoid clashing with Arduino.h's bit(b) macro.
    auto getBit = [](uint16_t v, uint8_t pin) -> bool {
        return (v >> pin) & 0x1;
    };

    for (uint8_t i = 0; i < gates::NUM_GATES; i++) {
        const auto& loc = gates::TABLE[i];
        // A gate on a disabled bank is skipped, not read as "clear": its
        // emitters are dark, so the phototransistor is only reporting ambient
        // light, and sun into the entrance would fabricate crossings on gates
        // the operator deliberately switched off. The chip itself is still read
        // above, so mcps_healthy keeps meaning "expanders answering".
        if (!bankstate::enabled(g_banks, loc.led_bank)) continue;
        const int8_t ch = mcpIndexForAddress(loc.mcp_address);
        if (ch < 0 || !g_mcp[ch].valid) continue;   // no trustworthy sample
        const uint16_t v = g_mcp[ch].value;

        // QRE1113 phototransistor: BLOCKED -> sensor line LOW (bit=0).
        // We pass "blocked = (bit == 0)" into the state machine.
        const bool inner_blocked = !getBit(v, loc.inner_pin);
        const bool outer_blocked = !getBit(v, loc.outer_pin);

        gatelogic::GateRuntime& rt = g_gate_rt[i];
        const bool inner_was = rt.inner.accepted;
        const bool outer_was = rt.outer.accepted;

        const gatelogic::GateUpdate u = gatelogic::updateGate(
            rt, inner_blocked, outer_blocked, now_ms, GATE_TUNING);

        // Debounced edges and counted crossings for the console's `events`
        // stream. A copy into a ring, and only while someone asked for it.
        if (console::eventsEnabled()) {
            if (rt.inner.accepted != inner_was) {
                console::noteEvent(now_ms, i, rt.inner.accepted
                    ? eventring::Kind::INNER_BLOCK : eventring::Kind::INNER_CLEAR);
            }
            if (rt.outer.accepted != outer_was) {
                console::noteEvent(now_ms, i, rt.outer.accepted
                    ? eventring::Kind::OUTER_BLOCK : eventring::Kind::OUTER_CLEAR);
            }
            switch (u.event) {
            case gatelogic::GateEvent::BEE_IN:
                console::noteEvent(now_ms, i, eventring::Kind::BEE_IN);
                break;
            case gatelogic::GateEvent::BEE_OUT:
                console::noteEvent(now_ms, i, eventring::Kind::BEE_OUT);
                break;
            case gatelogic::GateEvent::GLITCH:
                console::noteEvent(now_ms, i, eventring::Kind::GLITCH);
                break;
            case gatelogic::GateEvent::NONE:
            default:
                break;
            }
        }

        // In maintenance, crossings are test stimuli: they go to the
        // session's own counters and never reach the lifetime totals HiveHub
        // differences into traffic.
        if (g_maint.active) {
            switch (u.event) {
            case gatelogic::GateEvent::BEE_IN:  maintstate::saturatingInc(g_maint.test_in);     break;
            case gatelogic::GateEvent::BEE_OUT: maintstate::saturatingInc(g_maint.test_out);    break;
            case gatelogic::GateEvent::GLITCH:  maintstate::saturatingInc(g_maint.test_glitch); break;
            case gatelogic::GateEvent::NONE:
            default:
                break;
            }
            // Nor is a target held over a sensor for half a minute a fault:
            // the flag latches until reset, so a test must not raise it.
            continue;
        }

        switch (u.event) {
        case gatelogic::GateEvent::BEE_IN:  countCrossing(g_total_in);  break;
        case gatelogic::GateEvent::BEE_OUT: countCrossing(g_total_out); break;
        case gatelogic::GateEvent::GLITCH:
            // Saturating: a wrapping diagnostic counter can make a badly
            // unhealthy device look healthier than it did on the last read.
            gatelogic::countSaturating(g_glitch_count);
            break;
        case gatelogic::GateEvent::NONE:
        default:
            break;
        }

        if (u.sensor_stuck) {
            g_status_flags |= beecounter_proto::STATUS_SENSOR_FAULT_FLAG;
        }
    }

    return ok;
}


// ----------------------------------------------------------------------------
// BLE transport callbacks (defined here because the counter state lives in this
// translation unit). ble_link.cpp drives the GATT server and calls into these.
//
// They run on the NimBLE host task, which on this single-core chip outranks
// loopTask and can preempt it anywhere — in the middle of a lit sampling
// window, half-way through one gate's state update, or between a read and a
// write of g_status_flags. So nothing here changes counter state. The
// telemetry and read-back calls only read it; the two control writes are
// posted to a mailbox that loop() applies between polls (see
// serviceControlRequests() below), which makes loop() the one owner of the
// gates, the banks, night mode and the maintenance session.
// ----------------------------------------------------------------------------

// Control writes waiting for loop(). Latest wins within each kind, which is
// what both mean: a RESUME after a SET_IDLE in the same instant is a resume,
// and of two masks the second is the one HiveHub wants. Guarded by a critical
// section rather than a mutex because it is held for a few word copies.
struct ControlMailbox {
    bool     idle_pending = false;
    uint32_t idle_s = 0;
    bool     banks_pending = false;
    uint8_t  banks = 0;
};
static ControlMailbox g_ctrl_mailbox;
static portMUX_TYPE g_ctrl_mux = portMUX_INITIALIZER_UNLOCKED;

namespace ble {

void getTelemetry(Telemetry& t) {
    // No clamp since protocol v3: uptime_s is a uint32_t on both sides of the
    // link, and since 0.5.1 it no longer restarts with the millis() rollover
    // either (see uptimeSeconds()), so it only ever goes back on a real reset.
    const uint32_t up = uptimeSeconds();
    // Live health, not the boot-time snapshot this used to report: a chip that
    // died after setup() now shows up here (and in the STATUS_MCP_U*_OK bits),
    // and one that recovers is counted again.
    uint8_t healthy = 0;
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        if (g_mcp[i].ok) healthy++;
    }
    // Derived here, not read straight out of g_status_flags, so the bit and the
    // countdown can never disagree in a published document: the deadline can
    // pass between a GATT read and the loop() pass that services the expiry,
    // and "STATUS_NIGHT_IDLE set, idle_s 0" is a contradiction HiveHub would
    // have to guess its way out of.
    const uint32_t idle_left = idlestate::remainingSeconds(g_idle, millis());
    const uint8_t status = idle_left
        ? (uint8_t)(g_status_flags | beecounter_proto::STATUS_NIGHT_IDLE)
        : (uint8_t)(g_status_flags & ~beecounter_proto::STATUS_NIGHT_IDLE);

    t.protocol_version = beecounter_proto::PROTOCOL_VERSION;
    t.status_flags     = status;
    t.uptime_s         = up;
    t.num_gates        = gates::NUM_GATES;
    t.mcps_healthy     = healthy;
    t.total_in         = g_total_in;
    t.total_out        = g_total_out;
    t.glitch_count     = g_glitch_count;
    t.idle_s           = idle_left;
    // Reported unconditionally, including the 0x07 of a counter nobody has
    // reconfigured. A consumer that only saw the field when it was interesting
    // could not tell "all banks on" from "counter too old to say", and those
    // are opposite readings of the same flat totals.
    t.bank_mask        = g_banks.mask;
    // Lifetime seconds in maintenance, including a session still running.
    // Monotonic like the totals, so HiveHub can difference it: any increase
    // over an interval means that interval's totals were frozen for a test.
    t.maint_s          = maintstate::totalSeconds(g_maint, millis());
}

}  // namespace ble

// Apply a night-mode request immediately. HiveHub's requests reach it through
// handleIdleRequest() below, outside maintenance; ending a maintenance session
// replays a deferred request through it. loop() only, like everything that
// changes the gates or the emitters.
static uint32_t applyIdleNow(uint32_t duration_s) {
    const bool was_sensing = idlestate::sensing(g_idle);
    const idlestate::Request r =
        idlestate::request(g_idle, millis(), duration_s);

    if (idlestate::sensing(g_idle)) {
        // Resumed (or refused to start): clear the flag and drop any pairing
        // state that predates the gap before the next poll can build on it.
        if (!was_sensing) {
            resetAllGates();
            slog::printf("[IDLE] sensing resumed on request");
        }
        g_status_flags &= (uint8_t)~beecounter_proto::STATUS_NIGHT_IDLE;
    } else {
        // Entering. Park the emitters immediately rather than waiting for the
        // next poll to not run — the whole point of the feature is the current
        // they draw. Re-arming an already-idle counter is the common case and
        // must not reset the gates again: nothing has sampled in between.
        if (was_sensing) {
            resetAllGates();
            driveIrLeds(false);
        }
        g_status_flags |= beecounter_proto::STATUS_NIGHT_IDLE;
    }
    return r.granted_s;
}

// Apply a bank mask immediately; the same split as applyIdleNow().
static uint8_t applyBankMaskNow(uint8_t mask) {
    const uint8_t before = g_banks.mask;
    const bankstate::Request r = bankstate::request(g_banks, mask);
    if (!r.accepted) {
        // bank_state.h refuses an all-off mask rather than blinding the
        // counter on one byte. Say so: the alternative is a HiveHub that
        // believes it switched everything off and a counter that did not.
        slog::printf("[BANKS] request 0x%02X refused; still 0x%02X\n",
                      (unsigned)mask, (unsigned)g_banks.mask);
        return g_banks.mask;
    }
    if (r.changed) {
        // Both edges matter. A bank going dark leaves any half-finished
        // pairing on its gates unfinishable; a bank coming back would otherwise
        // combine a pairing from before the gap with a sample from after it.
        // A bank whose state did not change is left alone: switching bank 3
        // off must not throw away a crossing in progress on bank 1.
        for (uint8_t b = 1; b <= pins::NUM_LED_BANKS; b++) {
            if ((before ^ g_banks.mask) & bankstate::bankBit(b)) resetGatesForBank(b);
        }
        // Park the FETs on the new mask immediately rather than waiting for the
        // next poll — drawing the current is the whole thing being switched off.
        driveIrLeds(false);
        slog::printf("[BANKS] mask 0x%02X — %u of %u banks, %u gates active\n",
                      (unsigned)r.granted,
                      (unsigned)bankstate::enabledCount(g_banks),
                      (unsigned)pins::NUM_LED_BANKS,
                      (unsigned)(bankstate::enabledCount(g_banks) * 8u));
    }
    return r.granted;
}

// Apply a HiveHub night-mode request. Outside maintenance it takes effect at
// once; maintenance owns the emitters until it ends, so the request is held —
// clamped exactly as it would have been — and what is left of it is replayed
// then, so the counter still ends up where HiveHub wants it. loop() only.
static void handleIdleRequest(uint32_t duration_s) {
    if (!g_maint.active) {
        applyIdleNow(duration_s);
        return;
    }
    idlestate::State probe;
    const uint32_t granted = idlestate::request(probe, millis(), duration_s).granted_s;
    maintstate::deferIdle(g_maint, millis(), granted);
    slog::printf("[MAINT] HiveHub night mode %lus held until maintenance ends",
                 (unsigned long)granted);
}

// The same for a bank mask, validated by the same rules (an all-off mask is
// still refused) and then held like a night-mode request. loop() only.
static void handleBankRequest(uint8_t mask) {
    if (!g_maint.active) {
        applyBankMaskNow(mask);
        return;
    }
    bankstate::State probe = g_banks;
    const bankstate::Request r = bankstate::request(probe, mask);
    if (!r.accepted) {
        slog::printf("[BANKS] request 0x%02X refused; still 0x%02X",
                     (unsigned)mask, (unsigned)g_banks.mask);
        return;
    }
    maintstate::deferBanks(g_maint, r.granted);
    slog::printf("[MAINT] HiveHub bank mask 0x%02X held until maintenance ends",
                 (unsigned)r.granted);
}

// Apply whatever the control characteristic posted since the last pass. Called
// at the top of loop(), so a request always lands between two polls.
static void serviceControlRequests() {
    portENTER_CRITICAL(&g_ctrl_mux);
    const ControlMailbox m = g_ctrl_mailbox;
    g_ctrl_mailbox = ControlMailbox();
    portEXIT_CRITICAL(&g_ctrl_mux);

    if (m.banks_pending) handleBankRequest(m.banks);
    if (m.idle_pending) handleIdleRequest(m.idle_s);
}

namespace ble {

// The return values are what the request WILL do, worked out on copies by the
// same pure rules loop() applies, so ble_link.cpp can log and answer at once.
uint32_t applyIdleRequest(uint32_t duration_s) {
    idlestate::State probe;
    const uint32_t granted = idlestate::request(probe, millis(), duration_s).granted_s;
    portENTER_CRITICAL(&g_ctrl_mux);
    g_ctrl_mailbox.idle_pending = true;
    g_ctrl_mailbox.idle_s = duration_s;
    portEXIT_CRITICAL(&g_ctrl_mux);
    return granted;
}

uint32_t idleRemainingSeconds() {
    return idlestate::remainingSeconds(g_idle, millis());
}

uint8_t applyBankMask(uint8_t mask) {
    bankstate::State probe = g_banks;
    const bankstate::Request r = bankstate::request(probe, mask);
    // A refused mask is still posted, so loop() logs the refusal where it
    // logs everything else about the banks; it changes nothing there either.
    portENTER_CRITICAL(&g_ctrl_mux);
    g_ctrl_mailbox.banks_pending = true;
    g_ctrl_mailbox.banks = mask;
    portEXIT_CRITICAL(&g_ctrl_mux);
    return r.accepted ? r.granted : g_banks.mask;
}

uint8_t bankMask() {
    return g_banks.mask;
}

}  // namespace ble


// ============================================================================
// Arduino setup() / loop()
// ============================================================================

static uint32_t g_last_poll_ms = 0;

// Poll-gap and poll-duration records; see poll_timing.h.
static polltiming::State g_timing;

// The one place the production loop polls from, so every poll is measured.
static void timedPoll() {
    const uint32_t start_us = micros();
    pollAllGates();   // pulses the emitters internally in AUTO mode
    polltiming::notePoll(g_timing, start_us, (uint32_t)(micros() - start_us),
                         LATE_POLL_GAP_US);
}

// End a maintenance session and put back everything it overrode: LED mode,
// bank mask (HiveHub's latest request if one arrived, else the mask at entry)
// and any night-mode suspension, with only the time it has left.
static void endMaintenance(const char* why) {
    const uint32_t in = g_maint.test_in;
    const uint32_t out = g_maint.test_out;
    const uint32_t glitch = g_maint.test_glitch;
    const uint32_t session_s = (millis() - g_maint.entered_ms) / 1000;
    const maintstate::Restore r = maintstate::exit(g_maint, millis());

    g_led_mode = LedMode::AUTO;
    applyBankMaskNow(r.bank_mask);
    // Test stimuli must not leave a half-finished pairing for real traffic to
    // complete.
    resetAllGates();
    if (r.idle_s > 0) applyIdleNow(r.idle_s);

    slog::printf("[MAINT] ended %s after %lus: test in=%lu out=%lu glitches=%lu; "
                 "banks 0x%02X, night mode %lus",
                 why, (unsigned long)session_s, (unsigned long)in,
                 (unsigned long)out, (unsigned long)glitch,
                 (unsigned)g_banks.mask, (unsigned long)r.idle_s);
}

// ----------------------------------------------------------------------------
// Diagnostic console views (declared in diag.h). Reads of state the sampler
// already produced: no I2C, no emitters, no side effects.
// ----------------------------------------------------------------------------
namespace diag {

uint8_t numGates() {
    return gates::NUM_GATES;
}

static bool sensingNow() {
    return !ble::isOtaActive() && idlestate::sensing(g_idle);
}

void gateView(uint8_t gate, GateView& out) {
    out = GateView();
    if (gate >= gates::NUM_GATES) return;
    const auto& loc = gates::TABLE[gate];
    const gatelogic::GateRuntime& rt = g_gate_rt[gate];
    const int8_t ch = mcpIndexForAddress(loc.mcp_address);

    out.tag = loc.tag;
    out.bank = loc.led_bank;
    out.chip = ch < 0 ? 0 : (uint8_t)ch;
    out.inner_debounced = rt.inner.accepted;
    out.outer_debounced = rt.outer.accepted;
    out.state = rt.state;

    // The same precedence the sampler applies: a dark bank is never sampled,
    // a paused sampler has nothing current, an unreadable chip has no state.
    SensorView v;
    if (!bankstate::enabled(g_banks, loc.led_bank)) {
        v = SensorView::BANK_OFF;
    } else if (!sensingNow()) {
        v = SensorView::STALE;
    } else if (ch < 0 || !g_mcp[ch].ok || !g_mcp[ch].valid) {
        v = SensorView::INVALID;
    } else if (!g_mcp[ch].stats.have_ok ||
               millis() - g_mcp[ch].stats.last_ok_ms > SNAPSHOT_STALE_MS) {
        v = SensorView::STALE;
    } else {
        v = SensorView::CLEAR;   // resolved per sensor below
    }
    if (v != SensorView::CLEAR) {
        out.inner_raw = v;
        out.outer_raw = v;
        return;
    }
    // BLOCKED == sensor line LOW == bit 0.
    const uint16_t w = g_mcp[ch].value;
    out.inner_raw = ((w >> loc.inner_pin) & 1) ? SensorView::CLEAR : SensorView::BLOCKED;
    out.outer_raw = ((w >> loc.outer_pin) & 1) ? SensorView::CLEAR : SensorView::BLOCKED;
}

void chipView(uint8_t chip, ChipView& out) {
    out = ChipView();
    if (chip >= NUM_MCP) return;
    const McpHealth& m = g_mcp[chip];
    out.name = m.name;
    out.addr = m.addr;
    out.healthy = m.ok;
    out.last_read_ok = m.valid;
    out.fail_streak = m.fail_streak;
    out.last_word = m.stats.last_word;
    out.last_ok_age_ms = m.stats.have_ok
        ? (uint32_t)(millis() - m.stats.last_ok_ms) : UINT32_MAX;
    out.reads_ok = m.stats.reads_ok;
    out.nacks = m.stats.nacks;
    out.short_reads = m.stats.short_reads;
    out.lost = m.stats.lost;
    out.recovered = m.stats.recovered;
    out.max_read_us = m.stats.max_read_us;
}

void statusView(StatusView& out) {
    out = StatusView();
    const uint32_t now = millis();
    out.uptime_s = uptimeSeconds();
    out.total_in = g_total_in;
    out.total_out = g_total_out;
    out.glitches = g_glitch_count;
    out.status_flags = g_status_flags;
    out.bank_mask = g_banks.mask;
    out.idle_s = idlestate::remainingSeconds(g_idle, now);
    out.ota_active = ble::isOtaActive();
    out.sensing = sensingNow();
    out.poll_interval_ms = POLL_INTERVAL_MS;
    out.late_after_us = LATE_POLL_GAP_US;
    out.polls = g_timing.boot.polls;
    out.late = g_timing.boot.late;
    out.max_gap_us = g_timing.boot.max_gap_us;
    out.max_poll_us = g_timing.boot.max_duration_us;
    out.window_polls = g_timing.window.polls;
    out.window_late = g_timing.window.late;
    out.window_max_gap_us = g_timing.window.max_gap_us;
    out.window_max_poll_us = g_timing.window.max_duration_us;
    out.maint = g_maint.active;
}

// ---- maintenance ----

static Result maintGate() {
    if (ble::isOtaActive()) return Result::OTA_ACTIVE;
    if (!g_maint.active) return Result::NOT_IN_MAINT;
    return Result::OK;
}

Result maintEnter() {
    if (ble::isOtaActive()) return Result::OTA_ACTIVE;
    if (g_maint.active) return Result::ALREADY;
    const uint32_t now = millis();
    // A night-mode suspension in force is taken over by the session (the
    // bench needs the emitters) and resumed, with whatever time it has left,
    // when the session ends.
    const uint32_t idle_left = idlestate::remainingSeconds(g_idle, now);
    if (idle_left > 0) applyIdleNow(0);
    maintstate::enter(g_maint, now, g_banks.mask, idle_left);
    // Nothing paired before the session may complete inside it.
    resetAllGates();
    slog::printf("[MAINT] started: totals frozen, crossings go to test counters, "
                 "ends after %lus without a command",
                 (unsigned long)(maintstate::TIMEOUT_MS / 1000));
    return Result::OK;
}

Result maintExit() {
    if (!g_maint.active) return Result::ALREADY;
    endMaintenance("on request");
    return Result::OK;
}

void maintTouch() {
    maintstate::touch(g_maint, millis());
}

void maintView(MaintView& out) {
    out = MaintView();
    const uint32_t now = millis();
    out.active = g_maint.active;
    out.timeout_s = maintstate::timeoutRemainingSeconds(g_maint, now);
    out.session_s = g_maint.active ? (now - g_maint.entered_ms) / 1000 : 0;
    out.total_s = maintstate::totalSeconds(g_maint, now);
    out.test_in = g_maint.test_in;
    out.test_out = g_maint.test_out;
    out.test_glitch = g_maint.test_glitch;
    out.leds = g_led_mode == LedMode::FORCE_ON  ? LedSetting::ON
             : g_led_mode == LedMode::FORCE_OFF ? LedSetting::OFF
                                                : LedSetting::AUTO;
    out.bank_mask = g_banks.mask;
    if (g_maint.active) {
        out.restore_mask = g_maint.banks_pending ? g_maint.banks_mask
                                                 : g_maint.banks_before;
        if (g_maint.idle_pending) {
            const uint32_t elapsed_s = (now - g_maint.idle_requested_ms) / 1000;
            out.restore_idle_s = elapsed_s < g_maint.idle_duration_s
                ? g_maint.idle_duration_s - elapsed_s : 0;
        }
    }
}

Result setLeds(LedSetting setting) {
    const Result gate = maintGate();
    if (gate != Result::OK) return gate;
    // Applied by loop()'s mode-change block, which sets the steady level.
    g_led_mode = setting == LedSetting::ON  ? LedMode::FORCE_ON
               : setting == LedSetting::OFF ? LedMode::FORCE_OFF
                                            : LedMode::AUTO;
    return Result::OK;
}

Result setBank(uint8_t bank, bool on) {
    const Result gate = maintGate();
    if (gate != Result::OK) return gate;
    const uint8_t bit = bankstate::bankBit(bank);
    if (bit == 0 || bank > pins::NUM_LED_BANKS) return Result::REFUSED;
    const uint8_t mask = on ? (uint8_t)(g_banks.mask | bit)
                            : (uint8_t)(g_banks.mask & ~bit);
    if (mask == g_banks.mask) return Result::ALREADY;
    // An all-off mask is refused here exactly as it is over BLE.
    const uint8_t granted = applyBankMaskNow(mask);
    if (granted != mask) return Result::REFUSED;
    // The FETs were parked by applyBankMaskNow(); restore the mode's level.
    setIrLeds(true);
    return Result::OK;
}

Result i2cScan(I2cScanView& out) {
    out = I2cScanView();
    const Result gate = maintGate();
    if (gate != Result::OK) return gate;
    // ~126 empty address phases: a few milliseconds of bus time, which is why
    // this is a maintenance command — counting is paused anyway.
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (!mcpPresent(addr)) continue;
        if (out.count < sizeof(out.addrs)) out.addrs[out.count] = addr;
        out.count++;
    }
    return Result::OK;
}

static constexpr uint16_t MCP_TEST_BURST = 100;

Result mcpTest(McpTestView (&out)[NUM_CHIPS]) {
    const Result gate = maintGate();
    if (gate != Result::OK) return gate;
    for (uint8_t i = 0; i < NUM_MCP && i < NUM_CHIPS; i++) {
        McpHealth& m = g_mcp[i];
        McpTestView& v = out[i];
        v = McpTestView();
        v.name = m.name;
        v.addr = m.addr;
        v.present = mcpPresent(m.addr);
        if (!v.present) continue;

        uint8_t r[MCP_CONFIG_BYTES] = {0};
        if (readMcpConfig(m.addr, r)) {
            v.regs_read = true;
            v.iodir_a = r[MCP_REG_IODIRA];
            v.iodir_b = r[MCP_REG_IODIRB];
            v.ipol_a  = r[MCP_REG_IPOLA];
            v.ipol_b  = r[MCP_REG_IPOLB];
            v.iocon   = r[MCP_REG_IOCON];
            v.gppu_a  = r[MCP_REG_GPPUA];
            v.gppu_b  = r[MCP_REG_GPPUB];
            v.config_ok = mcpConfigMatches(r);
        }

        // The same checked two-byte read the sampler does, back to back. Its
        // statistics are real reads and stay in the chip's McpStats too.
        const McpStats before = m.stats;
        for (uint16_t k = 0; k < MCP_TEST_BURST; k++) {
            uint16_t word = 0;
            const uint32_t t0 = micros();
            const bool ok = readMcpChecked(i, word);
            const uint32_t dt = (uint32_t)(micros() - t0);
            v.burst++;
            if (ok) {
                v.burst_ok++;
                if (dt > v.burst_max_us) v.burst_max_us = dt;
            }
        }
        v.burst_nack = (uint16_t)(m.stats.nacks - before.nacks);
        v.burst_short = (uint16_t)(m.stats.short_reads - before.short_reads);
    }
    return Result::OK;
}

static SensorView viewOf(bool read_ok, uint16_t word, uint8_t pin) {
    if (!read_ok) return SensorView::INVALID;
    return ((word >> pin) & 1) ? SensorView::CLEAR : SensorView::BLOCKED;
}

Result darkLit(DarkLitView* out, uint8_t n) {
    const Result gate = maintGate();
    if (gate != Result::OK) return gate;

    uint16_t dark[NUM_MCP] = {0};
    uint16_t lit[NUM_MCP] = {0};
    bool dark_ok[NUM_MCP] = {false};
    bool lit_ok[NUM_MCP] = {false};

    // Dark first, then lit, each after the same settle time the sampler uses.
    // driveIrLeds() honours the bank mask, so a disabled bank stays dark for
    // both and is reported as such below.
    driveIrLeds(false);
    delayMicroseconds(LED_SETTLE_US);
    for (uint8_t i = 0; i < NUM_MCP; i++) dark_ok[i] = readMcpChecked(i, dark[i]);
    driveIrLeds(true);
    delayMicroseconds(LED_SETTLE_US);
    for (uint8_t i = 0; i < NUM_MCP; i++) lit_ok[i] = readMcpChecked(i, lit[i]);
    setIrLeds(true);   // back to the LED mode's own steady level

    for (uint8_t g = 0; g < n && g < gates::NUM_GATES; g++) {
        const auto& loc = gates::TABLE[g];
        DarkLitView& v = out[g];
        v = DarkLitView();
        v.tag = loc.tag;
        v.bank = loc.led_bank;
        const int8_t ch = mcpIndexForAddress(loc.mcp_address);
        if (!bankstate::enabled(g_banks, loc.led_bank)) {
            v.inner_dark = v.inner_lit = v.outer_dark = v.outer_lit =
                SensorView::BANK_OFF;
            continue;
        }
        if (ch < 0) continue;   // left INVALID
        v.inner_dark = viewOf(dark_ok[ch], dark[ch], loc.inner_pin);
        v.outer_dark = viewOf(dark_ok[ch], dark[ch], loc.outer_pin);
        v.inner_lit  = viewOf(lit_ok[ch],  lit[ch],  loc.inner_pin);
        v.outer_lit  = viewOf(lit_ok[ch],  lit[ch],  loc.outer_pin);
    }
    return Result::OK;
}

}  // namespace diag

void setup() {
    slog::begin(115200);
    delay(200);
    slog::printf("==============================================");
    slog::printf("Easy Bee Counter 2026 — firmware booting (BLE/GATT link)");
    slog::printf("==============================================");

    // Configure LED enable pins -- start with LEDs off so we can verify
    // the MCP23017 pull-up baseline before powering the emitters.
    for (uint8_t b = 0; b < pins::NUM_LED_BANKS; b++) {
        pinMode(pins::IR_LED_BANK_EN[b], OUTPUT);
        digitalWrite(pins::IR_LED_BANK_EN[b], LOW);
    }

    // The MCP23017 bus. There is only one bus now: the second I2C controller
    // used to run a permanent slave for the HiveScale, and that link is gone.
    Wire.begin(pins::I2C_SDA, pins::I2C_SCL, (uint32_t)I2C_MASTER_HZ);

    // Boot-time discovery. A chip that is missing here is NOT written off: it
    // is left unhealthy, its gates are skipped, and pollAllGates() re-probes it
    // every MCP_RETRY_INTERVAL_MS until it answers.
    for (uint8_t i = 0; i < NUM_MCP; i++) {
        g_mcp[i].ok = initMcp(g_mcp[i].addr, g_mcp[i].tag);
        g_mcp[i].fail_streak = 0;
        if (g_mcp[i].ok) {
            g_status_flags |= g_mcp[i].status_bit;
        } else {
            g_mcp[i].next_retry_ms = millis() + MCP_RETRY_INTERVAL_MS;
        }
    }

    ble::begin();   // connectable NimBLE GATT server on the GPIO-less radio

    // Emitters now default to PULSED (AUTO): they stay dark between samples and
    // are lit only for the settle+read window inside pollAllGates(). Leave the
    // steady level OFF here; the first pollAllGates() below will pulse them.
    g_led_mode = LedMode::AUTO;
    setIrLeds(false);

    // One warm-up poll so the state machine has fresh baselines before any
    // crossings start counting. This also exercises the pulsed-sample path.
    pollAllGates();

    g_status_flags |= beecounter_proto::STATUS_READY;

    slog::printf("[SETUP] emitter banks 0x%02X (%u of %u, %u gates active)\n",
                  (unsigned)g_banks.mask,
                  (unsigned)bankstate::enabledCount(g_banks),
                  (unsigned)pins::NUM_LED_BANKS,
                  (unsigned)(bankstate::enabledCount(g_banks) * 8u));
    slog::printf("[SETUP] Entering normal counting loop (pulsed IR)");
    console::begin();
}

void loop() {
    // Services the deferred post-OTA reboot, once the central has had a chance
    // to read DONE off the status characteristic.
    ble::loopOta();
    ble::serviceAdvertising();

    // Night-mode and bank writes from HiveHub, applied here between polls
    // rather than on the NimBLE task that received them.
    serviceControlRequests();

    const uint32_t now = millis();

    // ---- Poll the MCP23017s every POLL_INTERVAL_MS ----
    // An OTA pauses sensing: flash writes must not leave an emitter pulse
    // active or corrupt a crossing in progress. Counting is deliberately
    // sacrificed for the duration of a transfer.
    // Night mode ends on its own deadline, checked here rather than inside the
    // poll branch so it still expires while an OTA is running — otherwise a
    // relay that straddles sunrise would leave the counter suspended until the
    // next HiveHub cycle re-armed or cleared it.
    if (idlestate::serviceExpiry(g_idle, now)) {
        // Nothing has sampled since the suspension began, so any pairing state
        // still on a gate is hours stale.
        resetAllGates();
        g_status_flags &= (uint8_t)~beecounter_proto::STATUS_NIGHT_IDLE;
        slog::printf("[IDLE] suspension expired; counting again");
    }

    // A maintenance session ends by itself when an OTA starts (the transfer
    // owns the counter, and the new image would boot out of it anyway) or
    // when nobody has typed a command for maintstate::TIMEOUT_MS.
    if (g_maint.active) {
        if (ble::isOtaActive()) {
            endMaintenance("for a firmware update");
        } else if (maintstate::timedOut(g_maint, now)) {
            endMaintenance("after inactivity timeout");
        }
    }

    // An OTA that ends without a reboot — aborted, failed, stalled or cut off by
    // a disconnect — hands the counter back after a pause of any length. The
    // gates still hold what they saw when it began: a sensor accepted as
    // blocked then would fail the stuck-sensor check on the very first poll
    // afterwards (and latch the fault flag until reset), and a half-finished
    // pairing could combine with a sample taken minutes later. So the first
    // pass after the pause only resets them, exactly as night mode does on both
    // of its edges; polling resumes on the next pass from a fresh baseline.
    static bool ota_paused = false;
    if (ble::isOtaActive()) {
        ota_paused = true;
        driveIrLeds(false);
        polltiming::pause(g_timing);
    } else if (ota_paused) {
        ota_paused = false;
        resetAllGates();
        slog::printf("[OTA] transfer ended without an update; gates reset, "
                     "counting resumes");
    } else if (!idlestate::sensing(g_idle)) {
        // Night mode: emitters dark, gates unpolled, totals frozen.
        //
        // The re-assert runs on the poll cadence rather than every loop pass.
        // It is here as a backstop — applyIdleRequest() already parked the
        // emitters on entry — so that nothing else (a FORCE_ON left set from a
        // bench session, a future code path) can leave a bank lit through the
        // night; doing it at full loop speed would spend the CPU cycles this
        // mode exists to save.
        if (now - g_last_poll_ms >= POLL_INTERVAL_MS) {
            g_last_poll_ms = now;
            driveIrLeds(false);
        }
        polltiming::pause(g_timing);
        // Yield: with no gates to poll there is nothing here to be responsive
        // to except BLE, which runs on its own task. A bare spin would burn the
        // MCU's share of the budget for the whole night.
        delay(1);
    } else if (now - g_last_poll_ms >= POLL_INTERVAL_MS) {
        g_last_poll_ms = now;
        timedPoll();
    }

    // Keep the steady LED level in sync with the current mode. In AUTO this
    // resolves to OFF (the pulsed sampler owns the emitters); FORCE_ON drives
    // them steady-on; FORCE_OFF holds them dark.
    static LedMode last_led_mode = LedMode::AUTO;
    if (g_led_mode != last_led_mode) {
        last_led_mode = g_led_mode;
        // Suspended: leave the emitters dark whatever the mode says. FORCE_ON
        // exists for bench work and must not be able to undo night mode from a
        // debug console that was left in that state.
        if (idlestate::sensing(g_idle)) {
            setIrLeds(true);   // setIrLeds() applies the mode-correct steady level
        } else {
            driveIrLeds(false);
        }
    }

    // USB console: input, paced report/watch/event output. After this pass's
    // sampling, so nothing it does can delay a poll that is already due.
    console::poll(now);

    // Periodic debug dump on serial -- once every 30 s.
    static uint32_t last_dump_ms = 0;
    if (now - last_dump_ms > STAT_INTERVAL_MS) {
        last_dump_ms = now;
        slog::printf(
            "[STAT] uptime=%lus total_in=%lu total_out=%lu "
            "glitches=%lu status=0x%02X idle=%lus banks=0x%02X\n",
            (unsigned long)uptimeSeconds(),
            (unsigned long)g_total_in,
            (unsigned long)g_total_out,
            (unsigned long)g_glitch_count,
            (unsigned)g_status_flags,
            (unsigned long)idlestate::remainingSeconds(g_idle, now),
            (unsigned)g_banks.mask
        );

        // Printed only when the window had something wrong in it, so a
        // healthy counter's log is unchanged. A late poll is a stall long
        // enough to risk a missed crossing; a dropped line means the USB host
        // stopped reading (or could not keep up) and the log has gaps.
        static uint32_t dropped_reported = 0;
        const uint32_t dropped = slog::droppedLines();
        const polltiming::Figures w = polltiming::takeWindow(g_timing);
        if (w.late > 0 || dropped != dropped_reported) {
            slog::printf(
                "[TIMING] polls=%lu late=%lu max_gap=%luus max_poll=%luus "
                "log_dropped=%lu (boot: late=%lu max_gap=%luus)\n",
                (unsigned long)w.polls,
                (unsigned long)w.late,
                (unsigned long)w.max_gap_us,
                (unsigned long)w.max_duration_us,
                (unsigned long)(dropped - dropped_reported),
                (unsigned long)g_timing.boot.late,
                (unsigned long)g_timing.boot.max_gap_us);
            dropped_reported = dropped;
        }
    }
}
