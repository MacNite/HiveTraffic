// ============================================================================
// ota_session.h — what an OTA frame is allowed to do, as pure rules
// ============================================================================
//
// Arduino-free for the same reason idle_state.h and bank_state.h are: the
// decisions here are the ones that are awkward to provoke on a bench — a relay
// that stops sending mid-image, a duplicated END, an ABORT that arrives after
// the image was already committed — and every one of them decides whether the
// counter counts. src/ble_link.cpp owns Update, the GATT characteristics and the
// reboot, and asks this header what each frame may do. Exercised on a host
// compiler by test/test_ota_session/.
//
// Two rules
// ---------
//   1. **A transfer cannot hold the counter forever.** Gate polling pauses for
//      the whole of RECEIVING, so a relay that sends BEGIN and then goes quiet
//      while staying connected would otherwise stop the counter until someone
//      power-cycled it. A disconnect already aborts; a connected relay that
//      makes no progress for PROGRESS_TIMEOUT_MS is now aborted the same way.
//
//   2. **DONE is terminal.** Once END has verified the image, Update.end() has
//      already made the new slot the boot partition. Nothing a client sends
//      afterwards can take that back, so nothing is allowed to look as if it
//      had: a repeated END, a stray ABORT or BEGIN, or a late DATA write is
//      ignored, DONE is re-published, and the reboot goes ahead. Before this,
//      any of them cancelled the reboot and reported an error, leaving a counter
//      that told HiveHub the update failed and then booted the new image at its
//      next reset anyway.
// ============================================================================

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "counter_protocol.h"

namespace otasession {

// OTA control opcodes — byte 0 of a write to the OTA control characteristic.
// See docs/ble-mode.md for the frame layout.
constexpr uint8_t OP_BEGIN = 0x01;
constexpr uint8_t OP_END   = 0x03;
constexpr uint8_t OP_ABORT = 0x04;

// How long a transfer may go without a successful BEGIN or DATA write before it
// is aborted. HiveHub streams the image straight from an HTTPS download, so a
// slow server shows up here as a gap between DATA writes; a minute is far
// longer than any healthy stall and still bounds how long a hung relay can keep
// the counter blind.
constexpr uint32_t PROGRESS_TIMEOUT_MS = 60000;

// What a control write does.
enum class Control : uint8_t {
    BEGIN,       // start (or restart) a transfer
    END,         // verify and commit
    ABORT,       // discard the partial write
    FAIL_SEQ,    // unknown opcode: fail the transfer
    KEEP_DONE,   // the image is committed: change nothing, re-publish DONE
};

inline Control control(uint8_t state, uint8_t opcode) {
    if (state == beecounter_proto::OTA_STATE_DONE) return Control::KEEP_DONE;
    switch (opcode) {
    case OP_BEGIN: return Control::BEGIN;
    case OP_END:   return Control::END;
    case OP_ABORT: return Control::ABORT;
    default:       return Control::FAIL_SEQ;
    }
}

// What a DATA write does.
enum class Data : uint8_t {
    ACCEPT,      // write it to the inactive slot
    IGNORE,      // empty, or arrived after the image was committed
    FAIL_SEQ,    // no transfer running, or more bytes than BEGIN declared
};

inline Data data(uint8_t state, uint32_t received, uint32_t size,
                 size_t length) {
    if (length == 0) return Data::IGNORE;
    // A write-without-response queued behind END can still land after it.
    if (state == beecounter_proto::OTA_STATE_DONE) return Data::IGNORE;
    if (state != beecounter_proto::OTA_STATE_RECEIVING) return Data::FAIL_SEQ;
    if (received > size || length > size - received) return Data::FAIL_SEQ;
    return Data::ACCEPT;
}

// May a failure (an error frame, a disconnect, a stall) end the session? Every
// state but DONE: see rule 2 above.
inline bool mayFail(uint8_t state) {
    return state != beecounter_proto::OTA_STATE_DONE;
}

// Has a running transfer gone PROGRESS_TIMEOUT_MS without progress?
// `last_progress_ms` is the millis() of the BEGIN or of the last DATA write
// accepted. Unsigned subtraction, so the millis() rollover needs no special
// case.
inline bool stalled(uint8_t state, uint32_t last_progress_ms, uint32_t now_ms) {
    return state == beecounter_proto::OTA_STATE_RECEIVING &&
           now_ms - last_progress_ms >= PROGRESS_TIMEOUT_MS;
}

}  // namespace otasession
