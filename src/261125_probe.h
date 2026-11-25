#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Same harmless request as the stock Studio handshake, on our own USB interface. */
static inline bool probe_request(uint8_t *matched, uint8_t byte) {
    static const uint8_t info[] = {0xab, 8, 1, 26, 2, 8, 1, 0xad};
    if (byte == info[*matched]) {
        if (++*matched == sizeof(info)) {
            *matched = 0;
            return true;
        }
    } else {
        *matched = byte == info[0] ? 1 : 0;
    }
    return false;
}

static inline bool probe_reply(uint8_t out[9], bool down, uint8_t layer) {
    if (layer >= 18) return false;
    const uint8_t frame[] = {0xab, 'L', '4', '0', 'P', 1, down ? 1 : 0, layer, 0xad};
    for (unsigned i = 0; i < sizeof(frame); ++i) out[i] = frame[i];
    return true;
}
