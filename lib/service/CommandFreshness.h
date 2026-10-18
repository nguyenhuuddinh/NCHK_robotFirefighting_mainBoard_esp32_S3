#pragma once

#include <stdint.h>

// Called by Motion on Core 1. The timestamp and command must come from the
// same cmdMutex snapshot. Unsigned subtraction stays valid across millis wrap.
static inline bool commandIsFresh(bool received, uint32_t received_ms,
                                  uint32_t now_ms, uint32_t timeout_ms) {
    return received && (uint32_t)(now_ms - received_ms) <= timeout_ms;
}
