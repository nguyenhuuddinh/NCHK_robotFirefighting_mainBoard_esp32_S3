#pragma once
#include <stdint.h>
#include <stddef.h>

static inline uint32_t millis() { return 0; }

extern uint32_t s_usb_tx_event_cnt;

class HardwareSerial {
public:
    size_t mock_avail = 512;
    size_t next_write_limit = 512;

    // For capturing write
    uint8_t capture_buf[1024];
    size_t capture_len = 0;

    // For injecting TX event inside write
    bool inject_tx_event_in_write = false;

    size_t availableForWrite() { return mock_avail; }
    size_t write(const uint8_t* buf, size_t size) {
        size_t w = (size < next_write_limit) ? size : next_write_limit;
        for(size_t i = 0; i < w; i++) {
            if (capture_len < sizeof(capture_buf)) {
                capture_buf[capture_len++] = buf[i];
            }
        }
        if (inject_tx_event_in_write) {
            s_usb_tx_event_cnt++;
        }
        return w;
    }

    void reset_capture() {
        capture_len = 0;
    }
};

extern HardwareSerial Serial;
