#pragma once

#include <stdint.h>

// Shared between the Arduino USB event task and the serial task. The caller
// must serialize ALL access, including snapshots, with the same critical section.
class UsbLineState {
public:
    bool dtr() const { return dtr_; }
    uint32_t disconnectEpoch() const { return disconnect_epoch_; }
    uint32_t revision() const { return revision_; }

    void onLineState(bool dtr) {
        ++revision_;
        if (dtr_ && !dtr) ++disconnect_epoch_;
        dtr_ = dtr;
    }

    // Also used for the existing synchronous invalidation before soft detach.
    void onDisconnect() {
        ++revision_;
        dtr_ = false;
        ++disconnect_epoch_;
    }

    // A host may open CDC before our event callback is registered. Seed from
    // TinyUSB once at startup, but never overwrite a callback/detach that ran
    // while that snapshot was being obtained. No probing/ONLINE state is set here.
    bool seedIfUnchanged(uint32_t expected_revision, bool connected) {
        if (revision_ != expected_revision) return false;
        onLineState(connected);
        return true;
    }

private:
    bool dtr_ = false;
    uint32_t disconnect_epoch_ = 0;
    uint32_t revision_ = 0;
};
