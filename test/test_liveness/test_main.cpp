#include <unity.h>
#include "../../lib/service/SerialComm.cpp"
#include "SerialComm.h"
#include "UsbLineState.h"
#include "../../lib/service/PumpSafety.h"
#include "../../lib/service/CommandFreshness.h"
#include "../../lib/service/ActuatorLease.h"

uint32_t s_usb_tx_event_cnt = 0;
HardwareSerial Serial;
SerialCommTelemetry telemetry;
TxLivenessStateMachine sm;
SerialParser parser;
TxScheduler txSched;

void setUp(void) {
    sm.reset();
    Serial.mock_avail = 512;
    Serial.next_write_limit = 512;
    Serial.reset_capture();
    Serial.inject_tx_event_in_write = false;
    s_usb_tx_event_cnt = 0;
    telemetry = SerialCommTelemetry();
    parser.reset();
    txSched = TxScheduler();
}

void tearDown(void) {}

void test_motion_command_age_stops_when_serial_task_stalls(void) {
    const uint32_t timeout_ms = 1000;
    const uint32_t received_ms = 500;
    TEST_ASSERT_FALSE(commandIsFresh(false, received_ms, received_ms, timeout_ms));
    TEST_ASSERT_TRUE(commandIsFresh(true, received_ms, 1500, timeout_ms));
    // Only Motion's clock advances. No serial callback or RobotMaster::update.
    TEST_ASSERT_FALSE(commandIsFresh(true, received_ms, 1501, timeout_ms));
}

void test_motion_command_age_wraps_without_rearming_stale_command(void) {
    const uint32_t received_ms = UINT32_MAX - 100;
    TEST_ASSERT_TRUE(commandIsFresh(true, received_ms, 50, 1000));
    TEST_ASSERT_FALSE(commandIsFresh(true, received_ms, 901, 1000));
}

void test_pump_owner_stops_when_serial_task_stalls(void) {
    PumpSafety pump;
    pump.request(true, 100);
    TEST_ASSERT_TRUE(pump.tick(100,
        actuatorSerialLinkFresh(true, 100, 100)));
    // No serial tick or PUMP refresh. The independent owner keeps ticking.
    TEST_ASSERT_TRUE(pump.tick(350,
        actuatorSerialLinkFresh(true, 100, 350)));
    TEST_ASSERT_FALSE(pump.tick(351,
        actuatorSerialLinkFresh(true, 100, 351)));
    TEST_ASSERT_TRUE(pump.lockedOff());
    pump.request(true, 352);
    TEST_ASSERT_FALSE(pump.tick(352, true));
    pump.request(false, 353);
    pump.request(true, 354);
    TEST_ASSERT_TRUE(pump.tick(354, true));
}

void test_pump_serial_lease_wraps_without_extending_deadline(void) {
    const uint32_t tick = UINT32_MAX - 100;
    TEST_ASSERT_TRUE(actuatorSerialLinkFresh(true, tick, tick + 250));
    TEST_ASSERT_FALSE(actuatorSerialLinkFresh(true, tick, tick + 251));
    TEST_ASSERT_FALSE(actuatorSerialLinkFresh(false, tick, tick));
}

void test_fleeting_pump_off_does_not_restart_continuous_run_cap(void) {
    PumpSafety pump;
    pump.request(true, 100);
    TEST_ASSERT_TRUE(pump.tick(100, true));
    // Both frames arrive before the actuator owner gets a cycle. PWM stayed ON.
    TEST_ASSERT_FALSE(actuatorObservedOff(1, 0, true));
    pump.request(true, 5100);
    TEST_ASSERT_FALSE(pump.tick(5100, true));
    TEST_ASSERT_TRUE(pump.lockedOff());
    // An OFF present at an owner tick still clears the latch.
    TEST_ASSERT_TRUE(actuatorObservedOff(2, 1, false));
    pump.request(false, 5101);
    pump.request(true, 5102);
    TEST_ASSERT_TRUE(pump.tick(5102, true));
}

void test_dtr_fall_and_cleanup_recovery(void) {
    // DTR fall requests logical cleanup, never a physical USB recovery.
    sm.state = SessionState::ONLINE;

    // DTR fall
    sm.update(false, s_usb_tx_event_cnt);
    TEST_ASSERT_TRUE(sm.recovery_pending);
    TEST_ASSERT_EQUAL(
        SessionRecoveryReason::HOST_DISCONNECTED, sm.recovery_reason);
    TEST_ASSERT_EQUAL(SessionState::ONLINE, sm.state); // hasn't actually dropped yet

    // Caller consumes recovery_pending
    sm.reset();
    TEST_ASSERT_EQUAL(SessionState::OFFLINE, sm.state);

    // DTR rise -> PROBING
    sm.update(true, s_usb_tx_event_cnt);
    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);
}

void test_dtr_low_while_offline_does_not_request_recovery(void) {
    for (int i = 0; i < 1000; ++i) {
        sm.update(false, s_usb_tx_event_cnt);
    }

    TEST_ASSERT_EQUAL(SessionState::OFFLINE, sm.state);
    TEST_ASSERT_FALSE(sm.recovery_pending);
    TEST_ASSERT_EQUAL(SessionRecoveryReason::NONE, sm.recovery_reason);
}

void test_probe_event_in_write_ordering(void) {
    // TX-complete may occur synchronously inside Serial.write(). The baseline
    // must already be armed so that this event proves host drain activity.
    sm.state = SessionState::PROBING;
    TEST_ASSERT_TRUE(sm.enqueue_probe(
        "STATE,2,1", 100, &s_usb_tx_event_cnt, telemetry));

    TEST_ASSERT_TRUE(sm.probe_armed);
    TEST_ASSERT_EQUAL(0, sm.probe_armed_tx_event_cnt);

    Serial.inject_tx_event_in_write = true;
    sm.pump_tx(0, telemetry);

    TEST_ASSERT_EQUAL(1, s_usb_tx_event_cnt);
    sm.update(true, s_usb_tx_event_cnt);
    TEST_ASSERT_EQUAL(SessionState::ONLINE, sm.state);
    TEST_ASSERT_FALSE(sm.probe_armed);
}

void test_probe_without_tx_event_stays_probing(void) {
    sm.state = SessionState::PROBING;
    TEST_ASSERT_TRUE(sm.enqueue_probe(
        "STATE,2,1", 100, &s_usb_tx_event_cnt, telemetry));

    sm.pump_tx(0, telemetry);
    sm.update(true, s_usb_tx_event_cnt);

    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);
    TEST_ASSERT_TRUE(sm.probe_armed);
}

void test_stale_event_before_arm_is_absorbed(void) {
    sm.state = SessionState::PROBING;
    s_usb_tx_event_cnt = 17;
    TEST_ASSERT_TRUE(sm.enqueue_probe(
        "STATE,2,1", 100, &s_usb_tx_event_cnt, telemetry));

    TEST_ASSERT_EQUAL(17, sm.probe_armed_tx_event_cnt);
    sm.pump_tx(0, telemetry);
    sm.update(true, s_usb_tx_event_cnt);

    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);
}

void test_probe_event_counter_wrap_goes_online(void) {
    sm.state = SessionState::PROBING;
    s_usb_tx_event_cnt = UINT32_MAX;
    TEST_ASSERT_TRUE(sm.enqueue_probe(
        "STATE,2,1", 100, &s_usb_tx_event_cnt, telemetry));

    Serial.inject_tx_event_in_write = true;
    sm.pump_tx(0, telemetry);

    TEST_ASSERT_EQUAL(0, s_usb_tx_event_cnt);
    sm.update(true, s_usb_tx_event_cnt);
    TEST_ASSERT_EQUAL(SessionState::ONLINE, sm.state);
}

void test_probe_retry_recovers_from_missed_event(void) {
    sm.state = SessionState::PROBING;
    TEST_ASSERT_TRUE(sm.enqueue_probe(
        "STATE,2,1", 100, &s_usb_tx_event_cnt, telemetry));
    sm.pump_tx(0, telemetry);

    sm.update(true, s_usb_tx_event_cnt);
    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);

    TEST_ASSERT_TRUE(sm.enqueue_probe(
        "STATE,2,2", 100, &s_usb_tx_event_cnt, telemetry));
    Serial.inject_tx_event_in_write = true;
    sm.pump_tx(0, telemetry);
    sm.update(true, s_usb_tx_event_cnt);

    TEST_ASSERT_EQUAL(SessionState::ONLINE, sm.state);
}

void test_valid_rx_activity_recovers_without_tx_event(void) {
    sm.state = SessionState::PROBING;

    char frame[128];
    const size_t length = serialEncodeFrame(
        frame, sizeof(frame), "CMD,2,1,0.0,0.0");
    SerialRxResult rx;
    bool valid_rx_activity = false;

    for (size_t i = 0; i < length; i++) {
        if (parser.feed(frame[i], rx, telemetry)) {
            valid_rx_activity = true;
        }
    }

    TEST_ASSERT_TRUE(valid_rx_activity);
    TEST_ASSERT_EQUAL(RX_CMD, rx.type);
    sm.update(false, s_usb_tx_event_cnt, valid_rx_activity);
    TEST_ASSERT_EQUAL(SessionState::ONLINE, sm.state);
    TEST_ASSERT_FALSE(sm.probe_armed);
}

void test_invalid_rx_cannot_confirm_liveness(void) {
    sm.state = SessionState::PROBING;

    const char* invalid_frame = "@CMD,2,broken\n";
    SerialRxResult rx;
    bool valid_rx_activity = false;

    for (size_t i = 0; invalid_frame[i] != '\0'; i++) {
        if (parser.feed(invalid_frame[i], rx, telemetry)) {
            valid_rx_activity = true;
        }
    }

    TEST_ASSERT_FALSE(valid_rx_activity);
    sm.update(true, s_usb_tx_event_cnt, valid_rx_activity);
    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);
}

void test_combined_cleanup_prevents_stale_gap(void) {
    // feed CMD hợp lệ seq=100 để tạo sequence history
    char buf1[128];
    size_t len1 = serialEncodeFrame(buf1, sizeof(buf1), "CMD,2,100,0.5,0.2");
    SerialRxResult rx;
    bool got_frame1 = false;
    for(size_t i = 0; i < len1; i++) {
        if (parser.feed(buf1[i], rx, telemetry)) got_frame1 = true;
    }
    TEST_ASSERT_TRUE(got_frame1);
    TEST_ASSERT_EQUAL(100, rx.seq);

    uint32_t init_gap = telemetry.rx_seq_gap;
    uint32_t init_dup = telemetry.rx_seq_dup;

    // feed thêm partial frame
    const char* partial = "@STATE,1";
    for(size_t i = 0; partial[i]; i++) {
        parser.feed(partial[i], rx, telemetry);
    }

    // chạy cleanup seam reset cả production SerialParser và TxLivenessStateMachine như runtime
    parser.reset();
    sm.reset();
    sm.update(true, s_usb_tx_event_cnt); // DTR rise về PROBING
    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);

    // rồi feed CMD seq=1
    char buf2[128];
    size_t len2 = serialEncodeFrame(buf2, sizeof(buf2), "CMD,2,1,0.5,0.2");
    bool got_frame2 = false;
    for(size_t i = 0; i < len2; i++) {
        if (parser.feed(buf2[i], rx, telemetry)) got_frame2 = true;
    }

    // Assert frame valid, is_duplicate=false
    TEST_ASSERT_TRUE(got_frame2);
    TEST_ASSERT_FALSE(rx.is_duplicate);
    TEST_ASSERT_EQUAL(1, rx.seq);

    // rx_seq_gap và rx_seq_duplicate không tăng
    TEST_ASSERT_EQUAL(init_gap, telemetry.rx_seq_gap);
    TEST_ASSERT_EQUAL(init_dup, telemetry.rx_seq_dup);
}

void test_actual_partial_abort_resync_parser(void) {
    // mock Serial capture byte đã write: actual partial frame, sau đó gửi
    // frame hợp lệ kế tiếp và feed capture vào production SerialParser để assert resync/valid;
    sm.state = SessionState::ONLINE;
    sm.enqueue("STATE,2,1", 1, 100, telemetry);

    Serial.mock_avail = 64;
    Serial.next_write_limit = 10; // actual partial write

    sm.pump_tx(0, telemetry); // Aborts and partials

    TEST_ASSERT_EQUAL(0, sm.tx_count);
    TEST_ASSERT_EQUAL(1, sm.consecutive_partials);

    // Queue next valid frame
    sm.enqueue("CMD,2,1,0.5,0.2", 1, 100, telemetry);
    Serial.next_write_limit = 512;
    sm.pump_tx(0, telemetry); // This one fully writes

    // Now feed capture_buf to parser
    SerialRxResult rx;
    bool got_frame = false;
    for(size_t i = 0; i < Serial.capture_len; i++) {
        if (parser.feed(Serial.capture_buf[i], rx, telemetry)) {
            got_frame = true;
        }
    }

    // The parser should have skipped the aborted partial bytes and successfully synced to the full frame
    TEST_ASSERT_TRUE(got_frame);
}

void test_scheduler_rate(void) {
    // chạy 50 tick/1 giây và assert đúng 10 STATE due + 2 ENV due
    int state_count = 0;
    int env_count = 0;

    for(int i = 0; i < 50; i++) {
        txSched.tick();
        if (txSched.state_due) state_count++;
        if (txSched.env_due) env_count++;
    }

    TEST_ASSERT_EQUAL(10, state_count);
    TEST_ASSERT_EQUAL(2, env_count);
}

void test_temporary_avail_zero_and_deadline_wrap(void) {
    // giữ test temporary avail=0, deadline/actual partial và thêm deadline wrap.
    sm.state = SessionState::ONLINE;
    sm.enqueue("STATE,2,1", 1, 0xFFFFFFF0, telemetry); // close to wrap

    Serial.mock_avail = 0;
    sm.pump_tx(0xFFFFFFE5, telemetry); // before deadline

    TEST_ASSERT_EQUAL(1, sm.tx_count);
    TEST_ASSERT_EQUAL(0, sm.consecutive_partials);

    // Exceed deadline after wrap
    sm.pump_tx(10, telemetry); // 10 is > 0xFFFFFFF0 + 20 in 32-bit wrap logic

    TEST_ASSERT_EQUAL(0, sm.tx_count);
    TEST_ASSERT_EQUAL(1, sm.consecutive_partials);
    TEST_ASSERT_EQUAL(1, telemetry.tx_deadline_expired);
    TEST_ASSERT_EQUAL(1, telemetry.tx_deadline_blocked);
    TEST_ASSERT_EQUAL(26, sm.last_abort.deadline_overrun_ms);
}

void test_usb_recovery_detach_is_one_shot(void) {
    UsbRecoveryController recovery;

    TEST_ASSERT_FALSE(recovery.armed());
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::NONE, recovery.request(999));
    recovery.observeValidRx();
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::DETACH, recovery.request(1000));
    TEST_ASSERT_TRUE(recovery.active());
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::NONE, recovery.request(1010, 250));
    TEST_ASSERT_EQUAL(UsbRecoveryAction::NONE, recovery.poll(1999));
    TEST_ASSERT_TRUE(recovery.active());
    TEST_ASSERT_EQUAL(UsbRecoveryAction::ATTACH, recovery.poll(2000));
    TEST_ASSERT_FALSE(recovery.active());
    TEST_ASSERT_EQUAL(UsbRecoveryAction::NONE, recovery.poll(2001));
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::NONE, recovery.request(10000, 250));
}

void test_unsent_expired_frames_do_not_prove_tx_stall(void) {
    // The task missed the entire deadline, so no transport attempt happened.
    // Even a full FIFO on its first observation does not prove a sustained stall.
    const size_t fifo_space[] = {64, 0};
    for (size_t available : fifo_space) {
        sm.reset();
        sm.state = SessionState::ONLINE;
        Serial.mock_avail = available;
        for (uint32_t now = 0; now < 600; now += 200) {
            TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, now + 100, telemetry));
            sm.pump_tx(now + 120, telemetry);
            sm.update(true, s_usb_tx_event_cnt, true);
            TEST_ASSERT_FALSE(sm.recovery_pending);
            TEST_ASSERT_EQUAL(0, sm.consecutive_partials);
        }
        TEST_ASSERT_EQUAL(0, Serial.capture_len);
        TEST_ASSERT_EQUAL(0, sm.tx_count);
    }
    TEST_ASSERT_EQUAL(6, telemetry.tx_deadline_expired);
    TEST_ASSERT_EQUAL(0, telemetry.tx_deadline_blocked);
    TEST_ASSERT_EQUAL(0, telemetry.tx_short_write);
    TEST_ASSERT_EQUAL(TxAbortReason::DEADLINE, sm.last_abort.reason);
    TEST_ASSERT_EQUAL(0, sm.last_abort.offset);
    TEST_ASSERT_EQUAL(20, sm.last_abort.deadline_overrun_ms);
    TEST_ASSERT_FALSE(sm.last_abort.transport_blocked);
}

void test_partly_sent_frame_expiry_with_free_fifo_is_not_stall(void) {
    sm.state = SessionState::ONLINE;
    Serial.mock_avail = 64;
    const char* long_payload =
        "STATE,2,123456789,123456789,-123.456,-123.456,-123.456,"
        "-123.456,-123.456,-123.456";
    for (uint32_t now = 0; now < 600; now += 200) {
        TEST_ASSERT_TRUE(sm.enqueue(long_payload, 1, now + 100, telemetry));
        sm.pump_tx(now + 20, telemetry);
        TEST_ASSERT_EQUAL(64, sm.tx_queue[sm.tx_head].offset);
        // Missing a later task tick must not turn a successful write into a stall.
        sm.pump_tx(now + 120, telemetry);
        sm.update(true, s_usb_tx_event_cnt, true);
        TEST_ASSERT_FALSE(sm.recovery_pending);
        TEST_ASSERT_EQUAL(0, sm.consecutive_partials);
    }
    TEST_ASSERT_EQUAL(192, Serial.capture_len);
    TEST_ASSERT_EQUAL(64, sm.last_abort.offset);
    TEST_ASSERT_EQUAL(64, sm.last_abort.available);
}

void test_transient_full_fifo_that_drains_before_expiry_is_not_stall(void) {
    sm.state = SessionState::ONLINE;
    for (uint32_t now = 0; now < 600; now += 200) {
        TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, now + 100, telemetry));
        Serial.mock_avail = 0;
        sm.pump_tx(now + 20, telemetry);
        Serial.mock_avail = 64;
        sm.pump_tx(now + 120, telemetry);
        sm.update(true, s_usb_tx_event_cnt, true);
        TEST_ASSERT_FALSE(sm.recovery_pending);
        TEST_ASSERT_EQUAL(0, sm.consecutive_partials);
    }
}

void test_tx_completion_invalidates_old_blocked_observation(void) {
    sm.state = SessionState::ONLINE;
    Serial.mock_avail = 0;
    s_usb_tx_event_cnt = UINT32_MAX;
    sm.update(true, s_usb_tx_event_cnt, true);
    for (uint32_t now = 0; now < 600; now += 200) {
        TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, now + 100, telemetry));
        sm.pump_tx(now + 20, telemetry);
        sm.update(true, ++s_usb_tx_event_cnt, true);
        // No new blocked attempt since the host made progress.
        sm.pump_tx(now + 120, telemetry);
        TEST_ASSERT_EQUAL(0, sm.consecutive_partials);
        TEST_ASSERT_FALSE(sm.recovery_pending);
    }
}

void test_sustained_full_fifo_still_recovers_with_valid_rx(void) {
    sm.state = SessionState::ONLINE;
    Serial.mock_avail = 0;
    for (uint32_t i = 0; i < 3; ++i) {
        const uint32_t now = i * 200;
        TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, now + 100, telemetry));
        sm.pump_tx(now + 20, telemetry);
        sm.pump_tx(now + 80, telemetry);
        sm.pump_tx(now + 100, telemetry);
        sm.update(true, s_usb_tx_event_cnt, true);
        TEST_ASSERT_EQUAL(i + 1, sm.consecutive_partials);
        TEST_ASSERT_EQUAL(i == 2, sm.recovery_pending);
    }
    TEST_ASSERT_EQUAL(SessionRecoveryReason::TX_STALLED, sm.recovery_reason);
    TEST_ASSERT_EQUAL(3, telemetry.tx_deadline_expired);
    TEST_ASSERT_EQUAL(3, telemetry.tx_deadline_blocked);
    TEST_ASSERT_TRUE(sm.last_abort.transport_blocked);
    TEST_ASSERT_EQUAL(0, sm.last_abort.available);
}

void test_short_writes_still_trigger_tx_recovery(void) {
    sm.state = SessionState::ONLINE;
    Serial.mock_avail = 64;
    Serial.next_write_limit = 1;
    for (uint32_t i = 0; i < 3; ++i) {
        TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, 100, telemetry));
        sm.pump_tx(20, telemetry);
        sm.update(true, s_usb_tx_event_cnt, true);
        TEST_ASSERT_EQUAL(i + 1, sm.consecutive_partials);
        TEST_ASSERT_EQUAL(i == 2, sm.recovery_pending);
    }
    TEST_ASSERT_EQUAL(SessionRecoveryReason::TX_STALLED, sm.recovery_reason);
    TEST_ASSERT_EQUAL(3, telemetry.tx_short_write);
    TEST_ASSERT_EQUAL(0, telemetry.tx_deadline_expired);
    TEST_ASSERT_EQUAL(TxAbortReason::SHORT_WRITE, sm.last_abort.reason);
    TEST_ASSERT_EQUAL(1, sm.last_abort.offset);
    TEST_ASSERT_TRUE(sm.last_abort.transport_blocked);
}

void test_buffer_acceptance_alone_cannot_hide_later_tx_stall(void) {
    sm.state = SessionState::ONLINE;
    // Serial.write accepting bytes only puts them into the device FIFO.
    // Without TX-complete, a subsequently full endpoint still needs recovery.
    TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, 100, telemetry));
    Serial.mock_avail = 64;
    sm.pump_tx(20, telemetry);
    TEST_ASSERT_EQUAL(1, telemetry.tx_completed);
    TEST_ASSERT_EQUAL(0, s_usb_tx_event_cnt);
    Serial.mock_avail = 0;
    for (uint32_t i = 0; i < 3; ++i) {
        const uint32_t now = 200 + i * 200;
        TEST_ASSERT_TRUE(sm.enqueue("STATE,2,2", 1, now + 100, telemetry));
        sm.pump_tx(now + 20, telemetry);
        sm.pump_tx(now + 100, telemetry);
        sm.update(true, s_usb_tx_event_cnt, true);
    }
    TEST_ASSERT_TRUE(sm.recovery_pending);
    TEST_ASSERT_EQUAL(SessionRecoveryReason::TX_STALLED, sm.recovery_reason);
    // Session cleanup clears diagnostics that refer to the previous queue.
    sm.reset();
    TEST_ASSERT_EQUAL(TxAbortReason::NONE, sm.last_abort.reason);
    TEST_ASSERT_EQUAL(0, sm.consecutive_partials);
}

void test_unserviced_queue_tail_does_not_inherit_head_stall(void) {
    sm.state = SessionState::ONLINE;
    Serial.mock_avail = 0;
    TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, 100, telemetry));
    TEST_ASSERT_TRUE(sm.enqueue("ENV,2,2", 2, 100, telemetry));
    sm.pump_tx(20, telemetry);
    sm.pump_tx(120, telemetry); // Head has observed a full FIFO twice.
    TEST_ASSERT_EQUAL(1, sm.consecutive_partials);
    sm.pump_tx(140, telemetry); // Tail first serviced after deadline.
    TEST_ASSERT_EQUAL(1, sm.consecutive_partials);
    // Reused ring slot must also start without the previous blocked evidence.
    TEST_ASSERT_TRUE(sm.enqueue("STATE,2,3", 1, 200, telemetry));
    sm.pump_tx(220, telemetry);
    sm.update(true, s_usb_tx_event_cnt, true);
    TEST_ASSERT_EQUAL(1, sm.consecutive_partials);
    TEST_ASSERT_FALSE(sm.recovery_pending);
}

void test_usb_recovery_deadline_wrap(void) {
    UsbRecoveryController recovery;

    recovery.observeValidRx();
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::DETACH,
        recovery.request(UINT32_MAX - 99, 250));
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::NONE, recovery.poll(UINT32_MAX - 1));
    TEST_ASSERT_EQUAL(UsbRecoveryAction::NONE, recovery.poll(149));
    TEST_ASSERT_EQUAL(UsbRecoveryAction::ATTACH, recovery.poll(150));
}

void test_usb_recovery_can_run_again_after_attach(void) {
    UsbRecoveryController recovery;

    recovery.observeValidRx();
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::DETACH, recovery.request(10, 1));
    TEST_ASSERT_EQUAL(UsbRecoveryAction::ATTACH, recovery.poll(11));
    TEST_ASSERT_FALSE(recovery.armed());
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::NONE, recovery.request(20, 1));

    recovery.observeValidRx();
    TEST_ASSERT_TRUE(recovery.armed());
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::DETACH, recovery.request(21, 1));
}

void test_usb_recovery_does_not_retry_without_new_valid_rx(void) {
    UsbRecoveryController recovery;

    recovery.observeValidRx();
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::DETACH, recovery.request(100, 1000));
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::ATTACH, recovery.poll(1100));

    for (uint32_t now = 4100; now < 600000; now += 4000) {
        TEST_ASSERT_EQUAL(
            UsbRecoveryAction::NONE, recovery.request(now, 1000));
    }
}

void test_valid_rx_rearms_one_shot_recovery(void) {
    UsbRecoveryController recovery;

    recovery.observeValidRx();
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::DETACH, recovery.request(100, 1000));
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::ATTACH, recovery.poll(1100));
    recovery.observeValidRx();

    // A later genuine session failure is eligible only after valid RX has
    // proved that a new host session is active.
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::DETACH, recovery.request(1200, 1000));
}

void test_host_disconnect_disarms_physical_recovery(void) {
    UsbRecoveryController recovery;

    recovery.observeValidRx();
    TEST_ASSERT_TRUE(recovery.armed());
    recovery.observeHostDisconnect();
    TEST_ASSERT_FALSE(recovery.armed());
    TEST_ASSERT_EQUAL(
        UsbRecoveryAction::NONE, recovery.request(100, 1000));
}

void test_valid_rx_cannot_hide_three_tx_stalls(void) {
    TxLivenessStateMachine local_sm;

    local_sm.state = SessionState::ONLINE;
    local_sm.last_tx_event_cnt = 77;
    local_sm.consecutive_partials = 3;
    local_sm.update(true, 77, true);

    TEST_ASSERT_TRUE(local_sm.recovery_pending);
    TEST_ASSERT_EQUAL(
        SessionRecoveryReason::TX_STALLED, local_sm.recovery_reason);
    TEST_ASSERT_EQUAL(3, local_sm.consecutive_partials);
}

void test_valid_rx_with_dtr_low_does_not_request_physical_recovery(void) {
    TxLivenessStateMachine local_sm;

    local_sm.state = SessionState::ONLINE;
    local_sm.consecutive_partials = 3;
    local_sm.update(false, 77, true);

    TEST_ASSERT_FALSE(local_sm.recovery_pending);
    TEST_ASSERT_EQUAL(SessionRecoveryReason::NONE, local_sm.recovery_reason);
}

void test_tx_complete_event_clears_stall_evidence(void) {
    TxLivenessStateMachine local_sm;

    local_sm.state = SessionState::ONLINE;
    local_sm.last_tx_event_cnt = UINT32_MAX;
    local_sm.consecutive_partials = 2;
    local_sm.update(true, 0, true);

    TEST_ASSERT_FALSE(local_sm.recovery_pending);
    TEST_ASSERT_EQUAL(0, local_sm.consecutive_partials);
    TEST_ASSERT_EQUAL(0, local_sm.last_tx_event_cnt);
}

void test_pump_requires_host_and_explicit_off_to_rearm(void) {
    PumpSafety pump;
    TEST_ASSERT_FALSE(pump.tick(0, true));
    pump.request(true, 100);
    TEST_ASSERT_FALSE(pump.tick(100, false));
    TEST_ASSERT_TRUE(pump.lockedOff());
    pump.request(true, 101);
    TEST_ASSERT_FALSE(pump.tick(101, true));
    pump.request(false, 102);
    pump.request(true, 103);
    TEST_ASSERT_TRUE(pump.tick(103, true));
}

void test_pump_lease_and_hard_run_cap(void) {
    PumpSafety lease;
    lease.request(true, 10);
    TEST_ASSERT_TRUE(lease.tick(760, true));
    TEST_ASSERT_FALSE(lease.tick(761, true));
    lease.request(true, 762);
    TEST_ASSERT_FALSE(lease.tick(762, true));

    PumpSafety run;
    run.request(true, 100);
    for (uint32_t now = 200;
         now < 100 + PumpSafety::kMaxRunMs; now += 100) {
        run.request(true, now);
        TEST_ASSERT_TRUE(run.tick(now, true));
    }
    run.request(true, 100 + PumpSafety::kMaxRunMs);
    TEST_ASSERT_FALSE(run.tick(100 + PumpSafety::kMaxRunMs, true));
    TEST_ASSERT_TRUE(run.lockedOff());
}

void test_pump_millis_wrap_does_not_extend_run(void) {
    PumpSafety pump;
    const uint32_t started_at = UINT32_MAX - 100;
    pump.request(true, started_at);
    for (uint32_t elapsed = 100;
         elapsed < PumpSafety::kMaxRunMs; elapsed += 100) {
        const uint32_t now = started_at + elapsed;
        pump.request(true, now);
        TEST_ASSERT_TRUE(pump.tick(now, true));
    }
    const uint32_t deadline = started_at + PumpSafety::kMaxRunMs;
    pump.request(true, deadline);
    TEST_ASSERT_FALSE(pump.tick(deadline, true));
    TEST_ASSERT_TRUE(pump.lockedOff());
}

void test_usb_recovery_snapshot_survives_session_cleanup_and_fits_wire(void) {
    UsbRecoveryDiagnostics d;
    d.recoveries = UINT32_MAX;
    d.recovery_ms = UINT32_MAX;
    d.abort.reason = TxAbortReason::DEADLINE;
    d.abort.offset = 64;
    d.abort.len = 192;
    d.abort.available = 0;
    d.abort.deadline_overrun_ms = UINT32_MAX;
    d.task_gap_ms = UINT32_MAX;
    d.tx_event_age_ms = UINT32_MAX;
    d.tx_event_seen = true;
    telemetry.tx_deadline_expired = UINT32_MAX;
    telemetry.tx_deadline_blocked = UINT32_MAX;
    telemetry.tx_short_write = UINT32_MAX;
    sm.reset(); // A session reset must not clear the caller's snapshot.
    char payload[SERIAL_BUFFER_SIZE];
    char wire[SERIAL_BUFFER_SIZE];
    TEST_ASSERT_TRUE(serialUsbDiagnosticsPayload(payload, sizeof(payload),
        UINT32_MAX, UINT32_MAX, d, telemetry) > 0);
    const char* expected =
        "USB,2,4294967295,4294967295,4294967295,4294967295,1,64,192,0,"
        "4294967295,4294967295,4294967295,1,4294967295,4294967295,4294967295";
    TEST_ASSERT_EQUAL_STRING(expected, payload);
    const size_t bytes = serialEncodeFrame(wire, sizeof(wire), payload);
    TEST_ASSERT_TRUE(bytes > 0 && bytes <= SERIAL_MAX_FRAME_SIZE);
    TEST_ASSERT_EQUAL(0, serialUsbDiagnosticsPayload(payload, 8,
        0, 0, d, telemetry));
}

void test_boot_identity_is_bounded_diagnostic_frame(void) {
    char sha[65];
    for (size_t i = 0; i < 64; ++i) sha[i] = 'a';
    sha[64] = '\0';
    char payload[SERIAL_BUFFER_SIZE];
    char wire[SERIAL_BUFFER_SIZE];
    TEST_ASSERT_TRUE(serialBootIdentityPayload(payload, sizeof(payload),
        UINT32_MAX, UINT32_MAX, 9, sha) > 0);
    TEST_ASSERT_EQUAL_STRING("BOOT,2,4294967295,4294967295,9,"
                             "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                             payload);
    const size_t bytes = serialEncodeFrame(wire, sizeof(wire), payload);
    TEST_ASSERT_TRUE(bytes > 0 && bytes <= SERIAL_MAX_FRAME_SIZE);
    TEST_ASSERT_TRUE(serialBootIdentityPayload(payload, sizeof(payload),
        1, 2, 0, "unknown") > 0);
    TEST_ASSERT_EQUAL_STRING("BOOT,2,1,2,0,unknown", payload);
    TEST_ASSERT_EQUAL(0, serialBootIdentityPayload(payload, 3,
        1, 2, 0, "unknown"));
    TEST_ASSERT_EQUAL(0, serialBootIdentityPayload(payload, sizeof(payload),
        1, 2, 0, "bad,field"));
}

void test_early_open_snapshot_survives_next_idle_rx_tick(void) {
    UsbLineState line;
    // The host opened before callback registration, so no event reached us.
    TEST_ASSERT_TRUE(line.seedIfUnchanged(line.revision(), true));
    sm.update(line.dtr(), 0, true);
    TEST_ASSERT_TRUE(sm.enqueue("STATE,2,1", 1, 100, telemetry));
    sm.update(line.dtr(), 0, false);
    TEST_ASSERT_EQUAL(SessionState::ONLINE, sm.state);
    TEST_ASSERT_FALSE(sm.recovery_pending);
    sm.pump_tx(20, telemetry);
    TEST_ASSERT_EQUAL(1, telemetry.tx_completed);
    TEST_ASSERT_EQUAL(0, line.disconnectEpoch());
}

void test_startup_snapshot_does_not_bypass_probe_or_disconnect(void) {
    UsbLineState line;
    TEST_ASSERT_TRUE(line.seedIfUnchanged(line.revision(), false));
    sm.update(line.dtr(), 0);
    TEST_ASSERT_EQUAL(SessionState::OFFLINE, sm.state);
    TEST_ASSERT_FALSE(sm.recovery_pending);
    line.onLineState(true); // Normal host open after startup.
    sm.update(line.dtr(), 0);
    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);
    TEST_ASSERT_FALSE(sm.recovery_pending);
    line.onLineState(false); // Real close must still tear down the session.
    sm.update(line.dtr(), 0);
    TEST_ASSERT_EQUAL(1, line.disconnectEpoch());
    TEST_ASSERT_EQUAL(SessionRecoveryReason::HOST_DISCONNECTED,
                      sm.recovery_reason);
}

void test_close_during_startup_snapshot_wins_over_stale_open(void) {
    UsbLineState line;
    line.onLineState(true);
    const uint32_t revision_before_read = line.revision();
    line.onLineState(false); // Callback arrives before snapshot commit.
    TEST_ASSERT_FALSE(line.seedIfUnchanged(revision_before_read, true));
    TEST_ASSERT_FALSE(line.dtr());
    TEST_ASSERT_EQUAL(1, line.disconnectEpoch());
}

void test_open_during_startup_snapshot_wins_over_stale_closed(void) {
    UsbLineState line;
    const uint32_t revision_before_read = line.revision();
    line.onLineState(true);
    TEST_ASSERT_FALSE(line.seedIfUnchanged(revision_before_read, false));
    TEST_ASSERT_TRUE(line.dtr());
    TEST_ASSERT_EQUAL(0, line.disconnectEpoch());
}

void test_close_after_startup_snapshot_still_invalidates_session(void) {
    UsbLineState line;
    TEST_ASSERT_TRUE(line.seedIfUnchanged(line.revision(), true));
    sm.update(line.dtr(), 0, true);
    line.onDisconnect(); // Callback arrives after snapshot commit.
    sm.update(line.dtr(), 0);
    TEST_ASSERT_FALSE(line.dtr());
    TEST_ASSERT_EQUAL(1, line.disconnectEpoch());
    TEST_ASSERT_EQUAL(SessionRecoveryReason::HOST_DISCONNECTED,
                      sm.recovery_reason);
}

void test_startup_snapshot_of_closed_usb_preserves_discard_epoch(void) {
    UsbLineState line;
    line.onLineState(true);
    // The USB stack sees close before the queued close callback arrives.
    TEST_ASSERT_TRUE(line.seedIfUnchanged(line.revision(), false));
    TEST_ASSERT_FALSE(line.dtr());
    TEST_ASSERT_EQUAL(1, line.disconnectEpoch());
    line.onLineState(false);
    TEST_ASSERT_EQUAL(1, line.disconnectEpoch());
}

void test_forced_detach_cannot_be_undone_by_startup_snapshot(void) {
    UsbLineState line;
    line.onLineState(true);
    const uint32_t revision_before_read = line.revision();
    line.onDisconnect(); // Same method used before firmware soft detach.
    TEST_ASSERT_FALSE(line.seedIfUnchanged(revision_before_read, true));
    TEST_ASSERT_FALSE(line.dtr());
    TEST_ASSERT_EQUAL(1, line.disconnectEpoch());
    sm.update(line.dtr(), 0);
    TEST_ASSERT_EQUAL(SessionState::OFFLINE, sm.state);
    TEST_ASSERT_FALSE(sm.recovery_pending);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_motion_command_age_stops_when_serial_task_stalls);
    RUN_TEST(test_motion_command_age_wraps_without_rearming_stale_command);
    RUN_TEST(test_pump_owner_stops_when_serial_task_stalls);
    RUN_TEST(test_pump_serial_lease_wraps_without_extending_deadline);
    RUN_TEST(test_fleeting_pump_off_does_not_restart_continuous_run_cap);
    RUN_TEST(test_dtr_fall_and_cleanup_recovery);
    RUN_TEST(test_dtr_low_while_offline_does_not_request_recovery);
    RUN_TEST(test_probe_event_in_write_ordering);
    RUN_TEST(test_probe_without_tx_event_stays_probing);
    RUN_TEST(test_stale_event_before_arm_is_absorbed);
    RUN_TEST(test_probe_event_counter_wrap_goes_online);
    RUN_TEST(test_probe_retry_recovers_from_missed_event);
    RUN_TEST(test_valid_rx_activity_recovers_without_tx_event);
    RUN_TEST(test_invalid_rx_cannot_confirm_liveness);
    RUN_TEST(test_combined_cleanup_prevents_stale_gap);
    RUN_TEST(test_actual_partial_abort_resync_parser);
    RUN_TEST(test_scheduler_rate);
    RUN_TEST(test_temporary_avail_zero_and_deadline_wrap);
    RUN_TEST(test_unsent_expired_frames_do_not_prove_tx_stall);
    RUN_TEST(test_partly_sent_frame_expiry_with_free_fifo_is_not_stall);
    RUN_TEST(test_transient_full_fifo_that_drains_before_expiry_is_not_stall);
    RUN_TEST(test_tx_completion_invalidates_old_blocked_observation);
    RUN_TEST(test_sustained_full_fifo_still_recovers_with_valid_rx);
    RUN_TEST(test_short_writes_still_trigger_tx_recovery);
    RUN_TEST(test_buffer_acceptance_alone_cannot_hide_later_tx_stall);
    RUN_TEST(test_unserviced_queue_tail_does_not_inherit_head_stall);
    RUN_TEST(test_usb_recovery_detach_is_one_shot);
    RUN_TEST(test_usb_recovery_deadline_wrap);
    RUN_TEST(test_usb_recovery_can_run_again_after_attach);
    RUN_TEST(test_usb_recovery_does_not_retry_without_new_valid_rx);
    RUN_TEST(test_valid_rx_rearms_one_shot_recovery);
    RUN_TEST(test_host_disconnect_disarms_physical_recovery);
    RUN_TEST(test_valid_rx_cannot_hide_three_tx_stalls);
    RUN_TEST(test_valid_rx_with_dtr_low_does_not_request_physical_recovery);
    RUN_TEST(test_tx_complete_event_clears_stall_evidence);
    RUN_TEST(test_pump_requires_host_and_explicit_off_to_rearm);
    RUN_TEST(test_pump_lease_and_hard_run_cap);
    RUN_TEST(test_pump_millis_wrap_does_not_extend_run);
    RUN_TEST(test_usb_recovery_snapshot_survives_session_cleanup_and_fits_wire);
    RUN_TEST(test_boot_identity_is_bounded_diagnostic_frame);
    RUN_TEST(test_early_open_snapshot_survives_next_idle_rx_tick);
    RUN_TEST(test_startup_snapshot_does_not_bypass_probe_or_disconnect);
    RUN_TEST(test_close_during_startup_snapshot_wins_over_stale_open);
    RUN_TEST(test_open_during_startup_snapshot_wins_over_stale_closed);
    RUN_TEST(test_close_after_startup_snapshot_still_invalidates_session);
    RUN_TEST(test_startup_snapshot_of_closed_usb_preserves_discard_epoch);
    RUN_TEST(test_forced_detach_cannot_be_undone_by_startup_snapshot);
    return UNITY_END();
}
