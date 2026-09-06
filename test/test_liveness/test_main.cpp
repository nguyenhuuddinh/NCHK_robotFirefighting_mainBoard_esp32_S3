#include <unity.h>
#include "../../lib/service/SerialComm.cpp"
#include "SerialComm.h"

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

void test_dtr_fall_and_cleanup_recovery(void) {
    // DTR fall -> recovery_pending -> cleanup/reset -> DTR rise -> PROBING;
    sm.state = SessionState::ONLINE;

    // DTR fall
    sm.update(false, s_usb_tx_event_cnt);
    TEST_ASSERT_TRUE(sm.recovery_pending);
    TEST_ASSERT_EQUAL(SessionState::ONLINE, sm.state); // hasn't actually dropped yet

    // Caller consumes recovery_pending
    sm.reset();
    TEST_ASSERT_EQUAL(SessionState::OFFLINE, sm.state);

    // DTR rise -> PROBING
    sm.update(true, s_usb_tx_event_cnt);
    TEST_ASSERT_EQUAL(SessionState::PROBING, sm.state);
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
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_dtr_fall_and_cleanup_recovery);
    RUN_TEST(test_probe_event_in_write_ordering);
    RUN_TEST(test_probe_without_tx_event_stays_probing);
    RUN_TEST(test_stale_event_before_arm_is_absorbed);
    RUN_TEST(test_probe_event_counter_wrap_goes_online);
    RUN_TEST(test_probe_retry_recovers_from_missed_event);
    RUN_TEST(test_combined_cleanup_prevents_stale_gap);
    RUN_TEST(test_actual_partial_abort_resync_parser);
    RUN_TEST(test_scheduler_rate);
    RUN_TEST(test_temporary_avail_zero_and_deadline_wrap);
    return UNITY_END();
}
