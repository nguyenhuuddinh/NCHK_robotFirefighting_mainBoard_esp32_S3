/**
 * @file SerialComm.h
 * @brief Raw Serial V2 protocol — thay thế micro-ROS/XRCE-DDS
 *
 * Layer: service/ (logic giao tiếp, không truy cập GPIO/I2C)
 *
 * Protocol V2:
 *   Frame: @PAYLOAD*CCCC\n
 *   CRC16: CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
 *   Encoding: ASCII, comma-separated fields
 *   Max frame: 192 bytes (kể cả LF)
 *
 * TX (ESP32 -> Pi):
 *   STATE,2,seq,esp_ms,x,y,yaw,vx,wz,gyro_z  @ 10Hz
 *   ENV,2,seq,fire_flags,gas_ppm,temp_c,batt_v,valid  @ 2Hz
 *
 * RX (Pi -> ESP32):
 *   CMD,2,seq,vx,wz
 *   FIRE,2,seq,x,y
 *   PUMP,2,seq,on
 *
 * Transport: Serial USB CDC native (cổng USB trên board)
 * Debug: DBG/Serial0 (UART0/CH340)
 * KHÔNG log lên Serial USB CDC.
 */

#pragma once

#include <Arduino.h>
#include <stdint.h>

// ============================================================
// PROTOCOL CONSTANTS
// ============================================================
static const uint8_t  SERIAL_PROTOCOL_VERSION = 2;
static const size_t   SERIAL_MAX_FRAME_SIZE   = 192;
static const size_t   SERIAL_BUFFER_SIZE      = SERIAL_MAX_FRAME_SIZE + 1; // [F2-L15-4] 193 byte: 192 byte wire limit + 1 NUL terminator cho encoder

// ============================================================
// DIAGNOSTIC COUNTERS
// ============================================================
struct SerialCommTelemetry {
    // RX counters
    uint32_t raw_rx_bytes    = 0;  // [F2-L11-1] Diagnostics raw byte đọc được
    uint32_t rx_valid        = 0;  // Frame hợp lệ (CRC + parse OK)
    uint32_t rx_crc_fail     = 0;  // CRC sai
    uint32_t rx_parse_fail   = 0;  // CRC đúng nhưng parse fail (field/range)
    uint32_t rx_overflow     = 0;  // Frame > 192 bytes, bị discard

    // RX sequence tracking
    uint32_t rx_seq_gap      = 0;  // Sequence gap (mất frame)
    uint32_t rx_seq_dup      = 0;  // Sequence duplicate

    // TX counters (Segmented Queue)
    uint32_t tx_generated    = 0;  // Frame được tạo ra
    uint32_t tx_queued       = 0;  // Frame được đẩy vào internal queue
    uint32_t tx_completed    = 0;  // Frame hoàn tất gửi (all chunks sent)
    uint32_t tx_drop         = 0;  // Frame bị bỏ (offline, NaN, queue đầy)
    uint32_t tx_partial      = 0;  // Chunk bị partial write / abort slot

    // USB CDC Events
    uint32_t usb_events      = 0;

    // CMD age
    uint32_t last_valid_cmd_ms = 0; // millis() lúc nhận CMD hợp lệ cuối
};

// ============================================================
// TX RESULT (cho caller phân biệt full/drop/partial)
// ============================================================
enum SerialTxStatus {
    TX_OK = 0,      // Toàn bộ frame đã ghi thành công
    TX_DROP,        // Frame bị drop (buffer đầy)
    TX_PARTIAL,     // Ghi được một phần (parser bên nhận sẽ resync)
    TX_ENCODE_FAIL  // [F2-L4-3] Lỗi tạo frame (vd: vượt quá 192 bytes)
};

// ============================================================
// CRC16-CCITT-FALSE
// ============================================================

/**
 * @brief Tính CRC16-CCITT-FALSE trên buffer ASCII
 * @param data Con trỏ tới PAYLOAD (không gồm @, *, LF)
 * @param len Số byte
 * @return CRC16 value
 */
uint16_t serialCrc16(const uint8_t* data, size_t len);

/**
 * @brief Tính CRC16-CCITT-FALSE trên chuỗi C
 */
uint16_t serialCrc16(const char* payload, size_t len);

// ============================================================
// FRAME ENCODER (TX)
// ============================================================

/**
 * @brief Format một frame hoàn chỉnh @PAYLOAD*CCCC\n vào buffer tĩnh
 *
 * @param outBuf   Buffer đích (phải >= SERIAL_BUFFER_SIZE (193))
 * @param outSize  Kích thước buffer đích
 * @param payload  Chuỗi PAYLOAD đã được format (không gồm @, *, CRC, LF)
 * @return Số byte đã format (bao gồm \n), hoặc 0 nếu overflow
 *
 * KHÔNG allocate heap. Dùng snprintf.
 */
size_t serialEncodeFrame(char* outBuf, size_t outSize, const char* payload);

/**
 * @brief Gửi frame non-blocking qua Serial USB CDC
 *
 * @param txBuf     Buffer chứa frame đã encode
 * @param frameLen  Kích thước frame
 * @param telemetry [in/out] Counters (tx_drop, tx_partial)
 * @return TX_OK nếu ghi hết, TX_DROP nếu drop, TX_PARTIAL nếu ghi thiếu
 */
SerialTxStatus serialSendFrame(char* txBuf, size_t txBufSize,
                                const char* payload,
                                SerialCommTelemetry& telemetry);

// ============================================================
// INCREMENTAL PARSER (RX)
// ============================================================

/**
 * @brief Kiểu frame RX đã parse xong
 */
enum SerialRxType {
    RX_NONE = 0,
    RX_CMD,       // CMD,2,seq,vx,wz
    RX_FIRE,      // FIRE,2,seq,x,y
    RX_PUMP,      // PUMP,2,seq,on
};

/**
 * @brief Kết quả parse một frame RX
 */
struct SerialRxResult {
    SerialRxType type  = RX_NONE;
    uint32_t     seq   = 0;
    float        f1    = 0.0f;   // CMD: vx   | FIRE: x
    float        f2    = 0.0f;   // CMD: wz   | FIRE: y
    uint8_t      pump  = 0;      // PUMP: 0 or 1
    bool         is_duplicate = false; // true nếu seq trùng frame trước
};

/**
 * @brief Incremental frame parser
 *
 * Xử lý từng byte từ Serial. Khi tìm được frame hoàn chỉnh (LF),
 * validate CRC, parse fields.
 *
 * - Bỏ byte rác trước @
 * - Nếu gặp @ mới giữa frame chưa overflow → resync từ @ mới
 * - Quá 192 bytes → discard tới LF (kể cả @ mới), tăng overflow counter
 * - Sai CRC/version/field/range → discard frame đó, frame sau vẫn OK
 * - Duplicate sequence → trả frame hợp lệ nhưng đánh dấu is_duplicate
 */
class SerialParser {
public:
    SerialParser() { reset(); }

    /**
     * @brief Feed một byte vào parser
     * @param b Byte đọc từ Serial
     * @param result [out] Kết quả parse nếu có frame hoàn chỉnh
     * @param telemetry [in/out] Counters
     * @return true nếu result chứa frame hợp lệ (kể cả duplicate)
     */
    bool feed(uint8_t b, SerialRxResult& result, SerialCommTelemetry& telemetry);

    /**
     * @brief Reset parser state (dùng khi reconnect)
     */
    void reset();

private:
    char    buf_[SERIAL_BUFFER_SIZE];
    size_t  pos_;
    bool    in_frame_;
    bool    overflow_;

    // Sequence tracking (chung cho mọi loại RX command)
    uint32_t last_rx_seq_;
    bool     has_rx_seq_;

    /**
     * @brief Parse một frame hoàn chỉnh (đã bỏ @ đầu và LF cuối)
     */
    bool parseFrame_(const char* frame, size_t len,
                     SerialRxResult& result, SerialCommTelemetry& telemetry);
};

// ============================================================
// LIVENESS STATE MACHINE & TX PUMP
// ============================================================

enum class SessionState {
    OFFLINE,
    PROBING,
    ONLINE
};

enum class SessionRecoveryReason {
    NONE,
    HOST_DISCONNECTED,
    TX_STALLED
};

enum class UsbRecoveryAction {
    NONE,
    DETACH,
    ATTACH
};

class UsbRecoveryController {
public:
    UsbRecoveryAction request(uint32_t now, uint32_t detach_ms = 1000);
    UsbRecoveryAction poll(uint32_t now);
    bool observeValidRx() {
        const bool changed = !recovery_armed_;
        recovery_armed_ = true;
        return changed;
    }
    void observeHostDisconnect() { recovery_armed_ = false; }
    bool active() const { return active_; }
    bool armed() const { return recovery_armed_; }

private:
    bool active_ = false;
    bool recovery_armed_ = false;
    uint32_t reconnect_deadline_ms_ = 0;
};

struct TxSlot {
    char data[SERIAL_BUFFER_SIZE];
    uint16_t len;
    uint16_t offset;
    uint8_t type; // 1=STATE, 2=ENV, 3=PROBE
    uint32_t deadline_ms;
};

class TxLivenessStateMachine {
public:
    SessionState state = SessionState::OFFLINE;
    uint32_t consecutive_partials = 0;

    // TX Ring Queue
    TxSlot tx_queue[2];
    uint8_t tx_head = 0;
    uint8_t tx_tail = 0;
    uint8_t tx_count = 0;

    // Probe arming state
    uint32_t probe_armed_tx_event_cnt = 0;
    uint32_t last_tx_event_cnt = 0;
    bool probe_armed = false;

    // Transition flags
    bool recovery_pending = false;
    SessionRecoveryReason recovery_reason = SessionRecoveryReason::NONE;

    void update(bool dtr, uint32_t current_tx_evt,
                bool valid_rx_activity = false);
    void reset();

    bool enqueue(const char* payload, uint8_t type, uint32_t deadline_ms,
                 SerialCommTelemetry& telemetry);
    bool enqueue_probe(const char* payload, uint32_t deadline_ms,
                       volatile uint32_t* event_cnt_ptr,
                       SerialCommTelemetry& telemetry);
    void pump_tx(uint32_t now, SerialCommTelemetry& telemetry);
};

class TxScheduler {
public:
    uint8_t stateDivider = 0;
    uint8_t envDivider = 0;
    bool state_due = false;
    bool env_due = false;

    void tick() {
        stateDivider++;
        if (stateDivider >= 5) {
            stateDivider = 0;
            state_due = true;
        } else {
            state_due = false;
        }

        envDivider++;
        if (envDivider >= 25) {
            envDivider = 0;
            env_due = true;
        } else {
            env_due = false;
        }
    }
};
