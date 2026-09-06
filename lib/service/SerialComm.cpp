/**
 * @file SerialComm.cpp
 * @brief Triển khai Raw Serial V2 protocol
 *
 * CRC16-CCITT-FALSE: poly 0x1021, init 0xFFFF, refin=false, refout=false
 *
 * Known-answer vectors (spec):
 *   CMD,2,1,0.000,0.000                  -> B006
 *   FIRE,2,2,0.000,0.000                 -> E53B
 *   PUMP,2,3,0                            -> D1AA
 *   STATE,2,1,1234,0.000,0.000,0.000,0.000,0.000,0.000 -> B1C7
 *   ENV,2,2,0,0.0,0.0,0.0,0             -> 5A2E
 */

#include "SerialComm.h"
#include "RobotConfig.h"
#include <string.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>

// ============================================================
// CRC16-CCITT-FALSE (byte-at-a-time, no lookup table)
// poly=0x1021, init=0xFFFF, refin=false, refout=false, xorout=0x0000
// ============================================================
uint16_t serialCrc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i]) << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

uint16_t serialCrc16(const char* payload, size_t len) {
    return serialCrc16((const uint8_t*)payload, len);
}

// ============================================================
// FRAME ENCODER
// ============================================================
size_t serialEncodeFrame(char* outBuf, size_t outSize, const char* payload) {
    size_t payloadLen = strlen(payload);

    // @PAYLOAD*CCCC\n = 1 + payloadLen + 1 + 4 + 1 = payloadLen + 7
    size_t totalLen = payloadLen + 7;
    // [F2-L14-3] Mức wire max là 192. outBuf phải đủ để chứa NUL (tức outSize >= totalLen + 1).
    if (totalLen > SERIAL_MAX_FRAME_SIZE || totalLen >= outSize) {
        return 0; // Overflow
    }

    // Tính CRC16 trên PAYLOAD
    uint16_t crc = serialCrc16(payload, payloadLen);

    // Format: @PAYLOAD*CCCC\n
    int written = snprintf(outBuf, outSize, "@%s*%04X\n", payload, crc);
    if (written <= 0 || (size_t)written >= outSize) {
        return 0;
    }

    return (size_t)written;
}

// ============================================================
// SEND FRAME — Non-blocking, trả status [F2-6]
// ============================================================
SerialTxStatus serialSendFrame(char* txBuf, size_t txBufSize,
                                const char* payload,
                                SerialCommTelemetry& telemetry) {
    size_t frameLen = serialEncodeFrame(txBuf, txBufSize, payload);
    if (frameLen == 0) {
        telemetry.tx_drop++;
        return TX_ENCODE_FAIL; // [F2-L4-3]
    }

    // Non-blocking: kiểm tra có đủ chỗ trong TX buffer không
    size_t avail = Serial.availableForWrite();
    if (avail < frameLen) {
        telemetry.tx_drop++;
        return TX_DROP;
    }

    size_t written = Serial.write((const uint8_t*)txBuf, frameLen);
    if (written < frameLen) {
        telemetry.tx_partial++;
        return TX_PARTIAL;
    }

    return TX_OK;
}

// ============================================================
// PARSER RESET
// ============================================================
void SerialParser::reset() {
    pos_ = 0;
    in_frame_ = false;
    overflow_ = false;
    last_rx_seq_ = 0;
    has_rx_seq_ = false;
}

// ============================================================
// FEED — Incremental byte parser
// [F2-4] Khi overflow, bỏ MỌI byte kể cả @ cho tới LF.
// ============================================================
bool SerialParser::feed(uint8_t b, SerialRxResult& result, SerialCommTelemetry& telemetry) {
    result.type = RX_NONE;
    result.is_duplicate = false;

    // [F2-4] Khi đang overflow, chỉ LF mới reset. Bỏ mọi byte khác kể cả @.
    if (overflow_) {
        if (b == '\n') {
            overflow_ = false;
            in_frame_ = false;
            pos_ = 0;
        }
        // Không xử lý @ hay bất kỳ byte nào khác khi overflow
        return false;
    }

    // Handle LF (end of frame)
    if (b == '\n') {
        if (in_frame_ && pos_ > 0) {
            // Null-terminate
            buf_[pos_] = '\0';

            // Bỏ CR cuối nếu có (chấp nhận CRLF)
            if (pos_ > 0 && buf_[pos_ - 1] == '\r') {
                buf_[pos_ - 1] = '\0';
                pos_--;
            }

            bool ok = parseFrame_(buf_, pos_, result, telemetry);
            in_frame_ = false;
            pos_ = 0;
            return ok;
        }
        // LF mà không có frame → ignore
        in_frame_ = false;
        pos_ = 0;
        return false;
    }

    // Handle @ (start of frame) — chỉ khi KHÔNG overflow
    if (b == '@') {
        // @ mới giữa frame chưa overflow → resync: bỏ frame cũ, bắt đầu mới
        in_frame_ = true;
        pos_ = 0;
        return false;
    }

    // Accumulate byte
    if (in_frame_) {
        if (pos_ < SERIAL_MAX_FRAME_SIZE - 2) {  // Chừa chỗ cho null-term
            buf_[pos_++] = (char)b;
        } else {
            // [F2-4] Overflow: đánh dấu, chờ LF để discard. Counter +1 đúng một lần.
            overflow_ = true;
            telemetry.rx_overflow++;
            in_frame_ = false;
            pos_ = 0;
        }
    }
    // Byte ngoài frame (trước @) → bỏ qua

    return false;
}

// ============================================================
// [F2-2] Helper: parse float NGHIÊM NGẶT
// - Token không rỗng
// - Parse hết chuỗi (endptr trỏ tới '\0')
// - Finite (không NaN/Inf)
// - Không ERANGE
// ============================================================
static bool safeParseFloat(const char* str, float& out) {
    if (str == nullptr || str[0] == '\0') return false;

    // [F2-L3-2] Reject mọi ASCII whitespace ở bất kỳ vị trí nào
    for (size_t i = 0; str[i] != '\0'; i++) {
        unsigned char c = (unsigned char)str[i];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r') {
            return false;
        }
    }

    errno = 0;
    char* endptr = nullptr;
    float val = strtof(str, &endptr);

    // endptr phải trỏ tới null terminator (parse hết token)
    if (endptr == nullptr || *endptr != '\0') return false;
    // endptr == str: không parse được gì
    if (endptr == str) return false;
    // ERANGE: overflow/underflow
    if (errno == ERANGE) return false;
    // Finite check
    if (!isfinite(val)) return false;

    out = val;
    return true;
}

// ============================================================
// [F2-2] Helper: parse uint32 NGHIÊM NGẶT
// - Token không rỗng
// - Chỉ số thập phân không âm (reject '-', '+' prefix cho simplicity)
// - Parse hết chuỗi
// - Không ERANGE
// - Không vượt UINT32_MAX
// ============================================================
static bool safeParseUint32(const char* str, uint32_t& out) {
    if (str == nullptr || str[0] == '\0') return false;

    // [F2-L3-2] uint32 phải đúng 0..9 ở mọi vị trí (loại bỏ âm, +, space, v.v.)
    for (size_t i = 0; str[i] != '\0'; i++) {
        if (str[i] < '0' || str[i] > '9') {
            return false;
        }
    }

    errno = 0;
    char* endptr = nullptr;
    unsigned long val = strtoul(str, &endptr, 10);

    // endptr phải trỏ tới null terminator
    if (endptr == nullptr || *endptr != '\0') return false;
    if (endptr == str) return false;
    if (errno == ERANGE) return false;

    // Trên platform 64-bit, unsigned long > UINT32_MAX là có thể
    // Trên ESP32 unsigned long = 32-bit nên luôn OK, nhưng check cho portable
    if (val > (unsigned long)UINT32_MAX) return false;

    out = (uint32_t)val;
    return true;
}

// ============================================================
// PARSE FRAME — Xử lý frame hoàn chỉnh (đã bỏ @ đầu và LF cuối)
// buf chứa: PAYLOAD*CCCC
//
// [F2-1] Sequence tracking SAU khi parse hoàn toàn thành công.
//        Duplicate CMD → trả frame hợp lệ nhưng is_duplicate=true.
// [F2-2] Numeric validation nghiêm ngặt.
// ============================================================
bool SerialParser::parseFrame_(const char* frame, size_t len,
                                SerialRxResult& result,
                                SerialCommTelemetry& telemetry) {
    // Tìm dấu * cuối cùng (ngăn cách payload và CRC)
    const char* star = nullptr;
    for (size_t i = len; i > 0; i--) {
        if (frame[i - 1] == '*') {
            star = &frame[i - 1];
            break;
        }
    }
    if (!star || (size_t)(star - frame) == 0) {
        telemetry.rx_parse_fail++;
        return false;
    }

    // Payload length
    size_t payloadLen = (size_t)(star - frame);

    // CRC string: phải đúng 4 hex chars
    const char* crcStr = star + 1;
    size_t crcLen = len - payloadLen - 1; // -1 cho *
    if (crcLen != 4) {
        telemetry.rx_crc_fail++;
        return false;
    }

    // [F2-L2-6] Enforce đúng 4 hex uppercase
    for (int i = 0; i < 4; i++) {
        char c = crcStr[i];
        if (!((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F'))) {
            telemetry.rx_crc_fail++;
            return false;
        }
    }

    // Parse CRC hex — phải parse hết 4 ký tự
    char* endptr = nullptr;
    unsigned long crcReceived = strtoul(crcStr, &endptr, 16);
    if (endptr != crcStr + 4) {
        telemetry.rx_crc_fail++;
        return false;
    }

    // Tính CRC trên payload
    uint16_t crcCalc = serialCrc16(frame, payloadLen);
    if ((uint16_t)crcReceived != crcCalc) {
        telemetry.rx_crc_fail++;
        return false;
    }

    // === CRC OK, parse payload ===
    // Copy payload để tokenize (strtok cần writable)
    char payloadBuf[SERIAL_BUFFER_SIZE];
    if (payloadLen >= sizeof(payloadBuf)) {
        telemetry.rx_parse_fail++;
        return false;
    }

    // [F2-L5-1] Bắt buộc không có byte NUL nhúng trong payload
    if (memchr(frame, '\0', payloadLen) != nullptr) {
        telemetry.rx_parse_fail++;
        return false;
    }

    // [F2-L4-1] Bắt buộc không có field rỗng (loại bỏ trường hợp strtok_r nuốt dấu phẩy)
    if (payloadLen == 0 || frame[0] == ',' || frame[payloadLen - 1] == ',') {
        telemetry.rx_parse_fail++;
        return false;
    }
    for (size_t i = 0; i < payloadLen - 1; i++) {
        if (frame[i] == ',' && frame[i + 1] == ',') {
            telemetry.rx_parse_fail++;
            return false;
        }
    }

    memcpy(payloadBuf, frame, payloadLen);
    payloadBuf[payloadLen] = '\0';

    // Tokenize bằng dấu phẩy
    const int MAX_FIELDS = 12;
    char* fields[MAX_FIELDS];
    int fieldCount = 0;

    char* saveptr = nullptr;
    char* token = strtok_r(payloadBuf, ",", &saveptr);
    while (token != nullptr && fieldCount < MAX_FIELDS) {
        fields[fieldCount++] = token;
        token = strtok_r(nullptr, ",", &saveptr);
    }

    if (fieldCount < 3) {
        telemetry.rx_parse_fail++;
        return false;
    }

    // Field 0: message type
    // Field 1: protocol version (phải là "2")
    // Field 2: sequence number
    const char* msgType = fields[0];
    uint32_t version;
    if (!safeParseUint32(fields[1], version) || version != SERIAL_PROTOCOL_VERSION) {
        telemetry.rx_parse_fail++;
        return false;
    }

    uint32_t seq;
    if (!safeParseUint32(fields[2], seq)) {
        telemetry.rx_parse_fail++;
        return false;
    }

    // === Parse theo message type TRƯỚC, sequence tracking SAU [F2-1] ===

    // CMD,2,seq,vx,wz  (5 fields)
    if (strcmp(msgType, "CMD") == 0) {
        if (fieldCount != 5) {
            telemetry.rx_parse_fail++;
            return false;
        }
        float vx, wz;
        if (!safeParseFloat(fields[3], vx) || !safeParseFloat(fields[4], wz)) {
            telemetry.rx_parse_fail++;
            return false;
        }

        // [F2-1] Sequence tracking CHỈ sau khi parse hoàn toàn thành công
        bool dup = false;
        if (has_rx_seq_) {
            if (seq == last_rx_seq_) {
                telemetry.rx_seq_dup++;
                dup = true;
            } else {
                uint32_t expected = last_rx_seq_ + 1;
                // Wrap chỉ hợp lệ khi UINT32_MAX -> 0
                if (seq != expected) {
                    if (!(last_rx_seq_ == UINT32_MAX && seq == 0)) {
                        telemetry.rx_seq_gap++;
                    }
                }
            }
        }
        last_rx_seq_ = seq;
        has_rx_seq_ = true;

        result.type = RX_CMD;
        result.seq  = seq;
        result.f1   = vx;
        result.f2   = wz;
        result.is_duplicate = dup;
        telemetry.rx_valid++;
        if (!dup) {
            telemetry.last_valid_cmd_ms = millis();
        }
        return true;
    }

    // FIRE,2,seq,x,y  (5 fields)
    if (strcmp(msgType, "FIRE") == 0) {
        if (fieldCount != 5) {
            telemetry.rx_parse_fail++;
            return false;
        }
        float fx, fy;
        if (!safeParseFloat(fields[3], fx) || !safeParseFloat(fields[4], fy)) {
            telemetry.rx_parse_fail++;
            return false;
        }
        // Validate range [-1, 1]
        if (fx < -1.0f || fx > 1.0f || fy < -1.0f || fy > 1.0f) {
            telemetry.rx_parse_fail++;
            return false;
        }

        // Sequence tracking sau parse thành công
        bool dup = false;
        if (has_rx_seq_) {
            if (seq == last_rx_seq_) {
                telemetry.rx_seq_dup++;
                dup = true;
            } else {
                uint32_t expected = last_rx_seq_ + 1;
                if (seq != expected) {
                    if (!(last_rx_seq_ == UINT32_MAX && seq == 0)) {
                        telemetry.rx_seq_gap++;
                    }
                }
            }
        }
        last_rx_seq_ = seq;
        has_rx_seq_ = true;

        result.type = RX_FIRE;
        result.seq  = seq;
        result.f1   = fx;
        result.f2   = fy;
        result.is_duplicate = dup;
        telemetry.rx_valid++;
        return true;
    }

    // PUMP,2,seq,on  (4 fields)
    if (strcmp(msgType, "PUMP") == 0) {
        if (fieldCount != 4) {
            telemetry.rx_parse_fail++;
            return false;
        }
        uint32_t on;
        if (!safeParseUint32(fields[3], on)) {
            telemetry.rx_parse_fail++;
            return false;
        }
        if (on != 0 && on != 1) {
            telemetry.rx_parse_fail++;
            return false;
        }

        // Sequence tracking sau parse thành công
        bool dup = false;
        if (has_rx_seq_) {
            if (seq == last_rx_seq_) {
                telemetry.rx_seq_dup++;
                dup = true;
            } else {
                uint32_t expected = last_rx_seq_ + 1;
                if (seq != expected) {
                    if (!(last_rx_seq_ == UINT32_MAX && seq == 0)) {
                        telemetry.rx_seq_gap++;
                    }
                }
            }
        }
        last_rx_seq_ = seq;
        has_rx_seq_ = true;

        result.type = RX_PUMP;
        result.seq  = seq;
        result.pump = (uint8_t)on;
        result.is_duplicate = dup;
        telemetry.rx_valid++;
        return true;
    }

    // Unknown message type
    telemetry.rx_parse_fail++;
    return false;
}

// ============================================================
// LIVENESS STATE MACHINE & TX PUMP
// ============================================================
void TxLivenessStateMachine::update(bool dtr, uint32_t current_tx_evt,
                                    bool valid_rx_activity) {
    if (recovery_pending) {
        return;
    }

    // A completely parsed command is stronger host-liveness evidence than
    // DTR or a CDC TX callback. This path lets a reopened host recover even
    // when TinyUSB does not emit another TX-complete event.
    if (valid_rx_activity) {
        state = SessionState::ONLINE;
        consecutive_partials = 0;
        probe_armed = false;
        return;
    }

    if (!dtr) {
        recovery_pending = true;
        return;
    }

    switch (state) {
        case SessionState::OFFLINE:
            state = SessionState::PROBING;
            break;
        case SessionState::PROBING:
            // [F2-L26-2] Chỉ TX progress sau khi probe được arm mới đưa ONLINE
            if (probe_armed) {
                if (current_tx_evt != probe_armed_tx_event_cnt) {
                    state = SessionState::ONLINE;
                    consecutive_partials = 0;
                    probe_armed = false;
                }
            }
            break;
        case SessionState::ONLINE:
            if (consecutive_partials >= 3) {
                recovery_pending = true; // [F2-L26-1] Caller must consume this and run cleanup
            }
            break;
    }
}

bool TxLivenessStateMachine::enqueue(const char* payload, uint8_t type,
                                     uint32_t deadline_ms,
                                     SerialCommTelemetry& telemetry) {
    telemetry.tx_generated++;
    if (tx_count >= 2) {
        telemetry.tx_drop++;
        return false;
    }

    size_t len = serialEncodeFrame(tx_queue[tx_tail].data, SERIAL_BUFFER_SIZE, payload);
    if (len > 0) {
        tx_queue[tx_tail].len = len;
        tx_queue[tx_tail].offset = 0;
        tx_queue[tx_tail].type = type;
        tx_queue[tx_tail].deadline_ms = deadline_ms;

        tx_tail = (tx_tail + 1) % 2;
        tx_count++;
        telemetry.tx_queued++;
        return true;
    }
    telemetry.tx_drop++;
    return false;
}

bool TxLivenessStateMachine::enqueue_probe(
        const char* payload, uint32_t deadline_ms,
        volatile uint32_t* event_cnt_ptr,
        SerialCommTelemetry& telemetry) {
    // Capture the event generation before the probe can be written. TinyUSB
    // may dispatch TX-complete synchronously from Serial.write(); sampling
    // after the write would absorb that proof and leave the session PROBING.
    const uint32_t baseline = *event_cnt_ptr;
    if (!enqueue(payload, 3, deadline_ms, telemetry)) {
        return false;
    }

    probe_armed_tx_event_cnt = baseline;
    probe_armed = true;
    return true;
}

void TxLivenessStateMachine::pump_tx(uint32_t now,
                                     SerialCommTelemetry& telemetry) {
    if (recovery_pending || state == SessionState::OFFLINE) return;

    if (tx_count > 0) {
        TxSlot* head = &tx_queue[tx_head];

        if ((int32_t)(now - head->deadline_ms) >= 0) {
            telemetry.tx_partial++;
            consecutive_partials++;
            tx_count--;
            tx_head = (tx_head + 1) % 2;
        } else {
            size_t remaining = head->len - head->offset;
            size_t avail = Serial.availableForWrite();
            size_t chunk = remaining;
            if (chunk > 64) chunk = 64;
            if (chunk > avail) chunk = avail;

            if (chunk > 0) {
                size_t written = Serial.write((const uint8_t*)&head->data[head->offset], chunk);
                if (written == chunk) {
                    head->offset += written;
                    if (head->offset >= head->len) {
                        telemetry.tx_completed++;
                        consecutive_partials = 0;
                        tx_count--;
                        tx_head = (tx_head + 1) % 2;

                    }
                } else {
                    // [F2-L24-3] Partial write -> Abort ngay lập tức, resync ở delimiter
                    telemetry.tx_partial++;
                    consecutive_partials++;
                    tx_count--;
                    tx_head = (tx_head + 1) % 2;
                }
            }
        }
    }
}

void TxLivenessStateMachine::reset() {
    state = SessionState::OFFLINE;
    consecutive_partials = 0;
    recovery_pending = false;
    probe_armed = false;
    tx_head = 0;
    tx_tail = 0;
    tx_count = 0;
}
