/*
  raw UBX capture into the dataflash log (UBXR)

  every checksum-valid frame from the u-blox driver is re-framed
  (B5 62 class id len payload ck_a ck_b) and written as consecutive
  UBXR records of up to 128 bytes. Seq increments per record so the
  extractor can detect dropped records.
 */
#include "AP_Periph.h"

#if AP_PERIPH_LOGGER

#include <AP_GPS/AP_GPS_UBLOX.h>

static_assert(UBLOX_MAX_RXM_RAWX_SATS >= 150, "all-band RAWX needs a larger driver buffer");

static constexpr uint8_t UBXR_RECORD_BYTES = 128;

class UBXRecordWriter {
public:
    void push(uint8_t byte) {
        bytes()[len++] = byte;
        if (len == UBXR_RECORD_BYTES) {
            flush();
        }
    }
    void push(const uint8_t *src, uint16_t n) {
        while (n--) {
            push(*src++);
        }
    }
    void flush() {
        if (len == 0) {
            return;
        }
        memset(&bytes()[len], 0, UBXR_RECORD_BYTES - len);
        AP::logger().Write("UBXR", "TimeUS,Seq,Len,Data0,Data1", "QHBaa",
                           time_us, seq++, len, &data[0], &data[32]);
        len = 0;
    }
    uint64_t time_us;
private:
    uint8_t *bytes() { return (uint8_t *)data; }
    int16_t data[UBXR_RECORD_BYTES / 2];
    uint8_t len;
    uint16_t seq;   // wraps; shared by all frames
};

static UBXRecordWriter ubxr_writer;

void AP_Periph_FW::log_ubx_frame(uint8_t instance, uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    (void)instance;
    const uint64_t arrival_us = AP_HAL::micros64();
    x20_on_frame(msg_class, msg_id, payload, payload_len);
    timesync_on_frame(msg_class, msg_id, payload, payload_len, arrival_us);
    if (!logger.logging_started()) {
        return;
    }
    const uint8_t header[6] { 0xB5, 0x62, msg_class, msg_id, uint8_t(payload_len & 0xFF), uint8_t(payload_len >> 8) };
    uint8_t ck_a = 0, ck_b = 0;
    for (uint8_t i = 2; i < sizeof(header); i++) {
        ck_b += (ck_a += header[i]);
    }
    for (uint16_t i = 0; i < payload_len; i++) {
        ck_b += (ck_a += payload[i]);
    }

    ubxr_writer.time_us = arrival_us;
    ubxr_writer.push(header, sizeof(header));
    ubxr_writer.push(payload, payload_len);
    ubxr_writer.push(ck_a);
    ubxr_writer.push(ck_b);
    ubxr_writer.flush();
}

#endif // AP_PERIPH_LOGGER
