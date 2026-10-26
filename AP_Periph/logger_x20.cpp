/*
  X20 receiver configuration for the logger build

  The AP_GPS u-blox driver can't configure an X20 (no MON-VER match,
  legacy CFG-MSG rates), so with GPS_AUTO_CONFIG 0 the logger writes a
  RAM-layer CFG-VALSET itself. The X20 emits no UBX by default, so the
  driver can't detect it until this config lands: the VALSET is written
  straight to the GPS UART every 250 ms while the driver probes baud
  rates, until the driver reports a receiver. The config is then read
  back with CFG-VALGET and compared, and MON-VER / MON-GNSS are polled.
  Every response also lands in UBXR. The readback and polls are
  repeated at each new log file.
 */
#include "AP_Periph.h"

#if AP_PERIPH_LOGGER

namespace {

struct ConfigItem {
    uint32_t key;
    uint32_t value;
};

// u-blox X20 HPG 2.02 keys, see docs/superpowers/specs/2026-10-26-herepro-logger-design.md
const ConfigItem x20_config[] {
    { 0x30210001, 200 },    // CFG-RATE-MEAS 200 ms (5 Hz)
    { 0x30210002, 1 },      // CFG-RATE-NAV
    { 0x20110021, 4 },      // CFG-NAVSPG-DYNMODEL automotive
    { 0x10740002, 0 },      // CFG-UART1OUTPROT-NMEA off
    { 0x20910007, 1 },      // MSGOUT NAV-PVT
    { 0x20910039, 1 },      // MSGOUT NAV-DOP
    { 0x209102a5, 1 },      // MSGOUT RXM-RAWX
    { 0x20910232, 1 },      // MSGOUT RXM-SFRBX
    { 0x2091035a, 5 },      // MSGOUT MON-RF
    { 0x20910346, 5 },      // MSGOUT NAV-SIG
    { 0x20910016, 5 },      // MSGOUT NAV-SAT
    { 0x2091069e, 5 },      // MSGOUT MON-SYS
    { 0x20910179, 1 },      // MSGOUT TIM-TM2
    { 0x2091017e, 1 },      // MSGOUT TIM-TP
    { 0x40520001, 921600 }, // CFG-UART1-BAUDRATE, applied last by the receiver
};

// read back only (logged, not compared): signal group, signal plan, timepulse group
const uint32_t x20_readback_only[] { 0x1031ffff, 0x2031003a, 0x1005ffff };

constexpr uint8_t CLASS_ACK = 0x05;
constexpr uint8_t CLASS_CFG = 0x06;
constexpr uint8_t CLASS_MON = 0x0A;
constexpr uint8_t MSG_CFG_VALSET = 0x8A;
constexpr uint8_t MSG_CFG_VALGET = 0x8B;
constexpr uint8_t MSG_MON_VER = 0x04;
constexpr uint8_t MSG_MON_GNSS = 0x28;
constexpr uint8_t LAYER_RAM = 0x01;

// value width in bytes from the key's size field (bits 28-30)
uint8_t config_value_size(uint32_t key)
{
    switch ((key >> 28) & 0x07) {
    case 1: // bit
    case 2: return 1;
    case 3: return 2;
    case 4: return 4;
    case 5: return 8;
    default: return 0;
    }
}

void put_le(uint8_t *out, uint64_t value, uint8_t size)
{
    for (uint8_t i = 0; i < size; i++) {
        out[i] = uint8_t(value >> (8 * i));
    }
}

}  // namespace

uint16_t AP_Periph_FW::ubx_build(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint8_t *out, uint16_t out_size)
{
    if (payload_len + 8U > out_size) {
        return 0;
    }
    out[0] = 0xB5;
    out[1] = 0x62;
    out[2] = msg_class;
    out[3] = msg_id;
    out[4] = payload_len & 0xFF;
    out[5] = payload_len >> 8;
    if (payload_len > 0) {
        memcpy(&out[6], payload, payload_len);
    }
    uint8_t ck_a = 0, ck_b = 0;
    for (uint16_t i = 2; i < payload_len + 6U; i++) {
        ck_b += (ck_a += out[i]);
    }
    out[payload_len + 6] = ck_a;
    out[payload_len + 7] = ck_b;
    return payload_len + 8;
}

bool AP_Periph_FW::x20_send(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    AP_HAL::UARTDriver *uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 0);
    uint8_t frame[160];
    const uint16_t len = ubx_build(msg_class, msg_id, payload, payload_len, frame, sizeof(frame));
    if (uart == nullptr || len == 0 || uart->txspace() < len) {
        return false;
    }
    return uart->write(frame, len) == len;
}

bool AP_Periph_FW::x20_send_valset()
{
    // version 0, layers, 2 reserved, then key/value pairs
    uint8_t payload[4 + ARRAY_SIZE(x20_config) * 8] {};
    uint16_t len = 0;
    payload[len++] = 0;
    payload[len++] = LAYER_RAM;
    len += 2;
    for (const auto &item : x20_config) {
        const uint8_t size = config_value_size(item.key);
        put_le(&payload[len], item.key, 4);
        put_le(&payload[len + 4], item.value, size);
        len += 4 + size;
    }
    return x20_send(CLASS_CFG, MSG_CFG_VALSET, payload, len);
}

bool AP_Periph_FW::x20_send_valget(bool readback_only_keys)
{
    // version 0, layer, position (2 bytes), then keys
    uint8_t payload[4 + 4 * ARRAY_SIZE(x20_config)] {};
    uint16_t len = 4;
    payload[1] = 0;     // layer 0 = RAM in VALGET
    if (readback_only_keys) {
        for (const auto key : x20_readback_only) {
            put_le(&payload[len], key, 4);
            len += 4;
        }
    } else {
        for (const auto &item : x20_config) {
            put_le(&payload[len], item.key, 4);
            len += 4;
        }
    }
    return x20_send(CLASS_CFG, MSG_CFG_VALGET, payload, len);
}

void AP_Periph_FW::x20_config_update()
{
    const uint32_t now_ms = AP_HAL::millis();
    const bool detected = gps.status() != AP_GPS::NO_GPS;

    switch (x20.state) {
    case X20State::SEND_VALSET:
        if (detected) {
            x20.state = X20State::READBACK;
            x20.step_ms = now_ms;
            break;
        }
        if (now_ms - x20.step_ms >= 250) {
            x20.step_ms = now_ms;
            x20_send_valset();
        }
        break;

    case X20State::READBACK:
        // let the driver settle at the new baud before polling
        if (now_ms - x20.step_ms < 1000) {
            break;
        }
        x20.matched_keys = 0;
        x20.readback_done = false;
        if (x20_send_valget(false) && x20_send_valget(true) &&
            x20_send(CLASS_MON, MSG_MON_VER, nullptr, 0) &&
            x20_send(CLASS_MON, MSG_MON_GNSS, nullptr, 0)) {
            x20.info_requested = false;
            x20.state = X20State::WAIT_READBACK;
            x20.step_ms = now_ms;
        }
        break;

    case X20State::WAIT_READBACK:
        if (x20.readback_done) {
            x20.ok = (x20.matched_keys == ARRAY_SIZE(x20_config));
            logger.Write_MessageF("X20 cfg %s %u/%u keys", x20.ok ? "ok" : "FAILED",
                                  unsigned(x20.matched_keys), unsigned(ARRAY_SIZE(x20_config)));
            x20.state = X20State::DONE;
        } else if (now_ms - x20.step_ms > 3000) {
            x20.ok = false;
            logger.Write_MessageF("X20 cfg FAILED no VALGET reply");
            x20.state = X20State::SEND_VALSET;
            x20.step_ms = now_ms;
        }
        break;

    case X20State::DONE:
        if (!detected) {
            // receiver lost (power cycle / reset): its RAM config is gone
            x20.ok = false;
            x20.state = X20State::SEND_VALSET;
        } else if (x20.info_requested) {
            x20.state = X20State::READBACK;
            x20.step_ms = now_ms - 1000;
        }
        break;
    }
}

void AP_Periph_FW::x20_on_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    if (msg_class == CLASS_ACK && payload_len >= 2 && payload[0] == CLASS_CFG && payload[1] == MSG_CFG_VALSET) {
        if (msg_id == 0x00) {
            logger.Write_MessageF("X20 VALSET NAK");
        }
        return;
    }
    if (msg_class != CLASS_CFG || msg_id != MSG_CFG_VALGET || payload_len < 4 || x20.state != X20State::WAIT_READBACK) {
        return;
    }
    // walk key/value pairs and compare against the config table
    uint16_t ofs = 4;
    bool known_key_seen = false;
    while (ofs + 4 <= payload_len) {
        uint32_t key;
        memcpy(&key, &payload[ofs], 4);
        const uint8_t size = config_value_size(key);
        if (size == 0 || ofs + 4 + size > payload_len) {
            break;
        }
        uint64_t value = 0;
        memcpy(&value, &payload[ofs + 4], size);
        for (const auto &item : x20_config) {
            if (item.key == key) {
                known_key_seen = true;
                if (item.value == value) {
                    x20.matched_keys++;
                } else {
                    logger.Write_MessageF("X20 key 0x%08lx=%lu want %lu", (unsigned long)key, (unsigned long)value, (unsigned long)item.value);
                }
            }
        }
        ofs += 4 + size;
    }
    if (known_key_seen) {
        x20.readback_done = true;
    }
}

#endif // AP_PERIPH_LOGGER
