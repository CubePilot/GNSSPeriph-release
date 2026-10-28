/*
  X20 receiver configuration for the logger build

  The AP_GPS u-blox driver can't configure an X20 (no MON-VER match,
  legacy CFG-MSG rates), so with GPS_AUTO_CONFIG 0 the logger writes a
  RAM-layer CFG-VALSET itself. The X20 emits no UBX by default, so the
  driver can't detect it until this config lands: the VALSET is written
  straight to the GPS UART every 250 ms while the driver probes baud
  rates. A receiver that already outputs UBX (e.g. production flash
  config at 38400) is detected at once; in both cases, once the driver
  has a lock the VALSET is sent at the known baud, its ACK/NAK logged,
  the driver re-detects after the baud change, and the config is read
  back with CFG-VALGET and compared (up to 5 attempts). MON-VER /
  MON-GNSS are polled with each readback.
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
constexpr uint32_t KEY_UART1_BAUDRATE = 0x40520001;
constexpr uint32_t MIN_UART1_BAUDRATE = 460800;
constexpr uint32_t X20_BLIND_PERIOD_MS = 250;
constexpr uint32_t X20_ACK_TIMEOUT_MS = 1000;
constexpr uint32_t X20_NO_BAUD_CHANGE_MS = 6000;      // driver keeps its lock if the baud didn't change
constexpr uint32_t X20_REDETECT_TIMEOUT_MS = 30000;
constexpr uint8_t X20_MAX_ATTEMPTS = 5;

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
    case X20State::SEND_VALSET_BLIND:
        // silent receiver: the driver can't detect it until the config lands
        if (detected) {
            x20.state = X20State::SEND_VALSET;
            break;
        }
        if (now_ms - x20.step_ms >= X20_BLIND_PERIOD_MS) {
            x20.step_ms = now_ms;
            x20_send_valset();
        }
        break;

    case X20State::SEND_VALSET:
        // driver has a lock, so the receiver's current baud is known
        if (x20.attempts >= X20_MAX_ATTEMPTS) {
            logger.Write_MessageF("X20 cfg FAILED after %u attempts", unsigned(x20.attempts));
            x20.state = X20State::FAILED;
            break;
        }
        if (x20_send_valset()) {
            x20.attempts++;
            x20.ack_result = X20Ack::NONE;
            x20.saw_lost = false;
            logger.Write_MessageF("X20 VALSET attempt %u", unsigned(x20.attempts));
            x20.state = X20State::WAIT_ACK;
            x20.step_ms = now_ms;
        }
        break;

    case X20State::WAIT_ACK:
        if (x20.ack_result == X20Ack::NONE && now_ms - x20.step_ms < X20_ACK_TIMEOUT_MS) {
            break;
        }
        logger.Write_MessageF("X20 VALSET %s", x20.ack_result == X20Ack::ACK ? "ACK" :
                              x20.ack_result == X20Ack::NAK ? "NAK" : "no ACK");
        x20.state = X20State::WAIT_REDETECT;
        x20.step_ms = now_ms;
        break;

    case X20State::WAIT_REDETECT:
        // a baud change makes the driver lose the receiver and re-probe
        if (!detected) {
            x20.saw_lost = true;
            if (now_ms - x20.step_ms > X20_REDETECT_TIMEOUT_MS) {
                logger.Write_MessageF("X20 not re-detected after VALSET");
                x20.state = X20State::SEND_VALSET_BLIND;
            }
        } else if (x20.saw_lost || now_ms - x20.step_ms > X20_NO_BAUD_CHANGE_MS) {
            x20.state = X20State::READBACK;
            x20.step_ms = now_ms;
        }
        break;

    case X20State::READBACK:
        // let the driver settle before polling
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
            logger.Write_MessageF("X20 cfg %s %u/%u keys", x20.ok ? "ok" : "mismatch",
                                  unsigned(x20.matched_keys), unsigned(ARRAY_SIZE(x20_config)));
            x20.state = x20.ok ? X20State::DONE : X20State::SEND_VALSET;
        } else if (now_ms - x20.step_ms > 3000) {
            logger.Write_MessageF("X20 no VALGET reply");
            x20.state = X20State::SEND_VALSET;
        }
        break;

    case X20State::DONE:
        if (!detected) {
            // receiver lost (power cycle / reset): its RAM config is gone
            x20.ok = false;
            x20.attempts = 0;
            x20.state = X20State::SEND_VALSET_BLIND;
        } else if (x20.info_requested) {
            x20.state = X20State::READBACK;
            x20.step_ms = now_ms - 1000;
        }
        break;

    case X20State::FAILED:
        break;
    }
}

void AP_Periph_FW::x20_on_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len)
{
    if (msg_class == CLASS_ACK && payload_len >= 2 && payload[0] == CLASS_CFG) {
        if (payload[1] == MSG_CFG_VALSET && x20.state == X20State::WAIT_ACK) {
            x20.ack_result = (msg_id == 0x01) ? X20Ack::ACK : X20Ack::NAK;
        } else if (payload[1] == MSG_CFG_VALGET && msg_id == 0x00) {
            logger.Write_MessageF("X20 VALGET NAK");
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
            if (item.key != key) {
                continue;
            }
            known_key_seen = true;
            // the driver's detection blob may move the receiver to 460800; any rate that carries the stream is fine
            const bool match = (key == KEY_UART1_BAUDRATE) ? (value >= MIN_UART1_BAUDRATE) : (item.value == value);
            if (match) {
                x20.matched_keys++;
            }
            if (!match || key == KEY_UART1_BAUDRATE) {
                logger.Write_MessageF("X20 key 0x%08lx=%lu want %lu", (unsigned long)key, (unsigned long)value, (unsigned long)item.value);
            }
        }
        ofs += 4 + size;
    }
    if (known_key_seen) {
        x20.readback_done = true;
    }
}

#endif // AP_PERIPH_LOGGER
