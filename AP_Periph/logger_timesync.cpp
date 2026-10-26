/*
  MCU <-> GPS time alignment for the logger build

  EXTS: once a second the MCU raises the receiver's EXTINT line and logs
        the MCU time of the edge; the X20 reports the GPS time of the
        same edge in UBX-TIM-TM2 (captured in UBXR).
  PPSE: MCU time of each receiver timepulse edge (cross-check against
        UBX-TIM-TP).
  GEPO: per RAWX epoch, receiver time of week paired with the MCU time
        predicted from the latest EXTS/TIM-TM2 pairs (0 until 2 pairs).
 */
#include "AP_Periph.h"

#if AP_PERIPH_LOGGER

extern const AP_HAL::HAL &hal;

#ifndef GPS_EXTINT
#error "logger time sync needs GPS_EXTINT"
#endif

static constexpr uint32_t STROBE_PERIOD_MS = 1000;
static constexpr uint32_t STROBE_HIGH_MS = 10;
static constexpr uint32_t TM2_PAIR_WINDOW_US = 900000;
static constexpr double WEEK_S = 604800.0;

void AP_Periph_FW::timesync_init()
{
#ifdef HAL_GPIO_PPS
    hal.gpio->attach_interrupt(HAL_GPIO_PPS, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::pps_edge_isr, void, uint8_t, bool, uint32_t),
                               AP_HAL::GPIO::INTERRUPT_RISING);
#endif
}

void AP_Periph_FW::pps_edge_isr(uint8_t pin, bool high, uint32_t timestamp_us)
{
    (void)pin;
    (void)high;
    timesync.pps_edge_us32 = timestamp_us;
    timesync.pps_count++;
}

void AP_Periph_FW::timesync_update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // EXTINT strobe: the edge time lies within [TimeUS, TimeUS + WriteUS]
    if (!timesync.strobe_high && now_ms - timesync.last_strobe_ms >= STROBE_PERIOD_MS) {
        const uint64_t before_us = AP_HAL::micros64();
        hal.gpio->write(GPS_EXTINT, 1);
        const uint64_t after_us = AP_HAL::micros64();
        timesync.last_strobe_ms = now_ms;
        timesync.strobe_high = true;
        timesync.last_edge_us = before_us;
        timesync.last_edge_paired = false;
        timesync.strobe_count++;
        logger.Write("EXTS", "TimeUS,Count,WriteUS", "QIH",
                     before_us, timesync.strobe_count, uint16_t(MIN(after_us - before_us, 65535U)));
    } else if (timesync.strobe_high && now_ms - timesync.last_strobe_ms >= STROBE_HIGH_MS) {
        hal.gpio->write(GPS_EXTINT, 0);
        timesync.strobe_high = false;
    }

    // timepulse edges captured in the ISR
    const uint32_t pps_count = timesync.pps_count;
    if (pps_count != timesync.pps_count_logged) {
        timesync.pps_count_logged = pps_count;
        const uint64_t now_us = AP_HAL::micros64();
        const uint64_t edge_us = now_us - uint32_t(uint32_t(now_us) - timesync.pps_edge_us32);
        logger.Write("PPSE", "TimeUS,Count", "QI", edge_us, pps_count);
    }
}

void AP_Periph_FW::timesync_on_frame(uint8_t msg_class, uint8_t msg_id, const uint8_t *payload, uint16_t payload_len, uint64_t arrival_us)
{
    if (msg_class == 0x0D && msg_id == 0x03 && payload_len == 28) {
        // UBX-TIM-TM2: pair with the latest strobe edge if it arrived within the window
        if (timesync.last_edge_paired || timesync.last_edge_us == 0 || arrival_us - timesync.last_edge_us > TM2_PAIR_WINDOW_US) {
            return;
        }
        uint16_t week;
        uint32_t tow_ms, tow_sub_ns;
        memcpy(&week, &payload[4], 2);          // wnR
        memcpy(&tow_ms, &payload[8], 4);        // towMsR
        memcpy(&tow_sub_ns, &payload[12], 4);   // towSubMsR (ns)
        auto &pair = timesync.pairs[timesync.pair_next];
        pair.mcu_us = timesync.last_edge_us;
        pair.week = week;
        pair.tow_s = tow_ms * 1e-3 + tow_sub_ns * 1e-9;
        timesync.pair_next = (timesync.pair_next + 1) % ARRAY_SIZE(timesync.pairs);
        if (timesync.pair_count < ARRAY_SIZE(timesync.pairs)) {
            timesync.pair_count++;
        }
        timesync.last_edge_paired = true;
        return;
    }

    if (msg_class == 0x02 && msg_id == 0x15 && payload_len >= 16) {
        // UBX-RXM-RAWX header: rcvTow R8, week U2, leapS I1, numMeas U1
        double rcv_tow;
        uint16_t week;
        int8_t leap_s;
        memcpy(&rcv_tow, &payload[0], 8);
        memcpy(&week, &payload[8], 2);
        memcpy(&leap_s, &payload[10], 1);
        const uint8_t num_meas = payload[11];
        logger.Write("GEPO", "TimeUS,RcvTow,Week,LeapS,NumMeas,StrobeFit", "QdHbBQ",
                     arrival_us, rcv_tow, week, leap_s, num_meas, timesync_predict_mcu_us(week, rcv_tow));
    }
}

uint64_t AP_Periph_FW::timesync_predict_mcu_us(uint16_t week, double tow_s) const
{
    if (timesync.pair_count < 2) {
        return 0;
    }
    const uint8_t n = ARRAY_SIZE(timesync.pairs);
    const auto &newest = timesync.pairs[(timesync.pair_next + n - 1) % n];
    const auto &oldest = timesync.pairs[(timesync.pair_next + n - timesync.pair_count) % n];
    const double gps_span_s = (int32_t(newest.week) - int32_t(oldest.week)) * WEEK_S + (newest.tow_s - oldest.tow_s);
    const double mcu_span_s = (newest.mcu_us - oldest.mcu_us) * 1e-6;
    if (gps_span_s <= 0 || mcu_span_s <= 0) {
        return 0;
    }
    const double gps_per_mcu = gps_span_s / mcu_span_s;
    const double gps_since_newest_s = (int32_t(week) - int32_t(newest.week)) * WEEK_S + (tow_s - newest.tow_s);
    return uint64_t(int64_t(newest.mcu_us) + int64_t(gps_since_newest_s / gps_per_mcu * 1e6));
}

#endif // AP_PERIPH_LOGGER
