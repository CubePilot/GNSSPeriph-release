/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "AP_Periph.h"
#include <AP_GPS/AP_GPS.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <stdio.h>
#include <AP_GPS/AP_GPS_UBLOX.h>

#ifdef ENABLE_BASE_MODE

#if 0
#define Debug(fmt, args...) can_printf(fmt, ##args)
#else
#define Debug(fmt, args...)
#endif

#define LOCK_ID 0x4c4f434b // "LOCK"
#define UBX_PREAMBLE1 0xb5
#define UBX_PREAMBLE2 0x62 

#define CLASS_ACK 0x05
#define CLASS_RXM 0x02
#define CLASS_CFG 0x06
#define CLASS_MON 0x0a
#define CLASS_NAV 0x01

#define MSG_ACK_NAK 0x00
#define MSG_ACK_ACK 0x01

#define MSG_CFG_MSG          0x01
#define MSG_CFG_RATE         0x08
#define MSG_CFG_CFG          0x09
#define MSG_CFG_NAV_SETTINGS 0x24
#define MSG_CFG_PRT          0x00
#define MSG_CFG_TMODE3       0x71

#define MSG_RXM_RAWX 0x15
#define MSG_RXM_SFRBX 0x13

#define MSG_MON_VER 0x04

#define MSG_NAV_SVIN 0x3b
#define MSG_NAV_PVT 0x07

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo GPS_Base::var_info[] = {
    // @Param: ENABLE
    // @DisplayName: Enable GPS Base Mode
    // @Description: Enable GPS Base Mode
    // @User: Standard
    AP_GROUPINFO_FLAGS("_ENABLE",  1, GPS_Base, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: LOGGING
    // @DisplayName: Enable Logging
    // @Description: Enable Logging
    // @User: Standard
    AP_GROUPINFO("_LOGGING",  2, GPS_Base, _logging, 1),

    // @Param: S_IN_ENABLE
    // @DisplayName: Enable Survey in
    // @Description: Enable Survey in
    // @User: Standard
    AP_GROUPINFO("_S_IN_EN",  3, GPS_Base, _s_in_enabled, 1),

    // @Param: S_IN_TIME
    // @DisplayName: Time to wait for Survey in to be asserted
    // @Description: Time to wait for Survey in to be asserted
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("_S_IN_TIME",  4, GPS_Base, _s_in_time, 20.0),

    // @Param: S_IN_ACC
    // @DisplayName: Accuracy to wait for Survey in to be asserted
    // @Description: Accuracy to wait for Survey in to be asserted
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("_S_IN_ACC",  5, GPS_Base, _s_in_acc, 2.0),

    // @Param: S_IN_LAT
    // @DisplayName: Latitude to wait for Survey in to be asserted
    // @Description: Latitude to wait for Survey in to be asserted
    // @Range: -90 90
    // @User: Standard
    AP_GROUPINFO("_S_IN_LAT",  6, GPS_Base, _s_in_lat, 0.0),

    // @Param: S_IN_LON
    // @DisplayName: Longitude to wait for Survey in to be asserted
    // @Description: Longitude to wait for Survey in to be asserted
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("_S_IN_LON",  7, GPS_Base, _s_in_lon, 0.0),

    // @Param: S_IN_ALT
    // @DisplayName: Altitude to wait for Survey in to be asserted
    // @Description: Altitude to wait for Survey in to be asserted
    // @Range: -1000 1000
    // @User: Standard
    AP_GROUPINFO("_S_IN_ALT",  8, GPS_Base, _s_in_alt, -1000.0),

    AP_GROUPEND
};

GPS_Base::GPS_Base() {
    // setup parameters
    AP_Param::setup_object_defaults(this, var_info);
}

// convert week number and time of week to date/time
void GPS_Base::gps_week_time(const uint16_t week, const uint32_t tow)
{
    // days since 1st epoch (6th Jan 1980)
    uint32_t days = (week * 7) + (tow / 86400000) + 5;

    uint32_t ms = tow % 86400000;
    uint32_t s = ms / 1000;
    ms = ms % 1000;
    uint32_t m = s / 60;
    s = s % 60;
    uint32_t h = m / 60;
    m = m % 60;
    uint32_t y = 1980;
    uint32_t d = 0;

    while (days >= 365) {
        if ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) {
            if (days >= 366) {
                days -= 366;
                y++;
            }
        } else {
            days -= 365;
            y++;
        }
    }
    dt.year = y;
    uint8_t month_days[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if ((y % 4 == 0 && y % 100 != 0) || y % 400 == 0) {
        month_days[1] = 29;
    }
    while (days >= month_days[d]) {
        days -= month_days[d];
        d++;
    }
    dt.utc_sec = (UNIX_OFFSET_MSEC + (week * AP_MSEC_PER_WEEK) + tow)/1000;
    dt.month = d + 1;
    dt.day = days + 1;
    dt.hour = h;
    dt.minute = m;
    dt.second = s;
}

void GPS_Base::parse_runtime_ubx(uint8_t byte) {
    if (parse_ubx(byte)) {
        // handle RAWX message
        if (_class == CLASS_RXM && _msg_id == MSG_RXM_RAWX) {
            if (_buffer.raw_rawx.week > 0 && _buffer.raw_rawx.rcvTow >= 0) {
                gps_week_time((uint16_t)_buffer.raw_rawx.week, (uint32_t)(_buffer.raw_rawx.rcvTow * 1000));
                can_printf("GPS: %d-%02d-%02d %02d:%02d:%02d\n", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
            }
        } else if (_class == CLASS_NAV && _msg_id == MSG_NAV_SVIN) {
            can_printf("GPS: Survey in status: %s Active:%d Acc:%fm\n", _buffer.nav_svin.valid ? "Valid":"Invalid", _buffer.nav_svin.active, _buffer.nav_svin.meanAcc/10000.0);
            if (_buffer.nav_svin.valid) {
                can_printf("GPS: Survey in complete\n");
            }
        }
    }
}

void GPS_Base::_update_checksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b)
{
    while (len--) {
        ck_a += *data;
        ck_b += ck_a;
        data++;
    }
}

bool GPS_Base::_send_message(uint8_t msg_class, uint8_t msg_id, const void *msg, uint16_t size)
{
    if (gps_uart->txspace() < (sizeof(struct ubx_header) + 2 + size)) {
        return false;
    }
    // Debug("GPS: sending message %d %d\n", msg_class, msg_id);
    struct ubx_header header;
    uint8_t ck_a=0, ck_b=0;
    header.preamble1 = UBX_PREAMBLE1;
    header.preamble2 = UBX_PREAMBLE2;
    header.msg_class = msg_class;
    header.msg_id    = msg_id;
    header.length    = size;

    _update_checksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
    _update_checksum((uint8_t *)msg, size, ck_a, ck_b);

    gps_uart->write_locked((const uint8_t *)&header, sizeof(header), LOCK_ID);
    gps_uart->write_locked((const uint8_t *)msg, size, LOCK_ID);
    gps_uart->write_locked((const uint8_t *)&ck_a, 1, LOCK_ID);
    gps_uart->write_locked((const uint8_t *)&ck_b, 1, LOCK_ID);
    return true;
}

bool GPS_Base::parse_ubx(uint8_t data) {
reset:
    switch(_step) {

        // Message preamble detection
        //
        // If we fail to match any of the expected bytes, we reset
        // the state machine and re-consider the failed byte as
        // the first byte of the preamble.  This improves our
        // chances of recovering from a mismatch and makes it less
        // likely that we will be fooled by the preamble appearing
        // as data in some other message.
        //
        case 1:
            if (UBX_PREAMBLE2 == data) {
                _step++;
                break;
            }
            _step = 0;
            FALLTHROUGH;
        case 0:
            if(UBX_PREAMBLE1 == data)
                _step++;
            break;

        // Message header processing
        //
        // We sniff the class and message ID to decide whether we
        // are going to gather the message bytes or just discard
        // them.
        //
        // We always collect the length so that we can avoid being
        // fooled by preamble bytes in messages.
        //
        case 2:
            _step++;
            _class = data;
            _ck_b = _ck_a = data;                       // reset the checksum accumulators
            break;
        case 3:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _msg_id = data;
            break;
        case 4:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length = data;                     // payload length low byte
            break;
        case 5:
            _step++;
            _ck_b += (_ck_a += data);                   // checksum byte
            _payload_length += (uint16_t)(data<<8);
            if (_payload_length > sizeof(_buffer)) {
                // assume any payload bigger then what we know about is noise
                _payload_length = 0;
                _step = 0;
                goto reset;
            }
            _payload_counter = 0;                       // prepare to receive payload
            if (_payload_length == 0) {
                // bypass payload and go straight to checksum
                _step++;
            }
            break;

        // Receive message data
        //
        case 6:
            _ck_b += (_ck_a += data);                   // checksum byte
            if (_payload_counter < sizeof(_buffer)) {
                _buffer[_payload_counter] = data;
            }
            if (++_payload_counter == _payload_length)
                _step++;
            break;

        // Checksum and message processing
        //
        case 7:
            _step++;
            if (_ck_a != data) {
                Debug("bad cka %x should be %x", data, _ck_a);
                _step = 0;
                goto reset;
            }
            break;
        case 8:
            _step = 0;
            if (_ck_b != data) {
                Debug("bad ckb %x should be %x", data, _ck_b);
                break;                                                  // bad checksum
            }
            return true;
    }
    return false;
}

void GPS_Base::handle_ubx_msg()
{
    Debug("GPS: got message 0x%x 0x%x", _class, _msg_id);
    switch (_class) {
        case CLASS_ACK:
            if (_msg_id == MSG_ACK_ACK) {
                if (ubx_config_state == SETTING_SAVE_CONFIG) {
                    if (_buffer.ack_ack.msg_class == CLASS_CFG &&
                        _buffer.ack_ack.msg_id == MSG_CFG_CFG) {
                        // move to next state
                        can_printf("GPS_Base: config saved");
                        ubx_config_state++;
                    }
                } else if (ubx_config_state == SETTING_SURVEYIN_CONFIG) {
                    if (_buffer.ack_ack.msg_class == CLASS_CFG &&
                        _buffer.ack_ack.msg_id == MSG_CFG_TMODE3) {
                        ubx_config_state++;
                    }
                }
            }
            break;
        case CLASS_MON:
            if (_msg_id == MSG_MON_VER) {
                can_printf("GPS_Base: SW Version: %s", _buffer.mon_ver.swVersion);
                can_printf("GPS_Base: HW Version: %s", _buffer.mon_ver.hwVersion);
                if (ubx_config_state == WAITING_FOR_VERSION) {
                    // move to next state
                    ubx_config_state++;
                }
            }
            break;
        case CLASS_CFG:
            switch (_msg_id) {
                case MSG_CFG_PRT:
                    if (ubx_config_state == GETTING_PORT_INDEX &&
                        (_buffer.cfg_prt.portID < ARRAY_SIZE(ubx_cfg_msg_rate_6::rates)) &&
                        (_buffer.cfg_prt.outProtoMask & ((1U << 5) | (1U << 0)))) {
                        Debug("Port ID: %d", _buffer.cfg_prt.portID);
                        _ublox_port = _buffer.cfg_prt.portID;
                        ubx_config_state++;
                    } else if (ubx_config_state == GETTING_PORT_INDEX) {
                        _buffer.cfg_prt.outProtoMask = ((1U << 5) | (1U << 0));
                        _send_message(CLASS_CFG, MSG_CFG_PRT, (void*)&_buffer.cfg_prt, sizeof(ubx_cfg_prt));
                    }
                    break;
                case MSG_CFG_RATE:
                    if (ubx_config_state == SETTING_NAV_RATE) {
                        // check that the rate has been set
                        if (_buffer.cfg_nav_rate.measure_rate_ms == 1000 &&
                            _buffer.cfg_nav_rate.nav_rate == 1 &&
                            _buffer.cfg_nav_rate.timeref == 0) {
                            // move to next state
                            ubx_config_state++;
                        } else {
                            Debug("NAV RATE Incorrect");
                            _update_setting = true;
                        }
                    }
                    break;
                case MSG_CFG_MSG:
                    if (_buffer.cfg_msg_rate_6.msg_class == curr_msg.msg_class &&
                        _buffer.cfg_msg_rate_6.msg_id == curr_msg.msg_id &&
                        _payload_length == sizeof(ubx_cfg_msg_rate_6)) {
                        // check that the rate has been set
                        if (_buffer.cfg_msg_rate_6.rates[_ublox_port] == curr_msg.rate) {
                            // move to next state
                            Debug("MSG(0x%x, 0x%x) Set RATE: %d", curr_msg.msg_class, curr_msg.msg_id, _buffer.cfg_msg_rate_6.rates[_ublox_port]);
                            ubx_config_state++;
                        } else {
                            Debug("MSG(0x%x, 0x%x) Incorrect RATE: %d", curr_msg.msg_class, curr_msg.msg_id, _buffer.cfg_msg_rate_6.rates[_ublox_port]);
                            _update_setting = true;
                        }
                    }
                    break;
                default:
                    break;
            }
        default:
            break;
    }
}

bool GPS_Base::configure_message_rate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
    if (!_update_setting) {
        struct ubx_cfg_msg msg;
        msg.msg_class = msg_class;
        msg.msg_id = msg_id;
        curr_msg.msg_class = msg_class;
        curr_msg.msg_id = msg_id;
        curr_msg.rate = rate;
        return _send_message(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
    } else {
        curr_msg.msg_class = msg_class;
        curr_msg.msg_id = msg_id;
        curr_msg.rate = rate;
        Debug("MSG(0x%x, 0x%x) Setting RATE: %d", curr_msg.msg_class, curr_msg.msg_id, curr_msg.rate);
        _update_setting = false;
        return _send_message(CLASS_CFG, MSG_CFG_MSG, &curr_msg, sizeof(curr_msg));
    }
}

static uint32_t baudrates[] = { 9600, 19200, 38400, 57600, 115200, 230400, 921600 };
uint8_t baudrate_index = 0;
void GPS_Base::do_configurations()
{
    // run at 50Hz
    if (AP_HAL::millis() - _last_config_ms < 20) {
        return;
    }
    // process bytes from GPS
    while (gps_uart->available_locked(LOCK_ID) > 0) {
        uint8_t c;
        if (gps_uart->read_locked(&c, 1, LOCK_ID)) {
            if (parse_ubx(c)) {
                handle_ubx_msg();
            }
        } else {
            break;
        }
    }
    _last_config_ms = AP_HAL::millis();
    if (ubx_config_state == WAITING_FOR_VERSION) {
        Debug("Trying baudrate %ld", baudrates[baudrate_index]);
        // try next baudrate
        gps_uart->begin_locked(baudrates[baudrate_index], 0,0, LOCK_ID);
        baudrate_index++;
        baudrate_index %= ARRAY_SIZE(baudrates);
        ubx_config_state = SETTING_BAUD;
    }
    switch (ubx_config_state) {
        case SETTING_BAUD: {
            // send the startup blob
            gps_uart->write_locked((const uint8_t*)UBLOX_SET_BINARY_460800, sizeof(UBLOX_SET_BINARY_460800), LOCK_ID);
            ubx_config_state++;
            break;
        }
        case CHECKING_VERSION:
            // set baud rate to 460800
            Debug("Checking version");
            gps_uart->begin_locked(460800, 0, 0, LOCK_ID);
            _send_message(CLASS_MON, MSG_MON_VER, NULL, 0);
            ubx_config_state++;
            break;
        case WAITING_FOR_VERSION:
            Debug("Waiting for version");
            break;
        case GETTING_PORT_INDEX:
            Debug("Getting port index");
            _send_message(CLASS_CFG, MSG_CFG_PRT, NULL, 0);
            break;
        case SETTING_NAV_RATE:
            Debug("Setting nav rate");
            struct ubx_cfg_nav_rate msg;
            msg.measure_rate_ms = 1000; //1Hz
            msg.nav_rate = 1;
            msg.timeref = 0;
            if (!_update_setting) {
                _send_message(CLASS_CFG, MSG_CFG_RATE, nullptr, 0);
            } else {
                _send_message(CLASS_CFG, MSG_CFG_RATE, (uint8_t*)&msg, sizeof(msg));
                _update_setting = false;    
            }
            break;
        case SETTING_SURVEY_IN_RATE:
            Debug("Setting survey in rate");
            configure_message_rate(CLASS_NAV, MSG_NAV_SVIN, 1);
            break;
        case SETTING_PVT_RATE:
            Debug("Setting PVT rate");
            configure_message_rate(CLASS_NAV, MSG_NAV_PVT, 1);
            break;
        case SETTING_1005_RATE:
            Debug("Setting 1005 rate");
            configure_message_rate(0xf5, 0x05, 1);
            break;
        case SETTING_1074_RATE:
            Debug("Setting 1074 rate");
            configure_message_rate(0xf5, 0x4a, 1);
            break;
        case SETTING_1084_RATE:
            Debug("Setting 1084 rate");
            configure_message_rate(0xf5, 0x54, 1);
            break;
        case SETTING_1094_RATE:
            Debug("Setting 1094 rate");
            configure_message_rate(0xf5, 0x5e, 1);
            break;
        case SETTING_1124_RATE:
            Debug("Setting 1124 rate");
            configure_message_rate(0xf5, 0x7c, 1);
            break;
        case SETTING_1230_RATE:
            Debug("Setting 1230 rate");
            configure_message_rate(0xf5, 0xe6, 1);
            break;
        case SETTING_RXM_RAWX:
            Debug("Setting RXM_RAWX rate");
            configure_message_rate(CLASS_RXM, MSG_RXM_RAWX, 1);
            break;
        case SETTING_RXM_SFRBX:
            Debug("Setting RXM_SFRBX rate");
            configure_message_rate(CLASS_RXM, MSG_RXM_SFRBX, 1);
            break;
        case SETTING_SURVEYIN_CONFIG:
            if ((AP_HAL::millis() - _last_surveyin_config_ms) > 1000) {
                can_printf("GPS_Base: Setting survey in config");
                ubx_cfg_tmode3 surveyin_cfg {};
                if (_s_in_lat == 0.0 && _s_in_lon == 0.0 && _s_in_alt == -1000.0) {
                    surveyin_cfg.flags = 1;
                    surveyin_cfg.fixedPosAcc = 0;
                    surveyin_cfg.svinAccLimit = _s_in_acc*10000.0;
                } else {
                    surveyin_cfg.flags = 256 + 2; // LLA
                    surveyin_cfg.ecefXOrLat = _s_in_lat*1e7;
                    surveyin_cfg.ecefXOrLatHP = (_s_in_lat*1e7 - surveyin_cfg.ecefXOrLat)*100.0;
                    surveyin_cfg.ecefYOrLon = _s_in_lon*1e7;
                    surveyin_cfg.ecefYOrLonHP = (_s_in_lon*1e7 - surveyin_cfg.ecefYOrLon)*100.0;
                    surveyin_cfg.ecefZOrAlt = _s_in_alt*100;
                    surveyin_cfg.ecefZOrAltHP = (_s_in_alt*100 - surveyin_cfg.ecefZOrAlt)*100.0;
                    surveyin_cfg.fixedPosAcc = 1;
                    surveyin_cfg.svinAccLimit = 2000;
                }
                surveyin_cfg.svinMinDur = _s_in_time;
                _last_surveyin_config_ms = AP_HAL::millis();
                _send_message(CLASS_CFG, MSG_CFG_TMODE3, &surveyin_cfg, sizeof(surveyin_cfg));
            }
            break;
        case SETTING_SAVE_CONFIG:
            if ((AP_HAL::millis() - _last_save_config_ms) > 1000) {
                can_printf("GPS_Base: Saving config");
                static const ubx_cfg_cfg save_cfg {
                    clearMask: 0,
                    saveMask: SAVE_CFG_ALL,
                    loadMask: 0
                };
                _last_save_config_ms = AP_HAL::millis();
                _send_message(CLASS_CFG, MSG_CFG_CFG, &save_cfg, sizeof(save_cfg));
            }
            break;
        case SETTING_FINISHED:
            can_printf("GPS_Base: Configuration Finished");
            _ppk_config_finished = true;
            break;
    };
}


void GPS_Base::update() {
    // get GPS port from serial_manager
    if (gps_uart == NULL) {
        gps_uart = periph.serial_manager.find_serial(AP_SerialManager::SerialProtocol_GPS, 0);
    }
    if (gcs_uart == NULL) {
        gcs_uart = hal.serial(0);
    }

    if (!_enabled || connected_to_gcs) {
        return;
    }

    // lock the gcs and gps ports
    gps_uart->lock_port(LOCK_ID, LOCK_ID);
    if (_s_in_enabled && !_ppk_config_finished) {
        do_configurations();
        return;
    }

    gcs_uart->lock_port(LOCK_ID, LOCK_ID);
    uint8_t byte, last_byte = 0;

    if (_s_in_enabled && (gps_uart->get_baud_rate() != 460800)) {
        // if we are doing survey in, we need to be at 460800 baud
        can_printf("GPS_Base: Setting baud rate to 460800");
        gps_uart->end();
        gps_uart->begin_locked(460800, 0, 0, LOCK_ID);
    } else if ((gcs_uart->get_usb_baud() != gps_uart->get_baud_rate()) && !_s_in_enabled) {
        gps_uart->end();
        gps_uart->begin_locked(gcs_uart->get_usb_baud(), 0, 0, LOCK_ID);
    }

    // read bytes from the GPS port and push them to the GCS port
    while (gps_uart->read_locked(&byte, 1, LOCK_ID) == 1) {
        if (gps_num_bytes_to_rx == 0 && ((byte == UBX_PREAMBLE2) && (last_byte == UBX_PREAMBLE1))) {
            gps_received_preamble = true;
            gps_length_counter = 5;
            gps_num_bytes_to_rx = 6;
        }
        if (gps_length_counter != 0) {
            gps_length_counter--;
            if (gps_length_counter == 0) {
                gps_num_bytes_to_rx = ((byte<<8) + last_byte) + 2;
            }
        }
        if (gps_buffer.write(&byte, 1)) {
            parse_runtime_ubx(byte);
            if (rtcm3_parser.read(byte)) {
                can_printf("GPS_Base: RTCM3: %d", rtcm3_parser.get_id());
            }
        }

        if ((gps_received_preamble && gps_num_bytes_to_rx == 0) || gps_buffer.space() == 0) {
            if (ubx_log_fd == -1 && dt.year >= 2023 && _logging.get()) {
                // open a log file with new date/time
                // check if ppk directory exists
                int ret = 0;
                struct stat st;
                ret = AP::FS().stat("ppk", &st);
                if (ret == -1) {
                    ret = AP::FS().mkdir("ppk");
                }
                if (ret == -1) {
                    can_printf("Failed to create ppk directory\n");
                } else {
                    snprintf(_ubx_log_filename, sizeof(_ubx_log_filename), "/ppk/UTC_%04d_%02d_%02d_%02d_%02d_%02d.ubx", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
                    ubx_log_fd = AP::FS().open(_ubx_log_filename, O_CREAT | O_WRONLY | O_TRUNC);
                    if (ubx_log_fd == -1) {
                        can_printf("Failed to open log file\n");
                    } else {
                        can_printf("Opened log file %s\n", _ubx_log_filename);
                    }
                    can_printf("Setting File Time: %lu", dt.utc_sec);
                }
            }
            // if we have proper time start a log file, if not done already
            // push all the bytes we have so far to the GCS port
            struct ByteBuffer::IoVec vecs[2];
            uint8_t num_iovecs = gps_buffer.peekiovec(vecs, gps_buffer.available());
            for (uint8_t i=0; i<num_iovecs; i++) {
                gcs_uart->write_locked(vecs[i].data, vecs[i].len, LOCK_ID);
                if (ubx_log_fd != -1) {
                    AP::FS().write(ubx_log_fd, vecs[i].data, vecs[i].len);
                    AP::FS().fsync(ubx_log_fd);
                    if (!AP::FS().set_mtime(_ubx_log_filename, dt.utc_sec)) {
                        can_printf("Failed to set file time %s\n", strerror(errno));
                    }
                }
                gps_buffer.advance(vecs[i].len);
            }
        }
        if (gps_num_bytes_to_rx != 0) {
            gps_num_bytes_to_rx--;
        } else {
            gps_received_preamble = false;
        }
        last_byte = byte;
    }

    // read bytes from the GCS port and push them to the GPS port
    while (gcs_uart->read_locked(&byte, 1, LOCK_ID) == 1) {
        periph.check_for_serial_reboot_cmd_byte(byte);
        connected_to_gcs = periph.mavlink.process_byte(byte);
        if (connected_to_gcs) {
            // we are connected to the GCS, so stop sending bytes to the GPS
            gcs_uart->lock_port(0, 0);
            break;
        }
        if (gcs_num_bytes_to_rx == 0 && ((byte == UBX_PREAMBLE2) && (last_byte == UBX_PREAMBLE1))) {
            gcs_received_preamble = true;
            gcs_length_counter = 5;
            gcs_num_bytes_to_rx = 6;
        }
        if (gcs_length_counter != 0) {
            gcs_length_counter--;
            if (gcs_length_counter == 0) {
                gcs_num_bytes_to_rx = ((byte<<8) + last_byte) + 2;
            }
        }
        gcs_buffer.write(&byte, 1);
        if ((gcs_received_preamble && gcs_num_bytes_to_rx == 0) || gcs_buffer.space() == 0) {
            // push all the bytes we have so far to the GPS port
            struct ByteBuffer::IoVec vecs[2];
            uint8_t num_iovecs = gcs_buffer.peekiovec(vecs, gcs_buffer.available());
            for (uint8_t i=0; i<num_iovecs; i++) {
                gps_uart->write_locked(vecs[i].data, vecs[i].len, LOCK_ID);
                gcs_buffer.advance(vecs[i].len);
            }
        }
        if (gcs_num_bytes_to_rx != 0) {
            gcs_num_bytes_to_rx--;
        } else {
            gcs_received_preamble = false;
        }
        last_byte = byte;
    }
}

static MAV_PARAM_TYPE mav_param_type(enum ap_var_type t)
{
    if (t == AP_PARAM_INT8) {
	    return MAV_PARAM_TYPE_INT8;
    }
    if (t == AP_PARAM_INT16) {
	    return MAV_PARAM_TYPE_INT16;
    }
    if (t == AP_PARAM_INT32) {
	    return MAV_PARAM_TYPE_INT32;
    }
    // treat any others as float
    return MAV_PARAM_TYPE_REAL32;
}

void GPS_Base::handle_param_request_list(const mavlink_message_t &msg)
{
    // unlock the gcs_port
    gcs_uart->lock_port(0,0);
    mavlink_param_request_list_t packet;
    mavlink_msg_param_request_list_decode(&msg, &packet);
    char key[AP_MAX_NAME_SIZE+1] = "B";
    if (_enabled) {
        uint8_t index = 0;
        // send parameter list
        for (auto var : var_info) {
            ap_var_type var_type;
            key[1] = '\0';
            strcat(key, var.name);
            AP_Param *vp = AP_Param::find(key, &var_type);
            if (vp == nullptr) {
                continue;
            }
            mavlink_msg_param_value_send(periph.mavlink.get_channel(),
                                        key,
                                        vp->cast_to_float(var_type),
                                        mav_param_type(var_type),
                                        ARRAY_SIZE(var_info) - 1,
                                        index++);
        }
    } else {
        // just send enable
        strcat(key, "_ENABLE");
        mavlink_msg_param_value_send(periph.mavlink.get_channel(),
                                    key,
                                    (float)_enabled.get(),
                                    MAV_PARAM_TYPE_INT8,
                                    1,
                                    0);
    }
}

void GPS_Base::handle_param_set(const mavlink_message_t &msg)
{
    mavlink_param_set_t packet;
    mavlink_msg_param_set_decode(&msg, &packet);
    enum ap_var_type var_type;
    gcs_uart->lock_port(0,0);

    // set parameter
    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    // we only allow parameter sets for BASE, check starts with B_
    if (strncmp(key, "B_", 2) != 0) {
        return;
    }

    // find existing param so we can get the old value
    uint16_t parameter_flags = 0;
    vp = AP_Param::find(key, &var_type, &parameter_flags);
    if (vp == nullptr || isnan(packet.param_value) || isinf(packet.param_value)) {
        return;
    }
    float old_value = vp->cast_to_float(var_type);

    // set the value
    vp->set_float(packet.param_value, var_type);

    /*
      we force the save if the value is not equal to the old
      value. This copes with the use of override values in
      constructors, such as PID elements. Otherwise a set to the
      default value which differs from the constructor value doesn't
      save the change
     */
    bool force_save = !is_equal(packet.param_value, old_value);

    // save the change
    vp->save(force_save);

    if (force_save && (parameter_flags & AP_PARAM_FLAG_ENABLE)) {
        AP_Param::invalidate_count();
    }

    // index of the key
    uint8_t index = 0;
    for (auto var : var_info) {
        if (strcmp(var.name, key+2) == 0) {
            break;
        }
        index++;
    }

    
    // send back the new value
    mavlink_msg_param_value_send(periph.mavlink.get_channel(),
                                 key,
                                 vp->cast_to_float(var_type),
                                 mav_param_type(var_type),
                                 ARRAY_SIZE(var_info),
                                 index);
}

void GPS_Base::handle_param_request_read(const mavlink_message_t &msg)
{
    mavlink_param_request_read_t packet;
    mavlink_msg_param_request_read_decode(&msg, &packet);
    enum ap_var_type var_type;
    gcs_uart->lock_port(0,0);

    AP_Param *vp;
    char key[AP_MAX_NAME_SIZE+1];
    strncpy(key, (char *)packet.param_id, AP_MAX_NAME_SIZE);
    key[AP_MAX_NAME_SIZE] = 0;

    // we only allow parameter sets for BASE, check starts with B_
    if (strncmp(key, "B_", 2) != 0) {
        return;
    }
    uint16_t parameter_flags = 0;
    vp = AP_Param::find(key, &var_type, &parameter_flags);

    if (vp == nullptr) {
        return;
    }
    float value = vp->cast_to_float(var_type);
    // send parameter value
    mavlink_msg_param_value_send(
        periph.mavlink.get_channel(),
        key,
        value,
        mav_param_type(var_type),
        AP_Param::count_parameters(),
        -1);
}

#endif
