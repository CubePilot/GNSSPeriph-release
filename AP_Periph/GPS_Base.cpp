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
#include <AP_SerialLED/AP_SerialLED.h>
#include <AP_HAL_ChibiOS/sdcard.h>

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
#define MSG_CFG_RST          0x04

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
    AP_GROUPINFO("_S_IN_ALT",  8, GPS_Base, _s_in_alt, 0.0),

    AP_GROUPEND
};

GPS_Base::GPS_Base() {
    // setup parameters
    AP_Param::setup_object_defaults(this, var_info);
}

void GPS_Base::prepare_ubx_base_cfg() {
    if (!_enabled) {
        return;
    }

#define UBX_CFG_PUSH(config_key, value) \
    ppk_config.push<AP::UBXConfigKey::config_key>(value);

    // Enable RTCM3X output on UART1
    UBX_CFG_PUSH(CFG_UART1OUTPROT_RTCM3X, (uint8_t)1);

    // Enable necessary messages on UART1
    UBX_CFG_PUSH(CFG_MSGOUT_UBX_NAV_SVIN_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_UBX_NAV_PVT_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_RTCM_3X_TYPE1005_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_RTCM_3X_TYPE1074_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_RTCM_3X_TYPE1084_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_RTCM_3X_TYPE1094_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_RTCM_3X_TYPE1124_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_UBX_RXM_RAWX_UART1, (uint8_t)1);
    UBX_CFG_PUSH(CFG_MSGOUT_UBX_RXM_SFRBX_UART1, (uint8_t)1);

    // Configure TMODE based on survey-in parameters
    if (int(_s_in_lat*1000) == 0 && int(_s_in_lon*1000) == 0 && int(_s_in_alt*1000) == 0) {
        // Survey-in mode
        UBX_CFG_PUSH(CFG_TMODE_MODE, (uint8_t)1);
        UBX_CFG_PUSH(CFG_TMODE_SVIN_MIN_DUR, uint32_t(ceil(_s_in_time)));
        UBX_CFG_PUSH(CFG_TMODE_SVIN_ACC_LIMIT, uint32_t(_s_in_acc * 10000));
    } else {
        // Fixed position mode
        UBX_CFG_PUSH(CFG_TMODE_MODE, (uint8_t)2);
        UBX_CFG_PUSH(CFG_TMODE_POS_TYPE, (uint8_t)1);
        UBX_CFG_PUSH(CFG_TMODE_FIXED_POS_ACC, (uint32_t)2000);
        UBX_CFG_PUSH(CFG_TMODE_LAT, (uint32_t)(_s_in_lat * 1e7));
        UBX_CFG_PUSH(CFG_TMODE_LAT_HP, (uint8_t)((_s_in_lat * 1e7 - int32_t(_s_in_lat * 1e7)) * 10));
        UBX_CFG_PUSH(CFG_TMODE_LON, (uint32_t)(_s_in_lon * 1e7));
        UBX_CFG_PUSH(CFG_TMODE_LON_HP, (uint8_t)((_s_in_lon * 1e7 - int32_t(_s_in_lon * 1e7)) * 10));
        UBX_CFG_PUSH(CFG_TMODE_HEIGHT, (uint32_t)(_s_in_alt * 100));
        UBX_CFG_PUSH(CFG_TMODE_HEIGHT_HP, (uint8_t)((_s_in_alt * 100 - int32_t(_s_in_alt * 100)) * 10));
    }

#undef UBX_CFG_PUSH
    // Set override config for GPS instance 0
    AP_GPS_UBLOX_CFGv2::override_ubx_cfg(0, ppk_config_data, ppk_config.get_size());
}

// convert week number and time of week to date/time
void GPS_Base::gps_week_time(struct date_time &dt, const uint16_t week, const uint32_t tow)
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
        // handle RAWX message for timestamp extraction
        if (_class == CLASS_RXM && _msg_id == MSG_RXM_RAWX) {
            if (_buffer.raw_rawx.week > 0 && _buffer.raw_rawx.rcvTow >= 0) {
                gps_week_time(dt, (uint16_t)_buffer.raw_rawx.week, (uint32_t)(_buffer.raw_rawx.rcvTow * 1000));
                can_printf("GPS: %d-%02d-%02d %02d:%02d:%02d\n", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
            }
        } else if (_class == CLASS_NAV && _msg_id == MSG_NAV_SVIN) {
            // handle survey-in status for LED feedback
            can_printf("GPS: Survey in status: %s Active:%d Acc:%fm\n", _buffer.nav_svin.valid ? "Valid":"Invalid", _buffer.nav_svin.active, _buffer.nav_svin.meanAcc/10000.0);
            if (_buffer.nav_svin.valid) {
                can_printf("GPS: Survey in complete\n");
            }
            if (_start_mean_acc == 0 && _buffer.nav_svin.active && (_buffer.nav_svin.meanAcc < (50*10000))) {
                _start_mean_acc = _buffer.nav_svin.meanAcc;
            }
            memcpy(&curr_svin, &_buffer.nav_svin, sizeof(curr_svin));
        }
    }
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

void GPS_Base::update_leds()
{
    if (!_s_in_enabled) {
        // turn off all the leds
        periph.notify.handle_rgb(0, 0, 0);
        return;
    }
    auto serial_led = AP_SerialLED::get_singleton();
    bool no_lock = !_ppk_config_finished || _start_mean_acc == 0;
    const float brightness = (hal.gpio->usb_connected() ? LED_CONNECTED_BRIGHTNESS : periph.notify.get_rgb_led_brightness_percent()) * 0.01f;
    if (no_lock && !curr_svin.valid) {
        // set all the leds to yellow while configuring
        for (uint8_t i=0; i<periph.notify.get_led_len(); i++) {
            serial_led->set_RGB(SRV_LED_DATA_CHANNEL, i, 255*brightness, 255*brightness, 0);
        }
    } else {
        float progress = (_start_mean_acc - curr_svin.meanAcc)/(_start_mean_acc - (_s_in_acc*10000));
        if (progress < 0.0) {
            progress = 0.0;
        } else if (progress > 1.0) {
            progress = 1.0;
        }
        // set the number of leds to the progress
        uint8_t num_leds = (uint8_t)(progress * periph.notify.get_led_len());
        for (uint8_t i=0; i<periph.notify.get_led_len(); i++) {
            if (i < num_leds) {
                serial_led->set_RGB(SRV_LED_DATA_CHANNEL, i, 0, 255*brightness, 0);
            } else {
                serial_led->set_RGB(SRV_LED_DATA_CHANNEL, i, 0, 0, 0);
            }
        }
    }
    serial_led->send(SRV_LED_DATA_CHANNEL);
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

    // Close log file if USB host has mounted the disk to prevent deadlock
    // When host mounts disk, filesystem access blocks waiting for USB MSD
    // which causes watchdog timeout (IERR 0x800 302)
    if (sdcard_is_mounted_by_host() && ubx_log_fd != -1) {
        _logging.set(false);
        AP::FS().close(ubx_log_fd);
        ubx_log_fd = -1;
        can_printf("GPS_Base: Closed log file - USB disk mounted by host\n");
    }

    update_leds();

    // lock the gcs and gps ports
    if (_s_in_enabled && !_ppk_config_finished) {
        // Wait for GPS driver CFGv2 configuration to complete, then cold start
        auto *gps = AP_GPS::get_singleton();
        if (gps && gps->is_configured(0)) {
            // CFGv2 configuration complete, now do cold start to apply settings
            static bool cold_start_sent = false;
            static uint32_t cold_start_time_ms = 0;
            gps_uart->lock_port(LOCK_ID, LOCK_ID);
            if (!cold_start_sent) {

                // Send UBX-CFG-RST for cold start
                struct PACKED {
                    uint16_t navBbrMask;
                    uint8_t resetMode;
                    uint8_t reserved0;
                } cold_start_msg = {
                    0xFFFF,  // navBbrMask: clear all
                    0x02,    // resetMode: cold start (SW reset, GNSS only)
                    0x00     // reserved
                };

                struct PACKED {
                    uint8_t preamble1;
                    uint8_t preamble2;
                    uint8_t msg_class;
                    uint8_t msg_id;
                    uint16_t length;
                } header = {
                    0xb5, 0x62,  // UBX preamble
                    CLASS_CFG,   // CFG class
                    MSG_CFG_RST, // RST message
                    sizeof(cold_start_msg)
                };

                uint8_t ck_a = 0, ck_b = 0;
                // Calculate checksum
                const uint8_t* ptr = (uint8_t*)&header.msg_class;
                for (unsigned i = 0; i < sizeof(header) - 2; i++) {
                    ck_a += ptr[i];
                    ck_b += ck_a;
                }
                ptr = (uint8_t*)&cold_start_msg;
                for (unsigned i = 0; i < sizeof(cold_start_msg); i++) {
                    ck_a += ptr[i];
                    ck_b += ck_a;
                }

                gps_uart->write_locked((const uint8_t*)&header, sizeof(header), LOCK_ID);
                gps_uart->write_locked((const uint8_t*)&cold_start_msg, sizeof(cold_start_msg), LOCK_ID);
                gps_uart->write_locked(&ck_a, 1, LOCK_ID);
                gps_uart->write_locked(&ck_b, 1, LOCK_ID);

                cold_start_sent = true;
                cold_start_time_ms = AP_HAL::millis();
            } else if (AP_HAL::millis() - cold_start_time_ms > 2000) {
                // Wait 2 seconds after cold start for GPS to restart
                can_printf("GPS_Base: Cold start complete, configuration finished");
                _ppk_config_finished = true;
                cold_start_sent = false;  // Reset for next time
            }
        }
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
    char key[AP_MAX_NAME_SIZE+1] = "FORMAT_VERSION";
    uint8_t index = 0;
    ap_var_type var_type;
    // set format_version
    AP_Param *vp = AP_Param::find("FORMAT_VERSION", &var_type);
    if (vp == nullptr) {
        return;
    }
    mavlink_msg_param_value_send(periph.mavlink.get_channel(),
                                key,
                                vp->cast_to_float(var_type),
                                mav_param_type(var_type),
                                ARRAY_SIZE(var_info), // also includes FORMAT_VERSION
                                index++);
    strcpy(key, "B");

    if (_enabled) {
        // send parameter list
        for (auto var : var_info) {
            key[1] = '\0';
            strcat(key, var.name);
            vp = AP_Param::find(key, &var_type);
            if (vp == nullptr) {
                continue;
            }
            mavlink_msg_param_value_send(periph.mavlink.get_channel(),
                                        key,
                                        vp->cast_to_float(var_type),
                                        mav_param_type(var_type),
                                        ARRAY_SIZE(var_info), // also includes FORMAT_VERSION
                                        index++);
        }
    } else {
        // just send enable
        strcat(key, "_ENABLE");
        mavlink_msg_param_value_send(periph.mavlink.get_channel(),
                                    key,
                                    (float)_enabled.get(),
                                    MAV_PARAM_TYPE_INT8,
                                    2,
                                    index);
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

    // we only allow parameter sets for BASE, check starts with B_, and FORMAT_VERSION
    if ((strncmp(key, "B_", 2) != 0) && (strncmp(key, "FORMAT_VERSION", sizeof("FORMAT_VERSION")) != 0)) {
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

    // we only allow parameter sets for BASE, check starts with B_, and FORMAT_VERSION
    if ((strncmp(key, "B_", 2) != 0) && (strncmp(key, "FORMAT_VERSION", sizeof("FORMAT_VERSION")) != 0)) {
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
