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

#ifdef ENABLE_BASE_MODE

#define LOCK_ID 0x4c4f434b // "LOCK"
#define UBX_PREAMBLE1 0xb5
#define UBX_PREAMBLE2 0x62 

#define CLASS_RXM 0x02
#define MSG_RXM_RAW  0x10
#define MSG_RXM_RAWX 0x15

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

    // @Param: S_IN_TIME
    // @DisplayName: Time to wait for Survey in to be asserted
    // @Description: Time to wait for Survey in to be asserted
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("_S_IN_TIME",  3, GPS_Base, _s_in_time, 20.0),

    // @Param: S_IN_ACC
    // @DisplayName: Accuracy to wait for Survey in to be asserted
    // @Description: Accuracy to wait for Survey in to be asserted
    // @Range: 0 1000
    // @User: Standard
    AP_GROUPINFO("_S_IN_ACC",  4, GPS_Base, _s_in_acc, 2.0),

    // @Param: S_IN_LAT
    // @DisplayName: Latitude to wait for Survey in to be asserted
    // @Description: Latitude to wait for Survey in to be asserted
    // @Range: -90 90
    // @User: Standard
    AP_GROUPINFO("_S_IN_LAT",  5, GPS_Base, _s_in_lat, 0.0),

    // @Param: S_IN_LON
    // @DisplayName: Longitude to wait for Survey in to be asserted
    // @Description: Longitude to wait for Survey in to be asserted
    // @Range: -180 180
    // @User: Standard
    AP_GROUPINFO("_S_IN_LON",  6, GPS_Base, _s_in_lon, 0.0),

    // @Param: S_IN_ALT
    // @DisplayName: Altitude to wait for Survey in to be asserted
    // @Description: Altitude to wait for Survey in to be asserted
    // @Range: -1000 1000
    // @User: Standard
    AP_GROUPINFO("_S_IN_ALT",  7, GPS_Base, _s_in_alt, -1000.0),

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

void GPS_Base::parse_time_ubx() {
    uint8_t tmp[8];
    if (gps_buffer.peek(0) == UBX_PREAMBLE1 &&
        gps_buffer.peek(1) == UBX_PREAMBLE2 &&
        gps_buffer.peek(2) == CLASS_RXM) {
        // we have a UBX RXM-RAW or RXM-RAWX message
        if (gps_buffer.peek(3) == MSG_RXM_RAW) {
            int32_t tow;
            int16_t week;
            // peek 4 bytes at offset 6 to get tow
            for (uint8_t i=0; i<4; i++) {
                tmp[i] = gps_buffer.peek(6+i);
            }
            memcpy(&tow, tmp, 4);
            // peek 2 bytes at offset 10 to get week
            tmp[0] = gps_buffer.peek(10);
            tmp[1] = gps_buffer.peek(11);
            memcpy(&week, tmp, 2);
            if (week > 0 && tow >= 0) {
                gps_week_time((uint16_t)week, (uint32_t)tow);
            }
        }
        if (gps_buffer.peek(3) == MSG_RXM_RAWX) {
            double tow;
            int16_t week;
            // peek 8 bytes at offset 6 to get tow
            for (uint8_t i=0; i<8; i++) {
                tmp[i] = gps_buffer.peek(6+i);
            }
            memcpy(&tow, tmp, 8);
            // peek 2 bytes at offset 14 to get week
            tmp[0] = gps_buffer.peek(14);
            tmp[1] = gps_buffer.peek(15);
            memcpy(&week, tmp, 2);
            if (week > 0 && tow >= 0) {
                gps_week_time((uint16_t)week, (uint32_t)(tow * 1000));
            }
        }
        // print date/time
        can_printf("%04d-%02d-%02d %02d:%02d:%02d\n", dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second);
    }
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
    gcs_uart->lock_port(LOCK_ID, LOCK_ID);

    uint8_t byte, last_byte = 0;

    if (gcs_uart->get_usb_baud() != gps_uart->get_baud_rate()) {
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
        gps_buffer.write(&byte, 1);
        if ((gps_received_preamble && gps_num_bytes_to_rx == 0) || gps_buffer.space() == 0) {
            parse_time_ubx();
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
