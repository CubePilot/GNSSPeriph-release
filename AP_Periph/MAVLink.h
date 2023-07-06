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
#pragma once
#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>

#include <GCS_MAVLink/GCS_MAVLink.h>

/*
 *  GCS backend used for many examples and tools
 */
class MAVLink_Periph
{
public:
    MAVLink_Periph(mavlink_channel_t _chan) : chan(_chan) { }
    void init(uint8_t port, uint32_t baudrate);
    void send_heartbeat();
    void update();
    uint32_t txspace() const;
    uint32_t write(const uint8_t *tbuf, uint32_t len);
    mavlink_message_t* channel_buffer() { return &chan_buffer; }
    mavlink_status_t* channel_status() { return &chan_status; }

private:
    void handleMessage(const mavlink_message_t &msg);
    MAV_RESULT handle_preflight_reboot(const mavlink_command_long_t &packet);
    void handle_open_drone_id_arm_status(const mavlink_message_t &msg);
    uint8_t sysid_my_gcs() const;

    void handle_cubepilot_firmware_update_resp(const mavlink_message_t &msg);
    void handle_odid_heartbeat(const mavlink_message_t &msg);
    uint8_t sysid_this_mav() const;

    uint32_t cubeid_fw_size;
    uint32_t cubeid_fw_crc;
    int cubeid_fw_fd = -1;
    uint8_t cubeid_fw_readbuf[252];
    bool cubeid_fw_updated;
    mavlink_channel_t chan;
    AP_HAL::UARTDriver *serial;
    mavlink_message_t chan_buffer;
    mavlink_status_t chan_status;
};

