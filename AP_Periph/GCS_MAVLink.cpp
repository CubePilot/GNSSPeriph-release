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

#include <AP_HAL/AP_HAL_Boards.h>
#include "GCS_MAVLink.h"
#include "AP_Periph.h"
#include <AP_Filesystem/AP_Filesystem.h>

#if HAL_GCS_ENABLED

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
    MSG_MCU_STATUS,
    MSG_MEMINFO,
    MSG_GPS_RAW,
    MSG_GPS_RTK,
};

static const ap_message STREAM_POSITION_msgs[] = {
#if defined(HAL_PERIPH_ENABLE_AHRS)
    MSG_LOCATION,
    MSG_LOCAL_POSITION
#endif
};

static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};

const struct GCS_MAVLINK::stream_entries GCS_MAVLINK::all_stream_entries[] = {
    MAV_STREAM_ENTRY(STREAM_RAW_SENSORS),
    MAV_STREAM_ENTRY(STREAM_POSITION),
    MAV_STREAM_ENTRY(STREAM_EXTENDED_STATUS),
    MAV_STREAM_ENTRY(STREAM_PARAMS),
    MAV_STREAM_TERMINATOR // must have this at end of stream_entries
};

const struct AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

uint8_t GCS_MAVLINK_Periph::sysid_my_gcs() const
{
    return periph.g.sysid_this_mav;
}

uint8_t GCS_Periph::sysid_this_mav() const
{
    return periph.g.sysid_this_mav;
}

MAV_RESULT GCS_MAVLINK_Periph::handle_preflight_reboot(const mavlink_command_long_t &packet)
{
    can_printf("RestartNode\n");
    hal.scheduler->delay(10);
    periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    NVIC_SystemReset();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    HAL_SITL::actually_reboot();
#endif
}

void GCS_MAVLINK_Periph::handle_cubepilot_firmware_update_resp(const mavlink_message_t &msg)
{
    mavlink_cubepilot_firmware_update_resp_t packet;
    mavlink_msg_cubepilot_firmware_update_resp_decode(&msg, &packet);
    if (packet.offset > cubeid_fw_size) {
        // update finished
        can_printf("CubeID fw update finished");
    }
    
    // read the requested chunk
    if (cubeid_fw_fd < 0) {
        // Nothing to do
        return;
    }

    AP::FS().lseek(cubeid_fw_fd, packet.offset, SEEK_SET);
    AP::FS().read(cubeid_fw_fd, cubeid_fw_readbuf, sizeof(cubeid_fw_readbuf));

    // send the requested chunk
    mavlink_encapsulated_data_t encap = {};
    encap.seqnr = packet.offset / sizeof(cubeid_fw_readbuf);
    memcpy(encap.data, cubeid_fw_readbuf, sizeof(cubeid_fw_readbuf));
    mavlink_msg_encapsulated_data_send_struct(chan, &encap);

    can_printf("CubeID fw update chunk %u/%lu sent", encap.seqnr, cubeid_fw_size / sizeof(cubeid_fw_readbuf));
}

// crc32
static uint32_t crc32(uint32_t crc, const uint8_t *buf, uint32_t len)
{
    uint32_t gen_table[] = {
        0x00000000, 0x1db71064, 0x3b6e20c8, 0x26d930ac,
        0x76dc4190, 0x6b6b51f4, 0x4db26158, 0x5005713c,
        0xedb88320, 0xf00f9344, 0xd6d6a3e8, 0xcb61b38c,
        0x9b64c2b0, 0x86d3d2d4, 0xa00ae278, 0xbdbdf21c,
    };
    crc ^= 0xffffffff;
    for (uint32_t i=0; i<len; i++) {
        uint8_t c = buf[i];
        crc = gen_table[(crc ^ c) & 0x0f] ^ (crc >> 4);
        crc = gen_table[(crc ^ (c >> 4)) & 0x0f] ^ (crc >> 4);
    }
    return crc ^ 0xffffffff;
}

// handle heartbeat from ODID module
void GCS_MAVLINK_Periph::handle_odid_heartbeat(const mavlink_message_t &msg)
{
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(&msg, &packet);
    if (packet.type == MAV_TYPE_ODID) {
        // open firmware from ROMFS
        if (cubeid_fw_fd == -1) {
            cubeid_fw_fd = AP::FS().open("@ROMFS/CubeID_fw.bin", O_RDONLY);
            if (cubeid_fw_fd < 0) {
                can_printf("Failed to open CubeID_fw.bin %d", cubeid_fw_fd);
                return;
            }
            can_printf("Opened cubeid_fw.bin");
            // calculate CRC32 of firmware
            while (true) {
                int n = AP::FS().read(cubeid_fw_fd, cubeid_fw_readbuf, sizeof(cubeid_fw_readbuf));
                if (n <= 0) {
                    break;
                }
                cubeid_fw_crc = crc32(cubeid_fw_crc, cubeid_fw_readbuf, n);
            }
            // seek to start of file
            AP::FS().lseek(cubeid_fw_fd, 0, SEEK_SET);
            cubeid_fw_size = AP::FS().lseek(cubeid_fw_fd, 0, SEEK_END);
        }
        // send firmware update start command
        mavlink_msg_cubepilot_firmware_update_start_send(chan, msg.sysid, msg.compid, cubeid_fw_size, cubeid_fw_crc);
    }
}

void GCS_MAVLINK_Periph::handleMessage(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP:
        handle_cubepilot_firmware_update_resp(msg);
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        can_printf("Got heartbeat from %u %u", msg.sysid, msg.compid);
        handle_odid_heartbeat(msg);
        // fallthrough
    default:
        handle_common_message(msg);
        break;
    }     // end switch
}


#endif // #if HAL_GCS_ENABLED
