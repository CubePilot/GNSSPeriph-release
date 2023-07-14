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
#include "MAVLink.h"
#include "AP_Periph.h"
#include <AP_Filesystem/AP_Filesystem.h>
#include <dronecan_msgs.h>

#ifdef MAVLINK_SEPARATE_HELPERS
// Shut up warnings about missing declarations; TODO: should be fixed on
// mavlink/pymavlink project for when MAVLINK_SEPARATE_HELPERS is defined
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#include "include/mavlink/v2.0/mavlink_helpers.h"
#pragma GCC diagnostic pop
#endif

mavlink_system_t mavlink_system = {3,1};

extern const AP_HAL::HAL &hal;

void MAVLink_Periph::init(uint8_t port, uint32_t baudrate)
{
    serial = hal.serial(port);
    // begin port
    serial->begin(baudrate);
    mavlink_system.sysid = periph.g.sysid_this_mav;
}

void MAVLink_Periph::send_heartbeat()
{
    mavlink_heartbeat_t hb = {};
    hb.type = MAV_TYPE_GPS;
    hb.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    hb.base_mode = 0;
    hb.system_status = MAV_STATE_ACTIVE;
    mavlink_msg_heartbeat_send_struct(chan, &hb);
}

void MAVLink_Periph::update()
{
    if (serial == nullptr) {
        return;
    }
    // read messages
    const uint16_t nbytes = serial->available();
    for (uint16_t i=0; i<nbytes; i++) {
        const uint8_t c = (uint8_t)serial->read();
        process_byte(c);
    }
}

bool MAVLink_Periph::process_byte(const uint8_t c)
{
    bool handled = false;
    if (mavlink_parse_char(chan, c, &chan_buffer, &chan_status)) {
        handled = true;
        handleMessage(chan_buffer);
    }
    return handled;
}

uint32_t MAVLink_Periph::txspace() const
{
    if (serial == nullptr) {
        return 0;
    }
    return serial->txspace();
}

uint32_t MAVLink_Periph::write(const uint8_t *buf, uint32_t len)
{
    if (serial == nullptr) {
        return 0;
    }
    return serial->write(buf, len);
}

uint8_t MAVLink_Periph::sysid_my_gcs() const
{
    return periph.g.sysid_this_mav;
}

uint8_t MAVLink_Periph::sysid_this_mav() const
{
    return periph.g.sysid_this_mav;
}

MAV_RESULT MAVLink_Periph::handle_preflight_reboot(const mavlink_command_long_t &packet)
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

void MAVLink_Periph::handle_cubepilot_firmware_update_resp(const mavlink_message_t &msg)
{
    mavlink_cubepilot_firmware_update_resp_t packet;
    mavlink_msg_cubepilot_firmware_update_resp_decode(&msg, &packet);
    if (packet.offset > cubeid_fw_size) {
        // update finished
        can_printf("CubeID Firmware update finished.");
        cubeid_fw_updated = true;
        return;
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
void MAVLink_Periph::handle_odid_heartbeat(const mavlink_message_t &msg)
{
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(&msg, &packet);
    if (packet.type == MAV_TYPE_ODID && !cubeid_fw_updated && periph.g.cubeid_fw_update_enabled) {
        // open firmware from ROMFS
        if (cubeid_fw_fd == -1) {
            cubeid_fw_fd = AP::FS().open("@ROMFS//CubeID_fw.bin", O_RDONLY);
            if (cubeid_fw_fd < 0) {
                can_printf("Failed to open CubeID_fw.bin %d %d", cubeid_fw_fd, errno);
                return;
            }

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
            can_printf("CubeID firmware size %lu crc 0x%08lx", cubeid_fw_size, cubeid_fw_crc);
        }
        // send firmware update start command
        mavlink_msg_cubepilot_firmware_update_start_send(chan, msg.sysid, msg.compid, cubeid_fw_size, cubeid_fw_crc);
    }
}

// handle arm status from ODID module
void MAVLink_Periph::handle_open_drone_id_arm_status(const mavlink_message_t &msg)
{
    mavlink_open_drone_id_arm_status_t packet;
    mavlink_msg_open_drone_id_arm_status_decode(&msg, &packet);
    periph.dronecan->handle_open_drone_id_arm_status(packet);
}

void MAVLink_Periph::handleMessage(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_CUBEPILOT_FIRMWARE_UPDATE_RESP:
        handle_cubepilot_firmware_update_resp(msg);
        break;
    case MAVLINK_MSG_ID_OPEN_DRONE_ID_ARM_STATUS:
        handle_open_drone_id_arm_status(msg);
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        handle_odid_heartbeat(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
#ifdef ENABLE_BASE_MODE
        periph.gps_base.handle_param_request_list(msg);
#endif
        break;
    case MAVLINK_MSG_ID_PARAM_SET:
#ifdef ENABLE_BASE_MODE
        periph.gps_base.handle_param_set(msg);
#endif
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
#ifdef ENABLE_BASE_MODE
        periph.gps_base.handle_param_request_read(msg);
#endif
        break;
    default:
        break;
    }     // end switch
}

bool gcs_alternative_active[MAVLINK_COMM_NUM_BUFFERS];

// per-channel lock
static HAL_Semaphore chan_locks[MAVLINK_COMM_NUM_BUFFERS];
static bool chan_discard[MAVLINK_COMM_NUM_BUFFERS];

/// Check for available transmit space on the nominated MAVLink channel
///
/// @param chan		Channel to check
/// @returns		Number of bytes available
uint16_t comm_get_txspace(mavlink_channel_t chan)
{
    MAVLink_Periph *link = periph.get_link(chan);
    if (link == nullptr) {
        return 0;
    }
    return link->txspace();
}

mavlink_message_t* mavlink_get_channel_buffer(uint8_t chan) {
    MAVLink_Periph *link = periph.get_link((mavlink_channel_t)chan);
    if (link == nullptr) {
        return nullptr;
    }
    return link->channel_buffer();
}

mavlink_status_t* mavlink_get_channel_status(uint8_t chan) {
    MAVLink_Periph *link = periph.get_link((mavlink_channel_t)chan);
    if (link == nullptr) {
        return nullptr;
    }
    return link->channel_status();
}

/*
  send a buffer out a MAVLink channel
 */
void comm_send_buffer(mavlink_channel_t chan, const uint8_t *buf, uint8_t len)
{
    MAVLink_Periph *link = periph.get_link(chan);

    if (!valid_channel(chan) || link == nullptr || chan_discard[chan]) {
        return;
    }
    if (gcs_alternative_active[chan]) {
        // an alternative protocol is active
        return;
    }
    const size_t written = link->write(buf, len);
    (void)written;
}

/*
  lock a channel for send
  if there is insufficient space to send size bytes then all bytes
  written to the channel by the mavlink library will be discarded
  while the lock is held.
 */
void comm_send_lock(mavlink_channel_t chan_m, uint16_t size)
{
    MAVLink_Periph *link = periph.get_link(chan_m);
    uint8_t chan = uint8_t(chan_m);
    chan_locks[chan].take_blocking();
    if (link->txspace() < size) {
        chan_discard[chan] = true;
    }
}

/*
  unlock a channel
 */
void comm_send_unlock(mavlink_channel_t chan_m)
{
    const uint8_t chan = uint8_t(chan_m);
    chan_discard[chan] = false;
    chan_locks[chan].give();
}

/*
  return reference to GCS channel lock, allowing for
  HAVE_PAYLOAD_SPACE() to be run with a locked channel
 */
HAL_Semaphore &comm_chan_lock(mavlink_channel_t chan)
{
    return chan_locks[uint8_t(chan)];
}
