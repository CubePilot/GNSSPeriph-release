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
/*
  handle tunnelling of serial data over DroneCAN
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"

#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

#define TUNNEL_LOCK_KEY 0xf2e460e4U

#ifndef TUNNEL_DEBUG
#define TUNNEL_DEBUG 0
#endif

#if TUNNEL_DEBUG
# define debug(fmt, args...) can_printf(fmt "\n", ##args)
#else
# define debug(fmt, args...)
#endif

/*
  get the default port to tunnel if the client requests port -1
 */
int8_t AP_Periph_FW::get_default_tunnel_serial_port(void)
{
    return serial_manager.find_portnum(AP_SerialManager::SerialProtocol_GPS, 0);
}

/*
  handle tunnel data
 */
void AP_Periph_FW::handle_tunnel_Targetted(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_tunnel_Targetted pkt;
    if (uavcan_tunnel_Targetted_decode(transfer, &pkt)) {
        return;
    }
    if (pkt.target_node != canardGetLocalNodeID(ins)) {
        return;
    }
    if (monitor.buffer == nullptr) {
        monitor.buffer = new ByteBuffer(1024);
        if (monitor.buffer == nullptr) {
            return;
        }
    }
    int8_t uart_num = pkt.serial_id;
    if (uart_num == -1) {
        uart_num = get_default_tunnel_serial_port();
    }
    if (uart_num < 0) {
        return;
    }
    auto *uart = hal.serial(uart_num);
    if (uart == nullptr) {
        return;
    }
    if (monitor.uart_num != uart_num && monitor.uart != nullptr) {
        // remove monitor from previous uart
        hal.serial(monitor.uart_num)->set_monitor_read_buffer(nullptr);
    }
    monitor.uart_num = uart_num;
    if (uart != monitor.uart) {
        // change of uart or expired, clear old data
        monitor.buffer->clear();
        monitor.uart = uart;
    }
    if (monitor.uart == nullptr) {
        return;
    }
    /*
      allow for locked state to change at any time, so users can
      switch between locked and unlocked while connected
     */
    monitor.locked = (pkt.options & UAVCAN_TUNNEL_TARGETTED_OPTION_LOCK_PORT) != 0;
    if (monitor.locked) {
        monitor.uart->lock_port(TUNNEL_LOCK_KEY, TUNNEL_LOCK_KEY);
    } else {
        monitor.uart->lock_port(0,0);
    }
    monitor.node_id = transfer->source_node_id;
    monitor.protocol = pkt.protocol.protocol;
    if (pkt.baudrate != monitor.baudrate) {
        if (monitor.locked && pkt.baudrate != 0) {
            monitor.uart->begin_locked(pkt.baudrate, 0, 0, TUNNEL_LOCK_KEY);
            debug("begin_locked %u", unsigned(pkt.baudrate));
        }
        monitor.baudrate = pkt.baudrate;
    }
    monitor.uart->set_monitor_read_buffer(monitor.buffer);
    monitor.last_request_ms = AP_HAL::millis();

    // write to device
    if (pkt.buffer.len > 0) {
        if (monitor.locked) {
            debug("write_locked %u", unsigned(pkt.buffer.len));
            monitor.uart->write_locked(pkt.buffer.data, pkt.buffer.len, TUNNEL_LOCK_KEY);
        } else {
            monitor.uart->write(pkt.buffer.data, pkt.buffer.len);
        }
    } else {
        debug("locked keepalive");
    }
}

/*
  send tunnelled serial data
 */
void AP_Periph_FW::send_serial_monitor_data()
{
    if (monitor.uart == nullptr ||
        monitor.node_id == 0 ||
        monitor.buffer == nullptr) {
        return;
    }
    const uint32_t last_req_ms = monitor.last_request_ms;
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_req_ms >= 3000) {
        // stop sending and unlock, but don't release the buffer
        if (monitor.locked) {
            debug("unlock");
            monitor.uart->lock_port(0, 0);
        }
        monitor.uart = nullptr;
        return;
    }
    if (monitor.locked) {
        /*
          when the port is locked nobody is reading the uart so the
          monitor doesn't fill. We read here to ensure it fills
         */
        uint8_t buf[120];
        for (uint8_t i=0; i<8; i++) {
            if (monitor.uart->read_locked(buf, sizeof(buf), TUNNEL_LOCK_KEY) <= 0) {
                break;
            }
        }
    }
    uint8_t sends = 8;
    while (monitor.buffer->available() > 0 && sends-- > 0) {
        uint32_t n;
        const uint8_t *buf = monitor.buffer->readptr(n);
        if (n == 0) {
            return;
        }
        // broadcast data as tunnel packets, can be used for uCenter debug and device fw update
        uavcan_tunnel_Targetted pkt {};
        n = MIN(n, sizeof(pkt.buffer.data));
        pkt.target_node = monitor.node_id;
        pkt.protocol.protocol = monitor.protocol;
        pkt.buffer.len = n;
        pkt.baudrate = monitor.baudrate;
        memcpy(pkt.buffer.data, buf, n);

        uint8_t buffer[UAVCAN_TUNNEL_TARGETTED_MAX_SIZE] {};
        const uint16_t total_size = uavcan_tunnel_Targetted_encode(&pkt, buffer, !canfdout());

        debug("read %u", unsigned(n));

        if (!canard_broadcast(UAVCAN_TUNNEL_TARGETTED_SIGNATURE,
                              UAVCAN_TUNNEL_TARGETTED_ID,
                              CANARD_TRANSFER_PRIORITY_MEDIUM,
                              &buffer[0],
                              total_size)) {
            break;
        }
        monitor.buffer->advance(n);
    }
}
