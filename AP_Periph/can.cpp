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
  AP_Periph can support
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <canard.h>
#include <AP_GPS/RTCM3_Parser.h>
#include <stdio.h>
#include <drivers/stm32/canard_stm32.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Common/AP_FWVersion.h>
#include <dronecan_msgs.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>

#include <AP_HAL_ChibiOS/CANIface.h>
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <AP_HAL_SITL/CANSocketIface.h>
#endif


#include "i2c.h"
#include <utility>

#if HAL_NUM_CAN_IFACES >= 2
#include <AP_CANManager/AP_CANSensor.h>
#endif

#if HAL_NUM_CAN_IFACES > 1
#define IFACE_ALL ((1<<(HAL_NUM_CAN_IFACES+1))-1)
#endif

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

#ifndef HAL_CAN_POOL_SIZE
#define HAL_CAN_POOL_SIZE 16384
#endif

#ifndef HAL_PERIPH_LOOP_DELAY_US
// delay between can loop updates. This needs to be longer on F4
#if defined(STM32H7)
#define HAL_PERIPH_LOOP_DELAY_US 64
#else
#define HAL_PERIPH_LOOP_DELAY_US 512
#endif
#endif

#define DEBUG_PRINTS 0
#define DEBUG_PKTS 0
#if DEBUG_PRINTS
 # define Debug(fmt, args ...)  do {can_printf(fmt "\n", ## args);} while(0)
#else
 # define Debug(fmt, args ...)
#endif

uint32_t canard_memory_pool[HAL_CAN_POOL_SIZE/sizeof(uint32_t)];
static struct instance_t {
    uint8_t index;
    ChibiOS::CANIface* iface;
} instances[HAL_NUM_CAN_IFACES];

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(HAL_GPIO_PIN_TERMCAN1)
static ioline_t can_term_lines[] = {
HAL_GPIO_PIN_TERMCAN1

#if HAL_NUM_CAN_IFACES > 2 
#ifdef HAL_GPIO_PIN_TERMCAN2
,HAL_GPIO_PIN_TERMCAN2
#else
#error "Only one Can Terminator defined with over two CAN Ifaces"
#endif
#endif

#if HAL_NUM_CAN_IFACES > 2 
#ifdef HAL_GPIO_PIN_TERMCAN3
,HAL_GPIO_PIN_TERMCAN3
#else
#error "Only two Can Terminator defined with three CAN Ifaces"
#endif
#endif

};
#endif // CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(HAL_GPIO_PIN_TERMCAN1)

#ifndef CAN_APP_NODE_NAME
#define CAN_APP_NODE_NAME                                               "org.ardupilot.ap_periph"
#endif

#ifndef HAL_CAN_DEFAULT_NODE_ID
#define HAL_CAN_DEFAULT_NODE_ID CANARD_BROADCAST_NODE_ID
#endif

uint8_t PreferredNodeID = HAL_CAN_DEFAULT_NODE_ID;


#ifndef CAN_PROBE_CONTINUOUS
#define CAN_PROBE_CONTINUOUS 0
#endif

#ifndef AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz
#define AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz 1
#endif

ChibiOS::CANIface* AP_Periph_FW::can_iface_periph[HAL_NUM_CAN_IFACES];

/*
 * Node status variables
 */
static uavcan_protocol_NodeStatus node_status;

/**
 * Returns a pseudo random integer in a given range
 */
static uint16_t get_random_range(uint16_t range)
{
    return get_random16() % range;
}


/*
  get cpu unique ID
 */
static void readUniqueID(uint8_t* out_uid)
{
    uint8_t len = sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data);
    memset(out_uid, 0, len);
    hal.util->get_system_id_unformatted(out_uid, len);
}


/*
  handle a GET_NODE_INFO request
 */
void AP_Periph_DroneCAN::handle_get_node_info(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest &req)
{
    uavcan_protocol_GetNodeInfoResponse pkt {};

    node_status.uptime_sec = AP_HAL::native_millis() / 1000U;

    pkt.status = node_status;
    pkt.software_version.major = AP::fwversion().major;
    pkt.software_version.minor = AP::fwversion().minor;
    pkt.software_version.optional_field_flags = UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_VCS_COMMIT | UAVCAN_PROTOCOL_SOFTWAREVERSION_OPTIONAL_FIELD_FLAG_IMAGE_CRC;
    pkt.software_version.vcs_commit = app_descriptor.git_hash;
    uint32_t *crc = (uint32_t *)&pkt.software_version.image_crc;
    crc[0] = app_descriptor.image_crc1;
    crc[1] = app_descriptor.image_crc2;

    readUniqueID(pkt.hardware_version.unique_id);

    // use hw major/minor for APJ_BOARD_ID so we know what fw is
    // compatible with this hardware
    pkt.hardware_version.major = APJ_BOARD_ID >> 8;
    pkt.hardware_version.minor = APJ_BOARD_ID & 0xFF;

    if (periph.g.serial_number > 0) {
        hal.util->snprintf((char*)pkt.name.data, sizeof(pkt.name.data), "%s(%u)", CAN_APP_NODE_NAME, (unsigned)periph.g.serial_number);
    } else {
        hal.util->snprintf((char*)pkt.name.data, sizeof(pkt.name.data), "%s", CAN_APP_NODE_NAME);
    }
    pkt.name.len = strnlen((char*)pkt.name.data, sizeof(pkt.name.data));

    periph.dronecan->get_node_info_server.respond(transfer, pkt);
}

/*
  handle parameter GetSet request
 */
void AP_Periph_DroneCAN::handle_param_getset(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest &req)
{
    // param fetch all can take a long time, so pat watchdog
    stm32_watchdog_pat();

    uavcan_protocol_param_GetSetResponse pkt {};

    AP_Param *vp;
    enum ap_var_type ptype;

    if (req.name.len != 0 && req.name.len > AP_MAX_NAME_SIZE) {
        vp = nullptr;
    } else if (req.name.len != 0 && req.name.len <= AP_MAX_NAME_SIZE) {
        memcpy((char *)pkt.name.data, (char *)req.name.data, req.name.len);
        vp = AP_Param::find((char *)pkt.name.data, &ptype);
    } else {
        AP_Param::ParamToken token {};
        vp = AP_Param::find_by_index(req.index, &ptype, &token);
        if (vp != nullptr) {
            vp->copy_name_token(token, (char *)pkt.name.data, AP_MAX_NAME_SIZE+1, true);
        }
    }
    if (vp != nullptr && req.name.len != 0 && req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY) {
        // param set
        switch (ptype) {
        case AP_PARAM_INT8:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                return;
            }
            ((AP_Int8 *)vp)->set_and_save_ifchanged(req.value.integer_value);
            break;
        case AP_PARAM_INT16:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                return;
            }
            ((AP_Int16 *)vp)->set_and_save_ifchanged(req.value.integer_value);
            break;
        case AP_PARAM_INT32:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
                return;
            }
            ((AP_Int32 *)vp)->set_and_save_ifchanged(req.value.integer_value);
            break;
        case AP_PARAM_FLOAT:
            if (req.value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
                return;
            }
            ((AP_Float *)vp)->set_and_save_ifchanged(req.value.real_value);
            break;
        default:
            return;
        }
    }
    if (vp != nullptr) {
        switch (ptype) {
        case AP_PARAM_INT8:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.value.integer_value = ((AP_Int8 *)vp)->get();
            break;
        case AP_PARAM_INT16:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.value.integer_value = ((AP_Int16 *)vp)->get();
            break;
        case AP_PARAM_INT32:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
            pkt.value.integer_value = ((AP_Int32 *)vp)->get();
            break;
        case AP_PARAM_FLOAT:
            pkt.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
            pkt.value.real_value = ((AP_Float *)vp)->get();
            break;
        default:
            return;
        }
        pkt.name.len = strnlen((char *)pkt.name.data, sizeof(pkt.name.data));
    }
    periph.dronecan->param_getset_server.respond(transfer, pkt);
}

/*
  handle parameter executeopcode request
 */
void AP_Periph_DroneCAN::handle_param_executeopcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest &req)
{
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        StorageManager::erase();
        AP_Param::erase_all();
        AP_Param::load_all();
        AP_Param::setup_sketch_defaults();
#ifdef HAL_PERIPH_ENABLE_GPS
        AP_Param::setup_object_defaults(&periph.gps, periph.gps.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
        AP_Param::setup_object_defaults(&periph.compass, periph.compass.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
        AP_Param::setup_object_defaults(&periph.baro, periph.baro.var_info);
#endif
    }

    uavcan_protocol_param_ExecuteOpcodeResponse pkt {};

    pkt.ok = true;

    periph.dronecan->param_executeopcode_server.respond(transfer, pkt);
}

void AP_Periph_DroneCAN::handle_begin_firmware_update(const CanardRxTransfer& transfer, const uavcan_protocol_file_BeginFirmwareUpdateRequest &req)
{
#if HAL_RAM_RESERVE_START >= 256
    // setup information on firmware request at start of ram
    struct app_bootloader_comms *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    memset(comms, 0, sizeof(struct app_bootloader_comms));
    comms->magic = APP_BOOTLOADER_COMMS_MAGIC;
    comms->server_node_id = req.source_node_id;
    if (comms->server_node_id == 0) {
        comms->server_node_id = transfer.source_node_id;
    }
    memset(comms->path, 0, sizeof(comms->path));
    memcpy(comms->path, req.image_file_remote_path.path.data, req.image_file_remote_path.path.len);
    comms->my_node_id = periph.dronecan->canard_iface.get_node_id();

    uavcan_protocol_file_BeginFirmwareUpdateResponse reply {};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    periph.dronecan->begin_firmware_update_server.respond(transfer, reply);
    uint8_t count = 50;
    while (count--) {
        periph.dronecan->canard_iface.processTx(false);
        hal.scheduler->delay(1);
    }
#endif

    // instant reboot, with backup register used to give bootloader
    // the node_id
    periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    set_fast_reboot((rtc_boot_magic)(RTC_BOOT_CANBL | periph.dronecan->canard_iface.get_node_id()));
    NVIC_SystemReset();
#endif
}

void AP_Periph_DroneCAN::handle_allocation_response(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation &msg)
{
    // Rule C - updating the randomized time interval
    periph.send_next_node_id_allocation_request_at_ms =
        AP_HAL::native_millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer.source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
        periph.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        printf("Mismatching allocation response\n");
        periph.node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        periph.node_id_allocation_unique_id_offset = msg.unique_id.len;
        periph.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d of %d sending in %ld\n", msg.unique_id.len, periph.send_next_node_id_allocation_request_at_ms);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        // canardSetLocalNodeID(ins, msg.node_id);
        periph.dronecan->canard_iface.set_node_id(msg.node_id);
    }
}

/*
  handle gnss::RTCMStream
 */
void AP_Periph_DroneCAN::handle_RTCMStream(const CanardRxTransfer& transfer, const uavcan_equipment_gnss_RTCMStream &req)
{
    periph.gps.handle_gps_rtcm_fragment(0, req.data.data, req.data.len);
}

/*
    handle gnss::MovingBaselineData
*/
#if GPS_MOVING_BASELINE
void AP_Periph_DroneCAN::handle_MovingBaselineData(const CanardRxTransfer& transfer, const ardupilot_gnss_MovingBaselineData &msg)
{
    periph.gps.inject_MBL_data((uint8_t*)msg.data.data, msg.data.len);
    Debug("MovingBaselineData: len=%u\n", msg.data.len);
}
#endif // GPS_MOVING_BASELINE


static void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue)
{
    periph.notify.handle_rgb(red, green, blue);
    periph.rcout_has_new_data_to_update = true;
}

/*
  handle lightscommand
 */
void AP_Periph_DroneCAN::handle_lightscommand(const CanardRxTransfer& transfer, const uavcan_equipment_indication_LightsCommand &req)
{
    periph.led_command_override();

    for (uint8_t i=0; i<req.commands.len; i++) {
        const uavcan_equipment_indication_SingleLightCommand &cmd = req.commands.data[i];
        // to get the right color proportions we scale the green so that is uses the
        // same number of bits as red and blue
        uint8_t red = cmd.color.red<<3;
        uint8_t green = (cmd.color.green>>1)<<3;
        uint8_t blue = cmd.color.blue<<3;
        const int8_t brightness = periph.notify.get_rgb_led_brightness_percent();
        if (brightness != 100 && brightness >= 0) {
            const float scale = brightness * 0.01;
            red = constrain_int16(red * scale, 0, 255);
            green = constrain_int16(green * scale, 0, 255);
            blue = constrain_int16(blue * scale, 0, 255);
        }
        set_rgb_led(red, green, blue);
    }
}

/*
    handle global time sync
*/
void AP_Periph_DroneCAN::handle_global_time_sync(const CanardRxTransfer& transfer, const uavcan_protocol_GlobalTimeSync &msg)
{
    if (periph.last_time_sync_usec != 0) {
        // calculate the time offset from master
        periph.time_offset_usec = periph.jitter.correct_offboard_timestamp_usec(periph.last_time_sync_usec, msg.previous_transmission_timestamp_usec);
    }
    periph.last_time_sync_usec = transfer.timestamp_usec;
}

void AP_Periph_DroneCAN::handle_notify_state(const CanardRxTransfer& transfer, const ardupilot_indication_NotifyState &msg)
{
    if (msg.aux_data.len == 2 && msg.aux_data_type == ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_YAW_EARTH_CENTIDEGREES) {
        uint16_t tmp = 0;
        memcpy(&tmp, msg.aux_data.data, sizeof(tmp));
        periph.yaw_earth = radians((float)tmp * 0.01f);
    }
    periph.vehicle_state = msg.vehicle_state;
    periph.last_vehicle_state = AP_HAL::millis();
}

void AP_Periph_DroneCAN::handle_restart_node(const CanardRxTransfer& transfer, const uavcan_protocol_RestartNodeRequest &req)
{
    printf("RestartNode\n");
    hal.scheduler->delay(10);
    periph.prepare_reboot();
    NVIC_SystemReset();
}

void AP_Periph_DroneCAN::node_status_send(void)
{
    {
        node_status.uptime_sec = AP_HAL::millis() / 1000U;
        node_status.vendor_specific_status_code = hal.util->available_memory();
        node_status_pub.broadcast(node_status);
    }
    // also send stats
    if (periph.debug_option_is_set(AP_Periph_FW::DebugOptions::ENABLE_STATS)) {
        {
            auto protocol_stats = canard_iface.get_protocol_stats();
            stats_pub.broadcast(protocol_stats);
        }

        for (auto &ins : instances) {
            dronecan_protocol_CanStats can_stats;
            const AP_HAL::CANIface::bus_stats_t *bus_stats = ins.iface->get_statistics();
            if (bus_stats == nullptr) {
                return;
            }
            can_stats.interface = ins.index;
            can_stats.tx_requests = bus_stats->tx_requests;
            can_stats.tx_rejected = bus_stats->tx_rejected;
            can_stats.tx_overflow = bus_stats->tx_overflow;
            can_stats.tx_success = bus_stats->tx_success;
            can_stats.tx_timedout = bus_stats->tx_timedout;
            can_stats.tx_abort = bus_stats->tx_abort;
            can_stats.rx_received = bus_stats->rx_received;
            can_stats.rx_overflow = bus_stats->rx_overflow;
            can_stats.rx_errors = bus_stats->rx_errors;
            can_stats.busoff_errors = bus_stats->num_busoff_err;
            can_stats_pub.broadcast(can_stats);
        }
    }
}


/**
 * This function is called at 1 Hz rate from the main loop.
 */
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Printing the memory usage statistics.
     */
    {
        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */
        if (periph.dronecan->canard_iface.pool_peak_percent() > 70) {
            printf("WARNING: ENLARGE MEMORY POOL\n");
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    periph.dronecan->node_status_send();

    if (periph.g.flash_bootloader.get()) {
        const uint8_t flash_bl = periph.g.flash_bootloader.get();
        periph.g.flash_bootloader.set_and_save_ifchanged(0);
        if (flash_bl == 42) {
            // magic developer value to test watchdog support with main loop lockup
            while (true) {
                can_printf("entering lockup\n");
                hal.scheduler->delay(100);
            }
        }
        if (flash_bl == 43) {
            // magic developer value to test watchdog support with hard fault
            can_printf("entering fault\n");
            void *foo = (void*)0xE000ED38;
            typedef void (*fptr)();
            fptr gptr = (fptr) (void *) foo;
            gptr();
        }
        EXPECT_DELAY_MS(2000);
        hal.scheduler->delay(1000);
        AP_HAL::Util::FlashBootloader res = hal.util->flash_bootloader();
        switch (res) {
        case AP_HAL::Util::FlashBootloader::OK:
            can_printf("Flash bootloader OK\n");
            break;
        case AP_HAL::Util::FlashBootloader::NO_CHANGE:
            can_printf("Bootloader unchanged\n");
            break;
        default:
            can_printf("Flash bootloader FAILED\n");
            break;
        }
    }

    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

    if (AP_HAL::native_millis() > 30000) {
        // use RTC to mark that we have been running fine for
        // 30s. This is used along with watchdog resets to ensure the
        // user has a chance to load a fixed firmware
        set_fast_reboot(RTC_BOOT_FWOK);
    }
}

/*
  wait for dynamic allocation of node ID
 */
bool AP_Periph_FW::no_iface_finished_dna = true;
bool AP_Periph_FW::can_do_dna()
{
    if (dronecan->canard_iface.get_node_id() != CANARD_BROADCAST_NODE_ID) {
        AP_Periph_FW::no_iface_finished_dna = false;
        return true;
    }

    const uint32_t now = AP_HAL::native_millis();

    if (AP_Periph_FW::no_iface_finished_dna) {
        printf("Waiting for dynamic node ID allocation... (pool %u)\n", periph.dronecan->canard_iface.pool_peak_percent());
    }

    send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uavcan_protocol_dynamic_node_id_Allocation allocation_request;

    allocation_request.node_id = PreferredNodeID;
    if (node_id_allocation_unique_id_offset == 0) {
        allocation_request.first_part_of_unique_id = true;
    }

    uint8_t my_unique_id[sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data)];
    readUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data) - node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request.unique_id.data, &my_unique_id[node_id_allocation_unique_id_offset], uid_size);
    allocation_request.unique_id.len = uid_size;

    // Broadcasting the request
    dronecan->dynamic_node_id_pub.broadcast(allocation_request);

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    node_id_allocation_unique_id_offset = 0;
    return false;
}

void AP_Periph_FW::can_start()
{
    node_status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_INITIALIZATION;
    node_status.uptime_sec = AP_HAL::native_millis() / 1000U;

    if (g.can_node >= 0 && g.can_node < 128) {
        PreferredNodeID = g.can_node;
    }

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
    periph.g.flash_bootloader.set_and_save_ifchanged(0);
#endif

#if AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz && HAL_NUM_CAN_IFACES >= 2
    bool has_uavcan_at_1MHz = false;
    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
        if (g.can_baudrate[i] == 1000000) {
            has_uavcan_at_1MHz = true;
        }
    }
    if (!has_uavcan_at_1MHz) {
        g.can_baudrate[0].set_and_save(1000000);
    }
#endif // HAL_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz

#ifdef HAL_GPIO_PIN_TERMCAN1
    for (uint8_t i=0; i<ARRAY_SIZE(can_term_lines); i++){
        palWriteLine(can_term_lines[i], g.can_terminator[i]);
    }
#endif
    dronecan = new AP_Periph_DroneCAN();

    if (dronecan == nullptr) {
        AP_HAL::panic("Failed to allocate dronecan");
    }

    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
        can_iface_periph[i] = new ChibiOS::CANIface();
        instances[i].iface = can_iface_periph[i];
        instances[i].index = i;
        if (can_iface_periph[i] != nullptr) {
            if (canfdout()) {
                can_iface_periph[i]->init(g.can_baudrate[i],  g.can_fdbaudrate[i], AP_HAL::CANIface::NormalMode);
            } else {
                can_iface_periph[i]->init(g.can_baudrate[i], AP_HAL::CANIface::NormalMode);
            }
            dronecan->canard_iface.add_interface(can_iface_periph[i]);
        }
    }
}

AP_Periph_DroneCAN::AP_Periph_DroneCAN()
{
    canard_iface.init(canard_memory_pool, sizeof(canard_memory_pool), PreferredNodeID);

    // setup message timeouts and priorities
    dynamic_node_id_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    dynamic_node_id_pub.set_timeout_ms(500);

    arm_status_pub.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);
    arm_status_pub.set_timeout_ms(100);

    node_status_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    node_status_pub.set_timeout_ms(1000);

    stats_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    stats_pub.set_timeout_ms(1000);

    can_stats_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOW);
    can_stats_pub.set_timeout_ms(1000);

    mag_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    mag_pub.set_timeout_ms(10);

    fix2_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    fix2_pub.set_timeout_ms(50);

    aux_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    aux_pub.set_timeout_ms(50);

    gnss_status_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    gnss_status_pub.set_timeout_ms(50);

    moving_baseline_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    moving_baseline_pub.set_timeout_ms(50);

    relposheading_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    relposheading_pub.set_timeout_ms(50);

    static_pressure_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    static_pressure_pub.set_timeout_ms(20);

    static_temperature_pub.set_priority(CANARD_TRANSFER_PRIORITY_MEDIUM);
    static_temperature_pub.set_timeout_ms(20);

    log_pub.set_priority(CANARD_TRANSFER_PRIORITY_LOWEST);
    log_pub.set_timeout_ms(1000);

    tunnel_pub.set_priority(CANARD_TRANSFER_PRIORITY_HIGH);
    tunnel_pub.set_timeout_ms(5);

    param_getset_server.set_timeout_ms(200);
    param_executeopcode_server.set_timeout_ms(200);
    begin_firmware_update_server.set_timeout_ms(200);
    restart_node_server.set_timeout_ms(200);
}

void AP_Periph_FW::can_update()
{
    const uint32_t now = AP_HAL::native_millis();
    const uint32_t led_pattern = 0xAAAA;
    const uint32_t led_change_period = 50;
    static uint8_t led_idx = 0;
    static uint32_t last_led_change;

    if ((now - last_led_change > led_change_period) && no_iface_finished_dna) {
        // blink LED in recognisable pattern while waiting for DNA
#ifdef HAL_GPIO_PIN_LED
        palWriteLine(HAL_GPIO_PIN_LED, (led_pattern & (1U<<led_idx))?1:0);
#elif defined(HAL_GPIO_PIN_SAFE_LED)
        // or use safety LED if defined
        palWriteLine(HAL_GPIO_PIN_SAFE_LED, (led_pattern & (1U<<led_idx))?1:0);
#else
        (void)led_pattern;
        (void)led_idx;
#endif
        led_idx = (led_idx+1) % 32;
        last_led_change = now;
    }

    if (AP_HAL::millis() > send_next_node_id_allocation_request_at_ms) {
        can_do_dna();
    }
    
    static uint32_t last_1Hz_ms;
    if (now - last_1Hz_ms >= 1000) {
        last_1Hz_ms = now;
        process1HzTasks(AP_HAL::native_micros64());
    }
    if (!g.serial_i2c_mode) {
        dronecan->can_gps_update();
        dronecan->send_serial_monitor_data();
    } else {
        // update LEDs as well
        if (i2c_new_led_data) {
            i2c_new_led_data = false;
            set_rgb_led(i2c_led_color_red, i2c_led_color_green, i2c_led_color_blue);
        }
    }
    
    dronecan->can_mag_update();
    dronecan->can_baro_update();

    rcout_update();

    dronecan->canard_iface.process(1);
}

/*
  update CAN magnetometer
 */
void AP_Periph_DroneCAN::can_mag_update(void)
{
#ifdef HAL_PERIPH_ENABLE_MAG
    auto &compass = periph.compass;
    if (!compass.available()) {
        return;
    }
    compass.read();
#if CAN_PROBE_CONTINUOUS
    if (compass.get_count() == 0) {
        static uint32_t last_probe_ms;
        uint32_t now = AP_HAL::native_millis();
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            compass.init();
        }
    }
#endif

    if (periph.last_mag_update_ms == compass.last_update_ms()) {
        return;
    }
    if (!compass.healthy()) {
        return;
    }

    periph.last_mag_update_ms = compass.last_update_ms();
    const Vector3f &field = compass.get_field();
    uavcan_equipment_ahrs_MagneticFieldStrength pkt {};

    // the canard dsdl compiler doesn't understand float16
    for (uint8_t i=0; i<3; i++) {
        pkt.magnetic_field_ga[i] = field[i] * 0.001;
    }

    mag_pub.broadcast(pkt);
#endif // HAL_PERIPH_ENABLE_MAG
}
/*
  update CAN GPS
 */
void AP_Periph_DroneCAN::can_gps_update(void)
{
    auto &gps = periph.gps;
    if (gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        return;
    }
    gps.update();
    send_moving_baseline_msg();
    send_relposheading_msg();
    if (periph.last_gps_update_ms == gps.last_message_time_ms()) {
        return;
    }
    periph.last_gps_update_ms = gps.last_message_time_ms();

    {
        /*
          send Fix2 packet
        */
        uavcan_equipment_gnss_Fix2 pkt {};
        const Location &loc = gps.location();
        const Vector3f &vel = gps.velocity();

        if (gps.status() < AP_GPS::GPS_OK_FIX_2D && !periph.saw_lock_once) {
            pkt.timestamp.usec = AP_HAL::micros64();
            pkt.gnss_timestamp.usec = 0;
        } else {
            periph.saw_lock_once = true;
            pkt.timestamp.usec = gps.time_epoch_usec();
            pkt.gnss_timestamp.usec = gps.last_message_epoch_usec();
        }
        if (pkt.gnss_timestamp.usec == 0) {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_NONE;
        } else {
            pkt.gnss_time_standard = UAVCAN_EQUIPMENT_GNSS_FIX_GNSS_TIME_STANDARD_UTC;
        }
        pkt.longitude_deg_1e8 = uint64_t(loc.lng) * 10ULL;
        pkt.latitude_deg_1e8 = uint64_t(loc.lat) * 10ULL;
        pkt.height_ellipsoid_mm = loc.alt * 10;
        pkt.height_msl_mm = loc.alt * 10;
        for (uint8_t i=0; i<3; i++) {
            pkt.ned_velocity[i] = vel[i];
        }
        pkt.sats_used = gps.num_sats();
        switch (gps.status()) {
        case AP_GPS::GPS_Status::NO_GPS:
        case AP_GPS::GPS_Status::NO_FIX:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_NO_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_2D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_2D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_SINGLE;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_OTHER;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_DGPS:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_DGPS;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_DGPS_SBAS;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FLOAT:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FLOAT;
            break;
        case AP_GPS::GPS_Status::GPS_OK_FIX_3D_RTK_FIXED:
            pkt.status = UAVCAN_EQUIPMENT_GNSS_FIX2_STATUS_3D_FIX;
            pkt.mode = UAVCAN_EQUIPMENT_GNSS_FIX2_MODE_RTK;
            pkt.sub_mode = UAVCAN_EQUIPMENT_GNSS_FIX2_SUB_MODE_RTK_FIXED;
            break;
        }

        pkt.covariance.len = 6;

        float hacc;
        if (gps.horizontal_accuracy(hacc)) {
            pkt.covariance.data[0] = pkt.covariance.data[1] = sq(hacc);
        }
    
        float vacc;
        if (gps.vertical_accuracy(vacc)) {
            pkt.covariance.data[2] = sq(vacc);
        }

        float sacc;
        if (gps.speed_accuracy(sacc)) {
            float vc3 = sq(sacc);
            pkt.covariance.data[3] = pkt.covariance.data[4] = pkt.covariance.data[5] = vc3;
        }

        fix2_pub.broadcast(pkt);
    }
    
    /*
      send aux packet
     */
    {
        uavcan_equipment_gnss_Auxiliary aux {};
        aux.hdop = gps.get_hdop() * 0.01;
        aux.vdop = gps.get_vdop() * 0.01;

        aux_pub.broadcast(aux);
    }

    // send the gnss status packet
    {
        ardupilot_gnss_Status status {};

        status.healthy = gps.is_healthy();
        if (gps.logging_present() && gps.logging_enabled() && !gps.logging_failed()) {
            status.status |= ARDUPILOT_GNSS_STATUS_STATUS_LOGGING;
        }
        uint8_t idx; // unused
        if (status.healthy && !gps.first_unconfigured_gps(idx)) {
            status.status |= ARDUPILOT_GNSS_STATUS_STATUS_ARMABLE;
        }

        uint32_t error_codes;
        if (gps.get_error_codes(error_codes)) {
            status.error_codes = error_codes;
        }

        gnss_status_pub.broadcast(status);
    }
}

void AP_Periph_DroneCAN::send_moving_baseline_msg()
{
#if defined(HAL_PERIPH_ENABLE_GPS) && GPS_MOVING_BASELINE
    const uint8_t *data = nullptr;
    uint16_t len = 0;
    if (!gps.get_RTCMV3(data, len)) {
        return;
    }
    if (len == 0 || data == nullptr) {
        return;
    }
    // send the packet from Moving Base to be used RelPosHeading calc by GPS module
    ardupilot_gnss_MovingBaselineData mbldata {};
    // get the data from the moving base
    static_assert(sizeof(ardupilot_gnss_MovingBaselineData::data.data) == RTCM3_MAX_PACKET_LEN, "Size of Moving Base data is wrong");
    mbldata.data.len = len;
    memcpy(mbldata.data.data, data, len);

    moving_baseline_pub.broadcast(mbldata);
    gps.clear_RTCMV3();
#endif // HAL_PERIPH_ENABLE_GPS && GPS_MOVING_BASELINE
}

void AP_Periph_DroneCAN::send_relposheading_msg() {
#if defined(HAL_PERIPH_ENABLE_GPS) && GPS_MOVING_BASELINE
    float reported_heading;
    float relative_distance;
    float relative_down_pos;
    float reported_heading_acc;
    static uint32_t last_timestamp = 0;
    uint32_t curr_timestamp = 0;
    gps.get_RelPosHeading(curr_timestamp, reported_heading, relative_distance, relative_down_pos, reported_heading_acc);
    if (last_timestamp == curr_timestamp) {
        return;
    }
    last_timestamp = curr_timestamp;
    ardupilot_gnss_RelPosHeading relpos {};
    relpos.timestamp.usec = uint64_t(curr_timestamp)*1000LLU;
    relpos.reported_heading_deg = reported_heading;
    relpos.relative_distance_m = relative_distance;
    relpos.relative_down_pos_m = relative_down_pos;
    relpos.reported_heading_acc_deg = reported_heading_acc;
    relpos.reported_heading_acc_available = true;

    relposheading_pub.broadcast(relpos);
#endif // HAL_PERIPH_ENABLE_GPS && GPS_MOVING_BASELINE
}

/*
  update CAN baro
 */
void AP_Periph_DroneCAN::can_baro_update(void)
{
#ifdef HAL_PERIPH_ENABLE_BARO
    auto &baro = periph.baro;
    if (!periph.g.baro_enable) {
        return;
    }
    baro.update();
    if (periph.last_baro_update_ms == baro.get_last_update()) {
        return;
    }

    periph.last_baro_update_ms = baro.get_last_update();
    if (!baro.healthy()) {
        // don't send any data
        return;
    }
    const float press = baro.get_pressure();
    const float temp = baro.get_temperature();

    {
        uavcan_equipment_air_data_StaticPressure pkt {};
        pkt.static_pressure = press;
        pkt.static_pressure_variance = 0; // should we make this a parameter?
        static_pressure_pub.broadcast(pkt);
    }

    {
        uavcan_equipment_air_data_StaticTemperature pkt {};
        pkt.static_temperature = C_TO_KELVIN(temp);
        pkt.static_temperature_variance = 0; // should we make this a parameter?
        static_temperature_pub.broadcast(pkt);
    }
#endif // HAL_PERIPH_ENABLE_BARO
}

// printf to CAN LogMessage for debugging
void can_printf(const char *fmt, ...)
{
    uavcan_protocol_debug_LogMessage pkt {};
    va_list ap;
    va_start(ap, fmt);
    uint32_t n = vsnprintf((char*)pkt.text.data, sizeof(pkt.text.data), fmt, ap);
    va_end(ap);
    pkt.text.len = MIN(n, sizeof(pkt.text.data));

    periph.dronecan->log_pub.broadcast(pkt);
}
