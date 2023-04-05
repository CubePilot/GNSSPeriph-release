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

#ifndef IFACE_ALL
#define IFACE_ALL ((1<<(HAL_NUM_CAN_IFACES+1))-1)
#endif

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

#ifndef HAL_CAN_POOL_SIZE
#define HAL_CAN_POOL_SIZE 4000
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

static struct instance_t {
    uint8_t index;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    ChibiOS::CANIface* iface;
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    HALSITL::CANIface* iface;
#endif
} instances[HAL_NUM_CAN_IFACES];

static struct dronecan_protocol_t {
    CanardInstance canard;
    uint32_t canard_memory_pool[HAL_CAN_POOL_SIZE/sizeof(uint32_t)];
    struct tid_map {
        uint32_t transfer_desc;
        uint8_t tid;
        tid_map *next;
    } *tid_map_head;
    /*
    * Variables used for dynamic node ID allocation.
    * RTFM at http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    */
    uint32_t send_next_node_id_allocation_request_at_ms; ///< When the next node ID allocation request should be sent
    uint8_t node_id_allocation_unique_id_offset;         ///< Depends on the stage of the next request
    uint8_t tx_fail_count;
    uint8_t dna_interface = 1;
} dronecan;

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

#ifndef AP_PERIPH_BATTERY_MODEL_NAME
#define AP_PERIPH_BATTERY_MODEL_NAME CAN_APP_NODE_NAME
#endif

#ifndef CAN_PROBE_CONTINUOUS
#define CAN_PROBE_CONTINUOUS 0
#endif

#ifndef AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz
#define AP_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz 1
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
ChibiOS::CANIface* AP_Periph_FW::can_iface_periph[HAL_NUM_CAN_IFACES];
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
HALSITL::CANIface* AP_Periph_FW::can_iface_periph[HAL_NUM_CAN_IFACES];
#endif


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
static void handle_get_node_info(CanardInstance* ins,
                                 CanardRxTransfer* transfer)
{
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE] {};
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

    uint16_t total_size = uavcan_protocol_GetNodeInfoResponse_encode(&pkt, buffer, !periph.canfdout());

    const int16_t resp_res = canardRequestOrRespond(ins,
                                                    transfer->source_node_id,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                                                    UAVCAN_PROTOCOL_GETNODEINFO_ID,
                                                    &transfer->transfer_id,
                                                    transfer->priority,
                                                    CanardResponse,
                                                    &buffer[0],
                                                    total_size,
                                                    IFACE_ALL,
                                                    periph.canfdout());
    if (resp_res <= 0) {
        printf("Could not respond to GetNodeInfo: %d\n", resp_res);
    }
}

/*
  handle parameter GetSet request
 */
static void handle_param_getset(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // param fetch all can take a long time, so pat watchdog
    stm32_watchdog_pat();

    uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req)) {
        return;
    }

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

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE] {};
    uint16_t total_size = uavcan_protocol_param_GetSetResponse_encode(&pkt, buffer, !periph.canfdout());

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_GETSET_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size,
                           IFACE_ALL,
                           periph.canfdout());

}

/*
  handle parameter executeopcode request
 */
static void handle_param_executeopcode(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req)) {
        return;
    }
    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        StorageManager::erase();
        AP_Param::erase_all();
        AP_Param::load_all();
        AP_Param::setup_sketch_defaults();
#ifdef HAL_PERIPH_ENABLE_GPS
        AP_Param::setup_object_defaults(&periph.gps, periph.gps.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_BATTERY
        AP_Param::setup_object_defaults(&periph.battery, periph.battery.lib.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
        AP_Param::setup_object_defaults(&periph.compass, periph.compass.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
        AP_Param::setup_object_defaults(&periph.baro, periph.baro.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
        AP_Param::setup_object_defaults(&periph.airspeed, periph.airspeed.var_info);
#endif
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
        AP_Param::setup_object_defaults(&periph.rangefinder, periph.rangefinder.var_info);
#endif
    }

    uavcan_protocol_param_ExecuteOpcodeResponse pkt {};

    pkt.ok = true;

    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE] {};
    uint16_t total_size = uavcan_protocol_param_ExecuteOpcodeResponse_encode(&pkt, buffer, !periph.canfdout());

    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size,
                           IFACE_ALL,
                           periph.canfdout());
}

static void canard_broadcast(uint64_t data_type_signature,
                                uint16_t data_type_id,
                                uint8_t priority,
                                const void* payload,
                                uint16_t payload_len);
static void processTx(void);
static void processRx(void);

static void handle_begin_firmware_update(CanardInstance* ins, CanardRxTransfer* transfer)
{
#if HAL_RAM_RESERVE_START >= 256
    // setup information on firmware request at start of ram
    struct app_bootloader_comms *comms = (struct app_bootloader_comms *)HAL_RAM0_START;
    memset(comms, 0, sizeof(struct app_bootloader_comms));
    comms->magic = APP_BOOTLOADER_COMMS_MAGIC;

    // manual decoding due to TAO bug in libcanard generated code
    if (transfer->payload_len < 1 || transfer->payload_len > sizeof(comms->path)+1) {
        return;
    }
    uint32_t offset = 0;
    canardDecodeScalar(transfer, 0, 8, false, (void*)&comms->server_node_id);
    offset += 8;
    for (uint8_t i=0; i<transfer->payload_len-1; i++) {
        canardDecodeScalar(transfer, offset, 8, false, (void*)&comms->path[i]);
        offset += 8;
    }
    if (comms->server_node_id == 0) {
        comms->server_node_id = transfer->source_node_id;
    }
    comms->my_node_id = canardGetLocalNodeID(ins);

    uint8_t buffer[UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_MAX_SIZE] {};
    uavcan_protocol_file_BeginFirmwareUpdateResponse reply {};
    reply.error = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_RESPONSE_ERROR_OK;

    uint32_t total_size = uavcan_protocol_file_BeginFirmwareUpdateResponse_encode(&reply, buffer, !periph.canfdout());
    canardRequestOrRespond(ins,
                           transfer->source_node_id,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE,
                           UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID,
                           &transfer->transfer_id,
                           transfer->priority,
                           CanardResponse,
                           &buffer[0],
                           total_size,
                           IFACE_ALL,
                           periph.canfdout());
    uint8_t count = 50;
    while (count--) {
        processTx();
        hal.scheduler->delay(1);
    }
#endif

    // instant reboot, with backup register used to give bootloader
    // the node_id
    periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    set_fast_reboot((rtc_boot_magic)(RTC_BOOT_CANBL | canardGetLocalNodeID(ins)));
    NVIC_SystemReset();
#endif
}

static void handle_allocation_response(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // Rule C - updating the randomized time interval
    dronecan.send_next_node_id_allocation_request_at_ms =
        AP_HAL::native_millis() + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID)
    {
        printf("Allocation request from another allocatee\n");
        dronecan.node_id_allocation_unique_id_offset = 0;
        return;
    }

    // Copying the unique ID from the message
    uavcan_protocol_dynamic_node_id_Allocation msg;

    uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg);

    // Obtaining the local unique ID
    uint8_t my_unique_id[sizeof(msg.unique_id.data)];
    readUniqueID(my_unique_id);

    // Matching the received UID against the local one
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        printf("Mismatching allocation response\n");
        dronecan.node_id_allocation_unique_id_offset = 0;
        return;         // No match, return
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // The allocator has confirmed part of unique ID, switching to the next stage and updating the timeout.
        dronecan.node_id_allocation_unique_id_offset = msg.unique_id.len;
        dronecan.send_next_node_id_allocation_request_at_ms -= UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;

        printf("Matching allocation response: %d of %d sending in %ld\n", msg.unique_id.len, dronecan.send_next_node_id_allocation_request_at_ms);
    } else {
        // Allocation complete - copying the allocated node ID from the message
        canardSetLocalNodeID(ins, msg.node_id);
        printf("IF%d Node ID allocated: %d\n", dronecan.dna_interface, msg.node_id);

#if defined(HAL_PERIPH_ENABLE_GPS) && (HAL_NUM_CAN_IFACES >= 2) && GPS_MOVING_BASELINE
        if (periph.g.gps_mb_only_can_port) {
            // we need to assign the unallocated port to be used for Moving Baseline only
            periph.gps_mb_can_port = (dronecan.dna_interface+1)%HAL_NUM_CAN_IFACES;
            if (canardGetLocalNodeID(&dronecan.canard) == CANARD_BROADCAST_NODE_ID) {
                // copy node id from the primary iface
                canardSetLocalNodeID(&dronecan.canard, msg.node_id);
#ifdef HAL_GPIO_PIN_TERMCAN1
                // also terminate the line as we don't have any other device on this port
                palWriteLine(can_term_lines[periph.gps_mb_can_port], 1);
#endif
            }
        }
#endif
    }
}

/*
  handle ArmingStatus
 */
static void handle_arming_status(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_equipment_safety_ArmingStatus req;
    if (uavcan_equipment_safety_ArmingStatus_decode(transfer, &req)) {
        return;
    }
    hal.util->set_soft_armed(req.status == UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_STATUS_FULLY_ARMED);
}

#ifdef HAL_PERIPH_ENABLE_GPS
/*
  handle gnss::RTCMStream
 */
static void handle_RTCMStream(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_equipment_gnss_RTCMStream req;
    if (uavcan_equipment_gnss_RTCMStream_decode(transfer, &req)) {
        return;
    }
    periph.gps.handle_gps_rtcm_fragment(0, req.data.data, req.data.len);
}

/*
    handle gnss::MovingBaselineData
*/
#if GPS_MOVING_BASELINE
static void handle_MovingBaselineData(CanardInstance* ins, CanardRxTransfer* transfer)
{
    ardupilot_gnss_MovingBaselineData msg;
    if (ardupilot_gnss_MovingBaselineData_decode(transfer, &msg)) {
        return;
    }
    periph.gps.inject_MBL_data(msg.data.data, msg.data.len);
    Debug("MovingBaselineData: len=%u\n", msg.data.len);
}
#endif // GPS_MOVING_BASELINE

#endif // HAL_PERIPH_ENABLE_GPS


static void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue)
{
    periph.notify.handle_rgb(red, green, blue);
    periph.rcout_has_new_data_to_update = true;
}

/*
  handle lightscommand
 */
static void handle_lightscommand(CanardInstance* ins, CanardRxTransfer* transfer)
{
    periph.led_command_override();
    uavcan_equipment_indication_LightsCommand req;
    if (uavcan_equipment_indication_LightsCommand_decode(transfer, &req)) {
        return;
    }
    for (uint8_t i=0; i<req.commands.len; i++) {
        uavcan_equipment_indication_SingleLightCommand &cmd = req.commands.data[i];
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
static void handle_global_time_sync(CanardInstance* ins, CanardRxTransfer* transfer)
{
    uavcan_protocol_GlobalTimeSync msg;
    if (uavcan_protocol_GlobalTimeSync_decode(transfer, &msg)) {
        return;
    }
    if (periph.last_time_sync_usec != 0) {
        // calculate the time offset from master
        periph.time_offset_usec = periph.jitter.correct_offboard_timestamp_usec(periph.last_time_sync_usec, msg.previous_transmission_timestamp_usec);
    }
    periph.last_time_sync_usec = transfer->timestamp_usec;
}

static void handle_notify_state(CanardInstance* ins, CanardRxTransfer* transfer)
{
    ardupilot_indication_NotifyState msg;
    if (ardupilot_indication_NotifyState_decode(transfer, &msg)) {
        return;
    }
    if (msg.aux_data.len == 2 && msg.aux_data_type == ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_YAW_EARTH_CENTIDEGREES) {
        uint16_t tmp = 0;
        memcpy(&tmp, msg.aux_data.data, sizeof(tmp));
        periph.yaw_earth = radians((float)tmp * 0.01f);
    }
    periph.vehicle_state = msg.vehicle_state;
    periph.last_vehicle_state = AP_HAL::millis();
}

#ifdef HAL_GPIO_PIN_SAFE_LED
/*
  update safety LED
 */
static void can_safety_LED_update(void)
{
    static uint32_t last_update_ms;
    switch (safety_state) {
    case ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_OFF:
        palWriteLine(HAL_GPIO_PIN_SAFE_LED, SAFE_LED_ON);
        break;
    case ARDUPILOT_INDICATION_SAFETYSTATE_STATUS_SAFETY_ON: {
        uint32_t now = AP_HAL::native_millis();
        if (now - last_update_ms > 100) {
            last_update_ms = now;
            static uint8_t led_counter;
            const uint16_t led_pattern = 0x5500;
            led_counter = (led_counter+1) % 16;
            palWriteLine(HAL_GPIO_PIN_SAFE_LED, (led_pattern & (1U << led_counter))?!SAFE_LED_ON:SAFE_LED_ON);
        }
        break;
    }
    default:
        palWriteLine(HAL_GPIO_PIN_SAFE_LED, !SAFE_LED_ON);
        break;
    }
}
#endif // HAL_GPIO_PIN_SAFE_LED


#ifdef HAL_GPIO_PIN_SAFE_BUTTON
#ifndef HAL_SAFE_BUTTON_ON
#define HAL_SAFE_BUTTON_ON 1
#endif
/*
  update safety button
 */
static void can_safety_button_update(void)
{
    static uint32_t last_update_ms;
    static uint8_t counter;
    uint32_t now = AP_HAL::native_millis();
    // send at 10Hz when pressed
    if (palReadLine(HAL_GPIO_PIN_SAFE_BUTTON) != HAL_SAFE_BUTTON_ON) {
        counter = 0;
        return;
    }
    if (now - last_update_ms < 100) {
        return;
    }
    if (counter < 255) {
        counter++;
    }

    last_update_ms = now;
    ardupilot_indication_Button pkt {};
    pkt.button = ARDUPILOT_INDICATION_BUTTON_BUTTON_SAFETY;
    pkt.press_time = counter;

    uint8_t buffer[ARDUPILOT_INDICATION_BUTTON_MAX_SIZE] {};
    uint16_t total_size = ardupilot_indication_Button_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(ARDUPILOT_INDICATION_BUTTON_SIGNATURE,
                    ARDUPILOT_INDICATION_BUTTON_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}
#endif // HAL_GPIO_PIN_SAFE_BUTTON


#define ODID_COPY(name) pkt.name = msg.name
#define ODID_COPY_STR(name) do { strncpy_noterm((char*)pkt.name, (const char*)msg.name.data, sizeof(pkt.name)); } while(0)

#ifndef DRONEID_MODULE_CHAN
#define DRONEID_MODULE_CHAN MAVLINK_COMM_0
#endif
static void handle_remoteid_location(CanardInstance* ins, CanardRxTransfer* transfer)
{
    dronecan_remoteid_Location msg {};
    dronecan_remoteid_Location_decode(transfer, &msg);

    mavlink_open_drone_id_location_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(status);
    ODID_COPY(direction);
    ODID_COPY(speed_horizontal);
    ODID_COPY(speed_vertical);
    ODID_COPY(latitude);
    ODID_COPY(longitude);
    ODID_COPY(altitude_barometric);
    ODID_COPY(altitude_geodetic);
    ODID_COPY(height_reference);
    ODID_COPY(height);
    ODID_COPY(horizontal_accuracy);
    ODID_COPY(vertical_accuracy);
    ODID_COPY(barometer_accuracy);
    ODID_COPY(speed_accuracy);
    ODID_COPY(timestamp);
    ODID_COPY(timestamp_accuracy);
    mavlink_msg_open_drone_id_location_send_struct(DRONEID_MODULE_CHAN, &pkt);
}

static void handle_remoteid_basicid(CanardInstance* ins, CanardRxTransfer* transfer)
{
    dronecan_remoteid_BasicID msg {};
    dronecan_remoteid_BasicID_decode(transfer, &msg);

    mavlink_open_drone_id_basic_id_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(id_type);
    ODID_COPY(ua_type);
    ODID_COPY_STR(uas_id);
    mavlink_msg_open_drone_id_basic_id_send_struct(DRONEID_MODULE_CHAN, &pkt);
}

static void handle_remoteid_system(CanardInstance* ins, CanardRxTransfer* transfer)
{
    dronecan_remoteid_System msg {};
    dronecan_remoteid_System_decode(transfer, &msg);

    mavlink_open_drone_id_system_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(operator_location_type);
    ODID_COPY(classification_type);
    ODID_COPY(operator_latitude);
    ODID_COPY(operator_longitude);
    ODID_COPY(area_count);
    ODID_COPY(area_radius);
    ODID_COPY(area_ceiling);
    ODID_COPY(area_floor);
    ODID_COPY(category_eu);
    ODID_COPY(class_eu);
    ODID_COPY(operator_altitude_geo);
    ODID_COPY(timestamp);
    mavlink_msg_open_drone_id_system_send_struct((mavlink_channel_t)DRONEID_MODULE_CHAN, &pkt);
}

static void handle_remoteid_selfid(CanardInstance* ins, CanardRxTransfer* transfer)
{
    dronecan_remoteid_SelfID msg {};
    dronecan_remoteid_SelfID_decode(transfer, &msg);

    mavlink_open_drone_id_self_id_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(description_type);
    ODID_COPY_STR(description);
    mavlink_msg_open_drone_id_self_id_send_struct(DRONEID_MODULE_CHAN, &pkt);
}

static void handle_remoteid_operatorid(CanardInstance* ins, CanardRxTransfer* transfer)
{
    dronecan_remoteid_OperatorID msg {};
    dronecan_remoteid_OperatorID_decode(transfer, &msg);

    mavlink_open_drone_id_operator_id_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(operator_id_type);
    ODID_COPY_STR(operator_id);
    mavlink_msg_open_drone_id_operator_id_send_struct(DRONEID_MODULE_CHAN, &pkt);
}

void AP_Periph_FW::handle_open_drone_id_arm_status(mavlink_open_drone_id_arm_status_t &pkt)
{
    dronecan_remoteid_ArmStatus msg {};
    strncpy_noterm((char*)msg.error.data, pkt.error, sizeof(msg.error.data));
    msg.error.len = strnlen((const char*)msg.error.data, sizeof(msg.error.data));
    msg.status = pkt.status;

    uint8_t buffer[DRONECAN_REMOTEID_ARMSTATUS_MAX_SIZE];
    uint32_t len = dronecan_remoteid_ArmStatus_encode(&msg, buffer, !periph.canfdout());

    canard_broadcast(DRONECAN_REMOTEID_ARMSTATUS_SIGNATURE,
                    DRONECAN_REMOTEID_ARMSTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

/**
 * This callback is invoked by the library when a new message or request or response is received.
 */
static void onTransferReceived(CanardInstance* ins,
                               CanardRxTransfer* transfer)
{
#ifdef HAL_GPIO_PIN_LED_CAN1
    palToggleLine(HAL_GPIO_PIN_LED_CAN1);
#endif

    /*
     * Dynamic node ID allocation protocol.
     * Taking this branch only if we don't have a node ID, ignoring otherwise.
     */
    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID) {
        if (transfer->transfer_type == CanardTransferTypeBroadcast &&
            transfer->data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID) {
            handle_allocation_response(ins, transfer);
        }
        return;
    }

    switch (transfer->data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        handle_get_node_info(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        handle_begin_firmware_update(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        printf("RestartNode\n");
        hal.scheduler->delay(10);
        periph.prepare_reboot();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        NVIC_SystemReset();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        HAL_SITL::actually_reboot();
#endif
        break;

    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        handle_param_getset(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        handle_param_executeopcode(ins, transfer);
        break;

    case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID:
        handle_arming_status(ins, transfer);
        break;

#ifdef HAL_PERIPH_ENABLE_GPS
    case UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID:
        handle_RTCMStream(ins, transfer);
        break;

#if GPS_MOVING_BASELINE
    case ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID:
        handle_MovingBaselineData(ins, transfer);
        break;
#endif
#endif
        
    case UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID:
        handle_lightscommand(ins, transfer);
        break;

    case UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID:
        handle_global_time_sync(ins, transfer);
        break;

    case ARDUPILOT_INDICATION_NOTIFYSTATE_ID:
        handle_notify_state(ins, transfer);
        break;

    case DRONECAN_REMOTEID_LOCATION_ID:
        handle_remoteid_location(ins, transfer);
        break;

    case DRONECAN_REMOTEID_BASICID_ID:
        handle_remoteid_basicid(ins, transfer);
        break;

    case DRONECAN_REMOTEID_SYSTEM_ID:
        handle_remoteid_system(ins, transfer);
        break;

    case DRONECAN_REMOTEID_SELFID_ID:
        handle_remoteid_selfid(ins, transfer);
        break;
    
    case DRONECAN_REMOTEID_OPERATORID_ID:
        handle_remoteid_operatorid(ins, transfer);
        break;

    }
}


/**
 * This callback is invoked by the library when it detects beginning of a new transfer on the bus that can be received
 * by the local node.
 * If the callback returns true, the library will receive the transfer.
 * If the callback returns false, the library will ignore the transfer.
 * All transfers that are addressed to other nodes are always ignored.
 */
static bool shouldAcceptTransfer(const CanardInstance* ins,
                                 uint64_t* out_data_type_signature,
                                 uint16_t data_type_id,
                                 CanardTransferType transfer_type,
                                 uint8_t source_node_id)
{
    (void)source_node_id;

    if (canardGetLocalNodeID(ins) == CANARD_BROADCAST_NODE_ID)
    {
        /*
         * If we're in the process of allocation of dynamic node ID, accept only relevant transfers.
         */
        if ((transfer_type == CanardTransferTypeBroadcast) &&
            (data_type_id == UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID))
        {
            *out_data_type_signature = UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        return false;
    }

    switch (data_type_id) {
    case UAVCAN_PROTOCOL_GETNODEINFO_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_FILE_BEGINFIRMWAREUPDATE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_RESTARTNODE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_RESTARTNODE_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_PARAM_GETSET_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
        return true;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_SAFETY_ARMINGSTATUS_SIGNATURE;
        return true;
    case UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_INDICATION_LIGHTSCOMMAND_SIGNATURE;
        return true;

    case UAVCAN_PROTOCOL_GLOBALTIMESYNC_ID:
        *out_data_type_signature = UAVCAN_PROTOCOL_GLOBALTIMESYNC_SIGNATURE;
        return true;

#ifdef HAL_PERIPH_ENABLE_GPS
    case UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_ID:
        *out_data_type_signature = UAVCAN_EQUIPMENT_GNSS_RTCMSTREAM_SIGNATURE;
        return true;

#if GPS_MOVING_BASELINE
    case ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID:
        *out_data_type_signature = ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE;
        return true;
#endif
#endif
    case ARDUPILOT_INDICATION_NOTIFYSTATE_ID:
        *out_data_type_signature = ARDUPILOT_INDICATION_NOTIFYSTATE_SIGNATURE;
        return true;

    case DRONECAN_REMOTEID_LOCATION_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_LOCATION_SIGNATURE;
        return true;

    case DRONECAN_REMOTEID_BASICID_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_BASICID_SIGNATURE;
        return true;

    case DRONECAN_REMOTEID_SYSTEM_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_SYSTEM_SIGNATURE;
        return true;

    case DRONECAN_REMOTEID_SELFID_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_SELFID_SIGNATURE;
        return true;

    case DRONECAN_REMOTEID_OPERATORID_ID:
        *out_data_type_signature = DRONECAN_REMOTEID_OPERATORID_SIGNATURE;
        return true;

    default:
        break;
    }

    return false;
}

static void cleanup_stale_transactions(uint64_t &timestamp_usec)
{
    canardCleanupStaleTransfers(&dronecan.canard, timestamp_usec);
}

#define MAKE_TRANSFER_DESCRIPTOR(data_type_id, transfer_type, src_node_id, dst_node_id)             \
    (((uint32_t)(data_type_id)) | (((uint32_t)(transfer_type)) << 16U) |                            \
    (((uint32_t)(src_node_id)) << 18U) | (((uint32_t)(dst_node_id)) << 25U))

static uint8_t* get_tid_ptr(uint32_t transfer_desc)
{
    // check head
    if (!dronecan.tid_map_head) {
        dronecan.tid_map_head = (dronecan_protocol_t::tid_map*)calloc(1, sizeof(dronecan_protocol_t::tid_map));
        if (dronecan.tid_map_head == nullptr) {
            return nullptr;
        }
        dronecan.tid_map_head->transfer_desc = transfer_desc;
        dronecan.tid_map_head->next = nullptr;
        return &dronecan.tid_map_head->tid;
    } else if (dronecan.tid_map_head->transfer_desc == transfer_desc) {
        return &dronecan.tid_map_head->tid;
    }

    // search through the list for an existing entry
    dronecan_protocol_t::tid_map *tid_map_ptr = dronecan.tid_map_head;
    while(tid_map_ptr->next) {
        tid_map_ptr = tid_map_ptr->next;
        if (tid_map_ptr->transfer_desc == transfer_desc) {
            return &tid_map_ptr->tid;
        }
    }

    // create a new entry, if not found
    tid_map_ptr->next = (dronecan_protocol_t::tid_map*)calloc(1, sizeof(dronecan_protocol_t::tid_map));
    if (tid_map_ptr->next == nullptr) {
        return nullptr;
    }
    tid_map_ptr->next->transfer_desc = transfer_desc;
    tid_map_ptr->next->next = nullptr;
    return &tid_map_ptr->next->tid;
}

static void canard_broadcast(uint64_t data_type_signature,
                                uint16_t data_type_id,
                                uint8_t priority,
                                const void* payload,
                                uint16_t payload_len)
{
    if (canardGetLocalNodeID(&dronecan.canard) == CANARD_BROADCAST_NODE_ID) {
        return;
    }

    uint8_t *tid_ptr = get_tid_ptr(MAKE_TRANSFER_DESCRIPTOR(data_type_signature, data_type_id, 0, CANARD_BROADCAST_NODE_ID));
    if (tid_ptr == nullptr) {
        return;
    }
#if DEBUG_PKTS
    const int16_t res = 
#endif
    canardBroadcast(&dronecan.canard,
                    data_type_signature,
                    data_type_id,
                    tid_ptr,
                    priority,
                    payload,
                    payload_len,
                    IFACE_ALL, // send over all ifaces
                    periph.canfdout());
#if DEBUG_PKTS
    if (res < 0) {
        can_printf("Tx error %d\n", res);
    }
#endif
}

static void processTx(void)
{
    for (const CanardCANFrame* txf = NULL; (txf = canardPeekTxQueue(&dronecan.canard)) != NULL;) {
        AP_HAL::CANFrame txmsg {};
        txmsg.dlc = AP_HAL::CANFrame::dataLengthToDlc(txf->data_len);
        memcpy(txmsg.data, txf->data, txf->data_len);
        txmsg.id = (txf->id | AP_HAL::CANFrame::FlagEFF);
#if HAL_CANFD_SUPPORTED
        txmsg.canfd = txf->canfd;
#endif
        // push message with 1s timeout
        bool sent = true;
        const uint64_t deadline = AP_HAL::native_micros64() + 1000000;
        // try sending to all interfaces
        for (auto &ins : instances) {
            if (ins.iface == NULL) {
                continue;
            }
            if (!(txf->iface_mask & (1<<ins.index))) {
                continue;
            }
    #if HAL_NUM_CAN_IFACES >= 2
            if (periph.can_protocol_cached[ins.index] != AP_CANManager::Driver_Type_UAVCAN) {
                continue;
            }
    #endif
            if (ins.iface->send(txmsg, deadline, 0) <= 0) {
                sent = false;
            }
        }
        if (sent) {
            canardPopTxQueue(&dronecan.canard);
            dronecan.tx_fail_count = 0;
        } else {
            // just exit and try again later. If we fail 8 times in a row
            // then start discarding to prevent the pool filling up
            if (dronecan.tx_fail_count < 8) {
                dronecan.tx_fail_count++;
            } else {
                canardPopTxQueue(&dronecan.canard);
            }
            break;
        }
    }
}

static void processRx(void)
{
    AP_HAL::CANFrame rxmsg;
    for (auto &ins : instances) {
        if (ins.iface == NULL) {
            continue;
        }
#if HAL_NUM_CAN_IFACES >= 2
        if (periph.can_protocol_cached[ins.index] != AP_CANManager::Driver_Type_UAVCAN) {
            continue;
        }
#endif
        while (true) {
            bool read_select = true;
            bool write_select = false;
            ins.iface->select(read_select, write_select, nullptr, 0);
            if (!read_select) { // No data pending
                break;
            }
            CanardCANFrame rx_frame {};

            //palToggleLine(HAL_GPIO_PIN_LED);
            uint64_t timestamp;
            AP_HAL::CANIface::CanIOFlags flags;
            if (ins.iface->receive(rxmsg, timestamp, flags) <= 0) {
                break;
            }
            rx_frame.data_len = AP_HAL::CANFrame::dlcToDataLength(rxmsg.dlc);
            rx_frame.canfd = rxmsg.canfd;
            memcpy(rx_frame.data, rxmsg.data, rx_frame.data_len);
            rx_frame.id = rxmsg.id;
            rx_frame.iface_id = ins.index;
#if DEBUG_PKTS
            const int16_t res = 
#endif
            canardHandleRxFrame(&dronecan.canard, &rx_frame, timestamp);
#if DEBUG_PKTS
            if (res < 0 &&
                res != -CANARD_ERROR_RX_NOT_WANTED &&
                res != -CANARD_ERROR_RX_WRONG_ADDRESS &&
                res != -CANARD_ERROR_RX_MISSED_START) {
                printf("Rx error %d, IF%d %lx: ", res, ins.index, rx_frame.id);
                for (uint8_t i = 0; i < rx_frame.data_len; i++) {
                    printf("%02x", rx_frame.data[i]);
                }
                printf("\n");
            }
#endif
        }
    }
}

static uint16_t pool_peak_percent()
{
    const CanardPoolAllocatorStatistics stats = canardGetPoolAllocatorStatistics(&dronecan.canard);
    const uint16_t peak_percent = (uint16_t)(100U * stats.peak_usage_blocks / stats.capacity_blocks);
    return peak_percent;
}

static void node_status_send(void)
{
    uint8_t buffer[UAVCAN_PROTOCOL_NODESTATUS_MAX_SIZE];
    node_status.uptime_sec = AP_HAL::millis() / 1000U;

    node_status.vendor_specific_status_code = hal.util->available_memory();

    uint32_t len = uavcan_protocol_NodeStatus_encode(&node_status, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}


/**
 * This function is called at 1 Hz rate from the main loop.
 */
static void process1HzTasks(uint64_t timestamp_usec)
{
    /*
     * Purging transfers that are no longer transmitted. This will occasionally free up some memory.
     */
    cleanup_stale_transactions(timestamp_usec);

    /*
     * Printing the memory usage statistics.
     */
    {
        /*
         * The recommended way to establish the minimal size of the memory pool is to stress-test the application and
         * record the worst case memory usage.
         */
        for (auto &ins : instances) {
            if (pool_peak_percent() > 70) {
                printf("WARNING: ENLARGE MEMORY POOL on Iface %d\n", ins.index);
            }
        }
    }

    /*
     * Transmitting the node status message periodically.
     */
    node_status_send();

#if !defined(HAL_NO_FLASH_SUPPORT) && !defined(HAL_NO_ROMFS_SUPPORT)
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
#endif

    node_status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

#if 0
    // test code for watchdog reset
    if (AP_HAL::native_millis() > 15000) {
        while (true) ;
    }
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    if (AP_HAL::native_millis() > 30000) {
        // use RTC to mark that we have been running fine for
        // 30s. This is used along with watchdog resets to ensure the
        // user has a chance to load a fixed firmware
        set_fast_reboot(RTC_BOOT_FWOK);
    }
#endif
}

/*
  wait for dynamic allocation of node ID
 */
bool AP_Periph_FW::no_iface_finished_dna = true;
static bool can_do_dna()
{
    if (canardGetLocalNodeID(&dronecan.canard) != CANARD_BROADCAST_NODE_ID) {
        AP_Periph_FW::no_iface_finished_dna = false;
        return true;
    }

    const uint32_t now = AP_HAL::native_millis();

    static uint8_t node_id_allocation_transfer_id = 0;

    if (AP_Periph_FW::no_iface_finished_dna) {
        printf("Waiting for dynamic node ID allocation... (pool %u)\n", pool_peak_percent());
    }

    dronecan.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        get_random_range(UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    // Structure of the request is documented in the DSDL definition
    // See http://uavcan.org/Specification/6._Application_level_functions/#dynamic-node-id-allocation
    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = (uint8_t)(PreferredNodeID << 1U);

    if (dronecan.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;     // First part of unique ID
        // set interface to try
        dronecan.dna_interface++;
        dronecan.dna_interface %= HAL_NUM_CAN_IFACES;
    }

    uint8_t my_unique_id[sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data)];
    readUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size = (uint8_t)(sizeof(uavcan_protocol_dynamic_node_id_Allocation::unique_id.data) - dronecan.node_id_allocation_unique_id_offset);
    
    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1], &my_unique_id[dronecan.node_id_allocation_unique_id_offset], uid_size);

    // Broadcasting the request
    const int16_t bcast_res = canardBroadcast(&dronecan.canard,
                                                UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
                                                UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
                                                &node_id_allocation_transfer_id,
                                                CANARD_TRANSFER_PRIORITY_LOW,
                                                &allocation_request[0],
                                                (uint16_t) (uid_size + 1),
                                                (1<<dronecan.dna_interface),
                                                false);
    if (bcast_res < 0) {
        printf("Could not broadcast ID allocation req; error %d\n", bcast_res);
    }

    // Preparing for timeout; if response is received, this value will be updated from the callback.
    dronecan.node_id_allocation_unique_id_offset = 0;
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
        if (g.can_protocol[i] == AP_CANManager::Driver_Type_UAVCAN && g.can_baudrate[i] == 1000000) {
            has_uavcan_at_1MHz = true;
        }
    }
    if (!has_uavcan_at_1MHz) {
        g.can_protocol[0].set_and_save(AP_CANManager::Driver_Type_UAVCAN);
        g.can_baudrate[0].set_and_save(1000000);
    }
#endif // HAL_PERIPH_ENFORCE_AT_LEAST_ONE_PORT_IS_UAVCAN_1MHz

#ifdef HAL_GPIO_PIN_TERMCAN1
    for (uint8_t i=0; i<ARRAY_SIZE(can_term_lines); i++){
        palWriteLine(can_term_lines[i], g.can_terminator[i]);
    }
#endif

    for (uint8_t i=0; i<HAL_NUM_CAN_IFACES; i++) {
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        can_iface_periph[i] = new ChibiOS::CANIface();
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
        can_iface_periph[i] = new HALSITL::CANIface();
#endif
        instances[i].iface = can_iface_periph[i];
        instances[i].index = i;
#if HAL_NUM_CAN_IFACES >= 2
        can_protocol_cached[i] = g.can_protocol[i];
        CANSensor::set_periph(i, can_protocol_cached[i], can_iface_periph[i]);
#endif
        if (can_iface_periph[i] != nullptr) {
            if (canfdout()) {
                can_iface_periph[i]->init(g.can_baudrate[i],  g.can_fdbaudrate[i], AP_HAL::CANIface::NormalMode);
            } else {
                can_iface_periph[i]->init(g.can_baudrate[i], AP_HAL::CANIface::NormalMode);
            }
        }
    }
    canardInit(&dronecan.canard, (uint8_t *)dronecan.canard_memory_pool, sizeof(dronecan.canard_memory_pool),
            onTransferReceived, shouldAcceptTransfer, NULL);

    if (PreferredNodeID != CANARD_BROADCAST_NODE_ID) {
        canardSetLocalNodeID(&dronecan.canard, PreferredNodeID);
    }

}

#ifdef HAL_PERIPH_ENABLE_PWM_HARDPOINT
void AP_Periph_FW::pwm_hardpoint_init()
{
    hal.gpio->attach_interrupt(
        PWM_HARDPOINT_PIN,
        FUNCTOR_BIND_MEMBER(&AP_Periph_FW::pwm_irq_handler, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH);

}

/*
  called on PWM pin transition
 */
void AP_Periph_FW::pwm_irq_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    if (pin_state == 0 && pwm_hardpoint.last_state == 1 && pwm_hardpoint.last_ts_us != 0) {
        uint32_t width = timestamp - pwm_hardpoint.last_ts_us;
        if (width > 500 && width < 2500) {
            pwm_hardpoint.pwm_value = width;
            if (width > pwm_hardpoint.highest_pwm) {
                pwm_hardpoint.highest_pwm = width;
            }
        }
    }
    pwm_hardpoint.last_state = pin_state;
    pwm_hardpoint.last_ts_us = timestamp;
}

void AP_Periph_FW::pwm_hardpoint_update()
{
    uint32_t now = AP_HAL::native_millis();
    // send at 10Hz
    void *save = hal.scheduler->disable_interrupts_save();
    uint16_t value = pwm_hardpoint.highest_pwm;
    pwm_hardpoint.highest_pwm = 0;
    hal.scheduler->restore_interrupts(save);
    float rate = g.hardpoint_rate;
    rate = constrain_float(rate, 10, 100);
    if (value > 0 && now - pwm_hardpoint.last_send_ms >= 1000U/rate) {
        pwm_hardpoint.last_send_ms = now;
        uavcan_equipment_hardpoint_Command cmd {};
        cmd.hardpoint_id = g.hardpoint_id;
        cmd.command = value;

        uint8_t buffer[UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_hardpoint_Command_encode(&cmd, buffer, !periph.canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_SIGNATURE,
                        UAVCAN_EQUIPMENT_HARDPOINT_COMMAND_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
}
#endif // HAL_PERIPH_ENABLE_PWM_HARDPOINT

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

    if (AP_HAL::millis() > dronecan.send_next_node_id_allocation_request_at_ms) {
        can_do_dna();
    }
    
    static uint32_t last_1Hz_ms;
    if (now - last_1Hz_ms >= 1000) {
        last_1Hz_ms = now;
        process1HzTasks(AP_HAL::native_micros64());
    }
    if (!g.serial_i2c_mode) {
        can_gps_update();
    } else {
        // update LEDs as well
        if (i2c_new_led_data) {
            i2c_new_led_data = false;
            set_rgb_led(i2c_led_color_red, i2c_led_color_green, i2c_led_color_blue);
        }
    }
    
    can_mag_update();
    can_baro_update();

#ifdef HAL_GPIO_PIN_SAFE_LED
    can_safety_LED_update();
#endif
#ifdef HAL_GPIO_PIN_SAFE_BUTTON
    can_safety_button_update();
#endif
    rcout_update();

    if (!g.serial_i2c_mode) {
        can_imu_update();
    }
#if !HAL_INS_ENABLED // we wait in INS method otherwise
    const uint32_t now_us = AP_HAL::micros();
    while ((AP_HAL::micros() - now_us) < 1000) {
        hal.scheduler->delay_microseconds(HAL_PERIPH_LOOP_DELAY_US);
#else
    {
#endif
        processTx();
        processRx();
    }
}

/*
    update IMU
*/
void AP_Periph_FW::can_imu_update(void)
{
#if HAL_INS_ENABLED
    if (imu.get_accel_count()) {
        imu.update();
    } else {
        return;
    }
    static uint8_t pkt_count;//, update_count;
    // update_count++;
    // if (update_count % 10 != 0) {
    //     return;
    // }
    dronecan_sensors_inertial_PointValues pkt {};
    const Vector3f accel_pnt = imu.get_accel(0);
    const Vector3f gyro_pnt = imu.get_gyro(0);

    for (uint8_t i=0; i<3; i++) {
        pkt.accel[i] = accel_pnt[i];
        pkt.gyro[i] = gyro_pnt[i];
    }
    pkt.count = pkt_count++;

    uint8_t buffer[DRONECAN_SENSORS_INERTIAL_POINTVALUES_ID] {};
    uint16_t total_size = dronecan_sensors_inertial_PointValues_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(DRONECAN_SENSORS_INERTIAL_POINTVALUES_SIGNATURE,
                    DRONECAN_SENSORS_INERTIAL_POINTVALUES_ID,
                    CANARD_TRANSFER_PRIORITY_HIGH,
                    &buffer[0],
                    total_size);
#endif
}
/*
  update CAN magnetometer
 */
void AP_Periph_FW::can_mag_update(void)
{
#ifdef HAL_PERIPH_ENABLE_MAG
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

    if (last_mag_update_ms == compass.last_update_ms()) {
        return;
    }
    if (!compass.healthy()) {
        return;
    }

    last_mag_update_ms = compass.last_update_ms();
    const Vector3f &field = compass.get_field();
    uavcan_equipment_ahrs_MagneticFieldStrength pkt {};

    // the canard dsdl compiler doesn't understand float16
    for (uint8_t i=0; i<3; i++) {
        pkt.magnetic_field_ga[i] = field[i] * 0.001;
    }

    uint8_t buffer[UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_MAX_SIZE] {};
    uint16_t total_size = uavcan_equipment_ahrs_MagneticFieldStrength_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_SIGNATURE,
                    UAVCAN_EQUIPMENT_AHRS_MAGNETICFIELDSTRENGTH_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_MAG
}

/*
  update CAN battery monitor
 */
void AP_Periph_FW::can_battery_update(void)
{
#ifdef HAL_PERIPH_ENABLE_BATTERY
    const uint32_t now_ms = AP_HAL::native_millis();
    if (now_ms - battery.last_can_send_ms < 100) {
        return;
    }
    battery.last_can_send_ms = now_ms;

    const uint8_t battery_instances = battery.lib.num_instances();
    for (uint8_t i=0; i<battery_instances; i++) {
        if (!battery.lib.healthy(i)) {
            continue;
        }

        uavcan_equipment_power_BatteryInfo pkt {};

        // if a battery serial number is assigned, use that as the ID. Else, use the index.
        const int32_t serial_number = battery.lib.get_serial_number(i);
        pkt.battery_id = (serial_number >= 0) ? serial_number : i+1;

        pkt.voltage = battery.lib.voltage(i);

        float current;
        if (battery.lib.current_amps(current, i)) {
            pkt.current = current;
        }
        float temperature;
        if (battery.lib.get_temperature(temperature, i)) {
            // Battery lib reports temperature in Celsius.
            // Convert Celsius to Kelvin for tranmission on CAN.
            pkt.temperature = temperature + C_TO_KELVIN;
        }

        pkt.state_of_health_pct = UAVCAN_EQUIPMENT_POWER_BATTERYINFO_STATE_OF_HEALTH_UNKNOWN;
        uint8_t percentage = 0;
        if (battery.lib.capacity_remaining_pct(percentage, i)) {
            pkt.state_of_charge_pct = percentage;
        }
        pkt.model_instance_id = i+1;

#if !defined(HAL_PERIPH_BATTERY_SKIP_NAME)
        // example model_name: "org.ardupilot.ap_periph SN 123"
        hal.util->snprintf((char*)pkt.model_name.data, sizeof(pkt.model_name.data), "%s %ld", AP_PERIPH_BATTERY_MODEL_NAME, (long int)serial_number);
        pkt.model_name.len = strnlen((char*)pkt.model_name.data, sizeof(pkt.model_name.data));
#endif //defined(HAL_PERIPH_BATTERY_SKIP_NAME)

        uint8_t buffer[UAVCAN_EQUIPMENT_POWER_BATTERYINFO_MAX_SIZE] {};
        const uint16_t total_size = uavcan_equipment_power_BatteryInfo_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_POWER_BATTERYINFO_SIGNATURE,
                        UAVCAN_EQUIPMENT_POWER_BATTERYINFO_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
#endif
}

/*
  update CAN GPS
 */
void AP_Periph_FW::can_gps_update(void)
{
#ifdef HAL_PERIPH_ENABLE_GPS
    if (gps.get_type(0) == AP_GPS::GPS_Type::GPS_TYPE_NONE) {
        return;
    }
    gps.update();
    send_moving_baseline_msg();
    send_relposheading_msg();
    if (last_gps_update_ms == gps.last_message_time_ms()) {
        return;
    }
    last_gps_update_ms = gps.last_message_time_ms();

    {
        /*
          send Fix2 packet
        */
        uavcan_equipment_gnss_Fix2 pkt {};
        const Location &loc = gps.location();
        const Vector3f &vel = gps.velocity();

        if (gps.status() < AP_GPS::GPS_OK_FIX_2D && !saw_lock_once) {
            pkt.timestamp.usec = AP_HAL::micros64();
            pkt.gnss_timestamp.usec = 0;
        } else {
            saw_lock_once = true;
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

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_FIX2_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_gnss_Fix2_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_GNSS_FIX2_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_FIX2_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
    
    /*
      send aux packet
     */
    {
        uavcan_equipment_gnss_Auxiliary aux {};
        aux.hdop = gps.get_hdop() * 0.01;
        aux.vdop = gps.get_vdop() * 0.01;

        uint8_t buffer[UAVCAN_EQUIPMENT_GNSS_AUXILIARY_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_gnss_Auxiliary_encode(&aux, buffer, !periph.canfdout());
        canard_broadcast(UAVCAN_EQUIPMENT_GNSS_AUXILIARY_SIGNATURE,
                        UAVCAN_EQUIPMENT_GNSS_AUXILIARY_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
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

        uint8_t buffer[ARDUPILOT_GNSS_STATUS_MAX_SIZE] {};
        const uint16_t total_size = ardupilot_gnss_Status_encode(&status, buffer, !periph.canfdout());
        canard_broadcast(ARDUPILOT_GNSS_STATUS_SIGNATURE,
                        ARDUPILOT_GNSS_STATUS_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);

    }
#endif // HAL_PERIPH_ENABLE_GPS
}


void AP_Periph_FW::send_moving_baseline_msg()
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
    uint8_t buffer[ARDUPILOT_GNSS_MOVINGBASELINEDATA_MAX_SIZE] {};
    const uint16_t total_size = ardupilot_gnss_MovingBaselineData_encode(&mbldata, buffer, !periph.canfdout());

#if HAL_NUM_CAN_IFACES >= 2
    if (gps_mb_can_port != -1 && (gps_mb_can_port < HAL_NUM_CAN_IFACES)) {
        uint8_t *tid_ptr = get_tid_ptr(MAKE_TRANSFER_DESCRIPTOR(ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE, ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID, 0, CANARD_BROADCAST_NODE_ID));
        canardBroadcast(&dronecan.canard,
                        ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE,
                        ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID,
                        tid_ptr,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size,
                        (1<<gps_mb_can_port),
                        periph.canfdout());
    } else 
#endif
    {
        canard_broadcast(ARDUPILOT_GNSS_MOVINGBASELINEDATA_SIGNATURE,
                        ARDUPILOT_GNSS_MOVINGBASELINEDATA_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
    gps.clear_RTCMV3();
#endif // HAL_PERIPH_ENABLE_GPS && GPS_MOVING_BASELINE
}

void AP_Periph_FW::send_relposheading_msg() {
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
    uint8_t buffer[ARDUPILOT_GNSS_RELPOSHEADING_MAX_SIZE] {};
    const uint16_t total_size = ardupilot_gnss_RelPosHeading_encode(&relpos, buffer, !periph.canfdout());
    canard_broadcast(ARDUPILOT_GNSS_RELPOSHEADING_SIGNATURE,
                    ARDUPILOT_GNSS_RELPOSHEADING_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_GPS && GPS_MOVING_BASELINE
}

/*
  update CAN baro
 */
void AP_Periph_FW::can_baro_update(void)
{
#ifdef HAL_PERIPH_ENABLE_BARO
    if (!periph.g.baro_enable) {
        return;
    }
    baro.update();
    if (last_baro_update_ms == baro.get_last_update()) {
        return;
    }

    last_baro_update_ms = baro.get_last_update();
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

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_air_data_StaticPressure_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICPRESSURE_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }

    {
        uavcan_equipment_air_data_StaticTemperature pkt {};
        pkt.static_temperature = C_TO_KELVIN(temp);
        pkt.static_temperature_variance = 0; // should we make this a parameter?

        uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_MAX_SIZE] {};
        uint16_t total_size = uavcan_equipment_air_data_StaticTemperature_encode(&pkt, buffer, !periph.canfdout());

        canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_SIGNATURE,
                        UAVCAN_EQUIPMENT_AIR_DATA_STATICTEMPERATURE_ID,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        &buffer[0],
                        total_size);
    }
#endif // HAL_PERIPH_ENABLE_BARO
}


/*
  update CAN airspeed
 */
void AP_Periph_FW::can_airspeed_update(void)
{
#ifdef HAL_PERIPH_ENABLE_AIRSPEED
    if (!airspeed.enabled()) {
        return;
    }
#if CAN_PROBE_CONTINUOUS
    if (!airspeed.healthy()) {
        uint32_t now = AP_HAL::native_millis();
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            airspeed.init();
        }
    }
#endif
    uint32_t now = AP_HAL::native_millis();
    if (now - last_airspeed_update_ms < 50) {
        // max 20Hz data
        return;
    }
    last_airspeed_update_ms = now;
    airspeed.update(false);
    if (!airspeed.healthy()) {
        // don't send any data
        return;
    }
    const float press = airspeed.get_corrected_pressure();
    float temp;
    if (!airspeed.get_temperature(temp)) {
        temp = nanf("");
    } else {
        temp += C_TO_KELVIN;
    }

    uavcan_equipment_air_data_RawAirData pkt {};
    pkt.differential_pressure = press;
    pkt.static_air_temperature = temp;

    // unfilled elements are NaN
    pkt.static_pressure = nanf("");
    pkt.static_pressure_sensor_temperature = nanf("");
    pkt.differential_pressure_sensor_temperature = nanf("");
    pkt.pitot_temperature = nanf("");

    uint8_t buffer[UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_MAX_SIZE] {};
    uint16_t total_size = uavcan_equipment_air_data_RawAirData_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_SIGNATURE,
                    UAVCAN_EQUIPMENT_AIR_DATA_RAWAIRDATA_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_AIRSPEED
}


/*
  update CAN rangefinder
 */
void AP_Periph_FW::can_rangefinder_update(void)
{
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
    if (rangefinder.get_type(0) == RangeFinder::Type::NONE) {
        return;
    }
#if CAN_PROBE_CONTINUOUS
    if (rangefinder.num_sensors() == 0) {
        uint32_t now = AP_HAL::native_millis();
        static uint32_t last_probe_ms;
        if (now - last_probe_ms >= 1000) {
            last_probe_ms = now;
            rangefinder.init(ROTATION_NONE);
        }
    }
#endif
    uint32_t now = AP_HAL::native_millis();
    static uint32_t last_update_ms;
    if (now - last_update_ms < 20) {
        // max 50Hz data
        return;
    }
    last_update_ms = now;
    rangefinder.update();
    RangeFinder::Status status = rangefinder.status_orient(ROTATION_NONE);
    if (status <= RangeFinder::Status::NoData) {
        // don't send any data
        return;
    }
    uint16_t dist_cm = rangefinder.distance_cm_orient(ROTATION_NONE);
    uavcan_equipment_range_sensor_Measurement pkt {};
    pkt.sensor_id = rangefinder.get_address(0);
    switch (status) {
    case RangeFinder::Status::OutOfRangeLow:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_CLOSE;
        break;
    case RangeFinder::Status::OutOfRangeHigh:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_TOO_FAR;
        break;
    case RangeFinder::Status::Good:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_VALID_RANGE;
        break;
    default:
        pkt.reading_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_READING_TYPE_UNDEFINED;
        break;
    }
    switch (rangefinder.get_mav_distance_sensor_type_orient(ROTATION_NONE)) {
    case MAV_DISTANCE_SENSOR_LASER:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_LIDAR;
        break;
    case MAV_DISTANCE_SENSOR_ULTRASOUND:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_SONAR;
        break;
    case MAV_DISTANCE_SENSOR_RADAR:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_RADAR;
        break;
    default:
        pkt.sensor_type = UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SENSOR_TYPE_UNDEFINED;
        break;
    }

    pkt.range = dist_cm * 0.01;

    uint8_t buffer[UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_MAX_SIZE] {};
    uint16_t total_size = uavcan_equipment_range_sensor_Measurement_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_SIGNATURE,
                    UAVCAN_EQUIPMENT_RANGE_SENSOR_MEASUREMENT_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
#endif // HAL_PERIPH_ENABLE_RANGEFINDER
}


#ifdef HAL_PERIPH_ENABLE_ADSB
/*
  map an ADSB_VEHICLE MAVLink message to a UAVCAN TrafficReport message
 */
void AP_Periph_FW::can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg)
{
    ardupilot_equipment_trafficmonitor_TrafficReport pkt {};
    pkt.timestamp.usec = 0;
    pkt.icao_address = msg.ICAO_address;
    pkt.tslc = msg.tslc;
    pkt.latitude_deg_1e7 = msg.lat;
    pkt.longitude_deg_1e7 = msg.lon;
    pkt.alt_m = msg.altitude * 1e-3;

    pkt.heading = radians(msg.heading * 1e-2);

    pkt.velocity[0] = cosf(pkt.heading) * msg.hor_velocity * 1e-2;
    pkt.velocity[1] = sinf(pkt.heading) * msg.hor_velocity * 1e-2;
    pkt.velocity[2] = -msg.ver_velocity * 1e-2;

    pkt.squawk = msg.squawk;
    memcpy(pkt.callsign, msg.callsign, MIN(sizeof(msg.callsign),sizeof(pkt.callsign)));
    if (msg.flags & 0x8000) {
        pkt.source = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SOURCE_ADSB_UAT;
    } else {
        pkt.source = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SOURCE_ADSB;
    }

    pkt.traffic_type = msg.emitter_type;

    if ((msg.flags & ADSB_FLAGS_VALID_ALTITUDE) != 0 && msg.altitude_type == 0) {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_PRESSURE_AMSL;
    } else if ((msg.flags & ADSB_FLAGS_VALID_ALTITUDE) != 0 && msg.altitude_type == 1) {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_WGS84;
    } else {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_ALT_UNKNOWN;
    }

    pkt.lat_lon_valid = (msg.flags & ADSB_FLAGS_VALID_COORDS) != 0;
    pkt.heading_valid = (msg.flags & ADSB_FLAGS_VALID_HEADING) != 0;
    pkt.velocity_valid = (msg.flags & ADSB_FLAGS_VALID_VELOCITY) != 0;
    pkt.callsign_valid = (msg.flags & ADSB_FLAGS_VALID_CALLSIGN) != 0;
    pkt.ident_valid = (msg.flags & ADSB_FLAGS_VALID_SQUAWK) != 0;
    pkt.simulated_report = (msg.flags & ADSB_FLAGS_SIMULATED) != 0;

    // these flags are not in common.xml
    pkt.vertical_velocity_valid = (msg.flags & 0x0080) != 0;
    pkt.baro_valid = (msg.flags & 0x0100) != 0;

    uint8_t buffer[ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_MAX_SIZE] {};
    uint16_t total_size = ardupilot_equipment_trafficmonitor_TrafficReport_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SIGNATURE,
                    ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}
#endif // HAL_PERIPH_ENABLE_ADSB

// printf to CAN LogMessage for debugging
void can_printf(const char *fmt, ...)
{
    uavcan_protocol_debug_LogMessage pkt {};
    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE] {};
    va_list ap;
    va_start(ap, fmt);
    uint32_t n = vsnprintf((char*)pkt.text.data, sizeof(pkt.text.data), fmt, ap);
    va_end(ap);
    pkt.text.len = MIN(n, sizeof(pkt.text.data));

    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);

}
