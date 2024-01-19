#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include "SRV_Channel/SRV_Channel.h"
#include <AP_Notify/AP_Notify.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_MSP/AP_MSP.h>
#include <AP_MSP/msp.h>
#include "../AP_Bootloader/app_comms.h"
#include <AP_CheckFirmware/AP_CheckFirmware.h>
#include <AP_CANManager/AP_CANManager.h>
#include <AP_Scripting/AP_Scripting.h>
#include <AP_InertialSensor/AP_InertialSensor.h>

#include <AP_RTC/JitterCorrection.h>
#include <AP_HAL/CANIface.h>
#include <AP_HAL_ChibiOS/EventSource.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <ch.h>
#include <AP_DroneCAN/AP_Canard_iface.h>
#include <dronecan_msgs.h>
#include <canard.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <AP_HAL/AP_HAL.h>
#include "MAVLink.h"
#include "GPS_Base.h"

#if defined(HAL_PERIPH_ENABLE_BATTERY_MPPT_PACKETDIGITAL) && HAL_MAX_CAN_PROTOCOL_DRIVERS < 2
#error "Battery MPPT PacketDigital driver requires at least two CAN Ports"
#endif


#include "Parameters.h"

#define LED_CONNECTED_BRIGHTNESS 10 // 10%

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init();
void stm32_watchdog_pat();
#endif
/*
  app descriptor compatible with MissionPlanner
 */
extern const app_descriptor_t app_descriptor;

class AP_Periph_DroneCAN;

class AP_Periph_FW {
public:
    AP_Periph_FW();

    CLASS_NO_COPY(AP_Periph_FW);

    static AP_Periph_FW* get_singleton()
    {
        if (_singleton == nullptr) {
            AP_HAL::panic("AP_Periph_FW used before allocation.");
        }
        return _singleton;
    }

    void init();
    void update();

    Parameters g;

    void can_start();
    void can_update();
    bool can_do_dna();
    uint32_t send_next_node_id_allocation_request_at_ms;
    uint8_t node_id_allocation_unique_id_offset;

    void load_parameters();
    void prepare_reboot();

    bool canfdout() const { return (g.can_fdmode == 1); }

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
    void check_for_serial_reboot_cmd(const int8_t serial_index);
    void check_for_serial_reboot_cmd_byte(uint8_t data);
#endif
    void gpio_passthrough_isr(uint8_t pin, bool pin_state, uint32_t timestamp);

    static ChibiOS::CANIface* can_iface_periph[HAL_NUM_CAN_IFACES];

    AP_SerialManager serial_manager;

    AP_GPS gps;

#ifdef HAL_PERIPH_ENABLE_MAG
    Compass compass;
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    AP_Baro baro;
#endif

    SRV_Channels servo_channels;
    bool rcout_has_new_data_to_update;
    void rcout_update();

    void update_rainbow();

    // notification object for LEDs, buzzers etc
    AP_Notify notify;
    uint64_t vehicle_state = 1; // default to initialisation
    float yaw_earth;
    uint32_t last_vehicle_state;
    bool led_cmd_override = false;
    // Handled under LUA script to control LEDs
    void led_command_override() { led_cmd_override = true; }

    void toshibaled_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte);

#ifdef I2C_SLAVE_ENABLED
    void i2c_setup();
#endif
    uint8_t compass_reg;
    struct reg_list {
        struct reg_list *next;
        uint8_t reg;
        uint8_t val;
        bool updated;
    } *reg_list_head;

    void compass_recv_byte(uint8_t idx, uint8_t byte);
    uint8_t compass_send_byte(uint8_t reg);
    void compass_register_rw_callback(uint8_t reg, uint8_t *data, uint32_t len, bool is_write);

    float get_yaw_earth() { return yaw_earth; }
    uint32_t get_vehicle_state() { return vehicle_state; }

#if AP_SCRIPTING_ENABLED
    AP_Scripting scripting;
#endif

    MAVLink_Periph mavlink{MAVLINK_COMM_0};

    MAVLink_Periph* get_link(mavlink_channel_t chan) {
        if (chan == MAVLINK_COMM_0) {
            return &mavlink;
        } else {
            return nullptr;
        }
    }

#if AP_INERTIALSENSOR_ENABLED
    AP_InertialSensor imu;
    uint32_t last_imu_update_usec;
#endif

    // setup the var_info table
    AP_Param param_loader{var_info};

    static const AP_Param::Info var_info[];

    uint32_t last_mag_update_ms;
    uint32_t last_gps_update_ms;
    uint32_t last_baro_update_ms;
    uint64_t last_time_sync_usec;
    int64_t time_offset_usec;

#ifdef I2C_SLAVE_ENABLED
    uint8_t i2c_led_color_red;
    uint8_t i2c_led_color_green;
    uint8_t i2c_led_color_blue;
    uint8_t i2c_led_reg;
    bool i2c_new_led_data;

    uint8_t i2c2_transfer_byte_idx;
    uint8_t i2c2_transfer_address;
    uint8_t i2c2_transfer_direction;
    bool _setup_ser_i2c_mode;

    HAL_EventHandle i2c_event_handle;
    ChibiOS::EventSource i2c_event_source;
#endif

    static AP_Periph_FW *_singleton;

    enum class DebugOptions {
        SHOW_STACK = 0,
        ENABLE_STATS = 2,
    };
    // check if an option is set
    bool debug_option_is_set(const DebugOptions option) const {
        return (uint8_t(g.debug.get()) & (1U<<uint8_t(option))) != 0;
    }
    // show stack as DEBUG msgs
    void show_stack_free();

    static bool no_iface_finished_dna;
    bool saw_lock_once;
    JitterCorrection jitter;

    struct {
        ByteBuffer *buffer;
        uint32_t last_request_ms;
        AP_HAL::UARTDriver *uart;
        int8_t uart_num;
        uint8_t node_id;
        uint8_t protocol;
        uint32_t baudrate;
        bool locked;
        uint32_t key;
    } monitor;

    int8_t get_default_tunnel_serial_port(void);
    static void set_rgb_led(uint8_t red, uint8_t green, uint8_t blue);

    AP_Periph_DroneCAN *dronecan;

    void show_progress(uint32_t pct);

    static uint64_t get_tracked_tx_timestamp(uint8_t i);

#ifdef HAL_USB_VBUS_SENS_CHAN
    AP_HAL::AnalogSource *vbus_voltage_source;
#endif
    uint16_t reboot_str_index;

#ifdef ENABLE_BASE_MODE
    GPS_Base gps_base;
#endif
};


class AP_Periph_DroneCAN {
public:
    AP_Periph_DroneCAN();

    CanardInterface canard_iface{0};

    void send_serial_monitor_data();
    Canard::Publisher<uavcan_tunnel_Targetted> tunnel_pub{canard_iface};

    Canard::Publisher<uavcan_protocol_dynamic_node_id_Allocation> dynamic_node_id_pub{canard_iface};
    void handle_open_drone_id_arm_status(mavlink_open_drone_id_arm_status_t &pkt);
    Canard::Publisher<dronecan_remoteid_ArmStatus> arm_status_pub{canard_iface};

    void node_status_send(void);
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{canard_iface};
    Canard::Publisher<dronecan_protocol_Stats> stats_pub{canard_iface};
    Canard::Publisher<dronecan_protocol_CanStats> can_stats_pub{canard_iface};

    void can_mag_update();
    Canard::Publisher<uavcan_equipment_ahrs_MagneticFieldStrength> mag_pub{canard_iface};

    void can_gps_init();
    void can_gps_update();
    Canard::Publisher<uavcan_equipment_gnss_Fix2> fix2_pub{canard_iface};
    Canard::Publisher<uavcan_equipment_gnss_Auxiliary> aux_pub{canard_iface};
    Canard::Publisher<ardupilot_gnss_Status> gnss_status_pub{canard_iface};
#if HAL_NUM_CAN_IFACES == 1
    Canard::Publisher<uavcan_protocol_GlobalTimeSync> global_time_sync_pub[HAL_NUM_CAN_IFACES] = {{canard_iface}};
#else
    Canard::Publisher<uavcan_protocol_GlobalTimeSync> global_time_sync_pub[HAL_NUM_CAN_IFACES] = {{canard_iface, 1}, {canard_iface, 1<<1}};
#endif
    void send_moving_baseline_msg();
    Canard::Publisher<ardupilot_gnss_MovingBaselineData> moving_baseline_pub{canard_iface};

    void send_relposheading_msg();
    Canard::Publisher<ardupilot_gnss_RelPosHeading> relposheading_pub{canard_iface};

    void can_baro_update();
    Canard::Publisher<uavcan_equipment_air_data_StaticPressure> static_pressure_pub{canard_iface};
    Canard::Publisher<uavcan_equipment_air_data_StaticTemperature> static_temperature_pub{canard_iface};

    Canard::Publisher<uavcan_protocol_debug_LogMessage> log_pub{canard_iface};

    void can_imu_update();
#if AP_INERTIALSENSOR_ENABLED
    Canard::Publisher<uavcan_equipment_ahrs_RawIMU> raw_imu_pub{canard_iface};
#endif

    // servers
    static void handle_get_node_info(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest &req);
    Canard::StaticCallback<uavcan_protocol_GetNodeInfoRequest> get_node_info_callback{&AP_Periph_DroneCAN::handle_get_node_info};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> get_node_info_server{canard_iface, get_node_info_callback};

    static void handle_param_getset(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest &req);
    Canard::StaticCallback<uavcan_protocol_param_GetSetRequest> param_getset_callback{&AP_Periph_DroneCAN::handle_param_getset};
    Canard::Server<uavcan_protocol_param_GetSetRequest> param_getset_server{canard_iface, param_getset_callback};

    static void handle_param_executeopcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest &req);
    Canard::StaticCallback<uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_callback{&AP_Periph_DroneCAN::handle_param_executeopcode};
    Canard::Server<uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_server{canard_iface, param_executeopcode_callback};

    static void handle_begin_firmware_update(const CanardRxTransfer& transfer, const uavcan_protocol_file_BeginFirmwareUpdateRequest &req);
    Canard::StaticCallback<uavcan_protocol_file_BeginFirmwareUpdateRequest> begin_firmware_update_callback{&AP_Periph_DroneCAN::handle_begin_firmware_update};
    Canard::Server<uavcan_protocol_file_BeginFirmwareUpdateRequest> begin_firmware_update_server{canard_iface, begin_firmware_update_callback};

    static void handle_restart_node(const CanardRxTransfer& transfer, const uavcan_protocol_RestartNodeRequest &req);
    Canard::StaticCallback<uavcan_protocol_RestartNodeRequest> restart_node_callback{&AP_Periph_DroneCAN::handle_restart_node};
    Canard::Server<uavcan_protocol_RestartNodeRequest> restart_node_server{canard_iface, restart_node_callback};


    // subscribers
    static void handle_allocation_response(const CanardRxTransfer& transfer, const uavcan_protocol_dynamic_node_id_Allocation &msg);
    Canard::StaticCallback<uavcan_protocol_dynamic_node_id_Allocation> allocation_response_callback{&AP_Periph_DroneCAN::handle_allocation_response};
    Canard::Subscriber<uavcan_protocol_dynamic_node_id_Allocation> allocation_response_sub{allocation_response_callback, 0};

    static void handle_RTCMStream(const CanardRxTransfer& transfer, const uavcan_equipment_gnss_RTCMStream &req);
    Canard::StaticCallback<uavcan_equipment_gnss_RTCMStream> RTCMStream_callback{&AP_Periph_DroneCAN::handle_RTCMStream};
    Canard::Subscriber<uavcan_equipment_gnss_RTCMStream> RTCMStream_sub{RTCMStream_callback, 0};

#if GPS_MOVING_BASELINE
    static void handle_MovingBaselineData(const CanardRxTransfer& transfer, const ardupilot_gnss_MovingBaselineData &msg);
    Canard::StaticCallback<ardupilot_gnss_MovingBaselineData> MovingBaselineData_callback{&AP_Periph_DroneCAN::handle_MovingBaselineData};
    Canard::Subscriber<ardupilot_gnss_MovingBaselineData> MovingBaselineData_sub{MovingBaselineData_callback, 0};
#endif

    static void handle_lightscommand(const CanardRxTransfer& transfer, const uavcan_equipment_indication_LightsCommand &msg);
    Canard::StaticCallback<uavcan_equipment_indication_LightsCommand> lightscommand_callback{&AP_Periph_DroneCAN::handle_lightscommand};
    Canard::Subscriber<uavcan_equipment_indication_LightsCommand> lightscommand_sub{lightscommand_callback, 0};

    static void handle_global_time_sync(const CanardRxTransfer& transfer, const uavcan_protocol_GlobalTimeSync &msg);
    Canard::StaticCallback<uavcan_protocol_GlobalTimeSync> global_time_sync_callback{&AP_Periph_DroneCAN::handle_global_time_sync};
    Canard::Subscriber<uavcan_protocol_GlobalTimeSync> global_time_sync_sub{global_time_sync_callback, 0};

    static void handle_notify_state(const CanardRxTransfer& transfer, const ardupilot_indication_NotifyState &msg);
    Canard::StaticCallback<ardupilot_indication_NotifyState> notify_state_callback{&AP_Periph_DroneCAN::handle_notify_state};
    Canard::Subscriber<ardupilot_indication_NotifyState> notify_state_sub{notify_state_callback, 0};

    static void handle_remoteid_location(const CanardRxTransfer& transfer, const dronecan_remoteid_Location &msg);
    Canard::StaticCallback<dronecan_remoteid_Location> remoteid_location_callback{&AP_Periph_DroneCAN::handle_remoteid_location};
    Canard::Subscriber<dronecan_remoteid_Location> remoteid_location_sub{remoteid_location_callback, 0};

    static void handle_remoteid_basicid(const CanardRxTransfer& transfer, const dronecan_remoteid_BasicID &msg);
    Canard::StaticCallback<dronecan_remoteid_BasicID> remoteid_basicid_callback{&AP_Periph_DroneCAN::handle_remoteid_basicid};
    Canard::Subscriber<dronecan_remoteid_BasicID> remoteid_basicid_sub{remoteid_basicid_callback, 0};

    static void handle_remoteid_system(const CanardRxTransfer& transfer, const dronecan_remoteid_System &msg);
    Canard::StaticCallback<dronecan_remoteid_System> remoteid_system_callback{&AP_Periph_DroneCAN::handle_remoteid_system};
    Canard::Subscriber<dronecan_remoteid_System> remoteid_system_sub{remoteid_system_callback, 0};

    static void handle_remoteid_selfid(const CanardRxTransfer& transfer, const dronecan_remoteid_SelfID &msg);
    Canard::StaticCallback<dronecan_remoteid_SelfID> remoteid_selfid_callback{&AP_Periph_DroneCAN::handle_remoteid_selfid};
    Canard::Subscriber<dronecan_remoteid_SelfID> remoteid_selfid_sub{remoteid_selfid_callback, 0};

    static void handle_remoteid_operatorid(const CanardRxTransfer& transfer, const dronecan_remoteid_OperatorID &msg);
    Canard::StaticCallback<dronecan_remoteid_OperatorID> remoteid_operatorid_callback{&AP_Periph_DroneCAN::handle_remoteid_operatorid};
    Canard::Subscriber<dronecan_remoteid_OperatorID> remoteid_operatorid_sub{remoteid_operatorid_callback, 0};

    static void handle_tunnel_Targetted(const CanardRxTransfer& transfer, const uavcan_tunnel_Targetted &pkt);
    Canard::StaticCallback<uavcan_tunnel_Targetted> tunnel_targetted_callback{&AP_Periph_DroneCAN::handle_tunnel_Targetted};
    Canard::Subscriber<uavcan_tunnel_Targetted> tunnel_targetted_sub{tunnel_targetted_callback, 0};
};

namespace AP
{
    AP_Periph_FW& periph();
}

extern AP_Periph_FW periph;

extern "C" {
void can_printf(const char *fmt, ...) FMT_PRINTF(1,2);
}

