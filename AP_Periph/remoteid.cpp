#include "AP_Periph.h"
#include <dronecan_msgs.h>

#define ODID_COPY(name) pkt.name = msg.name
#define ODID_COPY_STR(name) do { strncpy_noterm((char*)pkt.name, (const char*)msg.name.data, sizeof(pkt.name)); } while(0)

#ifndef DRONEID_MODULE_CHAN
#define DRONEID_MODULE_CHAN MAVLINK_COMM_0
#endif

void AP_Periph_DroneCAN::handle_remoteid_location(const CanardRxTransfer& transfer, const dronecan_remoteid_Location &msg)
{
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

void AP_Periph_DroneCAN::handle_remoteid_basicid(const CanardRxTransfer& transfer, const dronecan_remoteid_BasicID &msg)
{
    mavlink_open_drone_id_basic_id_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(id_type);
    ODID_COPY(ua_type);
    ODID_COPY_STR(uas_id);
    mavlink_msg_open_drone_id_basic_id_send_struct(DRONEID_MODULE_CHAN, &pkt);
}

void AP_Periph_DroneCAN::handle_remoteid_system(const CanardRxTransfer& transfer, const dronecan_remoteid_System &msg)
{
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

void AP_Periph_DroneCAN::handle_remoteid_selfid(const CanardRxTransfer& transfer, const dronecan_remoteid_SelfID &msg)
{
    mavlink_open_drone_id_self_id_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(description_type);
    ODID_COPY_STR(description);
    mavlink_msg_open_drone_id_self_id_send_struct(DRONEID_MODULE_CHAN, &pkt);
}

void AP_Periph_DroneCAN::handle_remoteid_operatorid(const CanardRxTransfer& transfer, const dronecan_remoteid_OperatorID &msg)
{
    mavlink_open_drone_id_operator_id_t pkt {};
    ODID_COPY_STR(id_or_mac);
    ODID_COPY(operator_id_type);
    ODID_COPY_STR(operator_id);
    mavlink_msg_open_drone_id_operator_id_send_struct(DRONEID_MODULE_CHAN, &pkt);
}

void AP_Periph_DroneCAN::handle_open_drone_id_arm_status(mavlink_open_drone_id_arm_status_t &pkt)
{
    dronecan_remoteid_ArmStatus msg {};
    strncpy_noterm((char*)msg.error.data, pkt.error, sizeof(msg.error.data));
    msg.error.len = strnlen((const char*)msg.error.data, sizeof(msg.error.data));
    msg.status = pkt.status;
    arm_status_pub.broadcast(msg);
}
