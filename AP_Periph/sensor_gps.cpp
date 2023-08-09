#include "AP_Periph.h"
#include <AP_GPS/RTCM3_Parser.h>

#if 0
#define Debug(...) do { hal.console->printf(__VA_ARGS__); } while(0)
#else
#define Debug(...)
#endif
 
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
    auto &gps = periph.gps;
    const uint8_t *data = nullptr;
    uint16_t len = 0, offset = 0;
    if (!gps.get_RTCMV3(data, len)) {
        return;
    }
    if (len == 0 || data == nullptr) {
        return;
    }
    if (len > sizeof(ardupilot_gnss_MovingBaselineData::data.data)) {
        can_printf("RTCM3 packet large (%u bytes)\n", len);
    }
    while (len) {
        // send the packet from Moving Base to be used RelPosHeading calc by GPS module
        ardupilot_gnss_MovingBaselineData mbldata {};
        // get the data from the moving base
        // static_assert(sizeof(ardupilot_gnss_MovingBaselineData::data.data) == RTCM3_MAX_PACKET_LEN, "Size of Moving Base data is wrong");
        mbldata.data.len = MIN(sizeof(mbldata.data.data), len);
        memcpy(mbldata.data.data, &data[offset], mbldata.data.len);

        if (moving_baseline_pub.broadcast(mbldata)) {
            len -= mbldata.data.len;
            offset += mbldata.data.len;
        } else {
            // free some space
            canard_iface.process(1);
        }
    }
    gps.clear_RTCMV3();
#endif // HAL_PERIPH_ENABLE_GPS && GPS_MOVING_BASELINE
}

void AP_Periph_DroneCAN::send_relposheading_msg() {
#if defined(HAL_PERIPH_ENABLE_GPS) && GPS_MOVING_BASELINE
    float reported_heading;
    float relative_distance;
    float relative_down_pos;
    float reported_heading_acc;
    static uint64_t last_timestamp = 0;
    uint64_t curr_timestamp = 0;
#ifdef ENABLE_RTKLIB
    periph.get_RelPosHeading(curr_timestamp, reported_heading, relative_distance, relative_down_pos, reported_heading_acc);
#else
    auto &gps = periph.gps;
    gps.get_RelPosHeading(curr_timestamp, reported_heading, relative_distance, relative_down_pos, reported_heading_acc);
#endif
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
#ifdef ENABLE_RTKLIB
    periph.rtklib_handle_rtcm_fragment(msg.data.data, msg.data.len);
#else
    periph.gps.inject_MBL_data((uint8_t*)msg.data.data, msg.data.len);
#endif
    Debug("MovingBaselineData: len=%u\n", msg.data.len);
}
#endif // GPS_MOVING_BASELINE
