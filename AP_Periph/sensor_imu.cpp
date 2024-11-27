#include "AP_Periph.h"

/*
  update CAN magnetometer
 */
void AP_Periph_DroneCAN::can_imu_update(void)
{
#if AP_INERTIALSENSOR_ENABLED
    auto &imu = periph.imu;
    while (true) {
        if (periph.accel_cal_gcs && periph.initialise_accel_cal) {
            can_printf("Starting accel cal\n");
            // start with gyro calibration
            if (!imu.calibrate_gyros()) {
                periph.initialise_accel_cal = false;
            }
            // start accel cal
            imu.acal_init();
            imu.get_acal()->start(periph.accel_cal_gcs);
            periph.initialise_accel_cal = false;
            mavlink_msg_command_ack_send(periph.accel_cal_gcs->get_chan(), MAV_CMD_PREFLIGHT_CALIBRATION, MAV_RESULT_ACCEPTED,
                                         0, 0,
                                         periph.accel_cal_sysid,
                                         periph.accel_cal_compid);
        }
        if (periph.accel_cal_gcs) {
            imu.acal_update();
        }
        periph.ahrs.update();

        if (!imu.healthy()) {
            continue;
        }

        uavcan_equipment_ahrs_RawIMU pkt {};
        Vector3f tmp;
        imu.get_delta_velocity(tmp, pkt.integration_interval);
        pkt.accelerometer_integral[0] = tmp.x;
        pkt.accelerometer_integral[1] = tmp.y;
        pkt.accelerometer_integral[2] = tmp.z;

        imu.get_delta_angle(tmp, pkt.integration_interval);
        pkt.rate_gyro_integral[0] = tmp.x;
        pkt.rate_gyro_integral[1] = tmp.y;
        pkt.rate_gyro_integral[2] = tmp.z;

        tmp = imu.get_accel();
        pkt.accelerometer_latest[0] = tmp.x;
        pkt.accelerometer_latest[1] = tmp.y;
        pkt.accelerometer_latest[2] = tmp.z;

        raw_imu_pub.broadcast(pkt);
    }
#endif // AP_INERTIALSENSOR_ENABLED
}
