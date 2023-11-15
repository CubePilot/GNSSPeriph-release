#include "AP_Periph.h"

/*
  update CAN magnetometer
 */
void AP_Periph_DroneCAN::can_imu_update(void)
{
#if AP_INERTIALSENSOR_ENABLED
    auto &imu = periph.imu;
    while (true) {
        imu.update();

        if (!imu.healthy()) {
            continue;
        }

        uavcan_equipment_ahrs_RawIMU pkt {};
        if (imu.get_last_update_usec() == periph.last_imu_update_usec) {
            return;
        }

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
