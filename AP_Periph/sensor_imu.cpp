#include "AP_Periph.h"

#if AP_PERIPH_LOGGER
/*
  log one IMU loop sample as delta angle / delta velocity (IMUD)
 */
static void log_imu_delta(AP_InertialSensor &imu)
{
    Vector3f delta_angle, delta_velocity;
    float delta_angle_dt = 0, delta_velocity_dt = 0;
    imu.get_delta_angle(delta_angle, delta_angle_dt);
    imu.get_delta_velocity(delta_velocity, delta_velocity_dt);
    AP::logger().WriteStreaming("IMUD",
                                "TimeUS,DelAngX,DelAngY,DelAngZ,DelAngDT,DelVelX,DelVelY,DelVelZ,DelVelDT,Temp",
                                "srrrsnnnsO",
                                "F000000000",
                                "Qfffffffff",
                                AP_HAL::micros64(),
                                delta_angle.x, delta_angle.y, delta_angle.z, delta_angle_dt,
                                delta_velocity.x, delta_velocity.y, delta_velocity.z, delta_velocity_dt,
                                imu.get_temperature(0));
}
#endif

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

#if AP_PERIPH_LOGGER
        // logger build: log every new sample, never exit the thread, no CAN broadcast
        if (imu.get_last_update_usec() == periph.last_imu_update_usec) {
            continue;
        }
        periph.last_imu_update_usec = imu.get_last_update_usec();
        log_imu_delta(imu);
        continue;
#endif

        uavcan_equipment_ahrs_RawIMU pkt {};
        if (imu.get_last_update_usec() == periph.last_imu_update_usec) {
            return;
        }
        pkt.timestamp.usec = imu.get_last_update_usec();
        periph.last_imu_update_usec = pkt.timestamp.usec;
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
