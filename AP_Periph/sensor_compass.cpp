#include "AP_Periph.h"

#ifndef SENSOR_ID_OFFSET
#define SENSOR_ID_OFFSET 0
#endif 

/*
  update CAN magnetometer
 */
void AP_Periph_DroneCAN::can_mag_update(void)
{
#ifdef HAL_PERIPH_ENABLE_MAG
#ifdef I2C_SLAVE_ENABLED
    if (periph.is_ak09916_available && periph.g.serial_i2c_mode) {
        return;
    }
#endif
    auto &compass = periph.compass;
    if (!compass.available()) {
        return;
    }
    compass.read();

    // estimate scale and offset from first compass
    if (compass.healthy(0) && compass.healthy(1) && compass.get_offsets(1).is_zero()) {
        const Vector3f &reference = compass.get_field(0);
        const Vector3f &field = compass.get_field(1);
        compass.set_and_save_offsets(1, reference - field);
    }

    uavcan_equipment_ahrs_MagneticFieldStrength2 pkt {};
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (!compass.healthy(i)) {
            continue;
        }
        if (periph.last_mag_update_ms[i] == compass.last_update_ms(i)) {
            continue;
        }
        periph.last_mag_update_ms[i] = compass.last_update_ms(i);
        const Vector3f &field = compass.get_field(i);
        if (((compass.get_detected_dev_id(i) >> 16) & 0xFF) == AP_Compass_Backend::DEVTYPE_AK09918) {
            pkt.sensor_id = 1;
        } else {
            pkt.sensor_id = 0;
        }
        for (uint8_t j=0; j<3; j++) {
            pkt.magnetic_field_ga[j] = field[j] * 0.001;
        }
        mag_pub.broadcast(pkt);
    }
#endif // HAL_PERIPH_ENABLE_MAG
}
