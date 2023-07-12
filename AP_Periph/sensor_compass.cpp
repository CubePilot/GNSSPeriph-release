#include "AP_Periph.h"

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
