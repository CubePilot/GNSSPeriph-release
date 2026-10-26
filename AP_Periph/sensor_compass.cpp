#include "AP_Periph.h"

#ifndef SENSOR_ID_OFFSET
#define SENSOR_ID_OFFSET 0
#endif 

#if AP_PERIPH_LOGGER
/*
  MAGF: every sample in board frame, before any calibration correction, mGauss
 */
void AP_Periph_FW::log_mag_raw(uint8_t instance, const Vector3f &field_board_frame_mgauss)
{
    AP::logger().WriteStreaming("MAGF", "TimeUS,I,MagX,MagY,MagZ", "s#GGG", "F-CCC", "QBfff",
                                AP_HAL::micros64(), instance,
                                field_board_frame_mgauss.x, field_board_frame_mgauss.y, field_board_frame_mgauss.z);
}
#endif

/*
  update CAN magnetometer
 */
void AP_Periph_DroneCAN::can_mag_update(void)
{
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
#if AP_PERIPH_LOGGER
        // log once per new sample; Compass::read() would log on every call
        if (i == 0 && AP::logger().should_log(uint32_t(AP_Periph_FW::LogBit::COMPASS))) {
            AP::logger().Write_Compass();
        }
#endif
        const Vector3f &field = compass.get_field(i);
        pkt.sensor_id = SENSOR_ID_OFFSET + i;
        for (uint8_t j=0; j<3; j++) {
            pkt.magnetic_field_ga[j] = field[j] * 0.001;
        }
        mag_pub.broadcast(pkt);
    }
}
