#include "AP_Periph.h"

/*
  update CAN baro
 */
void AP_Periph_DroneCAN::can_baro_update(void)
{
#ifdef HAL_PERIPH_ENABLE_BARO
    auto &baro = periph.baro;
    if (!periph.g.baro_enable) {
        return;
    }
    baro.update();
    if (periph.last_baro_update_ms == baro.get_last_update()) {
        return;
    }

    periph.last_baro_update_ms = baro.get_last_update();
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
        static_pressure_pub.broadcast(pkt);
    }

    {
        uavcan_equipment_air_data_StaticTemperature pkt {};
        pkt.static_temperature = C_TO_KELVIN(temp);
        pkt.static_temperature_variance = 0; // should we make this a parameter?
        static_temperature_pub.broadcast(pkt);
    }
#endif // HAL_PERIPH_ENABLE_BARO
}

