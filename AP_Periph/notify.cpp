#include "AP_Periph.h"

extern const AP_HAL::HAL& hal;

void AP_Periph_FW::set_rgb_led(uint8_t red, uint8_t green, uint8_t blue)
{
    periph.notify.handle_rgb(red, green, blue);
    periph.rcout_has_new_data_to_update = true;
}

/*
  handle lightscommand
 */
void AP_Periph_DroneCAN::handle_lightscommand(const CanardRxTransfer& transfer, const uavcan_equipment_indication_LightsCommand &req)
{
#ifdef ENABLE_BASE_MODE
    if (periph.gps_base.enabled()) {
        return;
    }
#endif
    periph.led_command_override();

    for (uint8_t i=0; i<req.commands.len; i++) {
        const uavcan_equipment_indication_SingleLightCommand &cmd = req.commands.data[i];
        // to get the right color proportions we scale the green so that is uses the
        // same number of bits as red and blue
        uint8_t red = cmd.color.red<<3;
        uint8_t green = (cmd.color.green>>1)<<3;
        uint8_t blue = cmd.color.blue<<3;
        const int8_t brightness = hal.gpio->usb_connected() ? LED_CONNECTED_BRIGHTNESS : periph.notify.get_rgb_led_brightness_percent();
        if (brightness != 100 && brightness >= 0) {
            const float scale = brightness * 0.01;
            red = constrain_int16(red * scale, 0, 255);
            green = constrain_int16(green * scale, 0, 255);
            blue = constrain_int16(blue * scale, 0, 255);
        }
        AP_Periph_FW::set_rgb_led(red, green, blue);
    }
}

void AP_Periph_DroneCAN::handle_notify_state(const CanardRxTransfer& transfer, const ardupilot_indication_NotifyState &msg)
{
    if (msg.aux_data.len == 2 && msg.aux_data_type == ARDUPILOT_INDICATION_NOTIFYSTATE_VEHICLE_YAW_EARTH_CENTIDEGREES) {
        uint16_t tmp = 0;
        memcpy(&tmp, msg.aux_data.data, sizeof(tmp));
        periph.yaw_earth = radians((float)tmp * 0.01f);
    }
    periph.vehicle_state = msg.vehicle_state;
    periph.last_vehicle_state = AP_HAL::millis();
}

/*
  rotating rainbow pattern on startup
 */
void AP_Periph_FW::update_rainbow()
{
#ifdef ENABLE_BASE_MODE
    if (gps_base.enabled()) {
        return;
    }
#endif
    if (led_cmd_override) {
        return;
    }
    uint32_t now = AP_HAL::millis();

    static uint32_t last_update_ms;
    const uint8_t step_ms = 100;
    if (now - last_update_ms < step_ms) {
        return;
    }
    const struct {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    } rgb_rainbow[] = {
        { 255, 0, 0 },
        { 255, 127, 0 },
        { 255, 255, 0 },
        { 0,   255, 0 },
        { 0,   0,   255 },
        { 75,  0,   130 },
        { 143, 0,   255 },
        { 0,   0,   0 },
    };
    last_update_ms = now;
    static uint8_t step;
    const uint8_t nsteps = ARRAY_SIZE(rgb_rainbow);
    float brightness = (hal.gpio->usb_connected() ? LED_CONNECTED_BRIGHTNESS : notify.get_rgb_led_brightness_percent()) * 0.01f;
    for (uint8_t n=0; n<4; n++) {
        uint8_t i = (step + n) % nsteps;
        notify.handle_rgb(rgb_rainbow[i].red*brightness,
                                 rgb_rainbow[i].green*brightness,
                                 rgb_rainbow[i].blue*brightness);
    }
    step++;
}
