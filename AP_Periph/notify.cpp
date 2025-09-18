#include "AP_Periph.h"
#include <AP_HAL_ChibiOS/CANFDIface.h>

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

#if AP_SCRIPTING_ENABLED
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
#endif

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
    if (led_cmd_override
#if AP_SCRIPTING_ENABLED
     || AP::scripting()->enabled()
#endif
     ) {
        return;
    }
    uint32_t now = AP_HAL::millis();

    static uint32_t last_update_ms;
    const uint8_t step_ms = 100;
    if (now - last_update_ms < step_ms) {
        return;
    }
    struct color {
        uint8_t red;
        uint8_t green;
        uint8_t blue;
    };
    const color rgb_rainbow[] = {
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
    bool compass_healthy = false;
    for (uint8_t i=0; i<COMPASS_MAX_INSTANCES; i++) {
        if (compass.healthy(i)) {
            compass_healthy = true;
            break;
        }
    }

#if defined(HAL_CANFD_CCU_ENABLED) && HAL_CANFD_CCU_ENABLED
    const color amber_breathing[] = {
        { 191, 0, 191 },
        { 127, 0, 127 },
        { 63, 0, 63 },
        { 31, 0, 31 },
        { 15, 0, 15 },
        { 7, 0, 7 },
        { 0, 0, 0 },
        { 7, 0, 7 },
        { 15, 0, 15 },
        { 31, 0, 31 },
        { 63, 0, 63 },
        { 127, 0, 127 },
        { 191, 0, 191 }
    };
    if (AP_Periph_FW::can_iface_periph[0] != nullptr && !AP_Periph_FW::can_iface_periph[0]->is_precise_calibration_complete()) {
        // use amber breathing pattern
        const uint8_t nsteps = ARRAY_SIZE(amber_breathing);
        float brightness = (hal.gpio->usb_connected() ? LED_CONNECTED_BRIGHTNESS : notify.get_rgb_led_brightness_percent()) * 0.01f;
        for (uint8_t n=0; n<4; n++) {
            uint8_t i = (step + n) % nsteps;
            notify.handle_rgb(amber_breathing[i].red*brightness,
                                    amber_breathing[i].green*brightness,
                                    amber_breathing[i].blue*brightness);
        }
    } else
#endif
    if (!compass_healthy) {
        const color red_breathing[] = {
            { 191, 0, 0 },
            { 127, 0, 0 },
            { 63, 0, 0 },
            { 31, 0, 0 },
            { 15, 0, 0 },
            { 7, 0, 0 },
            { 0, 0, 0 },
            { 7, 0, 0 },
            { 15, 0, 0 },
            { 31, 0, 0 },
            { 63, 0, 0 },
            { 127, 0, 0 },
            { 191, 0, 0 }
        };
        // use red breathing pattern
        const uint8_t nsteps = ARRAY_SIZE(red_breathing);
        float brightness = (hal.gpio->usb_connected() ? LED_CONNECTED_BRIGHTNESS : notify.get_rgb_led_brightness_percent()) * 0.01f;
        for (uint8_t n=0; n<4; n++) {
            uint8_t i = (step + n) % nsteps;
            notify.handle_rgb(red_breathing[i].red*brightness,
                                    red_breathing[i].green*brightness,
                                    red_breathing[i].blue*brightness);
        }
    } else {
        const uint8_t nsteps = ARRAY_SIZE(rgb_rainbow);
        float brightness = (hal.gpio->usb_connected() ? LED_CONNECTED_BRIGHTNESS : notify.get_rgb_led_brightness_percent()) * 0.01f;
        for (uint8_t n=0; n<4; n++) {
            uint8_t i = (step + n) % nsteps;
            notify.handle_rgb(rgb_rainbow[i].red*brightness,
                                    rgb_rainbow[i].green*brightness,
                                    rgb_rainbow[i].blue*brightness);
        }
    }
    step++;
}
