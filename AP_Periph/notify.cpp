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

#if AP_PERIPH_LOGGER
/*
  logger status LED (replaces the rainbow):
    green blink 1 Hz  logging, 3D fix
    blue blink 1 Hz   logging, no fix
    solid red         not logging: no eMMC, write error, X20 config failed, or dropped log data in the last 10 s
    solid purple      USB connected (retrieval mode, not logging)
 */
void AP_Periph_FW::update_logger_led()
{
    const uint32_t now_ms = AP_HAL::millis();
    static uint32_t last_update_ms;
    if (now_ms - last_update_ms < 100) {
        return;
    }
    last_update_ms = now_ms;

    static uint32_t last_dropped, last_drop_ms;
    const uint32_t dropped = logger.num_dropped();
    if (dropped != last_dropped) {
        last_dropped = dropped;
        last_drop_ms = now_ms;
    }
    const bool recent_drops = last_drop_ms != 0 && now_ms - last_drop_ms < 10000;
    // allow the X20 30 s to be detected and configured before calling it failed
    const bool x20_failed = !x20_config_ok() && now_ms > 30000;

    uint8_t red = 0, green = 0, blue = 0;
    if (hal.gpio->usb_connected()) {
        red = 128; blue = 128;
    } else if (!logger.CardInserted() || logger.logging_failed() || !logger.logging_started() || recent_drops || x20_failed) {
        red = 255;
    } else if ((now_ms / 500) % 2 == 0) {
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {
            green = 255;
        } else {
            blue = 255;
        }
    }
    const int8_t brightness = hal.gpio->usb_connected() ? LED_CONNECTED_BRIGHTNESS : notify.get_rgb_led_brightness_percent();
    const float scale = constrain_float(brightness, 0, 100) * 0.01f;
    set_rgb_led(red * scale, green * scale, blue * scale);
}
#endif

/*
  rotating rainbow pattern on startup
 */
void AP_Periph_FW::update_rainbow()
{
#if AP_PERIPH_LOGGER
    update_logger_led();
    return;
#endif
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
