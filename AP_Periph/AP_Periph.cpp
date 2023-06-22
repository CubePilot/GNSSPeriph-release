/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  AP_Periph main firmware

  To flash this firmware on Linux use:

     st-flash write build/f103-periph/bin/AP_Periph.bin 0x8006000

 */
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <stdio.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>
#endif

extern const AP_HAL::HAL &hal;

AP_Periph_FW periph;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
void stm32_watchdog_init() {}
void stm32_watchdog_pat() {}
#endif

void setup(void)
{
    periph.init();
}

void loop(void)
{
    periph.update();
}

static uint32_t start_ms;

AP_Periph_FW::AP_Periph_FW()
#if HAL_LOGGING_ENABLED
    : logger(g.log_bitmask)
#endif
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Periph_FW must be singleton");
    }
    _singleton = this;
}

#if HAL_LOGGING_ENABLED
const struct LogStructure AP_Periph_FW::log_structure[] = {
    LOG_COMMON_STRUCTURES,
};
#endif

void AP_Periph_FW::gpio_passthrough_isr(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    if (pin == GPIO_USART1_RX) {
        hal.gpio->write(GPIO_USART2_TX, pin_state);
    } else if (pin == GPIO_USART2_RX) {
        hal.gpio->write(GPIO_USART1_TX, pin_state);
    }
}

void AP_Periph_FW::init()
{
    
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
#ifndef DISABLE_WATCHDOG
    stm32_watchdog_init();
#endif

    stm32_watchdog_pat();

#if !HAL_GCS_ENABLED
    hal.serial(0)->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 32);
#endif
    hal.serial(3)->begin(115200, 128, 256);

    load_parameters();

    stm32_watchdog_pat();

    can_start();

#if HAL_GCS_ENABLED
    stm32_watchdog_pat();
    gcs().init();
    
#endif
    serial_manager.init();

#if HAL_GCS_ENABLED
    gcs().setup_console();
    gcs().setup_uarts();
    gcs().send_text(MAV_SEVERITY_INFO, "AP_Periph GCS Initialised!");
#endif

    stm32_watchdog_pat();

#if HAL_LOGGING_ENABLED
    logger.Init(log_structure, ARRAY_SIZE(log_structure));
#endif

    printf("Booting %08x:%08x %u/%u len=%u 0x%08x\n",
           app_descriptor.image_crc1,
           app_descriptor.image_crc2,
           app_descriptor.version_major, app_descriptor.version_minor,
           app_descriptor.image_size,
           app_descriptor.git_hash);

    if (hal.util->was_watchdog_reset()) {
        printf("Reboot after watchdog reset\n");
    }

#if HAL_INS_ENABLED
    imu.init(1000);
#endif

#ifdef HAL_PERIPH_ENABLE_GPS
    if (gps.get_type(0) != AP_GPS::GPS_Type::GPS_TYPE_NONE && !g.serial_i2c_mode) {
#if HAL_LOGGING_ENABLED
        #define MASK_LOG_GPS (1<<2)
        gps.set_log_gps_bit(MASK_LOG_GPS);
#endif
        gps.init(serial_manager);

    } else {
        // setup gpio passthrough
        hal.gpio->set_mode(GPIO_USART1_RX, HAL_GPIO_INPUT);
        hal.gpio->set_mode(GPIO_USART1_TX, HAL_GPIO_OUTPUT);
        hal.gpio->set_mode(GPIO_USART2_RX, HAL_GPIO_INPUT);
        hal.gpio->set_mode(GPIO_USART2_TX, HAL_GPIO_OUTPUT);
        hal.gpio->attach_interrupt(GPIO_USART1_RX, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::gpio_passthrough_isr, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH);
        hal.gpio->attach_interrupt(GPIO_USART2_RX, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::gpio_passthrough_isr, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH);
    }

    i2c_event_handle.set_source(&i2c_event_source);
    i2c_event_handle.register_event(1);
    i2c_setup();
#endif

#ifdef HAL_PERIPH_ENABLE_MAG
    compass.init();
#endif

#ifdef HAL_PERIPH_ENABLE_BARO
    baro.init();
#endif


    hal.rcout->init();

    hal.rcout->force_safety_off();

    notify.init();

#if AP_SCRIPTING_ENABLED
    scripting.init();
#endif
    start_ms = AP_HAL::native_millis();
}

/*
  rotating rainbow pattern on startup
 */
void AP_Periph_FW::update_rainbow()
{
    if (led_cmd_override) {
        return;
    }
    uint32_t now = AP_HAL::native_millis();

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
    float brightness = 0.3;
    for (uint8_t n=0; n<4; n++) {
        uint8_t i = (step + n) % nsteps;
        periph.notify.handle_rgb(rgb_rainbow[i].red*brightness,
                                 rgb_rainbow[i].green*brightness,
                                 rgb_rainbow[i].blue*brightness);
    }
    step++;
}


#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && CH_DBG_ENABLE_STACK_CHECK == TRUE
void AP_Periph_FW::show_stack_free()
{
    const uint32_t isr_stack_size = uint32_t((const uint8_t *)&__main_stack_end__ - (const uint8_t *)&__main_stack_base__);
    can_printf("ISR %u/%u", unsigned(stack_free(&__main_stack_base__)), unsigned(isr_stack_size));

    for (thread_t *tp = chRegFirstThread(); tp; tp = chRegNextThread(tp)) {
        uint32_t total_stack;
        if (tp->wabase == (void*)&__main_thread_stack_base__) {
            // main thread has its stack separated from the thread context
            total_stack = uint32_t((const uint8_t *)&__main_thread_stack_end__ - (const uint8_t *)&__main_thread_stack_base__);
        } else {
            // all other threads have their thread context pointer
            // above the stack top
            total_stack = uint32_t(tp) - uint32_t(tp->wabase);
        }
        can_printf("%s STACK=%u/%u\n", tp->name, unsigned(stack_free(tp->wabase)), unsigned(total_stack));
    }
}
#endif

void AP_Periph_FW::rcout_update()
{
    if (!rcout_has_new_data_to_update) {
        return;
    }
    rcout_has_new_data_to_update = false;

    SRV_Channels::calc_pwm();
    SRV_Channels::cork();
    SRV_Channels::output_ch_all();
    SRV_Channels::push();
}


void AP_Periph_FW::update()
{
    static uint32_t last_led_ms;
    uint32_t now = AP_HAL::native_millis();
    if (now - last_led_ms > 1000) {
        last_led_ms = now;
#ifdef HAL_GPIO_PIN_LED
        if (!no_iface_finished_dna) {
            palToggleLine(HAL_GPIO_PIN_LED);
        }
#endif

#if 0
#ifdef HAL_PERIPH_ENABLE_GPS
        hal.serial(0)->printf("GPS status: %u\n", (unsigned)gps.status());
#endif
#ifdef HAL_PERIPH_ENABLE_MAG
        const Vector3f &field = compass.get_field();
        hal.serial(0)->printf("MAG (%d,%d,%d)\n", int(field.x), int(field.y), int(field.z));
#endif
#ifdef HAL_PERIPH_ENABLE_BARO
        hal.serial(0)->printf("BARO H=%u P=%.2f T=%.2f\n", baro.healthy(), baro.get_pressure(), baro.get_temperature());
#endif
#ifdef HAL_PERIPH_ENABLE_RANGEFINDER
        hal.serial(0)->printf("RNG %u %ucm\n", rangefinder.num_sensors(), rangefinder.distance_cm_orient(ROTATION_NONE));
#endif
        hal.scheduler->delay(1);
#endif
#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
        check_for_serial_reboot_cmd(HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT);
#endif

    SRV_Channels::enable_aux_servos();

#if HAL_GCS_ENABLED
        gcs().send_message(MSG_HEARTBEAT);
        gcs().send_message(MSG_SYS_STATUS);
#endif

        // print frequency dbms
        uint32_t freq_gain = 0, antenna_gain;
        // 20 frequencies to the left in 100MHz wide with 1191.47MHz as center
        static const float l5_test_frequencies[] = { 1141.47, 1146.47, 1151.47, 1156.47, 1161.47, 1166.47, 1171.47, 1176.47, 1181.47, 1186.47, 1191.47, 1196.47, 1201.47, 1206.47, 1211.47, 1216.47, 1221.47, 1226.47, 1231.47, 1236.47 };
        // float l5_good_freq_low = 1161.47;
        // float l5_good_freq_high = 1206.47;
        // 20 frequencies to the left in 100MHz wide with 1583.46MHz as center
        static const float l1_test_frequencies[] = { 1533.46, 1538.46, 1543.46, 1548.46, 1553.46, 1558.46, 1563.46, 1568.46, 1573.46, 1578.46, 1583.46, 1588.46, 1593.46, 1598.46, 1603.46, 1608.46, 1613.46, 1618.46, 1623.46, 1628.46 };
        // float l1_good_freq_low = 1548.46;
        // float l1_good_freq_high = 1613.46;
        // check gps for l5
        float frequency_with_peak_gain = 0;
        uint32_t peak_freq_gain = 0;
        for (uint8_t i=0; i<ARRAY_SIZE(l5_test_frequencies);i++) {
            if (AP::gps().get_frequency_db(l5_test_frequencies[i]*1000000LLU, freq_gain, antenna_gain)) {
                if (antenna_gain < 20) {
                    if ((peak_freq_gain < freq_gain) || peak_freq_gain == 0) {
                        peak_freq_gain = freq_gain;
                        frequency_with_peak_gain = l5_test_frequencies[i];
                    }
                }
            }
        }
        if (frequency_with_peak_gain != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "L5 %fMHz %ludbm %ludb\n", frequency_with_peak_gain, peak_freq_gain, antenna_gain);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "L5 not found\n");
        }
        frequency_with_peak_gain = 0;
        peak_freq_gain = 0;
        // check gps for l1
        for (uint8_t i=0; i<ARRAY_SIZE(l1_test_frequencies);i++) {
            if (AP::gps().get_frequency_db(l1_test_frequencies[i]*1000000LLU, freq_gain, antenna_gain)) {
                if (antenna_gain < 16) {
                    if ((peak_freq_gain < freq_gain) || peak_freq_gain == 0) {
                        peak_freq_gain = freq_gain;
                        frequency_with_peak_gain = l1_test_frequencies[i];
                    }
                }
            }
        }
        if (frequency_with_peak_gain != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "L1 %fMHz %ludb %ludb\n", frequency_with_peak_gain, peak_freq_gain, antenna_gain);
        } else {
            gcs().send_text(MAV_SEVERITY_INFO, "L1 not found\n");
        }
    }

    static uint32_t last_error_ms;
    const auto &ierr = AP::internalerror();
    if (now - last_error_ms > 5000 && ierr.errors()) {
        // display internal errors as DEBUG every 5s
        last_error_ms = now;
        can_printf("IERR 0x%x %u", unsigned(ierr.errors()), unsigned(ierr.last_error_line()));
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && CH_DBG_ENABLE_STACK_CHECK == TRUE
    static uint32_t last_debug_ms;
    if (g.debug==1 && now - last_debug_ms > 5000) {
        last_debug_ms = now;
        show_stack_free();
    }
#endif
    
#ifdef HAL_PERIPH_ENABLE_BATTERY
    if (now - battery.last_read_ms >= 100) {
        // update battery at 10Hz
        battery.last_read_ms = now;
        battery.lib.read();
    }
#endif

    static uint32_t fiftyhz_last_update_ms;
    if (now - fiftyhz_last_update_ms >= 20) {
        // update at 50Hz
        fiftyhz_last_update_ms = now;
        notify.update();
#if HAL_GCS_ENABLED
        gcs().update_receive();
        gcs().update_send();
#endif
    }

#if HAL_LOGGING_ENABLED
    logger.periodic_tasks();
#endif

    can_update();

    if (_setup_ser_i2c_mode && AP_Periph_FW::no_iface_finished_dna) {
        hal.scheduler->expect_delay_ms(100);
        g.serial_i2c_mode.set_and_save(1);
        prepare_reboot();
        hal.scheduler->reboot(false);
    }

    if (!AP_Periph_FW::no_iface_finished_dna && g.serial_i2c_mode) {
        hal.scheduler->expect_delay_ms(100);
        g.serial_i2c_mode.set_and_save(0);
        prepare_reboot();
        hal.scheduler->reboot(false);
    }

    if (!g.serial_i2c_mode) {
        update_rainbow();
    }

#ifdef HAL_PERIPH_ENABLE_ADSB
    adsb_update();
#endif
}

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
// check for uploader.py reboot command
void AP_Periph_FW::check_for_serial_reboot_cmd(const int8_t serial_index)
{
    // These are the string definitions in uploader.py
    //            NSH_INIT        = bytearray(b'\x0d\x0d\x0d')
    //            NSH_REBOOT_BL   = b"reboot -b\n"
    //            NSH_REBOOT      = b"reboot\n"

    // This is the command sequence that is sent from uploader.py
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT_BL)
    //            self.__send(uploader.NSH_INIT)
    //            self.__send(uploader.NSH_REBOOT)

    for (uint8_t i=0; i<hal.num_serial; i++) {
        if (serial_index >= 0 && serial_index != i) {
            // a specific serial port was selected but this is not it
            continue;
        }

        auto *uart = hal.serial(i);
        if (uart == nullptr || !uart->is_initialized()) {
            continue;
        }

        uint32_t available = MIN(uart->available(), 1000U);
        while (available-- > 0) {
            const char reboot_string[] = "\r\r\rreboot -b\n\r\r\rreboot\n";
            const char reboot_string_len = sizeof(reboot_string)-1; // -1 is to remove the null termination
            static uint16_t index[hal.num_serial];

            const int16_t data = uart->read();
            if (data < 0 || data > 0xff) {
                // read error
                continue;
            }
            if (index[i] >= reboot_string_len || (uint8_t)data != reboot_string[index[i]]) {
                // don't have a perfect match, start over
                index[i] = 0;
                continue;
            }
            index[i]++;
            if (index[i] == reboot_string_len) {
                // received reboot msg. Trigger a reboot and stay in the bootloader
                prepare_reboot();
                hal.scheduler->reboot(true);
            }
        }
    }
}
#endif // HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT

// prepare for a safe reboot where PWMs and params are gracefully disabled
// This is copied from AP_Vehicle::reboot(bool hold_in_bootloader) minus the actual reboot
void AP_Periph_FW::prepare_reboot()
{
        // flush pending parameter writes
        AP_Param::flush();

        // do not process incoming mavlink messages while we delay:
        hal.scheduler->register_delay_callback(nullptr, 5);

        // delay to give the ACK a chance to get out, the LEDs to flash,
        // the IO board safety to be forced on, the parameters to flush,
        hal.scheduler->delay(40);
}

AP_Periph_FW *AP_Periph_FW::_singleton;

AP_Periph_FW& AP::periph()
{
    return *AP_Periph_FW::get_singleton();
}

AP_HAL_MAIN();
