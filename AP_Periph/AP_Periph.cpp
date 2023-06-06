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
#include <hal.h>
#include <AP_HAL_ChibiOS/GPIO.h>
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
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_Periph_FW must be singleton");
    }
    _singleton = this;
}

#ifdef GPIO_USART1_RX
void AP_Periph_FW::gpio_passthrough_isr(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    if (pin == GPIO_USART1_RX) {
        hal.gpio->write(GPIO_USART2_TX, pin_state);
    } else if (pin == GPIO_USART2_RX) {
        hal.gpio->write(GPIO_USART1_TX, pin_state);
    }
}
#endif
void AP_Periph_FW::init()
{
    
    // always run with watchdog enabled. This should have already been
    // setup by the bootloader, but if not then enable now
#ifndef DISABLE_WATCHDOG
    stm32_watchdog_init();
#endif

    vbus_voltage_source = hal.analogin->channel(HAL_USB_VBUS_SENS_CHAN);

    // sleep for 2ms to fetch the VBUS voltage
    hal.scheduler->delay(2);

    if (vbus_voltage_source->voltage_latest() < 4.0f) {
        // we are not connected over USB, disable USB
        msdStop(&USBMSD1);
        usbDisconnectBus(&USBD2);
        hal.scheduler->delay(2);
        // reconfig the USB D7 pin as CAN RX
        palSetLineMode(HAL_GPIO_PIN_OTG_HS_ULPI_D7, PAL_MODE_ALTERNATE(9) | PAL_STM32_OSPEED_HIGHEST);
        palSetLine(HAL_GPIO_PIN_SWITCH_CAN2_USB);
        palClearLine(HAL_GPIO_PIN_SLEEP_CAN2);
    } else {
        ((ChibiOS::GPIO*)(hal.gpio))->set_usb_connected();
    }

    stm32_watchdog_pat();

    hal.serial(0)->begin(AP_SERIALMANAGER_CONSOLE_BAUD, 32, 32);
    hal.serial(3)->begin(115200, 128, 256);

    load_parameters();

    stm32_watchdog_pat();

    can_start();

    stm32_watchdog_pat();
#ifdef DRONEID_MODULE_PORT
    // initialise CUBEID
    mavlink.init(DRONEID_MODULE_PORT, 115200);
#endif
    serial_manager.init();

    stm32_watchdog_pat();

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

    bool enable_gps = true;
#ifdef I2C_SLAVE_ENABLED
    enable_gps = !g.serial_i2c_mode;
#endif
#ifdef ENABLE_BASE_MODE
    enable_gps = !g.gps_passthrough && !g.gps_ubx_log;
#endif

    if (gps.get_type(0) != AP_GPS::GPS_Type::GPS_TYPE_NONE && enable_gps) {
        gps.init(serial_manager);
    } else {
#ifdef GPIO_USART1_RX
        // setup gpio passthrough
        hal.gpio->set_mode(GPIO_USART1_RX, HAL_GPIO_INPUT);
        hal.gpio->set_mode(GPIO_USART1_TX, HAL_GPIO_OUTPUT);
        hal.gpio->set_mode(GPIO_USART2_RX, HAL_GPIO_INPUT);
        hal.gpio->set_mode(GPIO_USART2_TX, HAL_GPIO_OUTPUT);
        hal.gpio->attach_interrupt(GPIO_USART1_RX, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::gpio_passthrough_isr, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH);
        hal.gpio->attach_interrupt(GPIO_USART2_RX, FUNCTOR_BIND_MEMBER(&AP_Periph_FW::gpio_passthrough_isr, void, uint8_t, bool, uint32_t), AP_HAL::GPIO::INTERRUPT_BOTH);
#endif
    }

    i2c_event_handle.set_source(&i2c_event_source);
    i2c_event_handle.register_event(1);
    i2c_setup();

    compass.init();

#ifdef HAL_PERIPH_ENABLE_BARO
    baro.init();
#endif


    hal.rcout->init();

    hal.rcout->force_safety_off();

#ifdef SRV_LED_CLK_CHANNEL
    SRV_Channels::set_default_function(SRV_LED_CLK_CHANNEL - 1, SRV_LED_CLK_FUNCTION);
    SRV_Channels::set_default_function(SRV_LED_DATA_CHANNEL - 1, SRV_LED_DATA_FUNCTION);
#endif
    notify.init();

    if (hal.gpio->usb_connected()) {
        // set LED Brightness to low
        float value;
        AP_Param::get("NTF_LED_BRIGHT", value);
        if (value > 1.0) {
            AP_Param::set_by_name("NTF_LED_BRIGHT", 1);
        }
        AP_Param::set_by_name("GPS_PASSTHROUGH", 1);
    }

#if AP_SCRIPTING_ENABLED
    scripting.init();
#endif
    start_ms = AP_HAL::native_millis();
}


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
    if (vbus_voltage_source->voltage_latest() > 4.0f) {
        ((ChibiOS::GPIO*)(hal.gpio))->set_usb_connected();
    }

#ifdef ENABLE_BASE_MODE
    bool base_update = false;
    base_update = g.gps_passthrough || g.gps_ubx_log;
    if (base_update) {
        gps_base_update();
    }
#endif

    SRV_Channels::enable_aux_servos();

    static uint32_t last_led_ms;
    uint32_t now = AP_HAL::native_millis();
    if (now - last_led_ms > 1000) {
        last_led_ms = now;
#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
        check_for_serial_reboot_cmd(HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT);
#endif
        mavlink.send_heartbeat();
    }

    static uint32_t last_error_ms;
    const auto &ierr = AP::internalerror();
    if (now - last_error_ms > 5000 && ierr.errors()) {
        // display internal errors as DEBUG every 5s
        last_error_ms = now;
        can_printf("IERR 0x%x %u", unsigned(ierr.errors()), unsigned(ierr.last_error_line()));
    }

    static uint32_t last_debug_ms;
    if (debug_option_is_set(DebugOptions::SHOW_STACK) && now - last_debug_ms > 5000) {
        last_debug_ms = now;
        show_stack_free();
    }

    static uint32_t fiftyhz_last_update_ms;
    if (now - fiftyhz_last_update_ms >= 20) {
        // update at 50Hz
        fiftyhz_last_update_ms = now;
        notify.update();
        mavlink.update();
    }

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
}

#ifdef HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT
// check for uploader.py reboot command
void AP_Periph_FW::check_for_serial_reboot_cmd(const int8_t serial_index)
{
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
            const int16_t data = uart->read();
            if (data < 0 || data > 0xff) {
                // read error
                continue;
            }
            check_for_serial_reboot_cmd_byte(data);
        }
    }
}
#endif // HAL_PERIPH_LISTEN_FOR_SERIAL_UART_REBOOT_CMD_PORT

void AP_Periph_FW::check_for_serial_reboot_cmd_byte(uint8_t data)
{
    const char reboot_string[] = "\r\r\rreboot -b\n\r\r\rreboot\n";
    const char reboot_string_len = sizeof(reboot_string)-1; // -1 is to remove the null termination

    if (reboot_str_index >= reboot_string_len || (uint8_t)data != reboot_string[reboot_str_index]) {
        // don't have a perfect match, start over
        reboot_str_index = 0;
        return;
    }
    reboot_str_index++;
    if (reboot_str_index == reboot_string_len) {
        // received reboot msg. Trigger a reboot and stay in the bootloader
        prepare_reboot();
        hal.scheduler->reboot(true);
    }
}

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
