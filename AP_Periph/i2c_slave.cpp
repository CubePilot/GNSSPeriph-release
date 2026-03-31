#include "AP_Periph.h"
#include <ch.h>
#include <hal.h>

#include <AP_HAL_ChibiOS/hwdef/common/stm32_util.h>
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>

#ifdef I2C_SLAVE_ENABLED
#define TOSHIBALED_I2C_ADDRESS 0x55
#define RM3100_I2C_ADDR1 0x20
#define RM3100_I2C_ADDR2 0x21
#define RM3100_I2C_ADDR3 0x22
#define RM3100_I2C_ADDR4 0x23
#define AK09916_I2C_ADDR 0x0C
#define HAL_I2C_H7_400_TIMINGR 0x00300F38

// RM3100 register addresses
#define RM3100_CCX1_REG        0x04
#define RM3100_CCX0_REG        0x05
#define RM3100_CCY1_REG        0x06
#define RM3100_CCY0_REG        0x07
#define RM3100_CCZ1_REG        0x08
#define RM3100_CCZ0_REG        0x09
#define RM3100_TMRC_REG        0x0B
#define RM3100_CMM_REG         0x01
#define RM3100_BIST_REG        0x33

// RM3100 default/config values
#define CCP0    0xC8
#define CCP1    0x00
#define CCP0_DEFAULT 0xC8
#define CCP1_DEFAULT 0x00
#define GAIN_CC200 75.0f
#define TMRC    0x94
#define CMM     0x71
#define RUN_SELF_TEST 0xFF

extern const AP_HAL::HAL &hal;

#define TIMEOUT_MS 5

void AP_Periph_FW::i2c_setup()
{
    palSetLineMode(PAL_LINE(GPIOF, 0), PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    palSetLineMode(PAL_LINE(GPIOF, 1), PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);

    /*
    *
    *    Setup I2C Slave
    *
    */
    rccEnableI2C2(FALSE);
    rccResetI2C2();

    //Disable I2C
    I2C2->CR1 &= ~I2C_CR1_PE;

    //Enable Analog Filter
	I2C2->CR1 &= ~I2C_CR1_ANFOFF;

    //Disable Digital Filter
    I2C2->CR1 &=  ~(I2C_CR1_DNF);

    //Set Prescaler
    I2C2->TIMINGR = HAL_I2C_H7_400_TIMINGR;

    //Enable Stretching
	I2C2->CR1 &= ~I2C_CR1_NOSTRETCH;

    //7Bit Address Mode
    I2C2->CR2 &= ~I2C_CR2_ADD10;

    // Try RM3100 on SPI first
    bool rm3100_detected = false;
    for (uint8_t i = 0; i < 3; i++) {
        if (rm3100_spi_detect()) {
            rm3100_detected = true;
            break;
        }
        hal.scheduler->delay(10);
        stm32_watchdog_pat();
    }

    // Fall back to AK09916 on I2C4 if RM3100 not found
    if (!rm3100_detected) {
        for (uint8_t i = 0; i < 10; i++) {
            if (ak09916_i2c_init()) {
                is_ak09916_available = true;
                break;
            }
            hal.scheduler->delay(10);
            stm32_watchdog_pat();
        }
    }

    if (is_ak09916_available) {
        I2C2->OAR1 = (AK09916_I2C_ADDR & 0xFF) << 1; //Emulate AK09916 I2C Slave
    } else {
        I2C2->OAR1 = (RM3100_I2C_ADDR1 & 0xFF) << 1; //Emulate RM3100 I2C Slave
    }

    I2C2->OAR1 |= (1<<15);

    I2C2->OAR2 = (TOSHIBALED_I2C_ADDRESS & 0xFF) << 1; //Emulate Toshiba LED I2C Slave
    I2C2->OAR2 |= (1<<15);
    //Enable I2C interrupt
    nvicEnableVector(I2C2_EV_IRQn, 3);

    I2C2->CR1 |= (1<<1); // TXIE
    I2C2->CR1 |= (1<<2); // RXIE
    I2C2->CR1 |= (1<<3); // ADDRIE
    I2C2->CR1 |= (1<<5); // STOPIE
	I2C2->CR1 |= I2C_CR1_PE; // Enable I2C

    if (!is_ak09916_available) {
        hal.spi->set_register_rw_callback("rm3100", FUNCTOR_BIND_MEMBER(&AP_Periph_FW::compass_register_rw_callback, void, uint8_t, uint8_t*, uint32_t, bool));
    }
}

void AP_Periph_FW::toshibaled_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte)
{
    if (!g.serial_i2c_mode) {
        // we need to set param and reboot
        _setup_ser_i2c_mode  = true;
    }
    if (recv_byte_idx == 0 || ((recv_byte&(1<<7)) != 0)) {
        i2c_led_reg = recv_byte & ~(1<<7);
    } else {
        switch(i2c_led_reg) {
            case 1:
                i2c_led_color_blue = ((recv_byte << 4)&0xf0) | (recv_byte&0x0f);
                break;
            case 2:
                i2c_led_color_green = ((recv_byte << 4)&0xf0) | (recv_byte&0x0f);
                break;
            case 3:
                i2c_led_color_red = ((recv_byte << 4)&0xf0) | (recv_byte&0x0f);
                break;
        }
        i2c_led_reg++;
        i2c_new_led_data = true;
    }
}

void AP_Periph_FW::compass_register_rw_callback(uint8_t reg, uint8_t *buf, uint32_t size, bool is_write)
{
    uint8_t temp_buf[9], cnt = 0;
    if (size <= 9) {
        memcpy(temp_buf, buf, size);
    } else {
        // this is unexpected
        return;
    }
    if ((reg == 0x24) && (size == 9)) {
        // invert x and y axes
        // take 2's complement
        uint32_t val = (temp_buf[0] << 16) | (temp_buf[1] << 8) | (temp_buf[2]);
        val = ~val;
        val += 1;
        temp_buf[0] = (val >> 16) & 0xff;
        temp_buf[1] = (val >> 8) & 0xff;
        temp_buf[2] = (val) & 0xff;
        val = (temp_buf[3] << 16) | (temp_buf[4] << 8) | (temp_buf[5]);
        val = ~val;
        val += 1;
        temp_buf[3] = (val >> 16) & 0xff;
        temp_buf[4] = (val >> 8) & 0xff;
        temp_buf[5] = (val) & 0xff;
        val = (temp_buf[6] << 16) | (temp_buf[7] << 8) | (temp_buf[8]);
        val = ~val;
        val += 1;
        temp_buf[6] = (val >> 16) & 0xff;
        temp_buf[7] = (val >> 8) & 0xff;
        temp_buf[8] = (val) & 0xff;
    }
    // add register to singly linked list, if not already there
    if (reg_list_head == nullptr) {
        reg_list_head = new reg_list;
        reg_list_head->reg = reg;
        reg_list_head->val = temp_buf[cnt];
        reg_list_head->updated = true;
        reg++;
        size--;
        cnt++;
        return;
    }
    // add to list if not already there, otherwise update value
    reg_list *cur = reg_list_head;
    reg_list *prev = nullptr;
    while (size) {
        // search for the register in the list
        while (cur != nullptr && cur->reg != reg) {
            prev = cur;
            cur = cur->next;
        }
        // if found update it
        if (cur != nullptr) {
            cur->val = temp_buf[cnt];
            cur->updated = true;
            reg++;
            size--;
            cnt++;
        }
        // if not found add it
        else {
            prev->next = new reg_list;
            cur = prev->next;
            if (cur == nullptr) {
                AP_HAL::panic("Failed to add register to list"); // this is really bad, best to halt here
            }
            cur->reg = reg;
            cur->val = temp_buf[cnt];
            cur->updated = true;
            reg++;
            size--;
            cnt++;
        }
    }
}

uint8_t AP_Periph_FW::compass_send_byte(uint8_t reg) {
    // search for the register in the list
    reg_list *cur = reg_list_head;
    if (reg == 0x36) {
        return 0x22;
    }
    while (cur != nullptr && cur->reg != reg) {
        cur = cur->next;
    }
    // if found return it
    if (cur != nullptr) {
        if (reg == 0x34) { // this is a rm3100 status register request, only update if we have a new data at 0x24 already
            while (cur != nullptr && cur->reg != 0x24) {
                cur = cur->next;
            }
            if (cur != nullptr && cur->updated) {
                cur->updated = false;
                return 0x80;
            } else {
                return 0x00;
            }
        }
        cur->updated = false;
        return cur->val;
    }
    // if not found return 0
    return 0;
}

void AP_Periph_FW::compass_recv_byte(uint8_t idx, uint8_t byte)
{
    // TODO: implement writing back to registers
    if (idx == 0) {
        compass_reg = byte;
    }
}

bool AP_Periph_FW::ak09916_recv_byte(uint8_t idx, uint8_t byte)
{
    if (idx == 0) {
        // First byte is the register address
        ak09916_transfer_reg = byte;
        return true;
    } else {
        // Subsequent bytes are data to write to the register
        // Use raw I2C register access for direct hardware control
        return ak09916_write_register(ak09916_transfer_reg, byte);
    }
}

// Detect RM3100 on SPI using BIST self-test
bool AP_Periph_FW::rm3100_spi_detect()
{
    auto dev = hal.spi->get_device("rm3100");
    if (!dev) {
        return false;
    }
    dev->get_semaphore()->take_blocking();
    // read has high bit set for SPI
    dev->set_read_flag(0x80);

    // high retries for init
    dev->set_retries(10);

    // use default cycle count values as a whoami test
    uint8_t ccx0;
    uint8_t ccx1;
    uint8_t ccy0;
    uint8_t ccy1;
    uint8_t ccz0;
    uint8_t ccz1;
    if (!dev->read_registers(RM3100_CCX1_REG, &ccx1, 1) ||
        !dev->read_registers(RM3100_CCX0_REG, &ccx0, 1) ||
        !dev->read_registers(RM3100_CCY1_REG, &ccy1, 1) ||
        !dev->read_registers(RM3100_CCY0_REG, &ccy0, 1) ||
        !dev->read_registers(RM3100_CCZ1_REG, &ccz1, 1) ||
        !dev->read_registers(RM3100_CCZ0_REG, &ccz0, 1) ||
        ccx1 != CCP1_DEFAULT || ccx0 != CCP0_DEFAULT ||
        ccy1 != CCP1_DEFAULT || ccy0 != CCP0_DEFAULT ||
        ccz1 != CCP1_DEFAULT || ccz0 != CCP0_DEFAULT) {
        // couldn't read one of the cycle count registers or didn't recognize the default cycle count values
        dev->get_semaphore()->give();
        return false;
    }

    dev->setup_checked_registers(8);

    dev->write_register(RM3100_TMRC_REG, TMRC, true); // CMM data rate
    dev->write_register(RM3100_CMM_REG, CMM, true); // CMM configuration
    dev->write_register(RM3100_CCX1_REG, CCP1, true); // cycle count x
    dev->write_register(RM3100_CCX0_REG, CCP0, true); // cycle count x
    dev->write_register(RM3100_CCY1_REG, CCP1, true); // cycle count y
    dev->write_register(RM3100_CCY0_REG, CCP0, true); // cycle count y
    dev->write_register(RM3100_CCZ1_REG, CCP1, true); // cycle count z
    dev->write_register(RM3100_CCZ0_REG, CCP0, true); // cycle count z

    uint8_t bist;
    // do a self test of Coils
    dev->write_register(RM3100_BIST_REG, RUN_SELF_TEST);
    // sleep for 1ms
    hal.scheduler->delay(10);
    dev->read_registers(RM3100_BIST_REG, &bist, 1);

    if (bist != RUN_SELF_TEST) {
        // BIST failed
        dev->get_semaphore()->give();
        return false;
    }

    // turn off BIST
    dev->write_register(RM3100_BIST_REG, 0x00);

    // lower retries for run
    dev->set_retries(3);

    dev->get_semaphore()->give();
    return true;
}

// Initialize I2C4 for AK09916 master communication
bool AP_Periph_FW::ak09916_i2c_init()
{
    // Enable I2C4 clock
    rccEnableI2C4(FALSE);
    rccResetI2C4();

    // Disable I2C4
    I2C4->CR1 &= ~I2C_CR1_PE;

    // Configure timing for 400kHz operation on H7 
    I2C4->TIMINGR = HAL_I2C_H7_400_TIMINGR;

    // Configure as master mode
    I2C4->CR2 &= ~I2C_CR2_ADD10;  // 7-bit addressing

    // Enable I2C4
    I2C4->CR1 |= I2C_CR1_PE;

    if (!ak09916_write_register(0x32, 0x01)) {
        return false; // Failed to write to control register
    }

    if (!ak09916_write_register(0x32, 0x01)) {
        return false; // Failed to write to control register
    }

    // try reading the device ID register to confirm presence
    uint8_t id = 0x0;
    for (int i = 0; i < 10; i++) {
        if (ak09916_read_register(0x01, id)) {
            break; // AK09916 detected
        }
        hal.scheduler->delay(10); // wait before retrying
    }

    if (id != 0x0C) {
        return false; // AK09916 not detected
    }

    // setup AK09916 in continuous measurement mode 2
    if (!ak09916_write_register(0x31, 0x08)) {
        return false; // Failed to write to mode register
    }

    // try reading actual data register to confirm presence
    uint8_t data = 0x0;
    for (uint8_t i = 0; i < 5; i++) {
        hal.scheduler->delay(10); // wait before retrying
        if (!ak09916_read_register(0x11, data)) {
            continue;
        }
        if (data & 0x01) {
            return true; // AK09916 detected
        }
    }
    return false;
}

static bool i2c_wait_flag(volatile uint32_t &reg, uint32_t flag, bool set, uint32_t timeout_ms)
{
    uint32_t start = AP_HAL::millis();
    while (true) {
        if (set) {
            if (reg & flag) return true;
        } else {
            if ((reg & flag) == 0) return true;
        }
        if ((AP_HAL::millis() - start) > timeout_ms) {
            return false; // timeout
        }
    }
}

// Read a register from AK09916 using raw I2C register access
bool AP_Periph_FW::ak09916_read_register(uint8_t reg, uint8_t &data)
{
    data = 0;

    I2C4->ICR = 0xFFFFFFFF; // Clear all flags

    // Configure transfer: START + device address + register address + RESTART + device address + read
    I2C4->CR2 = (AK09916_I2C_ADDR << 1) | (1 << I2C_CR2_NBYTES_Pos) | (1 << I2C_CR2_START_Pos); // SADD, RD_WRN=0, START, NBYTES=1

    // Send register address
    I2C4->TXDR = reg;

    // Wait for transfer complete
    if (!i2c_wait_flag(I2C4->ISR, I2C_ISR_TC, true, TIMEOUT_MS)) {
        I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP
        return false; // timeout
    }

    // Configure for read: RESTART + device address + read 1 byte + STOP
    I2C4->CR2 = (AK09916_I2C_ADDR << 1) | (1 << I2C_CR2_RD_WRN_Pos) | (1 << I2C_CR2_START_Pos) | (1 << I2C_CR2_NBYTES_Pos); // RD_WRN=1, START, NBYTES=1

    // Wait for receive data
    if (!i2c_wait_flag(I2C4->ISR, I2C_ISR_RXNE, true, TIMEOUT_MS)) {
        I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP
        return false; // timeout
    }

    // Read the data
    data = I2C4->RXDR;

    I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP
    return true;
}

// Write a register to AK09916 using raw I2C register access
bool AP_Periph_FW::ak09916_write_register(uint8_t reg, uint8_t data)
{
    I2C4->ICR = 0xFFFFFFFF; // Clear all flags

    // Configure transfer: START + device address + register address + data + STOP
    I2C4->CR2 = (AK09916_I2C_ADDR << 1) | (2 << I2C_CR2_NBYTES_Pos) | I2C_CR2_START; // SADD, NBYTES=2, START, STOP

    if (!i2c_wait_flag(I2C4->ISR, I2C_ISR_TXIS, true, TIMEOUT_MS)) {
        I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP
        return false; // timeout
    }

    // Send register address
    I2C4->TXDR = reg;

    // Wait for TX ready
    if (!i2c_wait_flag(I2C4->ISR, I2C_ISR_TXIS, true, TIMEOUT_MS)) {
        I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP
        return false; // timeout
    }

    // Send data
    I2C4->TXDR = data;

    if (!i2c_wait_flag(I2C4->ISR, I2C_ISR_TC, true, TIMEOUT_MS)) {
        I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP
        return false; // timeout
    }

    I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP

    // Wait for STOP condition
    if (!i2c_wait_flag(I2C4->ISR, I2C_ISR_STOPF, true, TIMEOUT_MS)) {
        I2C4->CR2 |= I2C_CR2_STOP; // Generate STOP
        return false; // timeout
    }
    return true;
}

static void i2c_serve_interrupt(uint32_t isr)
{
    I2C2->ICR = isr & 0x3F38; // clear all interrupt flags we are servicing
    if (isr & (1<<3)) { // ADDR
        periph.i2c2_transfer_address = (isr >> 17) & 0x7FU; // ADDCODE
        periph.i2c2_transfer_direction = (isr >> 16) & 1; // direction
        periph.i2c2_transfer_byte_idx = 0;
        if (periph.i2c2_transfer_direction) {
            I2C2->ISR |= (1<<0); // TXE
        }
    }

    if (isr & I2C_ISR_RXNE) {
        uint8_t recv_byte = I2C2->RXDR & 0xff;; // reading clears our interrupt flag
        switch(periph.i2c2_transfer_address) {
            case TOSHIBALED_I2C_ADDRESS:
                periph.toshibaled_interface_recv_byte(periph.i2c2_transfer_byte_idx, recv_byte);
                break;
            case AK09916_I2C_ADDR:
                if (!periph.ak09916_recv_byte(periph.i2c2_transfer_byte_idx, recv_byte)) {
                    // nack
                    I2C2->CR2 |= I2C_CR2_NACK;
                }
                break;
            case RM3100_I2C_ADDR1:
            case RM3100_I2C_ADDR2:
            case RM3100_I2C_ADDR3:
            case RM3100_I2C_ADDR4:
                periph.compass_recv_byte(periph.i2c2_transfer_byte_idx, recv_byte);
                break;
        }
        periph.i2c2_transfer_byte_idx++;
    }

    if (isr & I2C_ISR_TXIS) {
        switch(periph.i2c2_transfer_address) {
            case TOSHIBALED_I2C_ADDRESS:
                I2C2->TXDR = 0x0; //TODO, return actual data
                break;
            case AK09916_I2C_ADDR: {
                uint8_t data = 0;
                if (periph.ak09916_read_register(periph.ak09916_transfer_reg + periph.i2c2_transfer_byte_idx, data)) {
                    I2C2->TXDR = data;
                } else {
                    // send NACK
                    I2C2->CR2 |= I2C_CR2_NACK;
                    I2C2->TXDR = 0xFF;
                }
            }
                break;
            case RM3100_I2C_ADDR1:
            case RM3100_I2C_ADDR2:
            case RM3100_I2C_ADDR3:
            case RM3100_I2C_ADDR4:
                I2C2->TXDR = periph.compass_send_byte(periph.compass_reg + periph.i2c2_transfer_byte_idx);
                break;
        }
        periph.i2c2_transfer_byte_idx++;
    }
}

OSAL_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER);
OSAL_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER) {
    uint32_t isr = I2C2->ISR;

    OSAL_IRQ_PROLOGUE();

    i2c_serve_interrupt(isr);

    OSAL_IRQ_EPILOGUE();
}
#endif
