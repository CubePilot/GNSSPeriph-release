#include "AP_Periph.h"
#include <ch.h>
#include <hal.h>
#ifdef I2C_SLAVE_ENABLED
#define TOSHIBALED_I2C_ADDRESS 0x55
#define RM3100_I2C_ADDR1 0x20
#define RM3100_I2C_ADDR2 0x21
#define RM3100_I2C_ADDR3 0x22
#define RM3100_I2C_ADDR4 0x23
#define AK09916_I2C_ADDR 0x0C
#define HAL_I2C_H7_400_TIMINGR 0x00300F38
extern const AP_HAL::HAL &hal;

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

    I2C2->OAR1 = (AK09916_I2C_ADDR & 0xFF) << 1; //Emulate AK09916 I2C Slave
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

    hal.spi->set_register_rw_callback("rm3100", FUNCTOR_BIND_MEMBER(&AP_Periph_FW::compass_register_rw_callback, void, uint8_t, uint8_t*, uint32_t, bool));

    // Initialize I2C4 for AK09916 master communication
    ak09916_i2c_init();
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

void AP_Periph_FW::ak09916_recv_byte(uint8_t idx, uint8_t byte)
{
    if (idx == 0) {
        // First byte is the register address
        ak09916_transfer_reg = byte;
    } else {
        // Subsequent bytes are data to write to the register
        // Use raw I2C register access for direct hardware control
        ak09916_write_register(ak09916_transfer_reg, byte);
    }
}

uint8_t AP_Periph_FW::ak09916_send_byte(uint8_t reg)
{
    // Use raw I2C register access for direct hardware control
    return ak09916_read_register(reg);
}

// Initialize I2C4 for AK09916 master communication
void AP_Periph_FW::ak09916_i2c_init()
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
}

// Read a register from AK09916 using raw I2C register access
uint8_t AP_Periph_FW::ak09916_read_register(uint8_t reg)
{
    uint8_t data = 0;

    // Wait for I2C4 to be ready
    while ((I2C4->ISR & I2C_ISR_BUSY) != 0) {
        // Busy wait - should be very short
    }

    // Configure transfer: START + device address + register address + RESTART + device address + read
    I2C4->CR2 = (AK09916_I2C_ADDR << 1) | (1 << 10) | (1 << 13) | (1 << 16); // SADD, RD_WRN=0, START, NBYTES=1

    // Send register address
    I2C4->TXDR = reg;

    // Wait for transfer complete
    while ((I2C4->ISR & I2C_ISR_TC) == 0) {
        // Wait for transmission complete
    }

    // Configure for read: RESTART + device address + read 1 byte + STOP
    I2C4->CR2 = (AK09916_I2C_ADDR << 1) | (1 << 10) | (1 << 13) | (1 << 14) | (1 << 16); // RD_WRN=1, START, STOP, NBYTES=1

    // Wait for receive data
    while ((I2C4->ISR & I2C_ISR_RXNE) == 0) {
        // Wait for data
    }

    // Read the data
    data = I2C4->RXDR;

    // Wait for STOP condition
    while ((I2C4->ISR & I2C_ISR_STOPF) == 0) {
        // Wait for stop
    }

    // Clear STOP flag
    I2C4->ICR |= I2C_ICR_STOPCF;

    return data;
}

// Write a register to AK09916 using raw I2C register access
void AP_Periph_FW::ak09916_write_register(uint8_t reg, uint8_t data)
{
    // Wait for I2C4 to be ready
    while ((I2C4->ISR & I2C_ISR_BUSY) != 0) {
        // Busy wait - should be very short
    }

    // Configure transfer: START + device address + register address + data + STOP
    I2C4->CR2 = (AK09916_I2C_ADDR << 1) | (2 << 16) | (1 << 13) | (1 << 14); // SADD, NBYTES=2, START, STOP

    // Send register address
    I2C4->TXDR = reg;

    // Wait for TX ready
    while ((I2C4->ISR & I2C_ISR_TXIS) == 0) {
        // Wait for TX buffer empty
    }

    // Send data
    I2C4->TXDR = data;

    // Wait for STOP condition
    while ((I2C4->ISR & I2C_ISR_STOPF) == 0) {
        // Wait for stop
    }

    // Clear STOP flag
    I2C4->ICR |= I2C_ICR_STOPCF;
}

static void i2c_serve_interrupt(uint32_t isr)
{
    if (isr & (1<<3)) { // ADDR
        periph.i2c2_transfer_address = (isr >> 17) & 0x7FU; // ADDCODE
        periph.i2c2_transfer_direction = (isr >> 16) & 1; // direction
        periph.i2c2_transfer_byte_idx = 0;
        if (periph.i2c2_transfer_direction) {
            I2C2->ISR |= (1<<0); // TXE
        }
        I2C2->ICR |= (1<<3); // ADDRCF
    }

    if (isr & I2C_ISR_RXNE) {
        uint8_t recv_byte = I2C2->RXDR & 0xff;; // reading clears our interrupt flag
        switch(periph.i2c2_transfer_address) {
            case TOSHIBALED_I2C_ADDRESS:
                periph.toshibaled_interface_recv_byte(periph.i2c2_transfer_byte_idx, recv_byte);
                break;
            case AK09916_I2C_ADDR:
                periph.ak09916_recv_byte(periph.i2c2_transfer_byte_idx, recv_byte);
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
            case AK09916_I2C_ADDR:
                I2C2->TXDR = periph.ak09916_send_byte(periph.ak09916_transfer_reg + periph.i2c2_transfer_byte_idx);
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

    if (isr & I2C_ISR_STOPF) {
        I2C2->ICR |= I2C_ISR_STOPF; // STOPCF
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
