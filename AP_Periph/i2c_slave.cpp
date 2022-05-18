#include "AP_Periph.h"
#include <ch.h>
#include <hal.h>

#define TOSHIBALED_I2C_ADDRESS 0x55
#define RM3100_I2C_ADDR1 0x20
#define RM3100_I2C_ADDR2 0x21
#define RM3100_I2C_ADDR3 0x22
#define RM3100_I2C_ADDR4 0x23
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

    I2C2->OAR1 = (RM3100_I2C_ADDR1 & 0xFF) << 1; //Emulate AK09916 I2C Slave
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
}

void AP_Periph_FW::toshibaled_interface_recv_byte(uint8_t recv_byte_idx, uint8_t recv_byte)
{
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

void AP_Periph_FW::rm3100_recv_byte(uint8_t idx, uint8_t byte)
{
    if (idx == 0) {
        // this is a register address
        rm3100_reg = byte;
    } else {
        // write is requested
        chSysLockFromISR();
        periph.rm3100_write_requested = true;
        periph.i2c_event_source.signalI(EVENT_MASK(0));
        chSysUnlockFromISR();
        rm3100_reg_val = byte;
    }
}

void i2c_serve_interrupt(uint32_t isr)
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
            case RM3100_I2C_ADDR1:
            case RM3100_I2C_ADDR2:
            case RM3100_I2C_ADDR3:
            case RM3100_I2C_ADDR4:
                periph.rm3100_recv_byte(periph.i2c2_transfer_byte_idx, recv_byte);
                break;
        }
        periph.i2c2_transfer_byte_idx++;
    }

    if (isr & I2C_ISR_TXIS) {
        switch(periph.i2c2_transfer_address) {
            case RM3100_I2C_ADDR1:
            case RM3100_I2C_ADDR2:
            case RM3100_I2C_ADDR3:
            case RM3100_I2C_ADDR4:
                if (periph.rm3100_reg == 0x34) {
                    if (periph.compass.raw_data_available()) {
                        I2C2->TXDR = 0x80;
                    } else {
                        I2C2->TXDR = 0x00;
                    }
                } else if (periph.rm3100_reg == 0x24) {
                    uint8_t data[9] = {};
                    periph.compass.get_raw_data(data, sizeof(data));
                    I2C2->TXDR = data[periph.i2c2_transfer_byte_idx];
                } else {
                    I2C2->TXDR = periph.compass.get_reg_value(periph.rm3100_reg);
                }
                break;
        }
        periph.i2c2_transfer_byte_idx++;
    }

    if (isr & I2C_ISR_STOPF) {
        I2C2->ICR |= I2C_ISR_STOPF; // STOPCF
    }
}

OSAL_IRQ_HANDLER(STM32_I2C2_EVENT_HANDLER) {
    uint32_t isr = I2C2->ISR;

    OSAL_IRQ_PROLOGUE();

    i2c_serve_interrupt(isr);

    OSAL_IRQ_EPILOGUE();
}
