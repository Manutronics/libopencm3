/* 
* This code is provided "as is" and without any warranty. This file now can be joined to libopencm3
* and follow any legal disclaimer requested by the original project. 
* 
* coded by manutronics
*/

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/lpc17xx/memorymap.h>
#include <libopencm3/lpc17xx/pwr.h>
#include <libopencm3/lpc17xx/i2c.h>
#include <libopencm3/lpc17xx/clock.h>
#include <libopencm3/lpc17xx/gpio.h>

//difinitios of the states of the LPC176x/5x I2C 
//Common states
#define I2C_BUS_ERROR       0x00
#define I2C_START           0x08
#define I2C_R_START         0x10
#define I2C_ARBIT_LOST      0x38

//As Master Transmitter possibles states are
#define I2C_ADDR_TX_ACK     0x18
#define I2C_ADDRN_TX_ACK    0x20
#define I2C_DATA_TX_ACK     0x28
#define I2C_DATA_TX_NACK    0x30

//As Master Recevier possibles states are
#define I2C_ADDR_RX_ACK     0x40
#define I2C_ADDR_RX_NACK    0x48
#define I2C_DATA_RX_ACK     0x50
#define I2C_DATA_RX_NACK    0x58

static volatile xI2C_transaction_t *i2c_transaction;

void i2c_start_transfer(i2c_num_t i2c, uint8_t dev_addr, uint8_t* data, uint32_t data_len, i2c_stop_rule_t stop);

uint8_t i2c_init(i2c_num_t i2c, i2c_clk_div_t i2c_pclk_div, uint32_t bus_clock, i2c_pin_t pins, xI2C_transaction_t *tr)
{
    volatile uint32_t *pclksel;
    uint32_t i2c_num = 0;
    uint32_t shift;
    uint8_t i2c_nvic;
    uint8_t i2c_cpu_clk_div = 0;
    uint32_t i2c_pin_mask = 0;
    i2c_transaction = tr;

    switch (i2c)
    {
    case I2C0:
        i2c_num = PWR_PCONP_I2C0;
        i2c_nvic = NVIC_I2C0_IRQ;
        pclksel = &CLK_PCLKSEL0;
        shift = CLK_PCLKSEL0_I2C0_SHIFT;
        i2c_pin_mask = ((1<<27) | (1<<28));
        // PIN configuration for P0[19] P0[20]
        // I2C0 on LPC17xx are open drain by default and has not option to enable pull-down
        PINCON_PINSEL1 &= ~(0xF << 22);  // Clear
        PINCON_PINSEL1 |=  (0x5 << 22);  // Enable I2C Pins: SDA, SCL
        break;
    case I2C1:
        i2c_num = PWR_PCONP_I2C1;
        i2c_nvic = NVIC_I2C1_IRQ;
        pclksel = &CLK_PCLKSEL1;
        shift = CLK_PCLKSEL1_I2C1_SHIFT;
        if(pins == I2C1_P0_19_SDA_P0_20_SCL)
        {
            // PIN configuration for P0[19] P0[20]
            i2c_pin_mask = ((1<<19) | (1<<20));
            PINCON_PINMODE1 &= ~(0xF << 6);  // Both pins with Pull-Up Enabled
            PINCON_PINMODE1 |=  (0xA << 6);  // Disable both pull-up and pull-down
            PINCON_PINMODE_OD0 |= (3 << 19); // Enable Open-drain
            PINCON_PINSEL1 &= ~(0xF << 6);   // Clear
            PINCON_PINSEL1 |=  (0xF << 6);   // Enable I2C Pins: SDA, SCL
        }
        else if(pins == I2C1_P0_0_SDA_P0_1_SCL)
        {
            // PIN configuration for P0[0] P0[1]
            i2c_pin_mask = ((1<<0) | (1<<1));
            PINCON_PINMODE0 &= ~(0xF << 0); // Both pins with Pull-Up Enabled
            PINCON_PINMODE0 |=  (0xA << 0); // Disable both pull-up and pull-down
            PINCON_PINMODE_OD0 |= (3 << 0); // Enable Open-drain for
            PINCON_PINSEL0 &= ~(0xF << 0);  // Clear
            PINCON_PINSEL0 |=  (0xF << 0);  // Enable I2C Pins: SDA, SCL
        }
        break;
    case I2C2:
        i2c_num = PWR_PCONP_I2C2;
        i2c_nvic = NVIC_I2C2_IRQ;
        pclksel = &CLK_PCLKSEL1;
        shift = CLK_PCLKSEL1_I2C2_SHIFT;
        // PIN configuration for P0[10] P0[11]
        i2c_pin_mask = ((1<<10) | (1<<11));
        PINCON_PINMODE0 &= ~(0xF << 20); // Both pins with Pull-Up Enabled
        PINCON_PINMODE0 |=  (0xA << 20); // Disable both pull-up and pull-down
        PINCON_PINMODE_OD0 |= (3 << 10); // Enable Open-drain for I2C2 on pins P0.10 and P0.11
        PINCON_PINSEL0 &= ~(0xF << 20);  // Clear
        PINCON_PINSEL0 |=  (0xA << 20);  // Enable I2C Pins: SDA, SCL
        break;
    default:
        return I2C_BAD_CONFIG;
    }

    if(i2c_pclk_div==I2C_PCLKSEL_DIV_8)
        i2c_cpu_clk_div = 3;
    else if(i2c_pclk_div==I2C_PCLKSEL_DIV_4)
        i2c_cpu_clk_div = 2;
    else if(i2c_pclk_div==I2C_PCLKSEL_DIV_2)
        i2c_cpu_clk_div = 1;

    pwr_enable_peripherals(i2c_num);
    *pclksel |= ((uint32_t)i2c_pclk_div<<shift);

    if(!(GPIO0_PIN & i2c_pin_mask))
        return I2C_BUS_INCOSISTENT;

    I2C_CONCLR(i2c) = I2C_CONTROL_AA|I2C_CONTROL_SI|I2C_CONTROL_START|I2C_CONTROL_EN;
    const uint32_t percent_high = 40;
    const uint32_t percent_low = (100 - percent_high);
    const uint32_t freq_hz = (bus_clock > 1000) ? (100 * 1000) : (bus_clock * 1000);
    const uint32_t half_clock_divider = ((clock_get_cpu_speed()>>i2c_cpu_clk_div) / freq_hz) / 2;
    uint32_t sclh = (half_clock_divider * percent_high) / 100;
    uint32_t scll = (half_clock_divider * percent_low ) / 100;

    I2C_SCLH(i2c) = sclh;
    I2C_SCLL(i2c) = scll;
    I2C_CONSET(i2c) = I2C_CONTROL_EN; //enable the i2c peripheral
    nvic_enable_irq(i2c_nvic);
    i2c_transaction->status = idle;
    return I2C_SUCCESS;
}

void i2c_send_no_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t *data, uint32_t data_len)
{
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_WRITE_ADDR(dev_addr), data, data_len, SND_STOP);
}

void i2c_send_8bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t internal_addr, uint8_t *data, uint32_t data_len)
{
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_WRITE_ADDR(dev_addr), &internal_addr, 1, NO_STOP);
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_WRITE_ADDR(dev_addr), data, data_len, SND_STOP);
}

void i2c_send_16bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint16_t internal_addr, uint8_t *data, uint32_t data_len)
{
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_WRITE_ADDR(dev_addr), &internal_addr, 2, NO_STOP);
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_WRITE_ADDR(dev_addr), data, data_len, SND_STOP);
}

void i2c_read_no_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t *data, uint32_t data_len)
{
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_READ_ADDR(dev_addr), data, data_len, SND_STOP);
}

void i2c_read_8bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t internal_addr, uint8_t *data, uint32_t data_len)
{
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_WRITE_ADDR(dev_addr), &internal_addr, 1, NO_STOP);
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_READ_ADDR(dev_addr), data, data_len, SND_STOP);
}

void i2c_read_16bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint16_t internal_addr, uint8_t *data, uint32_t data_len)
{
    uint8_t int_addr[2];
    int_addr[0] = (uint8_t)internal_addr>>8;
    int_addr[1] = (uint8_t)internal_addr;
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_WRITE_ADDR(dev_addr), int_addr, 2, NO_STOP);
    while(i2c_transaction->status == busy);
    i2c_start_transfer(i2c, I2C_READ_ADDR(dev_addr), data, data_len, SND_STOP);
}

void wait_i2c_transfer_finish(void)
{
    while(i2c_transaction->status == busy);
}

void i2c_start_transfer(i2c_num_t i2c, uint8_t dev_addr, uint8_t* data, uint32_t data_len, i2c_stop_rule_t stop)
{
    i2c_transaction->error = 0;
    i2c_transaction->slave_addr = dev_addr;
    i2c_transaction->xfer_size = data_len;
    i2c_transaction->p_data = data;
    i2c_transaction->stop = stop;
    i2c_transaction->status = busy;

    // Send START, I2C State Machine will finish the rest.
    I2C_CONSET(i2c) = I2C_CONTROL_START;
}

i2c_states i2cStateMachine(i2c_num_t i2c)
{
    switch (I2C_STAT(i2c))
    {
        case I2C_START://0x08
        case I2C_R_START://0x10
            I2C_DAT(i2c) = i2c_transaction->slave_addr;
            I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            break;

        case I2C_ADDR_TX_ACK://0x18
        case I2C_DATA_TX_ACK://0x28
            I2C_CONCLR(i2c) = I2C_CONTROL_START;

            if(0 == i2c_transaction->xfer_size)
            {
                // set_stop(i2c, &state);
                i2c_transaction->status = writeComplete;
                if(i2c_transaction->stop == SND_STOP)
                {
                    I2C_CONCLR(i2c) = I2C_CONTROL_START;
                    I2C_CONSET(i2c) = I2C_CONTROL_STO;
                    I2C_CONCLR(i2c) = I2C_CONTROL_SI;
                    while((I2C_CONSET(i2c)&I2C_CONTROL_STO));
                }
            }
            else
            {
                I2C_DAT(i2c) = *i2c_transaction->p_data;
                ++i2c_transaction->p_data;
                --i2c_transaction->xfer_size;
                I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            }

            I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            break;
        case I2C_ADDR_RX_ACK://0x40
            I2C_CONCLR(i2c) = I2C_CONTROL_START;

            if(i2c_transaction->xfer_size > 1)
            {
                //ACK to receive a byte and transition to dataAvailableAckSent
                I2C_CONSET(i2c) = I2C_CONTROL_AA;
            }
            else
            {
                //NACK next byte to go to dataAvailableNackSent for 1-byte read.
                I2C_CONCLR(i2c) = I2C_CONTROL_AA;
            }

            I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            break;
        case I2C_DATA_RX_ACK://0x50
            *i2c_transaction->p_data = I2C_DAT(i2c);
            ++i2c_transaction->p_data;
            --i2c_transaction->xfer_size;

            if(1 == i2c_transaction->xfer_size)
            {
                //NACK next byte --> Next state: dataAvailableNackSent
                I2C_CONCLR(i2c) = I2C_CONTROL_AA;
            }
            else
            {
                //ACK next byte --> Next state: dataAvailableAckSent(back to this state)
                I2C_CONSET(i2c) = I2C_CONTROL_AA;
            }

            I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            break;
        case I2C_DATA_RX_NACK: //0x58 Read last-byte from Slave
            *i2c_transaction->p_data = I2C_DAT(i2c);
            // set_stop(i2c, &state);
            i2c_transaction->status = readComplete;
            I2C_CONCLR(i2c) = I2C_CONTROL_START;
            I2C_CONSET(i2c) = I2C_CONTROL_STO;
            I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            while((I2C_CONSET(i2c)&I2C_CONTROL_STO));
            I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            break;

        case I2C_ARBIT_LOST://0x38
            i2c_transaction->status = I2C_READ_MODE(i2c_transaction->slave_addr) ? readComplete : writeComplete;
            i2c_transaction->error = I2C_STAT(i2c);
            break;

        case I2C_ADDRN_TX_ACK://slave Address Nacked:    //0x20 no break
        case I2C_DATA_TX_NACK://data Nacked By Slave:     //0x30 no break
        case I2C_ADDR_RX_NACK://read Mode Nacked By Slave: //0x48 no break
        case I2C_BUS_ERROR:   // no break //0x00
        default:
            i2c_transaction->error = I2C_STAT(i2c);
            I2C_CONCLR(i2c) = I2C_CONTROL_START;
            I2C_CONSET(i2c) = I2C_CONTROL_STO;
            I2C_CONCLR(i2c) = I2C_CONTROL_SI;
            while((I2C_CONSET(i2c)&I2C_CONTROL_STO));
            break;
    }
}
