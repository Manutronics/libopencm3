/* 
* This code is provided "as is" and without any warranty. This file now can be joined to libopencm3
* and follow any legal disclaimer requested by the original project. 
* 
* coded by manutronics
*/

#ifndef LPC17xx_I2C_H
#define LPC17xx_I2C_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/lpc17xx/memorymap.h>

#define I2C0                           I2C0_BASE /* APB0 */
#define I2C1                           I2C1_BASE /* APB0 */
#define I2C2                           I2C2_BASE /* APB1 */

#define I2C_SUCCESS                     (0x00)
#define I2C_BAD_CONFIG                  (0xFF)
#define I2C_BUS_INCOSISTENT             (0xFE)

#define I2C_CONSET(port)                MMIO32((port) + 0x000)
#define I2C_STAT(port)                  MMIO32((port) + 0x004)
#define I2C_DAT(port)                   MMIO32((port) + 0x008)
#define I2C_ADR0(port)                  MMIO32((port) + 0x00C)
#define I2C_SCLH(port)                  MMIO32((port) + 0x010)
#define I2C_SCLL(port)                  MMIO32((port) + 0x014)
#define I2C_CONCLR(port)                MMIO32((port) + 0x018)
#define I2C_MMCTRL(port)                MMIO32((port) + 0x01C)
#define I2C_ADR1(port)                  MMIO32((port) + 0x020)
#define I2C_ADR2(port)                  MMIO32((port) + 0x024)
#define I2C_ADR3(port)                  MMIO32((port) + 0x028)
#define I2C_DATABUFFER(port)            MMIO32((port) + 0x02C)
#define I2C_MASK0(port)                 MMIO32((port) + 0x030)
#define I2C_MASK1(port)                 MMIO32((port) + 0x034)
#define I2C_MASK2(port)                 MMIO32((port) + 0x038)
#define I2C_MASK3(port)                 MMIO32((port) + 0x03C)

#define I2C_WRITE_ADDR(addr)    (addr & 0xFE)
#define I2C_READ_ADDR(addr)     (addr | 0x01)
#define I2C_READ_MODE(addr)     (addr & 0x01)

#define I2C_CONTROL_AA                  (1<<2)
#define I2C_CONTROL_SI                  (1<<3)
#define I2C_CONTROL_STO                 (1<<4)
#define I2C_CONTROL_START               (1<<5)
#define I2C_CONTROL_EN                  (1<<6)

typedef enum {
    I2C0_NUM = I2C0,
    I2C1_NUM = I2C1,
    I2C2_NUM = I2C2
} i2c_num_t;

typedef enum {
    I2C0_P0_27_SDA_P0_28_SCL,
    I2C1_P0_0_SDA_P0_1_SCL,
    I2C1_P0_19_SDA_P0_20_SCL,
    I2C2_P0_10_SDA_P0_11_SCL
} i2c_pin_t;

typedef enum {
    I2C_PCLKSEL_DIV_4,
    I2C_PCLKSEL_DIV_1,
    I2C_PCLKSEL_DIV_2,
    I2C_PCLKSEL_DIV_8
} i2c_clk_div_t;

typedef enum {
    busy,
    writeComplete,
    readComplete,
    error,
    idle
}  i2c_states;

typedef enum {
    NO_STOP,
    SND_STOP
} i2c_stop_rule_t;

typedef struct
{
    uint32_t xfer_size;   ///< # of bytes to transfer.
    uint8_t slave_addr;  ///< Slave Device Address
    uint8_t error;      ///< Error if any occurred within I2C
    uint8_t *p_data;  ///< Buffer of the I2C Read or Write
    i2c_states status; 
    i2c_stop_rule_t stop;
} xI2C_transaction_t;

BEGIN_DECLS

void wait_i2c_transfer_finish(void);
uint8_t i2c_init(i2c_num_t i2c, i2c_clk_div_t i2c_pclk_div, uint32_t bus_clock, i2c_pin_t pins, xI2C_transaction_t *tr);
void i2c_send_no_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t *data, uint32_t data_len);
void i2c_send_8bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t internal_addr, uint8_t *data, uint32_t data_len);
void i2c_send_16bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint16_t internal_addr, uint8_t *data, uint32_t data_len);
void i2c_read_no_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t *data, uint32_t data_len);
void i2c_read_8bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint8_t internal_addr, uint8_t *data, uint32_t data_len);
void i2c_read_16bits_internal_addr(i2c_num_t i2c, uint8_t dev_addr, uint16_t internal_addr, uint8_t *data, uint32_t data_len);
i2c_states i2cStateMachine(i2c_num_t i2c);

END_DECLS

#endif //LPC17xx_I2C_H