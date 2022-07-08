#ifndef LPC17xx_SSP_H
#define LPC17xx_SSP_H

#include <libopencm3/cm3/common.h>
#include <libopencm3/lpc17xx/memorymap.h>

#define SPI_P             SPI_BASE  /* APB0 */
#define SSP0_P            SSP0_BASE /* APB1 */
#define SSP1_P            SSP1_BASE /* APB0 */


#define SSP_CR0(port)                   MMIO32((port) + 0x00)
#define SSP_CR1(port)                   MMIO32((port) + 0x04)
#define SSP_DR(port)                    MMIO32((port) + 0x08)
#define SSP_SR(port)                    MMIO32((port) + 0x0C)
#define SSP_CPSR(port)                  MMIO32((port) + 0x10)
#define SSP_IMSC(port)                  MMIO32((port) + 0x14)
#define SSP_RIS(port)                   MMIO32((port) + 0x18)
#define SSP_MIS(port)                   MMIO32((port) + 0x1C)
#define SSP_ICR(port)                   MMIO32((port) + 0x20)
#define SSP_DMACR(port)                 MMIO32((port) + 0x24)

// SSP_CR0 register fields:
// Data Size Select bits 3:0
#define SSP_DSS_MASK                    0x0F
#define SSP_DSS_4bit                    0x03
#define SSP_DSS_5bit                    0x04
#define SSP_DSS_6bit                    0x05
#define SSP_DSS_7bit                    0x06
#define SSP_DSS_8bit                    0x07
#define SSP_DSS_9bit                    0x08
#define SSP_DSS_10bit                   0x09
#define SSP_DSS_11bit                   0x0A
#define SSP_DSS_12bit                   0x0B
#define SSP_DSS_13bit                   0x0C
#define SSP_DSS_14bit                   0x0D
#define SSP_DSS_15bit                   0x0E
#define SSP_DSS_16bit                   0x0F
//Frame Format bits 5:4
#define SSP_FRF_MASK                    0x30
#define SSP_FRF_SPI                     0x00
#define SSP_FRF_TI                      0x10
#define SSP_FRF_uW                      0x20
//Clock out polarity bit 6
#define SSP_CPOL_MASK                   0x40
#define SSP_CPOL_BUS_L                  0x00
#define SSP_CPOL_BUS_H                  0x40
//Clock out phase bit 7
#define SSP_CPHA_MASK                   0x80
#define SSP_CPHA_FIRST_EDGE             0x00
#define SSP_CPHA_SECOND                 0x80
//Serial Clock Rate bits 15:8
//the frequency formula is PCLK/(CPSDVSR*[SCR+1])
#define SSP_SCR_MASK                    0xFF00
#define SSP_SCR(_cr)                     ((_cr)<<8)

// SSP_CR1 register fields:
// Loop Back Mode bit 0
#define SSP_LBM_MASK                    0x01
#define SSP_LBM_DIS                     0x00
#define SSP_LBM_EN                      0x01
//SSP_EN bit 1
#define SSP_EN_MASK                     0x02
#define SSP_EN                          SSP_EN_MASK
#define SSP_DIS                         0x00
//Master Slave Mode bit 2
#define SSP_MS_MODE_MASK                0x40
#define SSP_MS_MASTER                   0x00
#define SSP_MS_SLAVE                    SSP_MS_MODE_MASK
//Slave Output Disable bit 3
#define SSP_SOD_MASK                    0x80
#define SSP_SO_EN                       0x00
#define SSP_SO_DIS                      SSP_SOD_MASK

// SSP_DR register fileds:
// Data register bits 15:0
#define SSP_DR_MASK                     0xFFFF

// SSP_SR (read-only)
//Transmit FIFO empty; this bit is 1 if the tx FIFO is empty
#define SSP_TFE_MASK                    0x01
//Transmit FIFO not full: this bit is 0 if the tx FIFO is full
#define SSP_TNF_MASK                    0x02
//Receive FIFO not empty: this bit is 0 if the rx FIFO is empty
#define SSP_RNE_MASK                    0x04
//Receive FIFO is full: this bit is 1 if rx FIFO is full
#define SSP_RFF_MASK                    0x08
//Busy: this bit is 0 if the SSP controller is idle
#define SSP_BSY_MASK                    0x10

//SSP_CPSR register field
//Clock Prescaler Register
#define SSP_CPSR_MASK                   0xFF

//SSP_IMSC register fields
//Receive Overrun bit 0
#define SSP_RORIM_MASK                  0x01
#define SSP_RORIM_EN                    SSP_RORIM_MASK
#define SSP_RORIM_DIS                   0x00
//Receive Time Out bit 1
#define SSP_RTIM_MASK                   0x02
#define SSP_RTIM_EN                     SSP_RTIM_MASK
#define SSP_RTIM_DIS                    0x00
//Rx FIFO is at least half full interrupt
#define SSP_RXIM_MASK                   0x04
#define SSP_RXIM_EN                     SSP_RXIM_MASK
#define SSP_RXIM_DIS                    0x00
//Tx FIFO is at least half empty interrupt
#define SSP_TXIM_MASK                   0x08
#define SSP_TXIM_EN                     SSP_TXIM_MASK
#define SSP_TXIM_DIS                    0x00

//SSP_RIS register fields
//Rx Overrun bit 0
//This bit is 1 if another frame was received while the rx FIFO was still full
#define SSP_RORRIS_MASK                 0x01
//Rx Timeout bit 1
//This bit is 1 if rx FIFO was not empty and no data was read for a timeout period
#define SSP_RTRIS_MASK                  0x02
//Rx FIFO status bit 2
//This bit is 1 if the rx fifo is at least half full
#define SSP_RXIS_MASK                   0x04
//Tx FIFO status bit 3
//This bit is 1 if at least half of tx fifo is empty
#define SSP_XRIS_MASK                   0x08

//SSP_ICR register fields
//Read Overrun clear interrupt flag
#define SSP_RORIC                       0x01
//Read timeout clear interrupt flag
#define SSP_RTIC                        0x02

typedef enum {
	SPI = SPI_P,
	SSP0 = SSP0_P,
	SSP1 = SSP1_P
} ssp_num_t;

typedef enum {
	SSP_SUCCESS,
	SSP_ERROR
} ssp_ret_t;

BEGIN_DECLS

ssp_ret_t ssp_init(ssp_num_t spi_port, uint8_t spi_hard_ctrl_ssel);
uint16_t ssp_read(ssp_num_t ssp_num);
void ssp_write(ssp_num_t spi_num, uint16_t data);
uint16_t spi_xfer(ssp_num_t spi_num, uint16_t data);

END_DECLS

#endif