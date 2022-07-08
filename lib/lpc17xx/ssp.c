#include <libopencm3/lpc17xx/ssp.h>
#include <libopencm3/lpc17xx/gpio.h>
#include <libopencm3/lpc17xx/clock.h>
#include <libopencm3/lpc17xx/pwr.h>

/*
* SSP Init function
*/
ssp_ret_t ssp_init(ssp_num_t ssp_port, uint8_t spi_hard_ctrl_ssel)
{
	volatile uint32_t *pclksel;
	uint32_t ssp_pwr_shift;
    uint32_t div, shift;
	uint32_t enable_hard_ctrl_ssel = 0xFFFFFFFF;

    div = CLK_PCLKSEL_DIV1;

	if(!spi_hard_ctrl_ssel)
	{
		enable_hard_ctrl_ssel = 0xFC;
	}

	switch (ssp_port)
	{
	case SPI:
        /* use PLL0 as clock source for SPI */
        // 
        pclksel = &CLK_PCLKSEL0;
        shift = CLK_PCLKSEL0_SPI_SHIFT;
        ssp_pwr_shift = PWR_PCONP_SPI;
        // Configure PINSEL for SPI pins p0.15->sck p0.16->ssel p0.17->miso p0.18->mosi
		PINCON_PINSEL0 &= ~(0x3 << 30); // Clear values
		PINCON_PINSEL0 |=  (0x3 << 30); // Set values for SPI
		PINCON_PINSEL1 &= ~(0x3F << 0); // Clear values
        PINCON_PINSEL1 |=  ((0x3F&enable_hard_ctrl_ssel) << 0); // Set values for SPI
		if(!spi_hard_ctrl_ssel)
		{
			GPIO0_SET = (1<<16);
			GPIO0_DIR |= (1<<16);
		}
		break;

	case SSP0:
		/* use PLL0 as clock source for SSP0 */
        pclksel = &CLK_PCLKSEL1;
        shift = CLK_PCLKSEL1_SSP0_SHIFT;
        ssp_pwr_shift = PWR_PCONP_SSP0;
		// Configure PINSEL for SSP pins p0.15->sck p0.16->ssel p0.17->miso p0.18->mosi
		PINCON_PINSEL0 &= ~(0x3 << 30); // Clear values
        PINCON_PINSEL0 |=  (0x2 << 30); // Set values for SSP
		PINCON_PINSEL1 &= ~(0x3F << 0); // Clear values
        PINCON_PINSEL1 |=  ((0x2A&enable_hard_ctrl_ssel) << 0); // Set values for SSP
		if(!spi_hard_ctrl_ssel)
		{
			GPIO0_SET = (1<<16);
			GPIO0_DIR |= (1<<16);
		}
		break;

	case SSP1:
		/* use PLL0 as clock source for SSP1 */
        pclksel = &CLK_PCLKSEL0;
        shift = CLK_PCLKSEL0_SSP1_SHIFT;
        ssp_pwr_shift = PWR_PCONP_SSP1;
		// Configure PINSEL for SSP pins p0.6->ssel p0.7->sck p0.8->miso p0.9->mosi
		PINCON_PINSEL0 &= ~(0xFF << 12); // Clear values
        PINCON_PINSEL0 |=  ((0xAA&enable_hard_ctrl_ssel) << 12); // Set values for SSP
		if(!spi_hard_ctrl_ssel)
		{
			// GPIO0_SET = (1<<6);
			// GPIO0_DIR |= (1<<6);
			gpio_set(GPIO0, GPIOPIN6);
			gpio_set_mode(GPIO0, GPIOPIN6, GPIO_OUTPUT);
		}
		break;
	default:
		return SSP_ERROR; /* error */
	}

	pwr_enable_peripherals(ssp_pwr_shift);
    *pclksel |= (div<<shift);

	SSP_CR0(ssp_port) = SSP_FRF_SPI|SSP_DSS_8bit|SSP_CPHA_FIRST_EDGE|SSP_CPOL_BUS_L;
	SSP_CPSR(ssp_port) = 60; //freezed to produce 2MHz spi clock
	SSP_CR1(ssp_port) = SSP_LBM_DIS|SSP_MS_MASTER|SSP_EN;
	return SSP_SUCCESS;
}

/*
* This Function reads the data from SSP rx fifo
*/
uint16_t ssp_read_blocking(ssp_num_t ssp_num)
{
	while(!(SSP_SR(ssp_num)&SSP_RNE_MASK));
	return SSP_DR(ssp_num);
}

/* This Function Wait Data TX Ready, and Write Data to SPI
	if rx_timeout_nb_cycles = 0 Infinite wait
*/
void ssp_write_blocking(ssp_num_t spi_num, uint16_t data)
{
	while(!(SSP_SR(spi_num)&SSP_TNF_MASK));
	SSP_DR(spi_num) = data;
}

uint16_t spi_xfer(ssp_num_t spi_num, uint16_t data)
{
	ssp_write_blocking(spi_num, data);
	return ssp_read_blocking(spi_num);
}