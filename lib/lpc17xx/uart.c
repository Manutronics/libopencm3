#include <libopencm3/lpc17xx/uart.h>
#include <libopencm3/lpc17xx/clock.h>
#include <libopencm3/lpc17xx/pwr.h>

/*
* UART Init function
*/
uart_error_t uart_init(    uart_num_t uart_num, uart_databit_t data_nb_bits,
                        uart_stopbit_t data_nb_stop, uart_parity_t data_parity,
                           uint32_t baud_rate)
{
    volatile uint32_t *pclksel;
    uint32_t uart_pwr_shift;
    uint32_t lcr_config = 0;
    uint32_t div, shift;

    div = CLK_PCLKSEL_DIV1;

    switch (uart_num) {
    case UART0_NUM:
        /* use PLL0 as clock source for UART0 */
        pclksel = &CLK_PCLKSEL0;
        shift = CLK_PCLKSEL0_UART0_SHIFT;
        uart_pwr_shift = PWR_PCONP_UART0;
        // Configure PINSEL for UART0
        PINCON_PINSEL0 &= ~(0xF << 4); // Clear values
        PINCON_PINSEL0 |=  (0x5 << 4); // Set values for UART0 Rx/Tx
        break;

    case UART1_NUM:
        /* use PLL0 as clock source for UART1 */
        pclksel = &CLK_PCLKSEL0;
        shift = CLK_PCLKSEL0_UART1_SHIFT;
        uart_pwr_shift = PWR_PCONP_UART1;
        // Configure PINSEL for UART0
        PINCON_PINSEL0 &= ~(0x3 << 30); // Clear values
        PINCON_PINSEL0 |=  (0x1 << 30); // Set values for UART1 Tx in p0.15
        PINCON_PINSEL1 &= ~(0x3 << 0); // Clear values
        PINCON_PINSEL1 |=  (0x1 << 0); // Set values for UART1 Rx in p0.16
#if 0
        // Configure PINSEL for UART2
        PINCON_PINSEL4 &= ~(0xF << 0); // Clear values
        PINCON_PINSEL4 |=  (0xA << 0); // Set values for UART2 Rx/Tx in p2.1/p2.0
#endif
        break;

    case UART2_NUM:
        /* use PLL0 as clock source for UART2 */
        pclksel = &CLK_PCLKSEL1;
        shift = CLK_PCLKSEL1_UART2_SHIFT;
        uart_pwr_shift = PWR_PCONP_UART2;
        // Configure PINSEL for UART2
        PINCON_PINSEL0 &= ~(0xF << 20); // Clear values
        PINCON_PINSEL0 |=  (0x5 << 20); // Set values for UART2 Rx/Tx in p0.11/p0.10
#if 0
        // Configure PINSEL for UART2
        PINCON_PINSEL4 &= ~(0xF << 16); // Clear values
        PINCON_PINSEL4 |=  (0xA << 16); // Set values for UART2 Rx/Tx in p2.9/p2.8
#endif
        break;

    case UART3_NUM:
        /* use PLL0 as clock source for UART3 */
        pclksel = &CLK_PCLKSEL1;
        shift = CLK_PCLKSEL1_UART3_SHIFT;
        uart_pwr_shift = PWR_PCONP_UART3;
        // Configure PINSEL for UART3
        PINCON_PINSEL0 &= ~(0xF << 0); // Clear values
        PINCON_PINSEL0 |=  (0x5 << 0); // Set values for UART3 Rx/Tx in p0.1/p0.0
#if 0
        PINCON_PINSEL1 &= ~(0xF << 18); // Clear values
        PINCON_PINSEL1 |=  (0xF << 18); // Set values for UART1 Rx/Tx in p0.26/p0.25
        //or
        PINCON_PINSEL9 &= ~(0xF << 24); // Clear values
        PINCON_PINSEL9 |=  (0xF << 24); // Set values for UART1 Rx/Tx in p4.29/p4.28
#endif
        break;

    default:
        return UART_CONIFG_ERROR; /* error */
    }

    pwr_enable_peripherals(uart_pwr_shift);
    *pclksel |= (div<<shift);

    /* Disable Tx */
    UART_TER(uart_num) = 0;

    /* Read LCR config & Force Enable of Divisor Latches Access */
    lcr_config |= data_nb_bits; /* Set Nb Data Bits */
    lcr_config |= data_nb_stop; /* Set Nb Stop Bits */
    lcr_config |= data_parity; /* Set Data Parity */

    UART_LCR(uart_num) = (1 << 7); // Enable DLAB to access DLM, DLL, and IER

    div = (uint16_t)((float)(clock_get_cpu_speed() / (16 * baud_rate)) + 0.5);

    UART_DLM(uart_num) = (div >> 8);
    UART_DLL(uart_num) = (div);

    /* Write LCR (only 7bits) */
    UART_LCR(uart_num) = (lcr_config & UART_LCR_BITMASK);  // Disable DLAB and set 8bit per char

    /* Dummy read (to clear existing data) */
    volatile uint32_t dummy_read;
    (void)dummy_read;
    while (UART_LSR(uart_num) & UART_LSR_RDR) {
        dummy_read = UART_RBR(uart_num);
    }

    UART_FCR(uart_num) = 3 | (2<<6);

    /* Wait end of TX & disable TX */
    UART_TER(uart_num) = UART_TER_TXEN;

    /* Wait for current transmit complete */
    while (!(UART_LSR(uart_num) & UART_LSR_THRE));
    return UART_NO_ERROR;
}

/*
* This Function return if data are received or not received.
*/
uart_rx_data_ready_t uart_rx_data_ready(uart_num_t uart_num)
{
    uint32_t uart_port;
    uint8_t uart_status;
    uart_rx_data_ready_t data_ready;

    uart_port = uart_num;

    uart_status = UART_LSR(uart_port) & 0xFF;

    /* Check Error */
    if ((uart_status & UART_LSR_ERROR_MASK) == 0) {
        /* No errors check if data is ready */
        if ((uart_status & UART_LSR_RDR) == 0) {
            data_ready = UART_RX_NO_DATA;
        } else {
            data_ready = UART_RX_DATA_READY;
        }
    } else {
        /* UART Error */
        data_ready = UART_RX_DATA_ERROR;
    }

    return data_ready;
}

/*
* This Function Wait until Data RX Ready, and return Data Read from UART.
*/
uint8_t uart_read(uart_num_t uart_num)
{
    uint32_t uart_port;
    uint8_t uart_val;

    uart_port = uart_num;

    /* Wait Until Data Received (Rx Data Not Ready) */
    while ((UART_LSR(uart_port) & UART_LSR_RDR) == 0);

    uart_val = (UART_RBR(uart_port) & UART_RBR_MASKBIT);

    return uart_val;
}

/*
* This Function Wait until Data RX Ready, and return Data Read from UART.
*/
uint8_t uart_read_timeout(uart_num_t uart_num, uint32_t rx_timeout_nb_cycles,
                uart_error_t *err)
{
    uint32_t uart_port;
    uint8_t uart_val;
    uint32_t counter;

    uart_port = uart_num;

    /* Wait Until Data Received (Rx Data Not Ready) */
    counter = 0;
    while ((UART_LSR(uart_port) & UART_LSR_RDR) == 0) {
        if (rx_timeout_nb_cycles > 0) {
            counter++;
            if (counter >= rx_timeout_nb_cycles) {
                *err = UART_TIMEOUT_ERROR;
                return 0;
            }
        }
    }

    uart_val = (UART_RBR(uart_port) & UART_RBR_MASKBIT);

    /* Clear error */
    *err = UART_NO_ERROR;

    return uart_val;
}

/* This Function Wait Data TX Ready, and Write Data to UART
    if rx_timeout_nb_cycles = 0 Infinite wait
*/
void uart_write(uart_num_t uart_num, uint8_t data)
{
    uint32_t uart_port;

    uart_port = uart_num;

    /* Wait Until FIFO not full  */
    while ((UART_LSR(uart_port) & UART_LSR_THRE) == 0);

    UART_THR(uart_port) = data;
}