/*
 * nrf24l01_port.c
 *
 *  Created on: May 18, 2020
 *      Author: marek
 */

#include "stm32f1xx_ll_gpio.h"
#include "stm32f1xx_ll_spi.h"
#include "cmsis_os.h"
#include "lib/nrf24l01.h"
#include "main.h"

#define NRF24L01_PORT__STM32F103_72MHZ_DELAY_1US		(8u)

/*
 * nRF24L01 (RX FIFO:3, TX FIFO:3)
 *
 * SPI MAX 8(10) MHz
 * 8 bit transfer
 * MSB first
 * CPOL = Low
 * CPHA = 1 Edge
 *
 * PINS:
 * VCC  - (3.3) <1V9;3V6>
 * GND  - (G)
 *
 * CSN  - (B12) active low, normally high
 * SCK  - (B13) normally low, rising edge active
 * MISO - (B14)
 * MOSI - (B15)
 *
 * CE   - (B9) control data transmission and reception
 *      - active-high
 *      - RX mode: high - monitor the air
 *                 low  - standby
 *      - TX mode: high - transmit the packets from FIFO (>10 us)
 *                 low  - standby
 * IRQ  - (B8) active-low, normally high
 *      - 1. data received RX_DR interrupt
 *      - 2. data transmitted TX_DS interrupt
 *      - 3. maximum number of transmit retries reached MAX_RT interrupt
 *
 * SPI:
 * read
 *   CSN  ''''\________________________/'''''
 *   SCK  ______I_I*I___I_I-I___I_I*I________
 *   MOSI XXXXXC7--C0XXXXXXXXXXXXXXXXXXXXXXXX
 *   MISO XXXXXS7--S0 D7---D0 D15---D8XXXXXXX
 *
 * write
 *   CSN  ''''\________________________/'''''
 *   SCK  ______I_I*I___I_I-I___I_I*I________
 *   MOSI XXXXXC7--C0 D7---D0 D15---D8XXXXXXX
 *   MISO XXXXXS7--S0XXXXXXXXXXXXXXXXXXXXXXXX
 */


/////////////////////////////////////////////////////////////////////////////////
// Delay function requirements
//
// The user must define a function that delays for the specified number of
//	 microseconds. This function needs to be as precise as possible, and the use
//   of a timer module within your microcontroller is highly recommended. The
//   function used here has the prototype
//
//	    void delay_us(unsigned int microseconds);
//
// You should also change the include file name below to whatever the name of your
//   delay include file is.
//////////////////////////////////////////////////////////////////////////////////
void nrf24l01_port_delay_us(unsigned int delay_us)
{
	// Do not use active waiting for delays longer then 100us.
	if (delay_us > 100) {
		unsigned int delay_ms    = ((delay_us + 999) / 1000);
		TickType_t   delay_ticks = delay_ms * osKernelGetTickFreq() / 1000;
		osDelay(delay_ticks);
	} else {
		volatile unsigned int delay = delay_us * NRF24L01_PORT__STM32F103_72MHZ_DELAY_1US;
		while (delay--) {};
	}
}

/////////////////////////////////////////////////////////////////////////////////
// SPI function requirements
//
// The user must define a function to send one byte of data and also return the
//   resulting byte of data data through the SPI port. The function used here
//   has the function prototype
//
//	    unsigned char spi_send_read_byte(unsigned char byte);
//
// This function should take the argument unsigned char byte and send it through
//   the SPI port to the 24L01.  Then, it should wait until the 24L01 has returned
//   its response over SPI.  This received byte should be the return value of the
//   function.
//
// You should also change the include file name below to whatever the name of your
//   SPI include file is.
//////////////////////////////////////////////////////////////////////////////////
unsigned char spi_port_send_read_byte(unsigned char byte)
{
	if (!LL_SPI_IsEnabled(SPI2)){
		LL_SPI_Enable(SPI2);
	}

	LL_SPI_TransmitData8(SPI2, byte);

    while (!LL_SPI_IsActiveFlag_TXE(SPI2)) {};   // wait while tx not empty

    while (LL_SPI_IsActiveFlag_BSY(SPI2)) {};    // wait while busy

    while (!LL_SPI_IsActiveFlag_RXNE(SPI2)) {};  // wait while rx empty

	return LL_SPI_ReceiveData8(SPI2);
}

//clears the pin on the host microcontroller that is attached to the 24l01's CE pin
void nrf24l01_port_pin_ce_clear(void)
{
	LL_GPIO_ResetOutputPin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin);
}

//sets the pin on the host microcontroller that is attached to the 24l01's CE pin
void nrf24l01_port_pin_ce_set(void)
{
	LL_GPIO_SetOutputPin(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin);
}

//returns true if CE is high, false if not
bool nrf24l01_port_pin_ce_active(void)
{
	return !!LL_GPIO_IsOutputPinSet(NRF24L01_CE_GPIO_Port, NRF24L01_CE_Pin);
}

//sets the pin on the host microcontroller that is attached to the 24l01's CSN pin
void nrf24l01_port_pin_csn_clear(void)
{
	LL_GPIO_ResetOutputPin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin);

	// Tcc - CSN to SCK Setup - min 2ns
}

//clears the pin on the host microcontroller that is attached to the 24l01's CSN pin
void nrf24l01_port_pin_csn_set(void)
{
	// Tcch - SCK to CSN Hold - min 2ns

	// Because CSN is set before SCK is idle it is delayed by 3us
	// This is produces 1.1-1.4us delay after SCK for SPI at 141 kHz
	nrf24l01_port_delay_us(3u);

	LL_GPIO_SetOutputPin(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin);

	// Tcwh - CSN Inactive time - min 50ns
}

//returns true if CSN is high, false if not
bool nrf24l01_port_pin_csn_active(void)
{
	return !!LL_GPIO_IsOutputPinSet(NRF24L01_CSN_GPIO_Port, NRF24L01_CSN_Pin);
}

//returns true if IRQ pin is low, false otherwise
bool nrf24l01_port_pin_irq_active(void)
{
	return !LL_GPIO_IsInputPinSet(NRF24L01_IRQ_GPIO_Port, NRF24L01_IRQ_Pin);
}

