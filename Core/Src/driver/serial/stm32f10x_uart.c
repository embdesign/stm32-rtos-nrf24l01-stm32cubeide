// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <driver/serial/stm32f10x_uart.c>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Uart driver
 */

#include <common.h>
#include <driver/dm_device.h>
#include <driver/serial.h>
#include "stm32f1xx_ll_usart.h"

/* Conversions serial-class -> stm32f10x */
static const unsigned int s_serial_to_stm32f10x_parity[SERIAL_PARITY__MAX] =
{
		LL_USART_PARITY_NONE,
		LL_USART_PARITY_ODD,
		LL_USART_PARITY_EVEN
};

static const unsigned int s_serial_to_stm32f10x_stop_bits[SERIAL_STOP_BITS__MAX] =
{
		LL_USART_STOPBITS_1,
		LL_USART_STOPBITS_2
};

static const unsigned int s_serial_to_stm32f10x_word_length[SERIAL_WORD_LENGTH__MAX] =
{
		0,
		0,
		0,
		LL_USART_DATAWIDTH_8B
};

struct stm32f10x_uart_plat {
	USART_TypeDef * const * regs;
};

static struct stm32f10x_uart_plat s_stm32f10x_uart_plat;

static USART_TypeDef * const s_stm32f10x_uart_uart_ports[] = {
		(USART_TypeDef *) USART1,	/* UART1 */
		(USART_TypeDef *) USART2,	/* UART2 */
		(USART_TypeDef *) USART3,   /* UART3 */
};
#define STM32F10X_UART_PORT_MAX		ARRAY_COUNT(s_stm32f10x_uart_uart_ports)

static int stm32f10x_uart_op_setbrg(const struct dm_device *dev, unsigned int port, unsigned int baudrate)
{
	WARN_UNUSED(dev);
	WARN_UNUSED(port);

	return -ENOENT;
}

static int stm32f10x_uart_op_getc(const struct dm_device *dev, unsigned int port)
{
	struct stm32f10x_uart_plat *pdata = (struct stm32f10x_uart_plat *) dev->platdata;
	int                        ret    = -EBUSY;

	if (port >= STM32F10X_UART_PORT_MAX)
		return -ENOENT;

	if (LL_USART_IsActiveFlag_RXNE(pdata->regs[port])) {
		ret = LL_USART_ReceiveData8(pdata->regs[port]);
	};

	return ret;
}

static int stm32f10x_uart_op_putc(const struct dm_device *dev, unsigned int port, const char ch)
{
	struct stm32f10x_uart_plat *pdata = (struct stm32f10x_uart_plat *) dev->platdata;
	int                        ret    = -EBUSY;

	if (port >= STM32F10X_UART_PORT_MAX)
		return -ENOENT;

	if (LL_USART_IsActiveFlag_TXE(pdata->regs[port])) {
		LL_USART_TransmitData8(pdata->regs[port], ch);

		ret = 0;
	};

	return ret;
}

static int stm32f10x_uart_op_pending(const struct dm_device *dev, unsigned int port)
{
	WARN_UNUSED(dev);
	WARN_UNUSED(port);

	return -ENOENT;
}

static int stm32f10x_uart_op_clear(const struct dm_device *dev, unsigned int port)
{
	WARN_UNUSED(dev);
	WARN_UNUSED(port);

	return -ENOENT;
}

static int stm32f10x_uart_op_getconfig(const struct dm_device *dev, unsigned int port, uint32_t *config)
{
	WARN_UNUSED(dev);
	WARN_UNUSED(port);
	WARN_UNUSED(config);

	return -ENOENT;
}

static int stm32f10x_uart_op_setconfig(const struct dm_device *dev, unsigned int port, uint32_t config)
{
	struct stm32f10x_uart_plat *pdata = (struct stm32f10x_uart_plat *) dev->platdata;
	int                        ret    = -EBUSY;

	if (port >= STM32F10X_UART_PORT_MAX)
		return -ENOENT;

	{
		LL_USART_InitTypeDef init;
		ErrorStatus          status;

		LL_USART_Disable(pdata->regs[port]);

		init.BaudRate            = SERIAL_CONFIG_TO_BR(config);
		init.DataWidth           = s_serial_to_stm32f10x_word_length[SERIAL_CONFIG_TO_WL(config)];
		init.StopBits            = s_serial_to_stm32f10x_stop_bits[SERIAL_CONFIG_TO_SB(config)];
		init.Parity              = s_serial_to_stm32f10x_parity[SERIAL_CONFIG_TO_PB(config)];
		init.TransferDirection   = LL_USART_DIRECTION_TX_RX;
		init.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
		init.OverSampling        = LL_USART_OVERSAMPLING_16;

		status = LL_USART_Init(pdata->regs[port], &init);

		if (status != SUCCESS) {
			ret = -EIO;
		} else {
			LL_USART_ConfigAsyncMode(pdata->regs[port]);

			LL_USART_Enable(pdata->regs[port]);
		}
	}

	return ret;
}

static const struct dm_serial_ops uart_stm32f10x_ops = {
	.setbrg    = stm32f10x_uart_op_setbrg,
	.getc      = stm32f10x_uart_op_getc,
	.putc      = stm32f10x_uart_op_putc,
	.pending   = stm32f10x_uart_op_pending,
	.clear     = stm32f10x_uart_op_clear,
	.getconfig = stm32f10x_uart_op_getconfig,
	.setconfig = stm32f10x_uart_op_setconfig,
};

static int stm32f10x_uart_probe(struct dm_device *dev)
{
	struct stm32f10x_uart_plat *pdata = (struct stm32f10x_uart_plat *) dev->platdata;

	pdata->regs = s_stm32f10x_uart_uart_ports;

	return 0;
}

static int stm32f10x_uart_remove(struct dm_device *dev)
{
	struct stm32f10x_uart_plat *pdata = (struct stm32f10x_uart_plat *) dev->platdata;

	pdata->regs = NULL;

	return 0;
}

static int stm32f10x_uart_bind(struct dm_device *dev)
{
	dev->platdata = &s_stm32f10x_uart_plat;

	return 0;
}

static int stm32f10x_uart_unbind(struct dm_device *dev)
{
	dev->platdata = NULL;

	return 0;
}

DM_DRIVER_DEFINE(uart_stm32f10x) = {
	.name      = "uart_stm32f10x",
	.driver_id = DM_DRIVER_ID__STM32F10X_UART,
	.class_id  = DM_CLASS_ID__SERIAL,
	.bind      = stm32f10x_uart_bind,
	.unbind    = stm32f10x_uart_unbind,
	.probe     = stm32f10x_uart_probe,
	.remove    = stm32f10x_uart_remove,
	.ops       = &uart_stm32f10x_ops,
};
