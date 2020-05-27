// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/driver/serial.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Serial class driver
 */

#ifndef __MASU_SERIAL_H__
#define __MASU_SERIAL_H__

#include <common.h>
#include <driver/dm_device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * enum serial_port
 */
enum serial_port {
	SERIAL_PORT__TTYS0 = 0u,
	SERIAL_PORT__TTYS1,
	SERIAL_PORT__TTYS2,
	SERIAL_PORT__TTYS3
};

/**
 * enum serial_parity
 */
enum serial_parity {
	SERIAL_PARITY__NONE = 0ul,
	SERIAL_PARITY__ODD  = 1ul,
	SERIAL_PARITY__EVEN = 2ul,
	SERIAL_PARITY__MAX
};

/**
 * enum serial_stop_bits_e
 */
enum serial_stop_bits {
	SERIAL_STOP_BITS__1 = 0ul,
	SERIAL_STOP_BITS__2 = 1ul,
	SERIAL_STOP_BITS__MAX
};

/**
 * enum serial_word_length_e
 */
enum serial_word_length {
	SERIAL_WORD_LENGTH__5 = 0ul,
	SERIAL_WORD_LENGTH__6 = 1ul,
	SERIAL_WORD_LENGTH__7 = 2ul,
	SERIAL_WORD_LENGTH__8 = 3ul,
	SERIAL_WORD_LENGTH__MAX
};

/**
 * struct serial_desc - serial instance
 */
struct serial_desc {
	struct dm_device dev;												     /* Device */
	unsigned int     port;
#define SERIAL_PB_TO_CONFIG(m_pb)	 (((m_pb)  &  0x3ul)      << 28ul)       /* Parity */
#define SERIAL_CONFIG_TO_PB(m_cfg)   (((m_cfg) >> 28ul)       &  0x3ul)
#define SERIAL_SB_TO_CONFIG(m_sb)	 (((m_sb)  &  0x1ul)      << 27ul)       /* Stop bits */
#define SERIAL_CONFIG_TO_SB(m_cfg)   (((m_cfg) >> 27ul)       &  0x1ul)
#define SERIAL_WL_TO_CONFIG(m_wl)	 (((m_wl)  &  0x7ul)      << 24ul)	     /* Word length */
#define SERIAL_CONFIG_TO_WL(m_cfg)   (((m_cfg) >> 24ul)       &  0x7ul)
#define SERIAL_BR_TO_CONFIG(m_br)	 (((m_br)  &  0xFFFFFFul) << 0ul)        /* Baudrate */
#define SERIAL_CONFIG_TO_BR(m_cfg)   (((m_cfg) >> 0ul)        &  0xFFFFFFul)
	uint32_t         config;
};

int dm_serial_init      (      struct serial_desc *desc, unsigned int port, enum dm_driver_id driver_id);
int dm_serial_deinit    (      struct serial_desc *desc);
int dm_serial_getc      (const struct serial_desc *desc);
int dm_serial_getc_block(const struct serial_desc *desc);
int dm_serial_putc      (const struct serial_desc *desc, int c);
int dm_serial_putc_block(const struct serial_desc *desc, int c);
int dm_serial_puts      (const struct serial_desc *desc, const char *s);
int dm_serial_getconfig (const struct serial_desc *desc, uint32_t *config);
int dm_serial_setconfig (const struct serial_desc *desc, uint32_t config);

/**
 * struct dm_serial_ops - Driver model serial operations
 */
struct dm_serial_ops {
	int (*setbrg)   (const struct dm_device *dev, unsigned int port, unsigned int baudrate);
	int (*getc)     (const struct dm_device *dev, unsigned int port);
	int (*putc)     (const struct dm_device *dev, unsigned int port, char ch);
	int (*pending)  (const struct dm_device *dev, unsigned int port);
	int (*clear)    (const struct dm_device *dev, unsigned int port);
	int (*getconfig)(const struct dm_device *dev, unsigned int port, uint32_t *config);
	int (*setconfig)(const struct dm_device *dev, unsigned int port, uint32_t config);
};

/* Access macro for operations with the device */
#define serial_get_ops(dev)	((struct dm_serial_ops *)(dev)->driver->ops)

#ifdef __cplusplus
}
#endif

#endif /* __MASU_SERIAL_H__ */
