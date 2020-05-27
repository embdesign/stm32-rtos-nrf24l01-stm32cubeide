// SPDX-License-Identifier: GPL-2.0-only
/*
 *  <include/macro.h>
 *
 *  Copyright (C) 2020-2021 Marek Sujak
 *
 *  Generic macros
 */

#ifndef __MASU_MACRO_H__
#define __MASU_MACRO_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Arrays */
#define ARRAY_COUNT(m_a)				(sizeof(m_a)/sizeof(*(m_a)))

/* Align */
#define ALIGN_UP(m_num, m_align) 		(((m_num) & ~((m_align)-1)) + (m_align))
#define ALIGN_DOWN(m_num, m_align) 		(((m_num) & ~((m_align)-1)))

/* Bits */
#define BITS(m_val, m_shift)			((m_val) << (m_shift))
#define BIT(m_shift) 					BITS(1ul, (m_shift))

/* Compare */
#define CMP_MIN(m_a, m_b)               ((m_a) < (m_b) ? (m_a) : (m_b))

/* Containers */
/**
 * container_of - cast a member of a structure out to the containing structure
 * @m_ptr:        the pointer to the member.
 * @m_type:       the type of the container struct this is embedded in.
 * @m_member:     the name of the member within the struct.
 *
 */
#define CONTAINER_OF(m_ptr, m_type, m_member) ({                      \
        const typeof( ((m_type *)0)->m_member ) *__mptr = (m_ptr);    \
        (m_type *)(void *)( (char *)__mptr - offsetof(m_type, m_member) );})

/* Flags */
#define FLAG_ADD(m_a, m_f)				(m_a) |=  (m_f)
#define FLAG_DEL(m_a, m_f)				(m_a) &= ~(m_f)
#define FLAG_IS_SET(m_a, m_mask, m_f)	(((m_a) & (m_mask)) == (m_f))

/* IO */
#define IO_READ32(m_a)					*((volatile uint32_t * const)(m_a))
#define IO_WRITE32(m_a, m_d)			*((volatile uint32_t * const)(m_a)) = (uint32_t)(m_d);
#define IO_READ16(m_a)					*((volatile uint16_t * const)(m_a))
#define IO_WRITE16(m_a, m_d)			*((volatile uint16_t * const)(m_a)) = (uint16_t)(m_d);
#define IO_READ8(m_a)					*((volatile uint8_t  * const)(m_a))
#define IO_WRITE8(m_a, m_d)				*((volatile uint8_t  * const)(m_a)) = (uint8_t)(m_d);

/* Packing */
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#define __func__ __FUNCTION__
#define PACK( __Declaration__ ) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop) )
#define PACKED
#else
#define PACK( __Declaration__ ) __Declaration__
#define PACKED __attribute__((__packed__))
#endif // WIN32

/* Strings */
#define XSTR(m_s) 	STR(m_s)
#define STR(m_s) 	#m_s

/* Warnings */
#define WARN_UNUSED(m_a) 				(void)(m_a)
#define WARN_ASSUME_ALIGNED				(void *)

#ifdef __cplusplus
}
#endif

#endif /* __MASU_MACRO_H__ */
