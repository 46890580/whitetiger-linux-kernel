/*
 * Crystalfontz CFA-10049 FIQ handler
 *
 * Copyright (C) 2012 Free Electrons
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/linkage.h>
#include <asm/assembler.h>

#define BIT_SET 0x4
#define BIT_CLR	0x8
#define BIT_TOG	0xc

#define HW_PINCTRL_DOE0			0xb00
#define HW_PINCTRL_DOE3			0xb30
#define HW_PINCTRL_DOUT0		0x700
#define HW_PINCTRL_DOUT3		0x730
#define HW_TIMROT_TIMCTRL2		0xa0
#define HW_TIMROT_FIXED_COUNT2		0xc0

#define HW_PINCTRL_DOE			HW_PINCTRL_DOE0
#define HW_PINCTRL_DOUT			HW_PINCTRL_DOUT0

/*
 * This definition needs to match the fiq_cell structure layout
 * defined in cfa10049_fiq.h
 */
#define TIMER_OFFSET			0
#define CLR_OFFSET			4
#define SET_OFFSET			8

/*
 * This definition needs to match the fiq_buffer structure layout
 * defined in cfa10049_fiq.h
 */
#define READ_PTR_OFFSET			0
#define WRITE_PTR_OFFSET		4
#define SIZE_OFFSET			8
#define STATUS_OFFSET			12
#define DATA_OFFSET			16

#define ERR_URUN			2

	.text
	.global cfa10049_fiq_handler_end

ENTRY(cfa10049_fiq_handler)
	/*
	 * The interrupt ACK needs to be put first in
	 * the handler to have time for the write to the
	 * interrupt controller to propagates on the bus.
	 * See http://forums.arm.com/index.php?/topic/13692-pipeline-issue-on-return-from-fiq/
	 */
	/* Acknowledge the interrupt */
	mov	r11, #1
	lsl	r11, r11, #15
	str	r11, [r8, #HW_TIMROT_TIMCTRL2 + BIT_CLR]
	/* Get the read index */
	ldr	r11, [r10, #READ_PTR_OFFSET]

	/*
	 * Detect the underuns, and stop the execution if the
	 * application is too slow. Unprogram the timer and
	 * return directly from the FIQ
	 */
	ldr	r12, [r10, #WRITE_PTR_OFFSET]
	cmp	r11, r12
	moveq	r11, #0
	streq	r11, [r8, #HW_TIMROT_FIXED_COUNT2]
	moveq	r11, #ERR_URUN
	streq	r11, [r10, #STATUS_OFFSET]
	beq	out

	/* Increment it */
	add	r11, r11, #1
	/* If we are over the edge of the buffer, return to
	 * the beginning */
	ldr	r12, [r10, #SIZE_OFFSET]
	cmp	r11, r12
	movcs	r11, #0
	/* Update the read index in the buffer */
	str	r11, [r10, #READ_PTR_OFFSET]
	/* Get the start address of the array */
	add	r12, r10, #DATA_OFFSET
	/* cells have 3 * 4 bytes */
	add	r11, r11, r11, lsl #1
	lsl	r11, r11, #2
	/* Compute the ptr to the cell */
	add	r12, r12, r11
	/* Store the 1st cell in the timer control register */
	ldr	r11, [r12, #TIMER_OFFSET]
	add	r11, r11, r11, lsl #1
	lsl	r11, r11, #3
	str	r11, [r8, #HW_TIMROT_FIXED_COUNT2]
	/* Store the 2nd cell in the PIO clear register */
	ldr	r11, [r12, #CLR_OFFSET]
	str	r11, [r9, #HW_PINCTRL_DOUT + BIT_CLR]
	/* Store the 3rd cell in the PIO set register */
	ldr	r11, [r12, #SET_OFFSET]
	str	r11, [r9, #HW_PINCTRL_DOUT + BIT_SET]
out:
	/* return from FIQ */
	subs	pc, lr, #4
cfa10049_fiq_handler_end: