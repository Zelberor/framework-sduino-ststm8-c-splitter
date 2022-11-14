/*
  wiring_digital.c - digital input and output functions
  Part of Arduino - http://www.arduino.cc/

  Copyright (c) 2005-2006 David A. Mellis

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General
  Public License along with this library; if not, write to the
  Free Software Foundation, Inc., 59 Temple Place, Suite 330,
  Boston, MA  02111-1307  USA

  Modified 28 September 2010 by Mark Sproul
*/

#include "wiring_private.h"
#include "pins_arduino.h"



/* this typedef is a SDCC wordaround.
 * Defining this a type instead of defining an array of (unsigned char *)
 * keeps SDCC from storing the array in the xinit segment and copy it to RAM
 * at runtime.
 */
typedef unsigned char *uc_p;

extern const uc_p ccmrx[NUM_TIMERS];
extern const uc_p ccerx[NUM_TIMERS];
extern const unsigned char DISABLE_TIMER_OUTP_MASK[NUM_TIMERS];
void turnOffPWM(uint8_t timer);

// <--#SPLIT#--> //

/**
 * timer capture/compare mode register to control PWM mode
 */
const uc_p ccmrx[NUM_TIMERS]={
#ifdef NEED_TIMER_11_12
	TIM1->CCMR1,	/* for TIMER11 */
	TIM1->CCMR2,	/* for TIMER12 */
#endif
	TIM1->CCMR3,	/* for TIMER13 */
	TIM1->CCMR4,	/* for TIMER14 */
	TIM2->CCMR1,	/* for TIMER21 */
	TIM2->CCMR2,	/* for TIMER22 */
#ifdef NEED_TIMER_23
	TIM2->CCMR3,	/* for TIMER23 */
#endif
#ifdef NEED_TIMER_31_32
	TIM3->CCMR1,	/* for TIMER31 */
	TIM3->CCMR2	/* for TIMER32 */
#endif
};

// <--#SPLIT#--> //

/**
 * CCER register for each timer channel.
 *
 * Each 8-bit-register can contain the bits for two channels, so in addition to
 * the register address itself you also need the individual offset(s) of the
 * bit(s) you want to access.
 */
const uc_p ccerx[NUM_TIMERS]={
#ifdef NEED_TIMER_11_12
    TIM1->CCER1,    /* for TIMER11 */
    TIM1->CCER1,    /* for TIMER12 */
#endif
    TIM1->CCER2,    /* for TIMER13 */
    TIM1->CCER2,    /* for TIMER14 */
    TIM2->CCER1,    /* for TIMER21 */
    TIM2->CCER1,    /* for TIMER22 */
#ifdef NEED_TIMER_23
    TIM2->CCER2,    /* for TIMER23 */
#endif
#ifdef NEED_TIMER_31_32
    TIM3->CCER1,    /* for TIMER31 */
    TIM3->CCER1     /* for TIMER32 */
#endif
};

// <--#SPLIT#--> //

/**
 * These Bits have to be set to 0 in the timer channel's CCER
 * (Capture/compare enable register) to disable the output, so that the
 * physical pin is not driven by the timer.
 *
 * @see
 * RM0016 Reference Manual
 * STM8S Series and STM8AF Series 8-bit microcontrollers
 * DocID14587 Rev 14 (Oct 2017)
 * Table 38. Output control for complementary OCi and OCiN channels with break
 * feature
 */
const unsigned char DISABLE_TIMER_OUTP_MASK[NUM_TIMERS]={
#ifdef NEED_TIMER_11_12
    (1 << 0) | (1 << 2),    /* for TIMER11 */
    (1 << 4) | (1 << 6),    /* for TIMER12 */
#endif
    (1 << 0) | (1 << 2),    /* for TIMER13 */
    (1 << 4),               /* for TIMER14 */
    (1 << 0),               /* for TIMER21 */
    (1 << 4),               /* for TIMER22 */
#ifdef NEED_TIMER_23
    (1 << 0),               /* for TIMER23 */
#endif
#ifdef NEED_TIMER_31_32
    (1 << 0),               /* for TIMER31 */
    (1 << 4)                /* for TIMER32 */
#endif
};

// <--#SPLIT#--> //

/**
 * set the input or output mode of a pin
 */
#ifdef DONT_USE_ASSEMBLER
/***
 * Arduino-style pinMode
 *
 * This version compiles to 270 bytes.
 */
void pinMode(uint8_t pin, uint8_t mode)
{
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile GPIO_TypeDef *gpio;

	if (port == NOT_A_PORT) return;

	gpio = (GPIO_TypeDef *) portOutputRegister(port);

	if (mode == INPUT) {
		BEGIN_CRITICAL
		gpio->CR2 &= ~bit;	// first: deactivate interrupt
		gpio->CR1 &= ~bit;	// release top side
		gpio->DDR &= ~bit;	// now set direction
		END_CRITICAL
	} else if (mode == INPUT_PULLUP) {
		BEGIN_CRITICAL
		gpio->CR2 &= ~bit;	// first: deactivate interrupt
		gpio->DDR &= ~bit;	// set direction before
		gpio->CR1 |=  bit;	// activating the pull up
		END_CRITICAL
	} else if (mode == OUTPUT_FAST) {// output push-pull, fast
		BEGIN_CRITICAL
		gpio->CR1 |=  bit;
		gpio->DDR |=  bit;	// direction before setting CR2 to
		gpio->CR2 |=  bit;	// avoid accidental interrupt
		END_CRITICAL
	} else if (mode == OUTPUT_OD_FAST) {	// output open drain, fast
		BEGIN_CRITICAL
		gpio->CR1 &= ~bit;
		gpio->DDR |=  bit;	// direction before setting CR2 to
		gpio->CR2 |=  bit;	// avoid accidental interrupt
		END_CRITICAL
	} else if (mode == OUTPUT_OD) {	// output open drain, slow
		BEGIN_CRITICAL
		gpio->CR1 &= ~bit;
		gpio->CR2 &= ~bit;
		gpio->DDR |=  bit;
		END_CRITICAL
	} else {			// output push-pull, slow
		BEGIN_CRITICAL
		gpio->CR1 |=  bit;
		gpio->CR2 &= ~bit;
		gpio->DDR |=  bit;
		END_CRITICAL
	}
}

#else

/***
 * Hand-crafted and optimized assembler version of pinMode
 *
 * This version compiles to 147 bytes.
 */
#define DDR	2
#define CR1	3
#define CR2	4

#define BIT_CLEAR(REG) \
	ld	a,yh\
	and	a,(REG,x)\
	ld	(REG,x),a

#define BIT_SET(REG) \
	ld	a,yl\
	or	a,(REG,x)\
	ld	(REG,x),a


void pinMode(uint8_t pin, uint8_t mode)
{
	(void)	pin;	// empty code to avoid warning
	(void)	mode;
__asm

;
; position of the parameters on the stack:
; (3,sp)	pin
; (4,sp)	mode
;

;
; mapping pin => bit mask
;

;	pinmode-c.c: 9: uint8_t bit = digitalPinToBitMask(pin);
	clrw	x
	ld	a, (3, sp)
	ld	xl, a
	pushw	x
	addw	x, #(_digital_pin_to_bit_mask_PGM + 0)
	ld	a, (x)
	ld	yl,a		; yl = bit
	cpl	a
	ld	yh,a		; yh = ~bit

;
; mapping pin => port-ID
;
; The distance between the tables _digital_pin_to_bit_mask_PGM and
; _digital_pin_to_port_PGM is known and constant, but I could not find a way
; to do the math through the linker without defining a dummy variable.
;
; The constant NUM_DIGITAL_PINS could be used, but that requires that these
; two tables are always defined right after each other in all pins_arduino.h
; versions. This is possible, but a potential pitfall in the future.
;
; If this distance could be communicated into this module, a simple
; addw x, #(distance) could calculate the table start address.
;
; The second best way is reusing at least the offset part. Only 2 extra
; bytes and full flexibility.

;	pinmode-c.c: 10: uint8_t port = digitalPinToPort(pin);
;	pinmode-c.c: 13: if (port == NOT_A_PORT) return;
	popw	x
	addw	x, #(_digital_pin_to_port_PGM + 0)
	ld	a, (x)		; port-ID. Z-Flag wird gesetzt
	jreq	032$		; if (port == NOT_A_PORT) return;


;
; mapping port-ID => Port-Addresse
;
; x = (GPIO_TypeDef *) portOutputRegister(port)
;
00102$:
	clrw	x
	sll	a			; 8 bit shift is sufficient
	ld	xl, a
	addw	x, #(_port_to_output_PGM + 0)
	ldw	x, (x)			; jetzt ist gpio in x

;
; jump table/switch statement for mode parameter
;
	ld	a, (4, sp)
	jreq	000$
	dec	a
	jreq	010$
	dec	a
	jreq	020$
	dec	a
	jreq	030$
	dec	a		; there is no case 4
	dec	a
	jreq	050$
	dec	a		; there is no case 6
	dec	a
	jreq	070$

; fallthrough into OUTPUT as the default case

;
; case OUTPUT			// 1: output, push-pull, slow
;	gpio->CR1 |=  bit;
;	gpio->CR2 &= ~bit;
;	gpio->DDR |=  bit;
;
010$:
	sim
//	BIT_SET(CR1)
	ld	a,yl
	or	a,(CR1,x)
	jra 031$

;
; case OUTPUT_OD		// 3: output open drain, slow
;	gpio->CR1 &= ~bit;
;	gpio->CR2 &= ~bit;
;	gpio->DDR |=  bit;
;
030$:
	sim
//	BIT_CLEAR(CR1)
	ld	a,yh
	and	a,(CR1,x)
031$:	ld	(CR1,x),a
	BIT_CLEAR(CR2)
	BIT_SET(DDR)
	rim
032$:	ret

;
; case INPUT			// 0: input, floating
;	gpio->CR2 &= ~bit;	// first: deactivate interrupt
;	gpio->CR1 &= ~bit;	// release top side
;	gpio->DDR &= ~bit;	// now set direction
;
000$:
	sim
	BIT_CLEAR(CR2)		; first: deactivate interrupt
	BIT_CLEAR(CR1)		; release top side
	BIT_CLEAR(DDR)		; now set direction
	rim
001$:	ret

;
; case INPUT_PULLUP		// 2: input, internal pull up active
;	gpio->CR2 &= ~bit;	// first: deactivate interrupt
;	gpio->DDR &= ~bit;	// set direction before
;	gpio->CR1 |=  bit;	// activating the pull up
;
020$:
	sim
	BIT_CLEAR(CR2)		; first: deactivate interrupt
	BIT_CLEAR(DDR)		; set direction before
	BIT_SET(CR1)		; activating the pull up
	rim
	ret

;
; case OUTPUT_FAST		// 5: output push-pull, fast
;	gpio->CR1 |=  bit;
;	gpio->DDR |=  bit;	// direction before setting CR2 to
;	gpio->CR2 |=  bit;	// avoid accidental interrupt
;
050$:
	sim
//	BIT_SET(CR1)
	ld	a,yl
	or	a,(CR1,x)
	jra 071$

;
; case OUTPUT_OD_FAST		// 7: output open drain, fast
;	gpio->CR1 &= ~bit;
;	gpio->DDR |=  bit;	// direction before setting CR2 to
;	gpio->CR2 |=  bit;	// avoid accidental interrupt
;
070$:
	sim
//	BIT_CLEAR(CR1)
	ld	a,yh\
	and	a,(CR1,x)\
071$:	ld	(CR1,x),a
	BIT_SET(DDR)
	BIT_SET(CR2)
	rim

__endasm;
}
#endif

/* using an array of pointers compiles way more efficient than doing simple
 * pointer arithmetics like
 *
	if (timer<TIMER21) {
		*(&TIM1->CCMR1 + (timer-TIMER11)) &= 0x8f;
	} else {
		*(&TIM2->CCMR1 + (timer-TIMER21)) &= 0x8f;
	}
 *
 * or a simple switch/case statement like
 *
	switch (timer)
	{
		case TIMER11:   TIM1->CCMR1 &= 0x8f;    break;
		case TIMER12:   TIM1->CCMR2 &= 0x8f;    break;
		case TIMER13:   TIM1->CCMR3 &= 0x8f;    break;
		case TIMER14:   TIM1->CCMR4 &= 0x8f;    break;
		case TIMER21:   TIM2->CCMR1 &= 0x8f;    break;
		case TIMER22:   TIM2->CCMR2 &= 0x8f;    break;
		case TIMER23:   TIM2->CCMR3 &= 0x8f;    break;
	}
 *
 * The most efficient way is this:
 *
#define T1_BASE 0x5258
#define T2_BASE 0x5307
	uint8_t *reg = T1_BASE-1;
	if (timer>4) reg+= (T2_BASE - T1_BASE);
	reg[timer] &= ~TIM1_CCMR_OCM;
 *
 * Unfortunatly, SDCC can't figure out the values TIM1->CCMR1 und TIM->CCMR2
 * early enough in the compile process, so the adresses have to be hardcoded
 * into the code.
 *
 * SDCC is really, really not good in optimizing its code.
 */

// <--#SPLIT#--> //

/**
 * handle the PWM pins
 */
void turnOffPWM(uint8_t timer)
{
    // Output compare mode = 000: Frozen - The comparison between the output
    // compare register TIM1_CCR1 and the counter register TIM1_CNT has no
    // effect on the outputs.
    *((unsigned char *) ccmrx[timer-1]) &= ~TIM1_CCMR_OCM;

    // CCiE = CCiNE = 0: Output disabled (not driven by the timer)
    *((unsigned char *) ccerx[timer-1]) &=~ (DISABLE_TIMER_OUTP_MASK[timer-1]);
}

// <--#SPLIT#--> //

/**
 * set an output value for a pin
 */
void digitalWrite(uint8_t pin, uint8_t val)
{
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);
	volatile uint8_t *out;

	if (port == NOT_A_PIN) return;

	// If the pin that support PWM output, we need to turn it off
	// before doing a digital write.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	out = portOutputRegister(port);

	BEGIN_CRITICAL

	if (val == LOW) {
		*out &= ~bit;
	} else {
		*out |= bit;
	}

	END_CRITICAL
}

// <--#SPLIT#--> //

/**
 * read a pin value
 */
int digitalRead(uint8_t pin)
{
	uint8_t timer = digitalPinToTimer(pin);
	uint8_t bit = digitalPinToBitMask(pin);
	uint8_t port = digitalPinToPort(pin);

	if (port == NOT_A_PIN) return LOW;

	// If the pin that support PWM output, we need to turn it off
	// before getting a digital reading.
	if (timer != NOT_ON_TIMER) turnOffPWM(timer);

	if (*portInputRegister(port) & bit) return HIGH;
	return LOW;
}
