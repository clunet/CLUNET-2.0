/**************************************************************************************
The MIT License (MIT)
Copyright (c) 2016 Sergey V. DUDANOV
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************************/

#ifndef __CLUNET_CONFIG_H_INCLUDED__
#define __CLUNET_CONFIG_H_INCLUDED__

#include "bits.h"

/* CONFIGURE YOUR DEVICE HERE */

/* Device address (0-254) */
#define CLUNET_DEVICE_ID 99

/* Device name */
#define CLUNET_DEVICE_NAME "CLUNET device"

/* Buffer sized (memory usage) */
#define CLUNET_SEND_BUFFER_SIZE 128
#define CLUNET_READ_BUFFER_SIZE 128

/* MCUs pin, external interrupt with any logical change is required! */
#define CLUNET_PORT D
#define CLUNET_PIN 2

/* Timer prescaler */
#define CLUNET_TIMER_PRESCALER 64

/*
	Custom T (T >= 8 && T <= 24).
	T is frame unit size in timer ticks. Lower - faster, highter - more stable.
	If not defined T will be calculated as ~64us based on CLUNET_TIMER_PRESCALER value.
*/
//#define CLUNET_T 8

/* Timer initialization in NORMAL MODE */
#define CLUNET_TIMER_INIT \
			{ \
				unset_bit4(TCCR2, WGM21, WGM20, COM21, COM20); \
				set_bit(TCCR2, CS22); \
				unset_bit2(TCCR2, CS21, CS20); \
			}

/* 8-bit Timer/Counter definitions */

// Main Register
#define CLUNET_TIMER_REG TCNT2
// Output Compare Register
#define CLUNET_TIMER_REG_OCR OCR2
// Overflow Condition
#define CLUNET_TIMER_OVERFLOW (TIFR & (1 << TOV2))
// Reset Overflow Flag Command
#define CLUNET_TIMER_OVERFLOW_CLEAR TIFR = (1 << TOV2)

/* End of Timer/Counter definitions */

/* How to enable and disable timer interrupts */
#define CLUNET_ENABLE_TIMER_COMP { set_bit(TIFR, OCF2); set_bit(TIMSK, OCIE2); }
#define CLUNET_DISABLE_TIMER_COMP unset_bit(TIMSK, OCIE2)

/* How to init and enable external interrupt (read pin) */
#define CLUNET_INT_INIT { set_bit(MCUCR,ISC00); unset_bit(MCUCR,ISC01); set_bit(GICR, INT0); }

/* Interrupt vectors */
#define CLUNET_TIMER_COMP_VECTOR TIMER2_COMP_vect
#define CLUNET_INT_VECTOR INT0_vect

#endif
