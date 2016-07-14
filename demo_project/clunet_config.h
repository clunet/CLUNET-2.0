#ifndef __clunet_config_h_included__
#define __clunet_config_h_included__

/* CONFIGURE YOUR DEVICE HERE */

/* Device address (0-254) */
#define CLUNET_DEVICE_ID 99

/* Device name */
#define CLUNET_DEVICE_NAME "CLUNET device"

/* Buffer sized (memory usage) */
#define CLUNET_SEND_BUFFER_SIZE 128
#define CLUNET_READ_BUFFER_SIZE 128

/* Pin to send data */
#define CLUNET_WRITE_PORT D
#define CLUNET_WRITE_PIN 1

/* Using transistor? */
#define CLUNET_WRITE_TRANSISTOR

/* Pin to receive data, external interrupt required! */
#define CLUNET_READ_PORT D
#define CLUNET_READ_PIN 2

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
#define CLUNET_TIMER_OVERFLOW_CLEAR set_bit(TIFR, TOV2)

/* End of Timer/Counter definitions */

/* How to enable and disable timer interrupts */
#define CLUNET_ENABLE_TIMER_COMP { set_bit(TIFR, OCF2); set_bit(TIMSK, OCIE2); }
#define CLUNET_DISABLE_TIMER_COMP unset_bit(TIMSK, OCIE2)

/* How to init and enable external interrupt (read pin) */
#define CLUNET_INIT_INT { set_bit(MCUCR,ISC00); unset_bit(MCUCR,ISC01); set_bit(GICR, INT0); }

/* Interrupt vectors */
#define CLUNET_TIMER_COMP_VECTOR TIMER2_COMP_vect
#define CLUNET_INT_VECTOR INT0_vect

#endif
