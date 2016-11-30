/*
 * Copyright (c) 2010 by Cristian Maglie <c.maglie@arduino.cc>
 * Copyright (c) 2014 by Paul Stoffregen <paul@pjrc.com> (Transaction API)
 * Copyright (c) 2014 by Matthijs Kooijman <matthijs@stdin.nl> (SPISettings AVR)
 * Copyright (c) 2014 by Andrew J. Kroll <xxxajk@gmail.com> (atomicity fixes)
 * Copyright (c) 2016 by Augusto Dantas <aldantas@protonmail.com> (port to USI)
 * SPI Master library for arduino.
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */

#ifndef SPI_H_INCLUDED
#define SPI_H_INCLUDED

#include <stdlib.h>

#include <avr/io.h>
#include <util/atomic.h>

#include "tiny_usi_pins.h"

/* SPI_HAS_TRANSACTION means SPI has beginTransaction(), endTransaction(), */
/* usingInterrupt(), and SPISetting(clock, bitOrder, dataMode) */
#define SPI_HAS_TRANSACTION 1

// SPI_HAS_NOTUSINGINTERRUPT means that SPI has notUsingInterrupt() method
#define SPI_HAS_NOTUSINGINTERRUPT 1

/* SPI_ATOMIC_VERSION means that SPI has atomicity fixes and what version. */
/* This way when there is a bug fix you can check this define to alert users */
/* of your code if it uses better version of this library. */
/* This also implies everything that SPI_HAS_TRANSACTION as documented above is */
/* available too. */
#define SPI_ATOMIC_VERSION 1

/* Uncomment this line to add detection of mismatched begin/end transactions. */
/* A mismatch occurs if other libraries fail to use SPI.endTransaction() for */
/* each SPI.beginTransaction().  Connect an LED to this pin.  The LED will turn */
/* on if any mismatch is ever detected. */
//#define SPI_TRANSACTION_MISMATCH_LED 5

#ifdef SPI_TRANSACTION_MISMATCH_LED
/* Necessary to use the pinMode and digitalWrite routines */
#include <Arduino.h>
#endif

#ifndef LSBFIRST
#define LSBFIRST 0
#endif
#ifndef MSBFIRST
#define MSBFIRST 1
#endif

#define SPI_MODE0 0x08
#define SPI_MODE1 0x0C
#define SPI_MODE_MASK 0x0C

/* Syntax compability with the Arduino's default SPI Library */
/* The ATTiny MCUs do not have SPI clock configuration */
#define SPI_CLOCK_DIV4 0x00
#define SPI_CLOCK_DIV16 0x01
#define SPI_CLOCK_DIV64 0x02
#define SPI_CLOCK_DIV128 0x03
#define SPI_CLOCK_DIV2 0x04
#define SPI_CLOCK_DIV8 0x05
#define SPI_CLOCK_DIV32 0x06
#define SPI_CLOCK_MASK 0x03
#define SPI_2XCLOCK_MASK 0x01
#define SPI_MODE2 0x08
#define SPI_MODE3 0x08

/* define SPI_AVR_EIMSK for AVR boards with external interrupt pins */
#if defined(EIMSK)
  #define SPI_AVR_EIMSK  EIMSK
#elif defined(GICR)
  #define SPI_AVR_EIMSK  GICR
#elif defined(GIMSK)
  #define SPI_AVR_EIMSK  GIMSK
#endif

class TinySPISettings {
public:
	TinySPISettings(uint8_t bitOrder, uint8_t dataMode)
	{
		init_AlwaysInline(bitOrder, dataMode);
	}

	TinySPISettings()
	{
		init_AlwaysInline(LSBFIRST, SPI_MODE0);
	}

	/* Syntax compability with the Arduino's default SPI Library */
	/* The ATTiny MCUs do not have SPI clock configuration */
	TinySPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode)
	{
		init_AlwaysInline(bitOrder, dataMode);
	}
private:
	void init_AlwaysInline(uint8_t bitOrder, uint8_t dataMode)
		__attribute__((__always_inline__))
		{
			/* Pack into the TinySPISettings class */
			usicr = _BV(USIWM0) | _BV(USICLK) | (dataMode & SPI_MODE_MASK);
			this->bitOrder = bitOrder == LSBFIRST ? 0 : 1;
		}
	uint8_t usicr;
	uint8_t bitOrder;
	friend class TinySPIClass;
};

class TinySPIClass
{
public:

	/* Initialize the SPI library */
	void begin(void);

	/* If SPI is used from within an interrupt, this function registers */
	/* that interrupt with the SPI library, so beginTransaction() can */
	/* prevent conflicts.  The input interruptNumber is the number used */
	/* with attachInterrupt.  If SPI is used from a different interrupt */
	/* (eg, a timer), interruptNumber should be 255. */
	static void usingInterrupt(uint8_t interruptNumber);

	// And this does the opposite.
	static void notUsingInterrupt(uint8_t interruptNumber);
	/* Note: the usingInterrupt and notUsingInterrupt functions should */
	/* not to be called from ISR context or inside a transaction. */
	/* For details see: */
	/* https://github.com/arduino/Arduino/pull/2381 */
	/* https://github.com/arduino/Arduino/pull/2449 */

	/* Before using SPI.transfer() or asserting chip select pins, */
	/* this function is used to gain exclusive access to the SPI bus */
	/* and configure the correct settings. */
	inline static void beginTransaction(TinySPISettings settings)
	{
		if (interruptMode > 0) {
			uint8_t sreg = SREG;
			cli();

			#ifdef SPI_AVR_EIMSK
			if (interruptMode == 1) {
				interruptSave = SPI_AVR_EIMSK;
				SPI_AVR_EIMSK &= ~interruptMask;
				SREG = sreg;
			} else
			#endif
			{
				interruptSave = sreg;
			}
		}

		#ifdef SPI_TRANSACTION_MISMATCH_LED
		if (inTransactionFlag) {
			pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
			digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
		}
		inTransactionFlag = 1;
		#endif

		USICR = settings.usicr;
		TinySPIClass::bitOrder = settings.bitOrder;
	}

	/* Write to the SPI bus (MOSI pin) and also receive (MISO pin) */
	inline static uint8_t transfer(uint8_t data)
	{
		if (TinySPIClass::bitOrder == LSBFIRST) reverseByte(data);
		USIDR = data;
		/* Clear Counter Overwflow Interrupt Flag to be able to receive new data */
		USISR = _BV(USIOIF);
		/* Wait for transmission complete */
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
			/* The following NOP introduces a small delay that can prevent the wait */
			/* loop form iterating when running at the maximum speed. This gives */
			/* about 10% more speed, even if it seems counter-intuitive. At lower */
			/* speeds it is unnoticed. */
			asm volatile("nop");
			while (!(USISR & _BV(USIOIF))) {
				/* Toggle the USCK and clock the 4-bit counter, */
				/* When the transfer in complete, USIOIF is set */
				USICR |= _BV(USITC);
			}
		}
		if (TinySPIClass::bitOrder == LSBFIRST) return reverseByte(USIDR);
		else return USIDR;
	}

	inline static uint16_t transfer16(uint16_t data)
	{
		union { uint16_t val; struct { uint8_t lsb; uint8_t msb; }; } in, out;
		in.val = data;
		if (TinySPIClass::bitOrder == LSBFIRST) {
			USIDR = reverseByte(in.lsb);
			asm volatile("nop"); // See transfer(uint8_t) function
			while (!(USISR & _BV(USIOIF))) USICR |= _BV(USITC);
			out.lsb = reverseByte(USIDR);
			USIDR = reverseByte(in.msb);
			asm volatile("nop");
			while (!(USISR & _BV(USIOIF))) USICR |= _BV(USITC);
			out.msb = reverseByte(USIDR);
		} else {
			USIDR = in.msb;
			asm volatile("nop");
			while (!(USISR & _BV(USIOIF))) USICR |= _BV(USITC);
			out.msb = USIDR;
			USIDR = in.lsb;
			asm volatile("nop");
			while (!(USISR & _BV(USIOIF))) USICR |= _BV(USITC);
			out.lsb = USIDR;
		}
		return out.val;
	}

	/* Does not reverse bits and bytes order for LSBFISRT */
	inline static void transfer(void *buf, size_t count)
	{
		if (count == 0) return;
		uint8_t *p = (uint8_t *)buf;
		USIDR = *p;
		while (--count > 0) {
			uint8_t out = *(p + 1);
			while (!(USISR & _BV(USIOIF))) USICR |= _BV(USITC);
			uint8_t in = USIDR;
			USIDR = out;
			*p++ = in;
		}
		while (!(USISR & _BV(USIOIF))) USICR |= _BV(USITC);
		*p = USIDR;
	}

	/* After performing a group of transfers and releasing the chip select */
	/* signal, this function allows others to access the SPI bus */
	inline static void endTransaction(void)
	{
		#ifdef SPI_TRANSACTION_MISMATCH_LED
		if (!inTransactionFlag) {
			pinMode(SPI_TRANSACTION_MISMATCH_LED, OUTPUT);
			digitalWrite(SPI_TRANSACTION_MISMATCH_LED, HIGH);
		}
		inTransactionFlag = 0;
		#endif

		if (interruptMode > 0) {
			#ifdef SPI_AVR_EIMSK
			uint8_t sreg = SREG;
			#endif
			cli();
			#ifdef SPI_AVR_EIMSK
			if (interruptMode == 1) {
				SPI_AVR_EIMSK = interruptSave;
				SREG = sreg;
			} else
			#endif
			{
				SREG = interruptSave;
			}
		}
	}

	/* Disable the SPI bus */
	static void end();

	/* This function is deprecated. New applications should use */
	/* beginTransaction() to configure SPI settings. */
	inline static void setBitOrder(uint8_t bitOrder)
	{
		TinySPIClass::bitOrder = bitOrder == LSBFIRST ? 0 : 1;
	}

	/* This function is deprecated. New applications should use */
	/* beginTransaction() to configure SPI settings. */
	inline static void setDataMode(uint8_t dataMode)
	{
		USICR = (USICR & ~SPI_MODE_MASK) | dataMode;
	}

	/* This function is deprecated. New applications should use */
	/* beginTransaction() to configure SPI settings. */
	/* Function maintained for syntax compability purpose */
	inline static void setClockDivider(uint8_t clockDiv)
	{}

private:
	static uint8_t bitOrder;
	static uint8_t interruptMode; // 0=none, 1=mask, 2=global
	static uint8_t interruptMask; // which interrupts to mask
	static uint8_t interruptSave; // temp storage, to restore state
	#ifdef SPI_TRANSACTION_MISMATCH_LED
	static uint8_t inTransactionFlag;
	#endif
	inline static uint8_t reverseByte(uint8_t byte)
	{
		byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4;
		byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2;
		byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1;
		return byte;
	}
};

extern TinySPIClass SPI;

#endif
