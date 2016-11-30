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

#include "SPI.h"

TinySPIClass SPI;

uint8_t TinySPIClass::bitOrder = MSBFIRST;
uint8_t TinySPIClass::interruptMode = 0;
uint8_t TinySPIClass::interruptMask = 0;
uint8_t TinySPIClass::interruptSave = 0;
#ifdef SPI_TRANSACTION_MISMATCH_LED
uint8_t TinySPIClass::inTransactionFlag = 0;
#endif

void TinySPIClass::begin(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		USICR &= ~(_BV(USISIE) | _BV(USIOIE) | _BV(USIWM1));
		USICR |= _BV(USIWM0) | _BV(USICS1) | _BV(USICLK);

		SPI_DDR_PORT &= ~_BV(DI_DD_PIN);
		SPI_DDR_PORT |= _BV(USCK_DD_PIN) | _BV(DO_DD_PIN);
	}
}

void TinySPIClass::end(void)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		USICR &= ~(_BV(USIWM0) | _BV(USICS1) | _BV(USICLK));
		interruptMode = 0;
		#ifdef SPI_TRANSACTION_MISMATCH_LED
		inTransactionFlag = 0;
		#endif
	}
}

/* mapping of interrupt numbers to bits within SPI_AVR_EIMSK */
#ifdef INT0
#define SPI_INT0_MASK  (1<<INT0)
#endif
#ifdef INT1
#define SPI_INT1_MASK  (1<<INT1)
#endif
#ifdef INT2
#define SPI_INT2_MASK  (1<<INT2)
#endif

void TinySPIClass::usingInterrupt(uint8_t interruptNumber)
{
	uint8_t mask = 0;
	uint8_t sreg = SREG;
	cli(); // Protect from a scheduler and prevent transactionBegin
	switch (interruptNumber) {
	#ifdef SPI_INT0_MASK
	case 0: mask = SPI_INT0_MASK; break;
	#endif
	#ifdef SPI_INT1_MASK
	case 1: mask = SPI_INT1_MASK; break;
	#endif
	#ifdef SPI_INT2_MASK
	case 2: mask = SPI_INT2_MASK; break;
	#endif
	default:
		interruptMode = 2;
		break;
	}
	interruptMask |= mask;
	if (!interruptMode)
		interruptMode = 1;
	SREG = sreg;
}

void TinySPIClass::notUsingInterrupt(uint8_t interruptNumber)
{
	/* Once in mode 2 we can't go back to 0 without a proper reference count */
	if (interruptMode == 2)
		return;
	uint8_t mask = 0;
	uint8_t sreg = SREG;
	cli(); // Protect from a scheduler and prevent transactionBegin
	switch (interruptNumber) {
	#ifdef SPI_INT0_MASK
	case 0: mask = SPI_INT0_MASK; break;
	#endif
	#ifdef SPI_INT1_MASK
	case 1: mask = SPI_INT1_MASK; break;
	#endif
	#ifdef SPI_INT2_MASK
	case 2: mask = SPI_INT2_MASK; break;
	#endif
	default:
		break;
		/* this case can't be reached */
	}
	interruptMask &= ~mask;
	if (!interruptMask)
		interruptMode = 0;
	SREG = sreg;
}
