#ifndef TINY_USI_PINS_H_INCLUDED
#define TINY_USI_PINS_H_INCLUDED

#if defined(__AVR_ATtiny24__) ||defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
#define SPI_DDR_PORT DDRA
#define USCK_DD_PIN DDA4
#define DO_DD_PIN DDA5
#define DI_DD_PIN DDA6
#elif defined(__AVR_ATtiny25__) ||defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__) || \
	defined(__AVR_ATtiny261__) || defined(__AVR_ATtiny461__) || defined(__AVR_ATtiny861__) || \
	defined(__AVR_ATtiny87__) || defined(__AVR_ATtiny167__)
#define SPI_DDR_PORT DDRB
#define USCK_DD_PIN DDB2
#define DO_DD_PIN DDB1
#define DI_DD_PIN DDB0
#elif defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny2313A__) || defined(__AVR_ATtiny4313__)
#define SPI_DDR_PORT DDRB
#define USCK_DD_PIN DDB7
#define DO_DD_PIN DDB6
#define DI_DD_PIN DDB5
#endif

#endif
