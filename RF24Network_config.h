/*
 * Copyright (C) 2011 James Coliz, Jr. <maniacbug@ymail.com>
 * 2015 Ricardo Rodriguez.  <ricardo@ollinlabs.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 */

#ifndef __RF24_CONFIG_H__
#define __RF24_CONFIG_H__

#if defined (ARDUINO)
  #if ARDUINO < 100
    #include <WProgram.h>
  #else
    #include <Arduino.h>
  #endif
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/********** USER CONFIG **************/
/*
 * Remove the / at the beginning on the following comment block
 * to toggle the commented code and enable Serial debugging
 */
//*
#undef SERIAL_DEBUG
/*/
#define SERIAL_DEBUG
//*/

/*************************************/

#ifdef SERIAL_DEBUG
	#define IF_SERIAL_DEBUG(x) ({x;})
#else
	#define IF_SERIAL_DEBUG(x)
#endif

// Avoid spurious warnings
#if !defined( NATIVE ) && defined( ARDUINO )
	#undef PROGMEM
	#define PROGMEM __attribute__(( section(".progmem.data") ))
	#undef PSTR
	#define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];}))
#endif

// Progmem is Arduino-specific
#ifdef ARDUINO
	#include <avr/pgmspace.h>
	#define PRIPSTR "%S"
#else
	typedef char const prog_char;
	typedef uint16_t prog_uint16_t;
	#define PSTR(x) (x)
	#define printf_P printf
	#define strlen_P strlen
	#define PROGMEM
	#define pgm_read_word(p) (*(p))
	#define PRIPSTR "%s"
#endif

#endif // __RF24_CONFIG_H__
