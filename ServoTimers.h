/*
  Servo.h - Interrupt driven Servo library for Arduino using 16 bit timers- Version 2
  Copyright (c) 2009 Michael Margolis.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*
 * Defines for 16 bit timers used with Servo library
 *
 * If _useTimerX is defined then TimerX is a 16 bit timer on the current board
 * timer16_Sequence_t enumerates the sequence that the timers should be allocated
 * _Nbr_16timers indicates how many 16 bit timers are available.
 */

/**
 * AVR Only definitions
 * --------------------
 */


/*
#define _useTimer3
typedef enum { _timer3, _Nbr_16timers } timer16_Sequence_t;
*/


#define _useTimer1
typedef enum { _timer1, _Nbr_16timers } timer16_Sequence_t;



#define TCCR3A  _SFR_MEM8(0x90)
#define WGM30   0
#define WGM31   1
#define COM3B0  4
#define COM3B1  5
#define COM3A0  6
#define COM3A1  7

#define TCCR3B  _SFR_MEM8(0x91)
#define CS30    0
#define CS31    1
#define CS32    2
#define WGM32   3
#define WGM33   4
#define ICES3   6
#define ICNC3   7

#define TCCR3C  _SFR_MEM8(0x92)
#define FOC3B   6
#define FOC3A   7

/* Reserved [0x93] */

/* Combine TCNT3L and TCNT3H */
#define TCNT3   _SFR_MEM16(0x94)

#define TCNT3L  _SFR_MEM8(0x94)
#define TCNT3H  _SFR_MEM8(0x95)

/* Combine ICR3L and ICR3H */
#define ICR3    _SFR_MEM16(0x96)

#define ICR3L   _SFR_MEM8(0x96)
#define ICR3H   _SFR_MEM8(0x97)

/* Combine OCR3AL and OCR3AH */
#define OCR3A   _SFR_MEM16(0x98)

#define OCR3AL  _SFR_MEM8(0x98)
#define OCR3AH  _SFR_MEM8(0x99)

/* Combine OCR3BL and OCR3BH */
#define OCR3B   _SFR_MEM16(0x9A)

#define OCR3BL  _SFR_MEM8(0x9A)
#define OCR3BH  _SFR_MEM8(0x9B)

/* Reserved [0x9C..0x9F] */

#define TIFR3   _SFR_IO8(0x18)
#define TOV3    0
#define OCF3A   1
#define OCF3B   2
#define ICF3    5

#define TIMSK3  _SFR_MEM8(0x71)
#define TOIE3   0
#define OCIE3A  1
#define OCIE3B  2
#define ICIE3   5
