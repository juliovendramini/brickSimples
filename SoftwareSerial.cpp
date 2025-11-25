/*
SoftwareSerial.cpp (formerly NewSoftSerial.cpp) - 
Multi-instance software serial library for Arduino/Wiring
-- Interrupt-driven receive and other improvements by ladyada
   (http://ladyada.net)
-- Tuning, circular buffer, derivation from class Print/Stream,
   multi-instance support, porting to 8MHz processors,
   various optimizations, PROGMEM delay tables, inverse logic and 
   direct port writing by Mikal Hart (http://www.arduiniana.org)
-- Pin change interrupt macros by Paul Stoffregen (http://www.pjrc.com)
-- 20MHz processor support by Garrett Mace (http://www.macetech.com)
-- ATmega1280/2560 support by Brett Hagman (http://www.roguerobotics.com/)

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

The latest version of this library can always be found at
http://arduiniana.org.
*/

// 
// Includes
// 
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>
#include "SoftwareSerial.h"
#include <util/delay_basic.h>



//
// Private methods
//

/* static */ 
inline void SoftwareSerial::tunedDelay(uint16_t delay) { 
  _delay_loop_2(delay);
}

//
// Leitura bloqueante de um byte
//
int SoftwareSerial::readByteDirect()
{
  // Aguarda start bit (linha vai para LOW)
  unsigned long start = millis();
  while (rx_pin_read())
  {
    if (millis() - start > _timeout_ms)
      return -1;  // Timeout
  }

  uint8_t d = 0;

  // Wait approximately 1/2 of a bit width to "center" the sample
  tunedDelay(_rx_delay_centering);
  

  // Read each of the 8 bits
  for (uint8_t i=8; i > 0; --i)
  {
    tunedDelay(_rx_delay_intrabit);
    d >>= 1;
    
    if (rx_pin_read())
      d |= 0x80;
  }

  // skip the stop bit
  tunedDelay(_rx_delay_stopbit);
  

  return d;
}

uint8_t SoftwareSerial::rx_pin_read()
{
  return *_receivePortRegister & _receiveBitMask;
}

//
// Interrupt handling removido - agora usa leitura bloqueante
//

//
// Constructor
//
SoftwareSerial::SoftwareSerial(uint8_t receivePin, uint8_t transmitPin) : 
  _rx_delay_centering(0),
  _rx_delay_intrabit(0),
  _rx_delay_stopbit(0),
  _tx_delay(0),
  _timeout_ms(100)  // 100 ms de timeout padrão
{
  setTX(transmitPin);
  setRX(receivePin);
}

//
// Destructor
//
SoftwareSerial::~SoftwareSerial()
{
  end();
}

void SoftwareSerial::setTX(uint8_t tx)
{
  // First write, then set output. If we do this the other way around,
  // the pin would be output low for a short while before switching to
  // output high. Now, it is input with pullup for a short while, which is fine.
  digitalWrite(tx, HIGH);
  pinMode(tx, OUTPUT);
  _transmitBitMask = digitalPinToBitMask(tx);
  uint8_t port = digitalPinToPort(tx);
  _transmitPortRegister = portOutputRegister(port);
}

void SoftwareSerial::setRX(uint8_t rx)
{
  pinMode(rx, INPUT);
  digitalWrite(rx, HIGH);  // pullup for normal logic!
  _receivePin = rx;
  _receiveBitMask = digitalPinToBitMask(rx);
  uint8_t port = digitalPinToPort(rx);
  _receivePortRegister = portInputRegister(port);
}

uint16_t SoftwareSerial::subtract_cap(uint16_t num, uint16_t sub) {
  if (num > sub)
    return num - sub;
  else
    return 1;
}

//
// Public methods
//

void SoftwareSerial::begin(long speed)
{
  _rx_delay_centering = _rx_delay_intrabit = _rx_delay_stopbit = _tx_delay = 0;

  // Precalculate the various delays, in number of 4-cycle delays
  uint16_t bit_delay = (F_CPU / speed) / 4;

  // 12 (gcc 4.8.2) or 13 (gcc 4.3.2) cycles from start bit to first bit,
  // 15 (gcc 4.8.2) or 16 (gcc 4.3.2) cycles between bits,
  // 12 (gcc 4.8.2) or 14 (gcc 4.3.2) cycles from last bit to stop bit
  // These are all close enough to just use 15 cycles, since the inter-bit
  // timings are the most critical (deviations stack 8 times)
  _tx_delay = subtract_cap(bit_delay, 15 / 4);

  // Setup rx timings for blocking read (sem overhead de interrupção)
  #if GCC_VERSION > 40800
  // Timings para leitura bloqueante (mais simples que interrupção)
  // Apenas ~20 ciclos de overhead no loop
  _rx_delay_centering = subtract_cap(bit_delay / 2, 20 / 4);
  _rx_delay_intrabit = subtract_cap(bit_delay, 20 / 4);
  _rx_delay_stopbit = subtract_cap(bit_delay, 20 / 4);
  #else
  _rx_delay_centering = subtract_cap(bit_delay / 2, 20 / 4);
  _rx_delay_intrabit = subtract_cap(bit_delay, 20 / 4);
  _rx_delay_stopbit = subtract_cap(bit_delay, 20 / 4);
  #endif

  tunedDelay(_tx_delay); // if we were low this establishes the end


}

void SoftwareSerial::end()
{
  // Nada para fazer sem interrupções
}


// Leitura bloqueante de 1 byte com timeout
int SoftwareSerial::read()
{
  return readByteDirect();
}

// Leitura bloqueante de múltiplos bytes com timeout
size_t SoftwareSerial::readBytes(uint8_t *buffer, size_t length)
{
  size_t count = 0;
  while (count < length)
  {
    int c = readByteDirect();
    if (c < 0)
      break;  // Timeout
    buffer[count++] = (uint8_t)c;
  }
  return count;
}

int SoftwareSerial::available()
{
  // Sem buffer - verifica apenas se há start bit presente (LOW)
  return rx_pin_read() ? 0 : 1;
}

size_t SoftwareSerial::write(uint8_t b)
{
  if (_tx_delay == 0) {
    setWriteError();
    return 0;
  }

  // By declaring these as local variables, the compiler will put them
  // in registers _before_ disabling interrupts and entering the
  // critical timing sections below, which makes it a lot easier to
  // verify the cycle timings
  volatile uint8_t *reg = _transmitPortRegister;
  uint8_t reg_mask = _transmitBitMask;
  uint8_t inv_mask = ~_transmitBitMask;
  uint8_t oldSREG = SREG;
  uint16_t delay = _tx_delay;

  cli();  // turn off interrupts for a clean txmit

  // Write the start bit (LOW)
  *reg &= inv_mask;

  tunedDelay(delay);

  // Write each of the 8 bits
  for (uint8_t i = 8; i > 0; --i)
  {
    if (b & 1) // choose bit
      *reg |= reg_mask; // send 1
    else
      *reg &= inv_mask; // send 0

    tunedDelay(delay);
    b >>= 1;
  }

  // restore pin to natural state (HIGH)
  *reg |= reg_mask;

  SREG = oldSREG; // turn interrupts back on
  tunedDelay(_tx_delay);
  
  return 1;
}

void SoftwareSerial::flush()
{
  // There is no tx buffering, simply return
}

int SoftwareSerial::peek()
{
  // Peek não é possível sem buffer de interrupção
  return -1;
}
