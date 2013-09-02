#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "nespad.h"

/*
pin config:
  d2 = latch
  d3 = clock
  d4 = data
*/

u8 paddata;

void nespad_init(void)
{
  DDRD |= 0xC;
  DDRD &= ~0x10;
  PORTD |= 0x10;
  paddata = 0;
}

void nespad_poll()
{
  u8 ret = 0;
  int i;

  PORTD |= 4;
  _delay_ms(1);
  PORTD &= ~4;
  ret = (PIND & 0x10) >> 4;
  for(i=0;i<7;i++) {
    PORTD |= 8;
    _delay_ms(1);
    ret <<= 1;
    ret |= (PIND & 0x10) >> 4;
    PORTD &= ~8;
    _delay_ms(1);
  }
  paddata = ret ^ 0xFF;
}
