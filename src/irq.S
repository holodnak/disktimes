#include <avr/io.h>

.extern timer2
.extern rise

.global INT4_vect
INT4_vect:
  in    sreg_save, _SFR_IO_ADDR(SREG)
  push  r24
  ldi   r24,lo8(1)
  sts   rise,r24
  pop   r24
  out   _SFR_IO_ADDR(SREG), sreg_save
  reti

.global TIMER2_COMPA_vect
TIMER2_COMPA_vect:
  in    sreg_save, _SFR_IO_ADDR(SREG)
  push  r24
  lds   r24,timer2
  subi  r24,lo8(-(1))
  sts   timer2,r24
  pop   r24
  out   _SFR_IO_ADDR(SREG), sreg_save
  reti
