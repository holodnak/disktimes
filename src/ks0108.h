
#ifndef __ks0108_h__
#define __ks0108_h__

#include "types.h"

typedef struct {
  uint8_t x;
  uint8_t y;
  uint8_t page;
} lcdCoord;

typedef uint8_t (*FontCallback)(uint8_t*);

void ks0108_init(u8 invert);
void ks0108_reset(void);
void ks0108_clearpage(u8 page,u8 color);
void ks0108_clearscreen(u8 color);
void ks0108_enable(void);
void ks0108_selectchip(u8 chip);
void ks0108_waitready(u8 chip);
void ks0108_writecommand(u8 cmd,u8 chip);
u8 ks0108_readdata(void);
void ks0108_writedata(uint8_t data);
void ks0108_gotoxy(u8 x,u8 y);
void ks0108_invert(u8 state);

void ks0108_selectfont(u8 *font,FontCallback callback);
void ks0108_printnumber(long n);
int ks0108_putchar(char c);
void ks0108_puts(char* str);
void ks0108_puts_p(PGM_P str);
uint8_t ks0108_charwidth(char c);
uint16_t ks0108_stringwidth(char* str);
uint16_t ks0108_stringwidth_p(PGM_P str);

#endif
