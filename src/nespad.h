#ifndef __nespad_h__
#define __nespad_h__

#include "types.h"

#define BTN_A       0x80
#define BTN_B       0x40
#define BTN_SELECT  0x20
#define BTN_START   0x10
#define BTN_UP      0x08
#define BTN_DOWN    0x04
#define BTN_LEFT    0x02
#define BTN_RIGHT   0x01

extern u8 paddata;

void nespad_init(void);
void nespad_poll(void);

#endif
