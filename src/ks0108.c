#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "ks0108.h"

/*

pin config:

cs1   = e6
cs2   = e7
d0-d7 = c0-c7
e     = d7
r/w   = e0
d/i   = e1
reset = ??

*/

#define EN    0xD7
#define R_W   0xE0
#define D_I   0xE1
#define CSEL1 0xE6
#define CSEL2 0xE7


// macros for pasting port defines
#define GLUE(a, b)     a##b 
#define PORT(x)        GLUE(PORT, x)
#define PIN(x)         GLUE(PIN, x)
#define DDR(x)         GLUE(DDR, x)

// paste together the port definitions if using nibbles
#define LCD_DATA_LOW_NBL   C   // port for low nibble:  pins 10-13
#define LCD_DATA_HIGH_NBL  C   // port for high nibble: pins 14-17

#define LCD_DATA_IN_LOW   PIN(LCD_DATA_LOW_NBL) // Data I/O Register, low nibble
#define LCD_DATA_OUT_LOW  PORT(LCD_DATA_LOW_NBL)  // Data Output Register - low nibble
#define LCD_DATA_DIR_LOW  DDR(LCD_DATA_LOW_NBL) // Data Direction Register for Data Port, low nibble

#define LCD_DATA_IN_HIGH  PIN(LCD_DATA_HIGH_NBL)  // Data Input Register  high nibble
#define LCD_DATA_OUT_HIGH PORT(LCD_DATA_HIGH_NBL) // Data Output Register - high nibble
#define LCD_DATA_DIR_HIGH DDR(LCD_DATA_HIGH_NBL)  // Data Direction Register for Data Port, high nibble

#define lcdDataOut(_val_) LCD_DATA_OUT(_val_) 
#define lcdDataDir(_val_) LCD_DATA_DIR(_val_) 

// macros to handle data output
#define LCD_DATA_OUT(_val_) LCD_DATA_OUT_LOW = (_val_);   
#define LCD_DATA_DIR(_val_) LCD_DATA_DIR_LOW = (_val_);


#define DISPLAY_WIDTH   128
#define DISPLAY_HEIGHT  64

// panel controller chips
#define CHIP_WIDTH      64  // pixels per chip 

//commands
#define LCD_ON          0x3F
#define LCD_OFF         0x3E
#define LCD_DISP_START  0xC0
#define LCD_SET_ADD     0x40
#define LCD_SET_PAGE    0xB8

#define LCD_BUSY_FLAG   0x80 

// Colors
#define BLACK       0xFF
#define WHITE       0x00

// useful user contants
#define NON_INVERTED  0
#define INVERTED      1

// Font Indices
#define FONT_LENGTH       0
#define FONT_FIXED_WIDTH  2
#define FONT_HEIGHT       3
#define FONT_FIRST_CHAR   4
#define FONT_CHAR_COUNT   5
#define FONT_WIDTH_TABLE  6

#define EN_DELAY_VALUE 6 // this is the delay value that may need to be hand tuned for slow panels

#define EN_DELAY() \
  asm volatile( "ldi r24, %0 \n\t" "subi r24, 0x1 \n\t" "and r24, r24 \n\t" "brne .-6 \n\t" ::"M" (EN_DELAY_VALUE) : "r24" )

static FontCallback fontcb = 0;
static u8 *fontdata = 0;
static lcdCoord coord;
static u8 inverted;

static void fastWriteLow(int idx)
{
  int bit = 1 << (idx & 0xF);

  switch(idx & 0xF0) {
    case 0xA0:  PORTA &= ~bit;  break;
    case 0xB0:  PORTB &= ~bit;  break;
    case 0xC0:  PORTC &= ~bit;  break;
    case 0xD0:  PORTD &= ~bit;  break;
    case 0xE0:  PORTE &= ~bit;  break;
    case 0xF0:  PORTF &= ~bit;  break;
  }
}

static void fastWriteHigh(int idx)
{
  int bit = 1 << (idx & 0xF);

  switch(idx & 0xF0) {
    case 0xA0:  PORTA |= bit;  break;
    case 0xB0:  PORTB |= bit;  break;
    case 0xC0:  PORTC |= bit;  break;
    case 0xD0:  PORTD |= bit;  break;
    case 0xE0:  PORTE |= bit;  break;
    case 0xF0:  PORTF |= bit;  break;
  }
}

void ks0108_init(u8 invert)
{
  u8 chip;

  DDRC = 0xFF;
  DDRD |= 0x80;
  DDRE |= 0xC3;

  _delay_ms(10);

  fastWriteLow(D_I);
  fastWriteLow(R_W);
  fastWriteLow(EN);

  coord.x = 0;
  coord.y = 0;
  coord.page = 0;
  inverted = invert;
  
  for(chip=0; chip < DISPLAY_WIDTH/CHIP_WIDTH; chip++) {
    _delay_ms(10);  
    ks0108_writecommand(LCD_ON,chip);        // power on
    ks0108_writecommand(LCD_DISP_START,chip);    // display start line = 0
  }
  _delay_ms(50); 
  ks0108_clearscreen(invert ? BLACK : WHITE);      // display clear
  ks0108_gotoxy(0,0);
}

void ks0108_reset(void)
{
  //drive reset pin low to reset display
  PORTD &= ~0x40;
  _delay_ms(10);
  PORTD |= 0x40;
}

void ks0108_clearpage(u8 page,u8 color)
{
  for(uint8_t x=0; x < DISPLAY_WIDTH; x++) { 
    ks0108_gotoxy(x,page * 8);
    ks0108_writedata(color);
  }
}

void ks0108_clearscreen(u8 color)
{
  u8 page;

  for(page=0;page<8;page++) {
    ks0108_gotoxy(0, page * 8);
    ks0108_clearpage(page,color);
  }
}

void ks0108_enable(void)
{
  EN_DELAY();
  fastWriteHigh(EN);         // EN high level width min 450 ns
  EN_DELAY();
  fastWriteLow(EN);
}

static u8 chipSelect[] = {1,2};        // this is for 128 pixel displays

void ks0108_selectchip(u8 chip)
{
  if(chipSelect[chip] & 1)
    fastWriteHigh(CSEL1);
  else
    fastWriteLow(CSEL1);
  if(chipSelect[chip] & 2)
    fastWriteHigh(CSEL2);
  else
    fastWriteLow(CSEL2);
}

void ks0108_waitready(u8 chip)
{
  ks0108_selectchip(chip);
  lcdDataDir(0x00);
  fastWriteLow(D_I);  
  fastWriteHigh(R_W); 
  fastWriteHigh(EN);  
  EN_DELAY();
  while(LCD_DATA_IN_HIGH & LCD_BUSY_FLAG);
  fastWriteLow(EN);   
}

void ks0108_writecommand(u8 cmd,u8 chip)
{
  if(coord.x % CHIP_WIDTH == 0 && chip > 0){     // todo , ignore address 0???
    EN_DELAY();
  }
  ks0108_waitready(chip);
  fastWriteLow(D_I);          // D/I = 0
  fastWriteLow(R_W);          // R/W = 0  
  lcdDataDir(0xFF);

  EN_DELAY();
  lcdDataOut(cmd);
  ks0108_enable();            // enable
  EN_DELAY();
  EN_DELAY();
  lcdDataOut(0x00);
}

static u8 ks0108_doreaddata(u8 first)
{
  u8 data, chip;

  chip = coord.x/CHIP_WIDTH;
  ks0108_waitready(chip);
  if(first){
    if(coord.x % CHIP_WIDTH == 0 && chip > 0){    // todo , remove this test and call GotoXY always?
      ks0108_gotoxy(coord.x,coord.y);
      ks0108_waitready(chip);
    }
  } 
  fastWriteHigh(D_I);         // D/I = 1
  fastWriteHigh(R_W);         // R/W = 1
  
  fastWriteHigh(EN);          // EN high level width: min. 450ns
  EN_DELAY();

#ifdef LCD_DATA_NIBBLES
   data = (LCD_DATA_IN_LOW & 0x0F) | (LCD_DATA_IN_HIGH & 0xF0);
#else
   data = LCD_DATA_IN_LOW;        // low and high nibbles on same port so read all 8 bits at once
#endif 
  fastWriteLow(EN); 
    if(first == 0) 
    ks0108_gotoxy(coord.x,coord.y); 
  if(inverted)
    data = ~data;
  return data;
}

u8 ks0108_readdata()
{
  u8 ret;

  ks0108_doreaddata(1);
  ret = ks0108_doreaddata(0);
  coord.x++;
  return(ret);
}

void ks0108_writedata(uint8_t data)
{
  u8 displayData, yOffset, chip;
  //showHex("wrData",data);
    //showXY("wr", coord.x,coord.y);

#ifdef LCD_CMD_PORT 
  uint8_t cmdPort;  
#endif

  if(coord.x >= DISPLAY_WIDTH)
    return;
 chip = coord.x/CHIP_WIDTH; 
 ks0108_waitready(chip);


 if(coord.x % CHIP_WIDTH == 0 && chip > 0){     // todo , ignore address 0???
   ks0108_gotoxy(coord.x, coord.y);
 }

  fastWriteHigh(D_I);         // D/I = 1
  fastWriteLow(R_W);          // R/W = 0  
  lcdDataDir(0xFF);         // data port is output
  
  yOffset = coord.y%8;

  if(yOffset != 0) {
    // first page
#ifdef LCD_CMD_PORT 
    cmdPort = LCD_CMD_PORT;           // save command port
#endif
    displayData = ks0108_readdata();
#ifdef LCD_CMD_PORT     
    LCD_CMD_PORT = cmdPort;           // restore command port
#else
    fastWriteHigh(D_I);         // D/I = 1
    fastWriteLow(R_W);          // R/W = 0
    ks0108_selectchip(chip);
#endif
    lcdDataDir(0xFF);           // data port is output
    
    displayData |= data << yOffset;
    if(inverted)
      displayData = ~displayData;
    lcdDataOut(displayData);         // write data
    ks0108_enable();               // enable
    
    // second page
    ks0108_gotoxy(coord.x, coord.y+8);
    
    displayData = ks0108_readdata();

#ifdef LCD_CMD_PORT     
    LCD_CMD_PORT = cmdPort;           // restore command port
#else   
  fastWriteHigh(D_I);         // D/I = 1
  fastWriteLow(R_W);          // R/W = 0  
  ks0108_selectchip(chip);
#endif
    lcdDataDir(0xFF);       // data port is output
    
    displayData |= data >> (8-yOffset);
    if(inverted)
      displayData = ~displayData;
    lcdDataOut(displayData);    // write data
    ks0108_enable();         // enable
    ks0108_gotoxy(coord.x+1, coord.y-8);
  }
  else {
    // just this code gets executed if the write is on a single page
    if(inverted)
      data = ~data;   
        EN_DELAY();
    lcdDataOut(data);       // write data
    ks0108_enable();         // enable
    coord.x++;
  }
}

void ks0108_gotoxy(u8 x,u8 y)
{
  uint8_t chip, cmd;
  
  if((x > DISPLAY_WIDTH-1) || (y > DISPLAY_HEIGHT-1)) // exit if coordinates are not legal
    return;
  coord.x = x;                // save new coordinates
  coord.y = y;
  
  if((y / 8) != coord.page) {
    coord.page = y / 8;
    cmd = LCD_SET_PAGE | coord.page;      // set y address on all chips 
    for(chip=0; chip < DISPLAY_WIDTH/CHIP_WIDTH; chip++){
      ks0108_writecommand(cmd,chip); 
    }
  }
  chip = coord.x / CHIP_WIDTH;
  x = x % CHIP_WIDTH;
  cmd = LCD_SET_ADD | x;
  ks0108_writecommand(cmd,chip);          // set x address on active chip  
}

void ks0108_invert(u8 state)
{
  inverted = state;
}

//font stuff
uint8_t ReadPgmData(uint8_t* ptr) {  // note this is a static function
  return pgm_read_byte(ptr);
}

void ks0108_selectfont(u8 *font,FontCallback callback)
{
  if(callback == 0)
    callback = ReadPgmData;
  fontdata = font;
  fontcb = callback;
}

void ks0108_printnumber(long n)
{
  u8 buf[10];  // prints up to 10 digits  
  u8 i = 0;

  if(n == 0) {
    ks0108_putchar('0');
    return;
  }

  if(n < 0) {
    ks0108_putchar('-');
    n = -n;
  }
  while(n>0 && i <= 10) {
    buf[i++] = n % 10;  // n % base
    n /= 10;   // n/= base
  }
  for(; i >0; i--)
    ks0108_putchar((char) (buf[i-1] < 10 ? '0' + buf[i-1] : 'A' + buf[i-1] - 10));    
}

int ks0108_putchar(char c) {
  uint8_t width = 0;
  uint8_t height = fontcb(fontdata+FONT_HEIGHT);
  uint8_t bytes = (height+7)/8;
  
  uint8_t firstChar = fontcb(fontdata+FONT_FIRST_CHAR);
  uint8_t charCount = fontcb(fontdata+FONT_CHAR_COUNT);
  
  uint16_t index = 0;
  uint8_t x = coord.x, y = coord.y;
  
  if(c < firstChar || c >= (firstChar+charCount)) {
    return 1;
  }
  c-= firstChar;

  if( fontcb(fontdata+FONT_LENGTH) == 0 && fontcb(fontdata+FONT_LENGTH+1) == 0) {
    // zero length is flag indicating fixed width font (array does not contain width data entries)
     width = fontcb(fontdata+FONT_FIXED_WIDTH); 
     index = c*bytes*width+FONT_WIDTH_TABLE;
  }
  else{
  // variable width font, read width data, to get the index
     for(uint8_t i=0; i<c; i++) {  
     index += fontcb(fontdata+FONT_WIDTH_TABLE+i);
     }
     index = index*bytes+charCount+FONT_WIDTH_TABLE;
     width = fontcb(fontdata+FONT_WIDTH_TABLE+c);
    }

  // last but not least, draw the character
  for(uint8_t i=0; i<bytes; i++) {
    uint8_t page = i*width;
    for(uint8_t j=0; j<width; j++) {
      uint8_t data = fontcb(fontdata+index+page+j);
    
      if(height > 8 && height < (i+1)*8) {
        data >>= (i+1)*8-height;
      }
      
//      if(fontdataColor == BLACK) {
        ks0108_writedata(data);
//      } else {
//        ks0108_writedata(~data);
//      }
    }
    // 1px gap between chars
//    if(fontdataColor == BLACK) {
      ks0108_writedata(0x00);
//    } else {
//      ks0108_writedata(0xFF);
//    }
    ks0108_gotoxy(x, coord.y+8);
  }
  ks0108_gotoxy(x+width+1, y);

  return 0;
}

void ks0108_puts(char* str) {
  int x = coord.x;
  while(*str != 0) {
    if(*str == '\n') {
      ks0108_gotoxy(x, coord.y+fontcb(fontdata+FONT_HEIGHT));
    } else {
      ks0108_putchar(*str);
    }
    str++;
  }
}

void ks0108_puts_p(PGM_P str) {
  int x = coord.x;
  while(pgm_read_byte(str) != 0) {
    if(pgm_read_byte(str) == '\n') {
      ks0108_gotoxy(x, coord.y+fontcb(fontdata+FONT_HEIGHT));
    } else {
      ks0108_putchar(pgm_read_byte(str));
    }
    str++;
  }
}

uint8_t ks0108_charwidth(char c) {
  uint8_t width = 0;
  uint8_t firstChar = fontcb(fontdata+FONT_FIRST_CHAR);
  uint8_t charCount = fontcb(fontdata+FONT_CHAR_COUNT);
  
  // read width data
  if(c >= firstChar && c < (firstChar+charCount)) {
    c -= firstChar;
    width = fontcb(fontdata+FONT_WIDTH_TABLE+c)+1;
  }
  
  return width;
}

uint16_t ks0108_stringwidth(char* str) {
  uint16_t width = 0;
  
  while(*str != 0) {
    width += ks0108_charwidth(*str++);
  }
  
  return width;
}

uint16_t ks0108_stringwidth_p(PGM_P str) {
  uint16_t width = 0;
  
  while(pgm_read_byte(str) != 0) {
    width += ks0108_charwidth(pgm_read_byte(str++));
  }
  
  return width;
}
