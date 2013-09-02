#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include "types.h"
#include "ks0108.h"
#include "nespad.h"
#include "SystemFont5x7.h"
#include "util.h"
#include "../lib/sd-reader/fat.h"
#include "../lib/sd-reader/fat_config.h"
#include "../lib/sd-reader/partition.h"
#include "../lib/sd-reader/sd_raw.h"
#include "../lib/sd-reader/sd_raw_config.h"

#define CPU_PRESCALE(n) (CLKPR = 0x80, CLKPR = (n))
#define CPU_16MHz       0x00
#define CPU_8MHz        0x01
#define CPU_4MHz        0x02
#define CPU_2MHz        0x03
#define CPU_1MHz        0x04
#define CPU_500kHz      0x05
#define CPU_250kHz      0x06
#define CPU_125kHz      0x07
#define CPU_62kHz       0x08

#define is_mediaset()   ((PINF & 0x02) == 0)
#define is_motoron()    ((PINF & 0x04) != 0)
#define is_ready()      ((PINF & 0x08) == 0)
#define is_writable()   ((PINF & 0x10) == 0)

struct partition_struct *partition;
struct fat_fs_struct *fs;
struct fat_dir_entry_struct directory;
struct fat_dir_struct *dd;
struct fat_file_struct *fd;

#define SKIP_GAP_BITS   20000

#define TIMEBUFSIZE     (1024 * 6 + 512)
volatile u8 timebuf[TIMEBUFSIZE];
volatile u16 timebufpos;

volatile long skip = SKIP_GAP_BITS;
volatile long bits = 0;
volatile u8 started;

//external interrupt tied to change of signal connected to d0 (ready, active low)
ISR(INT0_vect)
{
  //if disk drive is ready, start outputting data
  if(is_ready()) {
    TCNT0 = 0;
    EIMSK |= (1 << INT4);     //enable INT4
  }

  //disk drive no longer ready, transfer complete
  else {
    EIMSK &= ~(1 << INT4);    //disable INT4
    started = 0;
  }
}

//external interrupt tied to rise of signal connected to e4 (read data)
ISR(INT4_vect)
{
  u8 time = TCNT0;

  TCNT0 = 0;

  bits++;

  if(skip) {
    skip--;
    return;
  }

  if(timebufpos < TIMEBUFSIZE)
    timebuf[timebufpos++] = time;
}

void timeunit_init(void)
{
  //initialize time unit (for receiving data from the disk drive)
  TCCR0A = 0x00;      //disable unused features
  TCCR0B = 0x02;      //div by 8 prescaler
  TIMSK0 = 0x00;      //disable interrupts
}

void diskdrive_init(void)
{
/*
pin config:

stop motor    = f0 (input)  (active low)
media set     = f1 (output) (active low)
batt/motor on = f2 (output)
ready         = f3 (output) (active low)
read data     = f4 (output)
rw media      = f5 (output) (active low)
write         = f6 (input)
scan media    = f7 (input)  (active low)
write data    = d5 (input)  (active low)
*/

  //set port input/outputs
  DDRF = 0xC1;
  DDRD |= 0x20;

  //enable pullups
  PORTF = 0x3E;

  //default values
  PORTD = 0x20;
  PORTF = 0x80;

  //e4 as input (irq for read data)
  DDRE &= ~0x10;
  PORTE |= 0x10;

  //d0 as input (irq for ready change)
  DDRD &= ~0x01;
  PORTD |= 0x01;

  //external interrupts
  EIMSK = 0x00;     //disable all external interrupts
  EICRA = 0x01;     //set INT0 to trigger with level change
  EICRB = 0x03;     //set INT4 to trigger with 0->1
}

void init(void)
{
  //led output
  DDRD |= (1 << 6);

  //power reduction register
  PRR0 = 0;

  //initialize subsystems
  diskdrive_init();
  ks0108_init(0);
  ks0108_clearscreen(0);
  ks0108_selectfont(System5x7,0);
  ks0108_gotoxy(0,0);
  timeunit_init();
  nespad_init();

  if(sd_raw_init() == 0) {
    ks0108_gotoxy(0,56);
    ks0108_puts("MMC/SD init failed");
    _delay_ms(500);
    return;
  }

  //open partition
  partition = partition_open(sd_raw_read,sd_raw_read_interval,sd_raw_write,sd_raw_write_interval,0);
  if(partition == 0) {
    //if it failed try in no-mbr mode
    partition = partition_open(sd_raw_read,sd_raw_read_interval,sd_raw_write,sd_raw_write_interval,-1);
    if(!partition) {
      ks0108_gotoxy(0,56);
      ks0108_puts("opening partition failed");
      _delay_ms(500);
      return;
    }
  }

  //open the fat filesystem
  if((fs = fat_open(partition)) == 0) {
    ks0108_gotoxy(0,56);
    ks0108_puts("opening filesystem failed");
    _delay_ms(500);
    return;
  }

  fat_get_dir_entry_of_path(fs,"/",&directory);
  dd = fat_open_dir(fs, &directory);
  if(dd == 0) {
    ks0108_gotoxy(0,56);
    ks0108_puts("opening root directory failed");
    _delay_ms(500);
    return;
  }

  ks0108_gotoxy(0,56);
  ks0108_puts("sd card init ok");
  _delay_ms(500);
}

uint8_t find_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name, struct fat_dir_entry_struct* dir_entry)
{
    while(fat_read_dir(dd, dir_entry))
    {
        if(strcmp(dir_entry->long_name, name) == 0)
        {
            fat_reset_dir(dd);
            return 1;
        }
    }

    return 0;
}

struct fat_file_struct* open_file_in_dir(struct fat_fs_struct* fs, struct fat_dir_struct* dd, const char* name)
{
    struct fat_dir_entry_struct file_entry;
    if(!find_file_in_dir(fs, dd, name, &file_entry))
        return 0;

    return fat_open_file(fs, &file_entry);
}

int main(void)
{
  struct fat_dir_entry_struct file_entry;
  char outputfile[] = "times.bin";

  //initialize clock speed and interrupts
  CPU_PRESCALE(CPU_16MHz);
//  set_sleep_mode(SLEEP_MODE_IDLE);

  cli();

  //initialize everything
  init();

  //clear screen
  ks0108_clearscreen(0);

  //enable interrupts
  sei();

  PORTD |= 0x20;
  PORTF |= 0x40;

  PORTF &= ~0x01;
  PORTF |= 0x80;

  PORTD &= ~(1 << 6);

  started = 0;

  //the main loop
  for(;;) {

    //toggle the led
    PORTD ^= (1 << 6);

    if(started || is_motoron()) {
      continue;
    }

    if(timebufpos) {
      ks0108_gotoxy(0,48);
      if(fat_write_file(fd,(uint8_t*)timebuf,timebufpos) != timebufpos) {
        ks0108_puts("wrote time buffer fail");
      }      
      else
        ks0108_puts("wrote time buffer ok");
      timebufpos = 0;
      ks0108_gotoxy(64,16);
      ks0108_printnumber(bits);
    }
    //needs to be polled 60 times a second
    nespad_poll();

    //check for jumping to the bootloader
    if(paddata & BTN_START) {
      PORTF &= ~0x01;
      PORTF |= 0x80;
      fat_close_file(fd);
      sd_raw_sync();
      bootloader();
    }

    //start transfer
    if((paddata & BTN_A) && is_mediaset()) {
      ks0108_gotoxy(0,40);
      ks0108_puts("opening output file");
      if(fat_create_file(dd,outputfile,&file_entry)) {
        int32_t offset = 0;

        fd = open_file_in_dir(fs,dd,outputfile);
        if(!fat_seek_file(fd,&offset,FAT_SEEK_SET)) {
          ks0108_gotoxy(0,48);
          ks0108_puts("error seeking");
          fat_close_file(fd);
        }
        else {
          ks0108_gotoxy(0,40);
          ks0108_puts("opened output file ");
        }
      }
      skip = SKIP_GAP_BITS;
      timebufpos = 0;
      started = 1;
      PORTF &= ~0x01;
      PORTF |= 0x80;
      EIMSK = 0x01;     //enable INT0, disable INT4
      _delay_ms(50);
      PORTF |= 0x01;
      PORTF &= ~0x80;
    }
  }
}
