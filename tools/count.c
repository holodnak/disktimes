#include <stdio.h>

int times[256];
int t[5];

int main(int argc,char *argv[])
{
  FILE *in;
  int i,n;

  if(argc < 2) {
    printf("usage:  %s times.bin\n",argv[0]);
    return(1);
  }

  if((in = fopen(argv[1],"rb")) == 0) {
    printf("error opening '%s'\n",argv[1]);
    return(2);
  }

  for(i=0;i<256;i++) {
    times[i] = 0;
    if(i <= 4)
    t[i] = 0;
  }

  while(feof(in) == 0) {
    times[fgetc(in)]++;
  }

  fclose(in);

  for(i=0;i<256;i++) {
    n = times[i];
    if(n)
      printf("  $%02X -- %d\n",i,n);
    if(i < 0x11)
      t[3] += n;
    else if(i < 0x1B)
      t[0] += n;
    else if(i < 0x25)
      t[1] += n;
    else if(i < 0x30)
      t[2] += n;
    else
      t[4] += n;
  }

  printf("%d  %d  %d  -- %d  %d",t[0],t[1],t[2],t[3],t[4]);

  return(0);
}
/*

0000-0001 0010-1010

*/