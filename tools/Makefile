TARGET = count

SOURCE = $(TARGET).c
OBJECT = $(SOURCE:.c=.o)

CC = gcc
LD = gcc

CFLAGS = -c -O3
LDFLAGS = -s

$(TARGET): $(OBJECT)
	$(CC) $(LDFLAGS) $(OBJECT) -o $@

.c.o:
	$(CC) $(CFLAGS) $< -o $@

all: $(SOURCE) $(TARGET)

clean:
	rm -f $(OBJECT) $(TARGET)

test: $(TARGET);
	$(TARGET) times.bin
