

CROSS_COMPILE=arm-none-eabi-
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy

SFLAGS=-mcpu=arm1176jzf-s -fpic -ffreestanding
CFLAGS=$(SFLAGS) -std=gnu99

.PHONY : clean

all :  kernel.img

clean :
	rm -f *.o *.elf *.img

start.o : start.S
	$(CC) $(SFLAGS) -c $^ -o $@

kernel.o : kernel.c
	$(CC) $(CFLAGS) -c $^ -o $@ -O2 -Wall -Wextra
 
kernel.elf : start.o kernel.o
	$(CC) -T rpi.ld -o $@ -ffreestanding -O2 -nostdlib $^

kernel.img : kernel.elf
	$(OBJCOPY) $^ -O binary $@
