#
# Make kernel.img for a Raspberry Pi
#

CROSS_COMPILE=arm-none-eabi-
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy

SFLAGS=-mcpu=arm1176jzf-s -mthumb -fpic -ffreestanding
CFLAGS=$(SFLAGS) -O1 -Wall -Wextra -std=gnu99 

RUST=rustc
RSFLAGS=-C opt-level=1 -Z no-landing-pads --target thumbv6-none-eabi -g --emit obj -L libcore-thumbv6

.PHONY : all clean

all :  kernel.img

clean :
	rm -f *.o *.elf *.img

start.o : start.S
	$(CC) $(SFLAGS) -c $^ -o $@

# C kernel
ckernel.o : ckernel.c
	$(CC) $(CFLAGS) -c $^ -o $@

# Pure Rust kernel
kernel.o : kernel.rs
	$(RUST) $(RSFLAGS) -o $@ $^
 
kernel.elf : start.o kernel.o
	$(CC) -T rpi.ld -o $@ -ffreestanding -nostdlib $^

kernel.img : kernel.elf
	$(OBJCOPY) $^ -O binary $@
