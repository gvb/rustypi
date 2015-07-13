#
# Make kernel.img for a Raspberry Pi
#

MACHINE=thumbv6
RUSTLIBSRC=../rust/src

CROSS_COMPILE=arm-none-eabi-
CC=$(CROSS_COMPILE)gcc
OBJCOPY=$(CROSS_COMPILE)objcopy

SFLAGS=-mcpu=arm1176jzf-s -mthumb -fpic -ffreestanding
CFLAGS=$(SFLAGS) -O1 -Wall -Wextra -std=gnu99 

RUST=rustc
RFLAGS=-C opt-level=1 -Z no-landing-pads --target $(MACHINE)-none-eabi -g --emit obj
RLFLAGS=-C opt-level=2 -Z no-landing-pads --target $(MACHINE)-none-eabi -g --crate-type=lib 


.PHONY : all clean distclean

all :  kernel.img

clean :
	rm -f *.o *.elf *.img \
	rm -f lib-$(MACHINE)/*

distclean : clean

# Start up code
start.o : start.S
	$(CC) $(SFLAGS) -c $^ -o $@

# CPU utilities, ARM (not Thumb) code
cpu.o : cpu.S
	$(CC) $(SFLAGS) -c $^ -o $@

# C kernel
ckernel.o : ckernel.c
	$(CC) $(CFLAGS) -c $^ -o $@

# Pure Rust kernel
kernel.o : kernel.rs \
		lib-$(MACHINE)/libcore.rlib \
		lib-$(MACHINE)/librpi.rlib
	$(RUST) $(RFLAGS) -L lib-$(MACHINE) -o $@ $<

# Target hardware library
lib-$(MACHINE)/librpi.rlib : rpi/src/lib.rs \
		lib-$(MACHINE)/libcore.rlib \
		cpu.o
	$(RUST) $(RLFLAGS) -L lib-$(MACHINE) --out-dir lib-$(MACHINE) $<

# Rust library
lib-$(MACHINE)/libcore.rlib : $(RUSTLIBSRC)/libcore/lib.rs \
			$(wildcard $(RUSTLIBSRC)/*/*.rs)
	$(RUST) $(RLFLAGS) --out-dir lib-$(MACHINE) $<

kernel.elf : start.o kernel.o \
		lib-$(MACHINE)/librpi.rlib \
		cpu.o
	$(CC) -T rpi.ld -o $@ -ffreestanding -nostdlib $^

kernel.img : kernel.elf
	$(OBJCOPY) $^ -O binary $@
