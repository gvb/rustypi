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
	rm -f *.o *.elf *.img librpi-$(MACHINE)/*.rlib

distclean : clean
	rm -f libcore-$(MACHINE)/*

start.o : start.S
	$(CC) $(SFLAGS) -c $^ -o $@

# C kernel
ckernel.o : ckernel.c
	$(CC) $(CFLAGS) -c $^ -o $@

# Pure Rust kernel
kernel.o : kernel.rs libcore-$(MACHINE)/libcore.rlib librpi-$(MACHINE)/librpi.rlib
	$(RUST) $(RFLAGS) \
		-L libcore-$(MACHINE) \
		-L librpi-$(MACHINE) \
		-o $@ $<

# Target hardware library
librpi-$(MACHINE)/librpi.rlib : rpi/src/lib.rs \
			$(wildcard rpi/src/*.rs)
	$(RUST) $(RLFLAGS) \
		-L libcore-$(MACHINE) \
		--out-dir librpi-$(MACHINE) $<

# Rust library
libcore-$(MACHINE)/libcore.rlib : $(RUSTLIBSRC)/libcore/lib.rs \
			$(wildcard $(RUSTLIBSRC)/*/*.rs)
	$(RUST) $(RLFLAGS) --out-dir libcore-$(MACHINE) $<

kernel.elf : start.o kernel.o librpi-$(MACHINE)/librpi.rlib
	$(CC) -T rpi.ld -o $@ -ffreestanding -nostdlib $^

kernel.img : kernel.elf
	$(OBJCOPY) $^ -O binary $@
