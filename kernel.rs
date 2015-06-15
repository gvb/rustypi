#![feature(no_std)]
#![feature(core)]
#![feature(asm)]
#![no_std]
#![crate_type="staticlib"]

// rustc -C opt-level=2 -Z no-landing-pads --target arm-none-eabi -g --emit obj -L libcore-armm  -o my_rust_file.o my_rust_file.rs

// **************************************
// These are here just to make the linker happy
// These functions are just used for critical error handling so for now we just loop forever
// For more information see: https://github.com/rust-lang/rust/blob/master/src/doc/trpl/unsafe.md

#![feature(lang_items)]

extern crate core;

#[lang="stack_exhausted"] extern fn stack_exhausted() {}
#[lang="eh_personality"] extern fn eh_personality() {}

#[lang="panic_fmt"]
pub fn panic_fmt(_fmt: &core::fmt::Arguments, _file_line: &(&'static str, usize)) -> ! {
    loop {}
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr0() -> () {
    loop {}
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr1() -> () {
    loop {}
}

// **************************************
// **************************************

// And now we can write some Rust!

enum MemMap {
    GPIO_BASE = 0x20200000,
    UART0_BASE = 0x20201000,
}
 
enum GpioMemMap {
    // Controls actuation of pull up/down to ALL GPIO pins.
    GPPUD = 0x20200000 + 0x94,
 
    // Controls actuation of pull up/down for specific GPIO pin.
    GPPUDCLK0 = 0x20200000 + 0x98,
}
 
enum UartMemMap {
 
    // The offsets for reach register for the UART.
    UART0_DR      = 0x20201000 + 0x00,
    UART0_RSRECR  = 0x20201000 + 0x04,
    UART0_FR      = 0x20201000 + 0x18,
    UART0_ILPR    = 0x20201000 + 0x20,
    UART0_IBRD    = 0x20201000 + 0x24,
    UART0_FBRD    = 0x20201000 + 0x28,
    UART0_LCRH    = 0x20201000 + 0x2C,
    UART0_CR      = 0x20201000 + 0x30,
    UART0_IFLS    = 0x20201000 + 0x34,
    UART0_IMSC    = 0x20201000 + 0x38,
    UART0_RIS     = 0x20201000 + 0x3C,
    UART0_MIS     = 0x20201000 + 0x40,
    UART0_ICR     = 0x20201000 + 0x44,
    UART0_DMACR   = 0x20201000 + 0x48,
    UART0_ITCR    = 0x20201000 + 0x80,
    UART0_ITIP    = 0x20201000 + 0x84,
    UART0_ITOP    = 0x20201000 + 0x88,
    UART0_TDR     = 0x20201000 + 0x8C,
}
 
/* Loop <delay> times in a way that the compiler won't optimize away. */
fn delay(count: i32) -> () {
    unsafe {
        asm!("1: subs $0, $0, #1; bne 1b\n"
             :
             : "r"(count)
             : "cc"
             : "volatile");
    }
}

unsafe fn mmio_write(reg: u32, data: u32) -> () {
    asm!("str $1, [$0]\n"
         :
         : "r"(reg), "r"(data)
         :
         : "volatile");
//  *(reg as *mut u32) = data as u32;
}
 
unsafe fn mmio_read(reg: u32) -> u32 {
    let mut data: u32;
    asm!("ldr $0, [$1]\n"
         : "=r"(data)
         : "r"(reg)
         :
         : "volatile");
    data
//  *(reg as *mut u32)
}
 
unsafe fn uart_init() {
    // Disable UART0.
    mmio_write(UartMemMap::UART0_CR as u32, 0x00000000);
    // Setup the GPIO pin 14 && 15.
 
    // Disable pull up/down for all GPIO pins & delay for 150 cycles.
    mmio_write(GpioMemMap::GPPUD as u32, 0x00000000);
    delay(150);
 
    // Disable pull up/down for pin 14,15 & delay for 150 cycles.
    mmio_write(GpioMemMap::GPPUDCLK0 as u32, (1 << 14) | (1 << 15));
    delay(151);
 
    // Write 0 to GPPUDCLK0 to make it take effect.
    mmio_write(GpioMemMap::GPPUDCLK0 as u32, 0x00000000);
 
    // Clear pending interrupts.
    mmio_write(UartMemMap::UART0_ICR as u32, 0x7FF);
 
    // Set integer & fractional part of baud rate.
    // Divider = UART_CLOCK/(16 * Baud)
    // Fraction part register = (Fractional part * 64) + 0.5
    // UART_CLOCK = 3000000; Baud = 115200.
 
    // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
    // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
    mmio_write(UartMemMap::UART0_IBRD as u32, 1);
    mmio_write(UartMemMap::UART0_FBRD as u32, 40);
 
    // Enable FIFO & 8 bit data transmissio (1 stop bit, no parity).
    mmio_write(UartMemMap::UART0_LCRH as u32, (1 << 4) | (1 << 5) | (1 << 6));
 
    // Mask all interrupts.
    mmio_write(UartMemMap::UART0_IMSC as u32, (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                           (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
 
    // Enable UART0, receive & transfer part of UART.
    mmio_write(UartMemMap::UART0_CR as u32, (1 << 0) | (1 << 8) | (1 << 9));
}

unsafe fn uart_putc(byte: u8) {
    // Wait for UART to become ready to transmit.
    while mmio_read(UartMemMap::UART0_FR as u32) & (1 << 5) != 0 {
    }
    mmio_write(UartMemMap::UART0_DR as u32, byte as u32);
}
 
unsafe fn uart_getc() -> u8 {
    // Wait for UART to have recieved something.
    while mmio_read(UartMemMap::UART0_FR as u32) & (1 << 4) != 0 {
    }
    return mmio_read(UartMemMap::UART0_DR as u32) as u8;
}

use core::str::StrExt;
unsafe fn uart_puts(str: &str)
{
    for b in str.as_bytes() {
        uart_putc(*b);
    }
}
 
#[no_mangle]
pub fn kernel() -> () {

    unsafe {
        uart_init();
    }

    unsafe {
        mmio_write(UartMemMap::UART0_DR as u32, 0x42);
        uart_putc(0x31);
        uart_putc(0x32);
        uart_putc(0x33);
        uart_putc(0x42);
        uart_putc(0x0D);
        uart_putc(0x0A);
        uart_puts("Hello, Rusty Raspberry Pi world!\r\n");
    }

    loop {
        unsafe {
            uart_putc(uart_getc());
        }
    }
}
