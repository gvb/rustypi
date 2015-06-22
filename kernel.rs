#![feature(no_std)]
#![feature(core)]
#![feature(asm)]
#![no_std]
#![crate_type="staticlib"]

// rustc -C opt-level=2 -Z no-landing-pads --target arm-none-eabi -g --emit obj -L libcore-armm  -o my_rust_file.o my_rust_file.rs

//****************************************************************************
// Make some functions that the compiler generates references to so that
// we can compile and link a minimal standalone program.
//
// If these functions are called, it is game over so go into a spin loop.
//

#![feature(lang_items)]

extern crate core;

#[lang="stack_exhausted"]
extern fn stack_exhausted() {
    unsafe {
        asm!("1: b 1b\n" :::: "volatile");
    }
}

#[lang="eh_personality"]
extern fn eh_personality() {
    unsafe {
        asm!("1: b 1b\n" :::: "volatile");
    }
}

#[lang="panic_fmt"]
pub fn panic_fmt(_fmt: &core::fmt::Arguments, _file_line: &(&'static str, usize)) -> ! {
    // The rust compiler wants an infinite loop, but doesn't realize
    // our asm!() is one.
    loop {
        unsafe {
            asm!("1: b 1b\n" :::: "volatile");
        }
    }
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr0() -> () {
    unsafe {
        asm!("1: b 1b\n" :::: "volatile");
    }
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr1() -> () {
    unsafe {
        asm!("1: b 1b\n" :::: "volatile");
    }
}

/****************************************************************************
 * Define the memory map
 ***************************************************************************/

enum MemMap {
    GPIO_BASE = 0x20200000,
    UART0_BASE = 0x20201000,
}
 
enum GpioMemMap {
    // Controls actuation of pull up/down to ALL GPIO pins.
    GPPUD = 0x94,
 
    // Controls actuation of pull up/down for specific GPIO pin.
    GPPUDCLK0 = 0x98,
}
 
enum UartMemMap {
 
    // The offsets for reach register for the UART.
    UART0_DR      = 0x00,
    UART0_RSRECR  = 0x04,
    UART0_FR      = 0x18,
    UART0_ILPR    = 0x20,
    UART0_IBRD    = 0x24,
    UART0_FBRD    = 0x28,
    UART0_LCRH    = 0x2C,
    UART0_CR      = 0x30,
    UART0_IFLS    = 0x34,
    UART0_IMSC    = 0x38,
    UART0_RIS     = 0x3C,
    UART0_MIS     = 0x40,
    UART0_ICR     = 0x44,
    UART0_DMACR   = 0x48,
    UART0_ITCR    = 0x80,
    UART0_ITIP    = 0x84,
    UART0_ITOP    = 0x88,
    UART0_TDR     = 0x8C,
}
 
/*
 * Convenience macro to take a base address and register enum and turn
 * them into a u32 address.
 */
macro_rules! toAddr {
    ($base:expr, $reg:expr) => ($base as u32 + $reg as u32)
}

/****************************************************************************
 * Some utility functions to make the compiler do what needs to be done.
 ***************************************************************************/

// Loop <delay> times, marked "volatile" for the optimizer.
fn delay(count: i32) -> () {
    unsafe {
        asm!("1: subs $0, $0, #1; bne 1b\n"
             :
             : "r"(count)
             : "cc"
             : "volatile");
    }
}

// Memory write marked "volatile" for the optimizer.
unsafe fn mmio_write(reg: u32, data: u32) -> () {
    asm!("str $1, [$0]\n"
         :
         : "r"(reg), "r"(data)
         :
         : "volatile");
//  *(reg as *mut u32) = data as u32;
}
 
// Memory read marked "volatile" for the optimizer.
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
 
/****************************************************************************
 * UART
 ***************************************************************************/

unsafe fn uart_init() {
    // Disable UART0.
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_CR),
               0x00000000);
    // Setup the GPIO pin 14 && 15.
 
    // Disable pull up/down for all GPIO pins & delay for 150 cycles.
    mmio_write(toAddr!(MemMap::GPIO_BASE, GpioMemMap::GPPUD),
               0x00000000);
    delay(150);
 
    // Disable pull up/down for pin 14,15 & delay for 150 cycles.
    mmio_write(toAddr!(MemMap::GPIO_BASE, GpioMemMap::GPPUDCLK0),
               (1 << 14) | (1 << 15));
    delay(151);
 
    // Write 0 to GPPUDCLK0 to make it take effect.
    mmio_write(toAddr!(MemMap::GPIO_BASE, GpioMemMap::GPPUDCLK0),
               0x00000000);
 
    // Clear pending interrupts.
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_ICR),
               0x7FF);
 
    // Set integer & fractional part of baud rate.
    // Divider = UART_CLOCK/(16 * Baud)
    // Fraction part register = (Fractional part * 64) + 0.5
    // UART_CLOCK = 3000000; Baud = 115200.
 
    // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
    // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_IBRD), 1);
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_FBRD), 40);
 
    // Enable FIFO & 8 bit data transmissio (1 stop bit, no parity).
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_LCRH),
               (1 << 4) | (1 << 5) | (1 << 6));
 
    // Mask all interrupts.
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_IMSC),
               (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
               (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
 
    // Enable UART0, receive & transfer part of UART.
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_CR),
               (1 << 0) | (1 << 8) | (1 << 9));
}

unsafe fn uart_putc(byte: u8) {
    // Wait for UART to become ready to transmit.
    while mmio_read(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_FR)) & (1 << 5) != 0 {
    }
    mmio_write(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_DR),
               byte as u32);
}
 
unsafe fn uart_getc() -> u8 {
    // Wait for UART to have recieved something.
    while mmio_read(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_FR)) & (1 << 4) != 0 {
    }
    return mmio_read(toAddr!(MemMap::UART0_BASE, UartMemMap::UART0_DR)) as u8;
}

use core::str::StrExt;
unsafe fn uart_puts(str: &str)
{
    for b in str.as_bytes() {
        uart_putc(*b);
    }
}
 
/****************************************************************************
 * Our mainline Rust function
 ***************************************************************************/

#[no_mangle]
pub fn kernel() -> () {

    unsafe {
        uart_init();
    }

    unsafe {
        uart_puts("Hello, Rusty Raspberry Pi world!\r\n");
    }

    loop {
        unsafe {
            uart_putc(uart_getc());
        }
    }
}
