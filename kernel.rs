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
    asm!("1: b 1b\n" :::: "volatile");
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr1() -> () {
    asm!("1: b 1b\n" :::: "volatile");
}

use core::str::StrExt;

/****************************************************************************
 * Define the memory map
 ***************************************************************************/

enum MemMap {
    GpioBase = 0x20200000,
    Uart0Base = 0x20201000,
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
 * GPIO - General Purpose Input / Output
 ***************************************************************************/

enum GpioMemMap {
    // Controls actuation of pull up/down to ALL GPIO pins.
    GPPUD = 0x94,
 
    // Controls actuation of pull up/down for specific GPIO pin.
    GPPUDCLK0 = 0x98,
}
 
struct Gpio {
    base_addr: u32
}

impl Gpio {

    // TODO: Verify operation and comment
    unsafe fn init(&self) {
        mmio_write(self.base_addr + GpioMemMap::GPPUD as u32, 0x00000000);
        delay(150);
 
        // Write a zero to GPPUDCLK0 to make it take effect.
        mmio_write(self.base_addr + GpioMemMap::GPPUDCLK0 as u32, 0x00000000);
    }

    unsafe fn config_uart0(&self) {
        mmio_write(self.base_addr + GpioMemMap::GPPUDCLK0 as u32,
                   (1 << 14) | (1 << 15));
        delay(150);
 
        // Write a zero to GPPUDCLK0 to make it take effect.
        mmio_write(self.base_addr + GpioMemMap::GPPUDCLK0 as u32, 0x00000000);
    }
}

/****************************************************************************
 * UART - Universal Asynchronous Recever / Transmitter
 ***************************************************************************/

// The offsets for reach register for the UART. Not all the registers
// are used.
#[allow(dead_code)]
enum UartMemMap {
    DR      = 0x00,
    RSRECR  = 0x04,
    FR      = 0x18,
    ILPR    = 0x20,
    IBRD    = 0x24,
    FBRD    = 0x28,
    LCRH    = 0x2C,
    CR      = 0x30,
    IFLS    = 0x34,
    IMSC    = 0x38,
    RIS     = 0x3C,
    MIS     = 0x40,
    ICR     = 0x44,
    DMACR   = 0x48,
    ITCR    = 0x80,
    ITIP    = 0x84,
    ITOP    = 0x88,
    TDR     = 0x8C,
}

struct Uart {
    base_addr: u32,
}

impl Uart {

    unsafe fn disable(&self) {
        // Disable UART.
        mmio_write(self.base_addr + UartMemMap::CR as u32, 0x00000000);
    }

    unsafe fn init(&self) {
        // Clear pending interrupts.
        mmio_write(self.base_addr + UartMemMap::ICR as u32, 0x7FF);
 
        // Set integer & fractional part of baud rate.
        // Divider = UART_CLOCK/(16 * Baud)
        // Fraction part register = (Fractional part * 64) + 0.5
        // UART_CLOCK = 3000000; Baud = 115200.
 
        // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
        // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
        mmio_write(self.base_addr + UartMemMap::IBRD as u32, 1);
        mmio_write(self.base_addr + UartMemMap::FBRD as u32, 40);
 
        // Enable FIFO & 8 bit data transmissio (1 stop bit, no parity).
        mmio_write(self.base_addr + UartMemMap::LCRH as u32,
                   (1 << 4) | (1 << 5) | (1 << 6));
 
        // Mask all interrupts.
        mmio_write(self.base_addr + UartMemMap::IMSC as u32,
                   (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                   (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
 
        // Enable UART, receive & transfer part of UART.
        mmio_write(self.base_addr + UartMemMap::CR as u32,
                   (1 << 0) | (1 << 8) | (1 << 9));
    }

    unsafe fn putc(&self, byte: u8) {
        // Wait for UART to become ready to transmit.
        while mmio_read(self.base_addr + UartMemMap::FR as u32) &
                                                             (1 << 5) != 0 {
        }
        mmio_write(self.base_addr + UartMemMap::DR as u32,
                   byte as u32);
    }
 
    unsafe fn getc(&self) -> u8 {
        // Wait for UART to have recieved something.
        while mmio_read(self.base_addr + UartMemMap::FR as u32) & (1 << 4) != 0 {
        }
        return mmio_read(self.base_addr + UartMemMap::DR as u32) as u8;
    }

    unsafe fn puts(&self, str: &str)
    {
        for b in str.as_bytes() {
            self.putc(*b);
        }
    }
}
 
/****************************************************************************
 * Our mainline Rust function
 ***************************************************************************/

#[no_mangle]
pub fn kernel() -> () {

    let uart0 = Uart{base_addr: MemMap::Uart0Base as u32};
    let gpio = Gpio{base_addr: MemMap::GpioBase as u32};

    unsafe {
        gpio.init();
        uart0.disable();
        gpio.config_uart0();
        uart0.init();
    }

    unsafe {
        uart0.puts("Hello, Rusty Raspberry Pi world!\r\n");
    }

    loop {
        unsafe {
            uart0.putc(uart0.getc());
        }
    }
}
