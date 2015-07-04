#![feature(no_std)]
#![feature(core)]
#![feature(asm)]
#![no_std]
#![crate_type="staticlib"]
#![feature(core_intrinsics)]

// rustc -C opt-level=2 -Z no-landing-pads --target arm-none-eabi -g --emit obj -L libcore-armm  -o my_rust_file.o my_rust_file.rs

//****************************************************************************
// Make some functions that the compiler generates references to so that
// we can compile and link a minimal standalone program.
//
// If these functions are called, it is game over so go into a spin loop.
//

#![feature(lang_items)]

extern crate core;
use core::intrinsics;
extern crate rpi;

// Our target hardware
use rpi::memory_map;
use rpi::gpio;
use rpi::uart;

#[lang="stack_exhausted"]
extern fn stack_exhausted() {
    unsafe {
        intrinsics::breakpoint();
    }
}

#[lang="eh_personality"]
extern fn eh_personality() {
    unsafe {
        intrinsics::breakpoint();
    }
}

#[lang="panic_fmt"]
pub fn panic_fmt(_fmt: &core::fmt::Arguments, _file_line: &(&'static str, usize)) -> ! {
    // The rust compiler wants an infinite loop, but doesn't realize
    // our breakpoing is one.
    unsafe {
        intrinsics::breakpoint();
    }
    loop {}
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr0() -> () {
    intrinsics::breakpoint();
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr1() -> () {
    intrinsics::breakpoint();
}


/****************************************************************************
 * Our mainline Rust function
 ***************************************************************************/

#[no_mangle]
pub fn kernel() -> () {

    let uart0 = uart::Uart{base_addr: memory_map::UART0BASE};
    let gpio = gpio::Gpio{base_addr: memory_map::GPIOBASE};

    gpio.init();
    uart0.disable();
    gpio.config_uart0();
    uart0.init();

    uart0.puts("Hello, Rusty Raspberry Pi world!\r\n");

    loop {
        uart0.putc(uart0.getc());
    }
}
