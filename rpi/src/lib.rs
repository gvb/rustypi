#![feature(no_std)]
#![feature(core)]
#![feature(asm)]
#![no_std]
#![crate_type="staticlib"]
#![crate_name = "rpi"]
#![feature(core_intrinsics)]
#![feature(core_str_ext)]

extern crate core;

/****************************************************************************
 * Define the memory map
 ***************************************************************************/

//pub mod memory_map;

pub mod memory_map {
    pub const GPIOBASE:  usize = 0x20200000;
    pub const UART0BASE: usize = 0x20201000;
}

/****************************************************************************
 * CPU utilities
 ***************************************************************************/

//pub mod cpu_util;

pub mod cpu_util {
    // Loop <delay> times, using "volatile" to prevent the optimizer from
    // optimizing it to nothing.
    pub fn delay(count: usize) -> () {
        unsafe {
            asm!("push {$0};
                  1: subs $0, $0, #1;
                  bne 1b;
                  pop {$0}\n"
                 :
                 : "r"(count)
                 : "cc"
                 : "volatile");
        }
    }
}
/****************************************************************************
 * MMIO - Memory Mapped Input / Output utility
 ***************************************************************************/

//pub mod mmio;

pub mod mmio {
    use core::intrinsics;

    // Memory write marked "volatile" for the optimizer.
    pub fn write(reg: usize, data: usize) -> () {
        unsafe {
            intrinsics::volatile_store(reg as *mut usize, data);
        }
    }
 
    // Memory read marked "volatile" for the optimizer.
    pub fn read(reg: usize) -> usize {
        unsafe {
            intrinsics::volatile_load(reg as *const usize) as usize
        }
    }
}

/****************************************************************************
 * GPIO - General Purpose Input / Output
 ***************************************************************************/

//pub mod gpio;

pub mod gpio {
    use super::*;

    // Function selections
    const GPFSEL0: usize = 0x00;
    const GPFSEL1: usize = 0x04;
    const GPFSEL2: usize = 0x08;
    const GPFSEL3: usize = 0x0C;
    const GPFSEL4: usize = 0x10;
    const GPFSEL5: usize = 0x14;

    // Pin control (set/clear/level)
//  const GPSET0: usize = 0x1C;
//  const GPSET1: usize = 0x20;

//  const GPCLR0: usize = 0x28;
//  const GPCLR1: usize = 0x2C;

//  const GPLEV0: usize = 0x34;
//  const GPLEV1: usize = 0x38;

    // Event detect status
//  const GPEDS0: usize = 0x40;
//  const GPEDS1: usize = 0x44;

    // Rising edge detect enables
//  const GPREN0: usize = 0x4C;
//  const GPREN1: usize = 0x50;

    // Falling edge detect enables
//  const GPFEN0: usize = 0x58;
//  const GPFEN1: usize = 0x5C;

    // High detect enables
//  const GPHEN0: usize = 0x64;
//  const GPHEN1: usize = 0x68;

    // Low detect enables
//  const GPLEN0: usize = 0x70;
//  const GPLEN1: usize = 0x74;

    // Async rising edge detect enables
//  const GPAREN0: usize = 0x7C;
//  const GPAREN1: usize = 0x80;

    // Async falling edge detect enables
//  const GPAFEN0: usize = 0x88;
//  const GPAFEN1: usize = 0x8C;

    // Pin pull-up/down enables
    const GPPUD: usize = 0x94;
 
    // Pin pull-up/down enable clocks
    const GPPUDCLK0: usize = 0x98;
    const GPPUDCLK1: usize = 0x9C;

    #[allow(dead_code)]
    pub enum GpioFunctionSelect {
        // Function selections for the pins.
        // Note the alt function selections are numbered a little oddly.
        Input = 0,
        Output = 1,
        Alt0 = 4,
        Alt1 = 5,
        Alt2 = 6,
        Alt3 = 7,
        Alt4 = 3,
        Alt5 = 2,
    }
 
    #[allow(dead_code)]
    pub enum GpioPullUpDown {
        // GPIO pin pullup/down disabled, pull up, pull down.
        Off = 0,
        PullDown = 1,
        PullUp = 2,
    }
 
    pub struct Gpio {
        pub base_addr: usize
    }

    impl Gpio {

        // TODO: Verify operation and comment
        pub fn init(&self) {
        }

        pub fn config_pull_up_down(&self, pin: usize, config: GpioPullUpDown) {
            // Ref: BCM2834 ARM Peripherals manual, page 95

            // Configure the control: off/pulldown/pullup
            mmio::write(self.base_addr + GPPUD,
                        config as usize);
            // Required setup time
            cpu_util::delay(150);
            // Clock the appropriate pin's clock
            match pin {
                0 ... 31 =>
                    mmio::write(self.base_addr + GPPUDCLK0,
                                1 << (pin as usize)),
                32 ... 53 =>
                    mmio::write(self.base_addr + GPPUDCLK1,
                                1 << (pin as usize - 32)),
                _ => {}
            }
            // Required setup time
            cpu_util::delay(150);
            // Write to the clock register
            match pin {
                0 ... 31 =>
                    mmio::write(self.base_addr + GPPUDCLK0, 0),
                32 ... 53 =>
                    mmio::write(self.base_addr + GPPUDCLK1, 0),
                _ => {}
            }
        }

        pub fn config_function(&self, pin: usize, config: GpioFunctionSelect) {
            let (adjpin, reg) = match pin {
                0 ... 9 =>
                    (pin,      self.base_addr + GPFSEL0),
                10 ... 19 =>
                    (pin - 10, self.base_addr + GPFSEL1),
                20 ... 29 =>
                    (pin - 20, self.base_addr + GPFSEL2),
                30 ... 39 =>
                    (pin - 30, self.base_addr + GPFSEL3),
                40 ... 49 =>
                    (pin - 40, self.base_addr + GPFSEL4),
                50 ... 53 =>
                    (pin - 50, self.base_addr + GPFSEL5),
                // TODO: Flag this as an error, probably with a panic.
                _ => (0,0)
            };
            mmio::write(reg, (mmio::read(reg) & 0b111 << (adjpin * 3)) |
                             (config as usize) << (adjpin * 3));
        }

        pub fn config_uart0(&self) {
            // Disable the pullup/down on the UART pins
            self.config_pull_up_down(14, GpioPullUpDown::Off);
            self.config_pull_up_down(15, GpioPullUpDown::Off);
//          self.config_function(14, GpioFunctionSelect::Alt0);
//          self.config_function(15, GpioFunctionSelect::Alt0);
        }
    }
}

/****************************************************************************
 * UART - Universal Asynchronous Recever / Transmitter
 ***************************************************************************/

//pub mod uart;

pub mod uart {
    use core::str::StrExt;
    use super::*;

    // The offsets for reach register for the UART.
    const DR:       usize = 0x00;
//  const RSRECR:   usize = 0x04;
    const FR:       usize = 0x18;
//  const ILPR:     usize = 0x20;
    const IBRD:     usize = 0x24;
    const FBRD:     usize = 0x28;
    const LCRH:     usize = 0x2C;
    const CR:       usize = 0x30;
//  const IFLS:     usize = 0x34;
    const IMSC:     usize = 0x38;
//  const RIS:      usize = 0x3C;
//  const MIS:      usize = 0x40;
    const ICR:      usize = 0x44;
//  const DMACR:    usize = 0x48;
//  const ITCR:     usize = 0x80;
//  const ITIP:     usize = 0x84;
//  const ITOP:     usize = 0x88;
//  const TDR:      usize = 0x8C;

    pub struct Uart {
        pub base_addr: usize,
    }

    impl Uart {

        pub fn disable(&self) {
            // Disable UART.
            mmio::write(self.base_addr + CR as usize, 0x00000000);
        }

        pub fn init(&self) {
            // Clear pending interrupts.
            mmio::write(self.base_addr + ICR as usize, 0x07FF);
 
            // Set integer & fractional part of baud rate.
            // Divider = UART_CLOCK/(16 * Baud)
            // Fraction part register = (Fractional part * 64) + 0.5
            // UART_CLOCK = 3000000; Baud = 115200.
 
            // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
            // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
            mmio::write(self.base_addr + IBRD as usize, 1);
            mmio::write(self.base_addr + FBRD as usize, 40);
 
            // Enable FIFO & 8 bit data transmission (1 stop bit, no parity).
            mmio::write(self.base_addr + LCRH as usize,
                       (1 << 4) | (1 << 5) | (1 << 6));
 
            // Mask all interrupts.
            mmio::write(self.base_addr + IMSC as usize,
                       (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                       (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
 
            // Enable UART, receive & transfer part of UART.
            mmio::write(self.base_addr + CR as usize,
                       (1 << 0) | (1 << 8) | (1 << 9));
        }

        pub fn putc(&self, byte: u8) {
            // Wait for UART to become ready to transmit.
            while mmio::read(self.base_addr + FR as usize) &
                                                                 (1 << 5) != 0 {
            }
            mmio::write(self.base_addr + DR as usize,
                       byte as usize);
        }
 
        pub fn getc(&self) -> u8 {
            // Wait for UART to have recieved something.
            while mmio::read(self.base_addr + FR as usize) & (1 << 4) != 0 {
            }
            return mmio::read(self.base_addr + DR as usize) as u8;
        }

        pub fn puts(&self, str: &str)
        {
            for b in str.bytes() {
                self.putc(b);
            }
        }
    }
}
