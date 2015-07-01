#![feature(no_std)]
#![feature(core)]
#![feature(asm)]
#![no_std]
#![crate_type="staticlib"]
#![crate_name = "rpi"]
#![feature(core_str_ext)]
#![feature(core_intrinsics)]

extern crate core;
use core::str::StrExt;

/****************************************************************************
 * Define the memory map
 ***************************************************************************/

pub enum MemMap {
    GpioBase = 0x20200000,
    Uart0Base = 0x20201000,
}
 

/****************************************************************************
 * Some utility functions to make the compiler do what needs to be done.
 ***************************************************************************/

// Loop <delay> times, using "volatile" to prevent the optimizer from
// optimizing it to nothing.
fn delay(count: usize) -> () {
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

mod mmio {
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

#[allow(dead_code)]
enum GpioMemMap {
    // Function selections
    GPFSEL0 = 0x00,
    GPFSEL1 = 0x04,
    GPFSEL2 = 0x08,
    GPFSEL3 = 0x0C,
    GPFSEL4 = 0x10,
    GPFSEL5 = 0x14,

    // Pin control (set/clear/level)
    GPSET0 = 0x1C,
    GPSET1 = 0x20,

    GPCLR0 = 0x28,
    GPCLR1 = 0x2C,

    GPLEV0 = 0x34,
    GPLEV1 = 0x38,

    // Event detect status
    GPEDS0 = 0x40,
    GPEDS1 = 0x44,

    // Rising edge detect enables
    GPREN0 = 0x4C,
    GPREN1 = 0x50,

    // Falling edge detect enables
    GPFEN0 = 0x58,
    GPFEN1 = 0x5C,

    // High detect enables
    GPHEN0 = 0x64,
    GPHEN1 = 0x68,

    // Low detect enables
    GPLEN0 = 0x70,
    GPLEN1 = 0x74,

    // Async rising edge detect enables
    GPAREN0 = 0x7C,
    GPAREN1 = 0x80,

    // Async falling edge detect enables
    GPAFEN0 = 0x88,
    GPAFEN1 = 0x8C,

    // Pin pull-up/down enables
    GPPUD = 0x94,
 
    // Pin pull-up/down enable clocks
    GPPUDCLK0 = 0x98,
    GPPUDCLK1 = 0x9C,
}

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

//  pub fn config_uart0(&self) {
//      mmio::write(self.base_addr + GpioMemMap::GPPUD as usize, 0x00000000);
//      delay(150);
//
//      mmio::write(self.base_addr + GpioMemMap::GPPUDCLK0 as usize,
//                 (1 << 14) | (1 << 15));
//      delay(150);
//
//      // Write a zero to GPPUDCLK0 to make it take effect.
//      mmio::write(self.base_addr + GpioMemMap::GPPUDCLK0 as usize, 0x00000000);
//  }

    pub fn config_pull_up_down(&self, pin: usize, config: GpioPullUpDown) {
        // Ref: BCM2834 ARM Peripherals manual, page 95

        // Configure the control: off/pulldown/pullup
        mmio::write(self.base_addr + GpioMemMap::GPPUD as usize,
                    config as usize);
        // Required setup time
        delay(150);
        // Clock the appropriate pin's clock
        match pin {
            0 ... 31 =>
                mmio::write(self.base_addr + GpioMemMap::GPPUDCLK0 as usize,
                            1 << (pin as usize)),
            32 ... 53 =>
                mmio::write(self.base_addr + GpioMemMap::GPPUDCLK1 as usize,
                            1 << (pin as usize - 32)),
            _ => {}
        }
        // Required setup time
        delay(150);
        // Write to the clock register
        match pin {
            0 ... 31 =>
                mmio::write(self.base_addr + GpioMemMap::GPPUDCLK0 as usize, 0),
            32 ... 53 =>
                mmio::write(self.base_addr + GpioMemMap::GPPUDCLK1 as usize, 0),
            _ => {}
        }
    }

    pub fn config_function(&self, pin: usize, config: GpioFunctionSelect) {
        let (adjpin, reg) = match pin {
            0 ... 9 =>
                (pin,      self.base_addr + GpioMemMap::GPFSEL0 as usize),
            10 ... 19 =>
                (pin - 10, self.base_addr + GpioMemMap::GPFSEL1 as usize),
            20 ... 29 =>
                (pin - 20, self.base_addr + GpioMemMap::GPFSEL2 as usize),
            30 ... 39 =>
                (pin - 30, self.base_addr + GpioMemMap::GPFSEL3 as usize),
            40 ... 49 =>
                (pin - 40, self.base_addr + GpioMemMap::GPFSEL4 as usize),
            50 ... 53 =>
                (pin - 50, self.base_addr + GpioMemMap::GPFSEL5 as usize),
            _ => (0,0)
        };
        mmio::write(reg, (mmio::read(reg) & 0b111 << (adjpin * 3)) |
                         (config as usize) << (adjpin * 3));
    }

    pub fn config_uart0(&self) {
        // Disable the pullup/down on the UART pins
        self.config_pull_up_down(14, GpioPullUpDown::Off);
        self.config_pull_up_down(15, GpioPullUpDown::Off);
//      self.config_function(14, GpioFunctionSelect::Alt0);
//      self.config_function(15, GpioFunctionSelect::Alt0);
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

pub struct Uart {
    pub base_addr: usize,
}

impl Uart {

    pub fn disable(&self) {
        // Disable UART.
        mmio::write(self.base_addr + UartMemMap::CR as usize, 0x00000000);
    }

    pub fn init(&self) {
        // Clear pending interrupts.
        mmio::write(self.base_addr + UartMemMap::ICR as usize, 0x07FF);
 
        // Set integer & fractional part of baud rate.
        // Divider = UART_CLOCK/(16 * Baud)
        // Fraction part register = (Fractional part * 64) + 0.5
        // UART_CLOCK = 3000000; Baud = 115200.
 
        // Divider = 3000000 / (16 * 115200) = 1.627 = ~1.
        // Fractional part register = (.627 * 64) + 0.5 = 40.6 = ~40.
        mmio::write(self.base_addr + UartMemMap::IBRD as usize, 1);
        mmio::write(self.base_addr + UartMemMap::FBRD as usize, 40);
 
        // Enable FIFO & 8 bit data transmission (1 stop bit, no parity).
        mmio::write(self.base_addr + UartMemMap::LCRH as usize,
                   (1 << 4) | (1 << 5) | (1 << 6));
 
        // Mask all interrupts.
        mmio::write(self.base_addr + UartMemMap::IMSC as usize,
                   (1 << 1) | (1 << 4) | (1 << 5) | (1 << 6) |
                   (1 << 7) | (1 << 8) | (1 << 9) | (1 << 10));
 
        // Enable UART, receive & transfer part of UART.
        mmio::write(self.base_addr + UartMemMap::CR as usize,
                   (1 << 0) | (1 << 8) | (1 << 9));
    }

    pub fn putc(&self, byte: u8) {
        // Wait for UART to become ready to transmit.
        while mmio::read(self.base_addr + UartMemMap::FR as usize) &
                                                             (1 << 5) != 0 {
        }
        mmio::write(self.base_addr + UartMemMap::DR as usize,
                   byte as usize);
    }
 
    pub fn getc(&self) -> u8 {
        // Wait for UART to have recieved something.
        while mmio::read(self.base_addr + UartMemMap::FR as usize) & (1 << 4) != 0 {
        }
        return mmio::read(self.base_addr + UartMemMap::DR as usize) as u8;
    }

    pub fn puts(&self, str: &str)
    {
        for b in str.bytes() {
            self.putc(b);
        }
    }
}
