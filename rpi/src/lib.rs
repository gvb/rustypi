#![feature(no_std)]
#![feature(core)]
#![feature(asm)]
#![no_std]
#![crate_type="staticlib"]
#![crate_name = "rpi"]
#![feature(core_intrinsics)]
#![feature(core_str_ext)]

extern crate core;

pub struct PutMem {
    // Print the name string, (*address >> shift) & mask, term string
    pub name: &'static str,
    pub addr: usize,
    pub shift: usize,
    pub mask: usize,
    pub term: &'static str,
}

// Print a hex representation for the given type.
pub trait PutHex<T> {
    fn puthex(&self, data: T);
}

// Print a string representation for the given type.
pub trait PutS<T> {
    fn puts(&self, data: T);
}


/****************************************************************************
 * Define the memory map
 ***************************************************************************/

//pub mod memory_map;

pub mod memory_map {
    pub const GPIOBASE:  usize = 0x20200000;
    pub const UART0BASE: usize = 0x20201000;
    pub const TIMERBASE: usize = 0x2020B000;
}

/****************************************************************************
 * CPU utilities
 ***************************************************************************/

pub mod cpu {

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

//  // Data Memory Barrier on ARMv6
//  llvm!(extern {
//      ir "declare dmb @llvm.dmb(i32)";
//
//      fn dmb(option: i32) {
//          "call i64 @llvm.dbm(option)"
//      }
//  });

//  pub fn dmb() {
//      unsafe {
//          intrinsics::atomic_fence();
//      }
//  }


    // Calls out to ARM (not Thumb) dmb() function
#[link(name = "dmb")]
    extern {
        pub fn dmb();
    }

//#[link_name = "llvm.arm.dsb"]
//  extern {
//      pub fn dmb(a: i32) -> ();
//      // The `llvm.arm.dsb` intrinsic; known as `__builtin_arm_dsb` in GCC.
//  }

    // Data Memory Barrier on ARMv6
//  pub fn dmb() {
//      unsafe {
//          // ee071f9a 	mcr	15, 0, r1, cr7, cr10, {4}
//          asm!("push {r0, r1}
//               movs r1,#0
//               ldr r0, 2f
//               bx r0
//               .align 2
//               .arm
//               1: .word 0xee071f9a
//               pop {r0, r1}
//               bx lr
//               2: .word 1b
//               .thumb
//               "
//               ::: "cc" : "volatile");
//      }
//  }
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
    const GPSET0: usize = 0x1C;
    const GPSET1: usize = 0x20;

    const GPCLR0: usize = 0x28;
    const GPCLR1: usize = 0x2C;

    const GPLEV0: usize = 0x34;
    const GPLEV1: usize = 0x38;

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

            unsafe{ cpu::dmb(); } // Data Memory Barrier

            // Configure the control: off/pulldown/pullup
            mmio::write(self.base_addr + GPPUD,
                        config as usize);
            // Required setup time
            cpu::delay(150);
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
            cpu::delay(150);
            // Write to the clock register
            match pin {
                0 ... 31 =>
                    mmio::write(self.base_addr + GPPUDCLK0, 0),
                32 ... 53 =>
                    mmio::write(self.base_addr + GPPUDCLK1, 0),
                _ => {}
            }

            unsafe{ cpu::dmb(); } // Data Memory Barrier
        }

        pub fn config_function(&self, pin: usize, config: GpioFunctionSelect) {
            let newconfig: u32 = config as u32;

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
            unsafe{ cpu::dmb(); } // Data Memory Barrier

            // Quirks:
            // * Using u32 because bitwise ops are not available for usize.
            // * Using XOR as bitwise "not" because Rust doesn't have BitNot.
            let data: u32 = (mmio::read(reg) as u32 &
                             ((0b111 << (adjpin * 3)) ^ 0xFFFFFFFF)) |
                             (newconfig << (adjpin * 3));
            mmio::write(reg, data as usize);
            unsafe{ cpu::dmb(); } // Data Memory Barrier
        }

        pub fn config_uart0(&self) {
            // Disable the pullup/down on the UART pins
            self.config_pull_up_down(14, GpioPullUpDown::Off);
            self.config_pull_up_down(15, GpioPullUpDown::Off);
            // Configure the proper function
            self.config_function(14, GpioFunctionSelect::Alt0);
            self.config_function(15, GpioFunctionSelect::Alt0);
        }

        pub fn get(&self, pin: usize) -> bool {
            let mut ret: bool;

            unsafe{ cpu::dmb(); } // Data Memory Barrier

            ret = match pin {
                0 ... 31 =>
                    (mmio::read(self.base_addr + GPLEV0) &
                        (1 << pin)) != 0,
                32 ... 53 =>
                    (mmio::read(self.base_addr + GPLEV1) &
                        (1 << (pin - 32))) != 0,
                _ => false  // unreachable!()
            };

            unsafe{ cpu::dmb(); } // Data Memory Barrier
            ret
        }

        pub fn set(&self, pin: usize) {
            unsafe{ cpu::dmb(); } // Data Memory Barrier

            match pin {
                0 ... 31 =>
                    mmio::write(self.base_addr + GPSET0, 1 << pin),
                32 ... 53 =>
                    mmio::write(self.base_addr + GPSET1, 1 << (pin - 32)),
                _ => {}  // unreachable!()
            }

            unsafe{ cpu::dmb(); } // Data Memory Barrier
        }

        pub fn clear(&self, pin: usize) {
            unsafe{ cpu::dmb(); } // Data Memory Barrier

            match pin {
                0 ... 31 =>
                    mmio::write(self.base_addr + GPCLR0, 1 << pin),
                32 ... 53 =>
                    mmio::write(self.base_addr + GPCLR1, 1 << (pin - 32)),
                _ => {}
            }

            unsafe{ cpu::dmb(); } // Data Memory Barrier
        }

        pub fn set_to(&self, pin: usize, value: bool) {
            if value {
                self.set(pin);
            } else {
                self.clear(pin);
            }
        }

        pub fn dump_reg(&self, uart: &uart::Uart) {
            let mem_dump: [PutMem; 8] = [
                PutMem {
                    name: "GPFSEL0 = ",
                    addr: 0x20200000,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "GPFSEL1 = ",
                    addr: 0x20200004,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "GPFSEL2 = ",
                    addr: 0x20200008,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "GPFSEL3 = ",
                    addr: 0x2020000C,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "GPFSEL4 = ",
                    addr: 0x20200010,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "GPFSEL5 = ",
                    addr: 0x20200014,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "GPLEV0 = ",
                    addr: 0x20200034,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "GPLEV1 = ",
                    addr: 0x20200038,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
            ];
            for loc in &mem_dump {
                uart.puts(loc);
            }
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

    // The offsets for the UART registers.
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
            unsafe{ cpu::dmb(); } // Data Memory Barrier

            // Disable UART.
            mmio::write(self.base_addr + CR as usize, 0x00000000);

            unsafe{ cpu::dmb(); } // Data Memory Barrier
        }

        pub fn init(&self) {
            unsafe{ cpu::dmb(); } // Data Memory Barrier

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

            unsafe{ cpu::dmb(); } // Data Memory Barrier
        }

        pub fn putc(&self, byte: u8) {
            unsafe{ cpu::dmb(); } // Data Memory Barrier

            // Wait for UART to become ready to transmit.
            while mmio::read(self.base_addr + FR as usize) &
                                                                 (1 << 5) != 0 {
            }
            mmio::write(self.base_addr + DR as usize,
                       byte as usize);

            unsafe{ cpu::dmb(); } // Data Memory Barrier
        }

        pub fn getc(&self) -> u8 {
            let mut ret: u8;

            unsafe{ cpu::dmb(); } // Data Memory Barrier

            // Wait for UART to have recieved something.
            while mmio::read(self.base_addr + FR as usize) & (1 << 4) != 0 {
            }
            ret = mmio::read(self.base_addr + DR as usize) as u8;

            unsafe{ cpu::dmb(); } // Data Memory Barrier
            ret
        }
    }

    impl PutS<u8> for Uart {
        fn puts(&self, data: u8) {
            self.putc(data);
        }
    }

    impl PutS<&'static str> for Uart {
        fn puts(&self, data: &'static str)
        {
            for b in data.bytes() {
                self.putc(b);
            }
        }
    }

    impl<'a> PutS<&'a PutMem> for Uart {
        fn puts(&self, data: &'a PutMem) {
            self.puts(data.name);
            self.puthex((mmio::read(data.addr) as u32
                        >> data.shift) & data.mask as u32);
            self.puts(data.term);
        }
    }
    impl PutHex<u8> for Uart {
        fn puthex(&self, data: u8) {
            let udata = data >> 4 & 0x0F;
            let ldata = data      & 0x0F;
            self.putc(match udata {
                0 ... 9 =>
                    '0' as u8 + udata,
                10 ... 15 =>
                    'A' as u8 + udata - 10,
                _ => '?' as u8
            });
            self.putc(match ldata {
                0 ... 9 =>
                    '0' as u8 + ldata,
                10 ... 15 =>
                    'A' as u8 + ldata - 10,
                _ => '?' as u8
            });
        }
    }

    impl PutHex<u16> for Uart {
        fn puthex(&self, data: u16) {
            self.puthex(((data >> 8) & 0x00FF) as u8);
            self.puthex( (data       & 0x00FF) as u8);
        }
    }

    impl PutHex<u32> for Uart {
        fn puthex(&self, data: u32) {
            self.puthex(((data >> 16) & 0x0000FFFF) as u16);
            self.puthex( (data        & 0x0000FFFF) as u16);
        }
    }

    impl PutHex<i8> for Uart {
        fn puthex(&self, data: i8) {
            self.puthex(data as u8);
        }
    }

    impl PutHex<i16> for Uart {
        fn puthex(&self, data: i16) {
            self.puthex(((data >> 8) & 0x00FF) as u8);
            self.puthex( (data       & 0x00FF) as u8);
        }
    }

    impl PutHex<i32> for Uart {
        fn puthex(&self, data: i32) {
            self.puthex(((data >> 16) & 0x0000FFFF) as u16);
            self.puthex( (data        & 0x0000FFFF) as u16);
        }
    }
}

/****************************************************************************
 * Timer
 ***************************************************************************/

//pub mod timer;

pub mod timer {
    use super::*;

    pub struct Timer{
        pub base_addr: usize
    }

    // The offsets for timer registers
    const LOAD:       usize = 0x400;
    const VALUE:      usize = 0x404;  // Read only
    const CONTROL:    usize = 0x408;
    const IRQ:        usize = 0x40C;  // IRQ Clear/Ack (Write only)
    const RAW_IRQ:    usize = 0x410;  // RAW IRQ (Read only)
    const MASKED_IRQ: usize = 0x414;  // Masked IRQ (Read only)
    const RELOAD:     usize = 0x418;
    const PREDIV:     usize = 0x41C;  // Pre-divider (Not in a real 804)
    const COUNTER:    usize = 0x420;  // Free running counter (Not in real 804)

    impl Timer {
        pub fn dump_reg(&self, uart: &uart::Uart) {
            let mem_dump: [PutMem; 10] = [
                PutMem {
                    name: "LOAD = ",
                    addr: 0x2020B400,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "VALUE = ",
                    addr: 0x2020B404,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "CONTROL = ",
                    addr: 0x2020B408,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "IRQ = ",
                    addr: 0x2020B40C,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "RAW_IRQ = ",
                    addr: 0x2020B410,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "MASKED_IRQ = ",
                    addr: 0x2020B414,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "RELOAD = ",
                    addr: 0x2020B418,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "PREDIV = ",
                    addr: 0x2020B41C,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
                PutMem {
                    name: "Free-running counter = ",
                    addr: 0x2020B424,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: " ",
                },
                PutMem {
                    name: "",
                    addr: 0x2020B420,
                    shift: 0,
                    mask: 0xFFFFFFFF,
                    term: "\r\n",
                },
            ];
            for loc in &mem_dump {
                uart.puts(loc);
            }
        }
    }
}
