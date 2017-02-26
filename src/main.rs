#![feature(lang_items,asm)]
#![no_std]
#![no_main]

use core::ptr;

const SRAM_ALIAS_REGION_BASE : u32   = 0x22000000;
const SRAM_BITBAND_REGION_BASE : u32 = 0x20000000;

const PERIPH_ALIAS_REGION_BASE : u32   = 0x42000000;
const PERIPH_BITBAND_REGION_BASE : u32 = 0x40000000;


macro_rules! bitband {
    ($bitband_offset:expr,$bit_offset:expr, sram) => {
        SRAM_ALIAS_REGION_BASE + (($bitband_offset - SRAM_BITBAND_REGION_BASE) * 32) + $bit_offset * 4;
    };
    ($bitband_offset:expr,$bit_offset:expr, periph) => {
        PERIPH_ALIAS_REGION_BASE + (($bitband_offset - PERIPH_BITBAND_REGION_BASE) * 32) + $bit_offset * 4;
    }

}
macro_rules! write_bitband {
    ($bitband_offset:expr, $bit_offset:expr, $bitval:expr, $loc:tt) => {
        *(bitband!($bitband_offset,$bit_offset, $loc) as *mut u32) = $bitval;
    }
}

macro_rules! read_bitband {
    ($bitband_offset:expr, $bit_offset:expr, $loc:tt) => {
        *(bitband!($bitband_offset,$bit_offset, $loc) as *mut u32);
    }
}

// macro_rules! bitband {
//     ($x:expr, $b:expr) => {{
//         let _x = $x;
//         ((_x & 0xF0000000) | 0x02000000 |           
//         ((_x & 0x000FFFFF) << 5) | (($b) << 2))
//     }}
// }

macro_rules! hwreg {
    ($x:expr, $t:ty) => {
            ptr::read_volatile(($x as *mut $t));
    };
    ($x:expr, $t:ty, $v:expr) => {
            *($x as *mut $t) = $v;
    }
}

const SYSCTL_BASE: u32 = 0x400fe000;
const SYSCTL_RCGCGPIO_BASE: u32 = SYSCTL_BASE + 0x608;
const SYSCTL_GPIOHBCTL_BASE: u32 = SYSCTL_BASE + 0x06c;

const SYSCTL_RCGBASE: u32 = 0x400fe600;
const SYSCTL_PERIPH_GPIOF: u32 = 0xf0000805;
const GPIO_LOCK_KEY: u32 = 0x4C4F434B;
const GPIO_LOCK_R_OFF: u32 = 0x520;
const GPIO_CR_R_OFF: u32 = 0x524;

const GPIO_DEN_R_OFF: u32 = 0x51c;
const GPIO_DIR_R_OFF: u32 = 0x400;
const GPIO_DATA_R_OFF: u32 = 0x000;

const GPIO_O_DR2R_OFF: u32 = 0x500;
const GPIO_O_DR4R_OFF: u32 = 0x504;
const GPIO_O_DR8R_OFF: u32 = 0x508;
const GPIO_O_SLR_OFF: u32 = 0x518;

 
// const GPIO_PORTF_BASE: u32 = 0x40025000;  // GPIO Port F APB (old)

const GPIO_PORTF_BASE: u32 = 0x4005D000;  // GPIO Port F AHB


pub unsafe fn sys_ctl_peripheral_enable(periph_id : u32) {
        write_bitband!((SYSCTL_RCGBASE + ((periph_id & 0xff00) >> 8)),(periph_id & 0xff),1, periph);
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr0() -> ()
{
    loop {}
}

#[no_mangle]
pub extern fn main() {

    const RCGCGPIO: *mut u32 = (0x400FE000 + 0x608) as *mut u32;

    const RCGCGPIO_PORTA: u32 = 0x1;
    const RCGCGPIO_PORTB: u32 = 0x1<<1;
    const RCGCGPIO_PORTC: u32 = 0x1<<2;
    const RCGCGPIO_PORTD: u32 = 0x1<<3;
    const RCGCGPIO_PORTE: u32 = 0x1<<4;
    const RCGCGPIO_PORTF: u32 = 0x1<<5;


    const GPIO_PORT_A: *mut u32 = (0x40058000) as *mut u32;
    const GPIO_PORT_B: *mut u32 = (0x40059000) as *mut u32;
    const GPIO_PORT_C: *mut u32 = (0x4005A000) as *mut u32;
    const GPIO_PORT_D: *mut u32 = (0x4005B000) as *mut u32;
    const GPIO_PORT_E: *mut u32 = (0x4005C000) as *mut u32;

    const GPIO_PORT_F: u32 = 0x4005D000;
    const GPIO_PORT_F_GPIODATA: *mut u32 = (GPIO_PORT_F) as *mut u32;    
    const GPIO_PORT_F_GPIODIR: *mut u32 = (GPIO_PORT_F + 0x400) as *mut u32;    
    const GPIO_PORT_F_GPIOIS: *mut u32 = (GPIO_PORT_F + 0x404) as *mut u32;    
    const GPIO_PORT_F_GPIODEN: *mut u32 = (GPIO_PORT_F + 0x51C) as *mut u32;    
    
    unsafe {
        
        // bit-band access
        write_bitband!(SYSCTL_RCGCGPIO_BASE, 5, 1, periph);

        // Use HPB instead of APB
        write_bitband!(SYSCTL_GPIOHBCTL_BASE, 5, 1, periph);

        // // unlock !
        // hwreg!(GPIO_PORTF_BASE + GPIO_LOCK_R_OFF, u32, GPIO_LOCK_KEY);

        // // allow for changing value of pin 3 (maybe useless)
        // write_bitband!(GPIO_PORTF_BASE + GPIO_CR_R_OFF, 3, 1, periph);

        // // lock back !
        //  hwreg!(GPIO_PORTF_BASE + GPIO_LOCK_R_OFF, u32);

        // Digital enable pin

        write_bitband!(GPIO_PORTF_BASE + GPIO_DEN_R_OFF,  3, 1, periph);
        write_bitband!(GPIO_PORTF_BASE + GPIO_DIR_R_OFF,  3, 1, periph);

        let mut i: u32 = 0;
        loop {
            if i == 0 {
                hwreg!(GPIO_PORTF_BASE + GPIO_DATA_R_OFF + 0x3FC, u32, 0x8);
                // write_bitband!(GPIO_PORTF_BASE + GPIO_DATA_R_OFF, 3, 1, periph);                
            }
            i = i + 1;
            if i == 50 {
                hwreg!(GPIO_PORTF_BASE + GPIO_DATA_R_OFF + 0x3FC, u32, 0x0);
                // write_bitband!(GPIO_PORTF_BASE + GPIO_DATA_R_OFF, 3, 0, periph);
                i = 0;
            }

        }
        // Set the output drive strength.
        // write_bitband!(GPIO_PORTF_BASE + GPIO_O_DR2R_OFF, 3, 1, periph);
        // write_bitband!(GPIO_PORTF_BASE + GPIO_O_DR4R_OFF, 3, 0, periph);
        // write_bitband!(GPIO_PORTF_BASE + GPIO_O_DR8R_OFF, 3, 0, periph);
        // write_bitband!(GPIO_PORTF_BASE + GPIO_O_SLR_OFF,  3, 0, periph);
        
        // hwreg!(GPIO_PORTF_BASE + GPIO_LOCK_R_OFF, u32, 0);
        // HWREGBITW(SYSCTL_RCGCBASE + ((ui32Peripheral & 0xff00) >> 8),
        //           ui32Peripheral & 0xff) = 1;
        

        // *RCGCGPIO |= RCGCGPIO_PORTF;
        // *GPIO_PORT_F_GPIODIR |= 0x1<<3;
        // *GPIO_PORT_F_GPIODEN |= 0x1<<3;
        
        // loop {
        //     *GPIO_PORT_F_GPIODATA |= 0x1<<3;
        //     // *GPIO_PORT_F_GPIODATA &= !(0x1<<3);

        // }
        loop{};

    }
}

// #[lang = "eh_personality"]
// extern "C" fn eh_personality() {
// }

#[lang = "panic_fmt"]
#[no_mangle]
pub extern fn rust_begin_panic(_msg: core::fmt::Arguments,
                               _file: &'static str,
                               _line: u32) -> ! {
    loop {};
}


extern {
    fn _stack_top();
}

pub extern fn error_handler() {
    loop{}
}


#[link_section = ".vectors"]
// #[no_mangle]
pub static _VECTORS: [unsafe extern fn(); 13] = [
    _stack_top,
    main,
    error_handler, // NMI
    error_handler, // Hard fault
    error_handler, // Mem Management
    error_handler, // bus fault
    error_handler, // usage fault
    error_handler, // NONE
    error_handler, // NONE
    error_handler, // NONE
    error_handler, // NONE
    error_handler, // SVcall
    error_handler, // Debug monitor
];

