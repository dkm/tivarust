#![feature(lang_items,asm)]
#![no_std]
#![no_main]

use core::ptr;

#[macro_use]
mod lowlevel {
    
    macro_rules! bitband {
        ($bitband_offset:expr,$bit_offset:expr) => {
            ((($bitband_offset) & 0xF0000000) |
             0x02000000) +
                (($bitband_offset & 0x000FFFFF) * 32) + ($bit_offset as u32) * 4;
        };
    }

    macro_rules! write_bitband {
        ($bitband_offset:expr, $bit_offset:expr, $bitval:expr) => {
            ptr::write_volatile(bitband!($bitband_offset,$bit_offset) as *mut u32,
                                $bitval);
        }
    }

    macro_rules! read_bitband {
        ($bitband_offset:expr, $bit_offset:expr) => {
            ptr::read_volatile(bitband!($bitband_offset,$bit_offset) as *mut u32);
        } 
    }

    macro_rules! hwreg {
        ($x:expr, $t:ty) => {
            ptr::read_volatile(($x as *mut $t));
        };
        ($x:expr, $t:ty, $v:expr) => {
            *($x as *mut $t) = $v;
        }
    }
}

mod driverlib;
mod tiva_uart;
mod tiva_gpio;

// enum TivaSysCtlSysDiv {
//     SysDiv1 = 0x0,
//     SysDiv2 = 0x1,
//     SysDiv3 = 0x2,
//     SysDiv4 = 0x3,
//     SysDiv5 = 0x4,
//     SysDiv6 = 0x5,
//     SysDiv7 = 0x6,
//     SysDiv8 = 0x7,
//     SysDiv9 = 0x8,
//     SysDiv10 = 0x9,
//     SysDiv11 = 0xA,
//     SysDiv12 = 0xB,
//     SysDiv13 = 0xC,
//     SysDiv14 = 0xD,
//     SysDiv15 = 0xE,
//     SysDiv16 = 0xF,
// }

// const SYSCTL_RCC_BYPASS: u32 =  0x0000_0800;  // PLL Bypass
// const SYSCTL_RCC2_BYPASS2: u32 =  0x0000_0800;  // PLL Bypass 2

// const SYSCTL_RCC_USESYSDIV: u32 = 0x0040_0000;  // Enable System Clock Divider
// const SYSCTL_RCC_MOSCDIS:u32 = 0x00000001;  // Main Oscillator Disable
// const SYSCTL_MAIN_OSC_DIS:u32 = 0x00000001;  // Disable main oscillator
// const SYSCTL_RIS_MOSCPUPRIS: u32 = 0x00000100;  // MOSC Power Up Raw Interrupt
// const SYSCTL_MISC: u32 = 0x400FE058;  // Masked Interrupt Status
// const SYSCTL_MISC_MOSCPUPMIS: u32 = 0x00000100;  // MOSC Power Up Masked Interrupt
// const SYSCTL_RIS: u32 = 0x400FE050;  // Raw Interrupt Status


#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr0() -> ()
{
    loop {}
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr1() -> ()
{
    loop {}
}

    
extern {
    static mut _data_start : u32;
    static mut _data_end : u32;
    static mut _data_load : u32;
    static mut _bss_start : u32;
    static mut _bss_end : u32;
}

static mut SOME_STATIC_DATA: u32 = 0xdeadbeef;

pub unsafe fn relocate(){
    let mut load_addr : *mut u32 = &mut _data_load;
    let mut dest_addr : *mut u32 = &mut _data_start;
    let dest_addr_end : *mut u32 = &mut _data_end;

    while dest_addr < dest_addr_end {
        *dest_addr = *load_addr;
        dest_addr = dest_addr.offset(1);
        load_addr = load_addr.offset(1);
    }

    dest_addr = &mut _bss_start;
    while dest_addr < &mut _bss_end as *mut u32 {
        *dest_addr = 0;
        dest_addr = dest_addr.offset(1);
    }
}


#[no_mangle]
pub extern fn main() {
   
    unsafe {

        driverlib::sysctl::cpu_clock_init(80);

        SOME_STATIC_DATA = 0xbeefdead;

        relocate();

        let uart_conf = tiva_uart::uart_init(0, 115200);

        for i in 0..10000 {
            tiva_uart::uart_write(&uart_conf, 'a');
            tiva_uart::uart_write(&uart_conf, '\n');
        }
        let gpio_f = tiva_gpio::TivaGpio {
            sysctl_idx:5,
            base_addr: driverlib::memmap::GPIO_PORTF_AHB_BASE,
            use_hpb:true
        };

        gpio_f.init();

        gpio_f.init_pin(1, tiva_gpio::TivaGpioMode::GpioOut);
        gpio_f.init_pin(2, tiva_gpio::TivaGpioMode::GpioOut);
        gpio_f.init_pin(3, tiva_gpio::TivaGpioMode::GpioOut);

        // gpio_f.write_pin(1,1);
        // gpio_f.write_pin(2,1);
        // gpio_f.write_pin(3,1);
        gpio_f.write_pins(0xC, 0xC);
        
        loop{};

    }
}

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

