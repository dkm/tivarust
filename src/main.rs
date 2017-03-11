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

mod tiva_sysctl;

const SYSCTL_BASE: u32 = 0x400F_E000;
const SYSCTL_RCGCGPIO_BASE: u32 = SYSCTL_BASE + 0x608;
const SYSCTL_GPIOHBCTL_BASE: u32 = SYSCTL_BASE + 0x06c;

const SYSCTL_RCC: u32 = SYSCTL_BASE + 0x60;
const SYSCTL_RCC2: u32 = SYSCTL_BASE + 0x70;

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


enum TivaSysCtlSysDiv {
    SysDiv1 = 0x0,
    SysDiv2 = 0x1,
    SysDiv3 = 0x2,
    SysDiv4 = 0x3,
    SysDiv5 = 0x4,
    SysDiv6 = 0x5,
    SysDiv7 = 0x6,
    SysDiv8 = 0x7,
    SysDiv9 = 0x8,
    SysDiv10 = 0x9,
    SysDiv11 = 0xA,
    SysDiv12 = 0xB,
    SysDiv13 = 0xC,
    SysDiv14 = 0xD,
    SysDiv15 = 0xE,
    SysDiv16 = 0xF,
}

// const SYSCTL_RCC_BYPASS: u32 =  0x0000_0800;  // PLL Bypass
// const SYSCTL_RCC2_BYPASS2: u32 =  0x0000_0800;  // PLL Bypass 2

// const SYSCTL_RCC_USESYSDIV: u32 = 0x0040_0000;  // Enable System Clock Divider
// const SYSCTL_RCC_MOSCDIS:u32 = 0x00000001;  // Main Oscillator Disable
// const SYSCTL_MAIN_OSC_DIS:u32 = 0x00000001;  // Disable main oscillator
// const SYSCTL_RIS_MOSCPUPRIS: u32 = 0x00000100;  // MOSC Power Up Raw Interrupt
// const SYSCTL_MISC: u32 = 0x400FE058;  // Masked Interrupt Status
// const SYSCTL_MISC_MOSCPUPMIS: u32 = 0x00000100;  // MOSC Power Up Masked Interrupt
// const SYSCTL_RIS: u32 = 0x400FE050;  // Raw Interrupt Status



pub unsafe fn sys_ctl_peripheral_enable(periph_id : u32) {
        write_bitband!((SYSCTL_RCGBASE + ((periph_id & 0xff00) >> 8)),(periph_id & 0xff),1);
}

#[no_mangle]
pub unsafe fn __aeabi_unwind_cpp_pr0() -> ()
{
    loop {}
}

struct TivaGpio {
    sysctl_idx : u32,
    base_addr : u32,
    use_hpb : bool,
}

enum TivaGpioMode {
    GpioIn,
    GpioInPd,
    GpioInPu,
    GpioOut,
    GpioOD,
    GpioODPu,
}

impl TivaGpio {
    unsafe fn init(&self) {
        // bit-band access
        write_bitband!(SYSCTL_RCGCGPIO_BASE, self.sysctl_idx, 1);

        // Use HPB instead of APB
        write_bitband!(SYSCTL_GPIOHBCTL_BASE, self.sysctl_idx, 1);
    }

    unsafe fn init_pin(&self, pin:u8, mode : TivaGpioMode){
        match mode {
            GpioOut => {
                write_bitband!(GPIO_PORTF_BASE + GPIO_DEN_R_OFF,  pin, 1);
                write_bitband!(GPIO_PORTF_BASE + GPIO_DIR_R_OFF,  pin, 1);
            },
            _ => {
            },
        }
    }

    unsafe fn write_pin(&self, pin:u8, val:u8){
        hwreg!(self.base_addr + GPIO_DATA_R_OFF | (1<<(pin + 2)),
               u32,
               if val != 0 {1<<pin} else {0}
        );
    }

    unsafe fn write_pins(&self, pin_mask:u8, val:u8){
        hwreg!(self.base_addr + GPIO_DATA_R_OFF | ((pin_mask as u32)<<2),
               u32,
               val as u32
        );
    }

    unsafe fn read_pin(&self, pin:u8) -> u8 {
        return hwreg!(self.base_addr + GPIO_DATA_R_OFF | (1<<(pin + 2)),
               u32
        ) as u8;
    }

    unsafe fn read_pins(&self, pin_mask:u8)-> u8 {
        return hwreg!(self.base_addr + GPIO_DATA_R_OFF | ((pin_mask as u32)<<2),
               u32
        ) as u8;
    }
}


extern {
    static mut _data_start : u32;
    static mut _data_end : u32;
    static mut _data_load : u32;
    static mut _bss_start : u32;
    static mut _bss_end : u32;
}


static mut SOME_STATIC_DATA: u32 = 0xdeadbeef;
static SOME_STATIC_STRING: &'static str = "A static string";

pub unsafe fn relocate(){
    let mut load_addr : *mut u32 = &mut _data_load;
    let mut dest_addr : *mut u32 = &mut _data_start;
    let mut dest_addr_end : *mut u32 = &mut _data_end;

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

    // const RCGCGPIO: *mut u32 = (0x400FE000 + 0x608) as *mut u32;

    // const RCGCGPIO_PORTA: u32 = 0x1;
    // const RCGCGPIO_PORTB: u32 = 0x1<<1;
    // const RCGCGPIO_PORTC: u32 = 0x1<<2;
    // const RCGCGPIO_PORTD: u32 = 0x1<<3;
    // const RCGCGPIO_PORTE: u32 = 0x1<<4;
    // const RCGCGPIO_PORTF: u32 = 0x1<<5;


    // const GPIO_PORT_A: *mut u32 = (0x40058000) as *mut u32;
    // const GPIO_PORT_B: *mut u32 = (0x40059000) as *mut u32;
    // const GPIO_PORT_C: *mut u32 = (0x4005A000) as *mut u32;
    // const GPIO_PORT_D: *mut u32 = (0x4005B000) as *mut u32;
    // const GPIO_PORT_E: *mut u32 = (0x4005C000) as *mut u32;

    // const GPIO_PORT_F: u32 = 0x4005D000;
    // const GPIO_PORT_F_GPIODATA: *mut u32 = (GPIO_PORT_F) as *mut u32;    
    // const GPIO_PORT_F_GPIODIR: *mut u32 = (GPIO_PORT_F + 0x400) as *mut u32;    
    // const GPIO_PORT_F_GPIOIS: *mut u32 = (GPIO_PORT_F + 0x404) as *mut u32;    
    // const GPIO_PORT_F_GPIODEN: *mut u32 = (GPIO_PORT_F + 0x51C) as *mut u32;    
    
    unsafe {

        tiva_sysctl::cpu_clock_init(80);

        SOME_STATIC_DATA = 0xbeefdead;

        relocate();
        
        let gpio_f = TivaGpio {
            sysctl_idx:5,
            base_addr:GPIO_PORTF_BASE,
            use_hpb:true
        };

        gpio_f.init();

        gpio_f.init_pin(1, TivaGpioMode::GpioOut);
        gpio_f.init_pin(2, TivaGpioMode::GpioOut);
        gpio_f.init_pin(3, TivaGpioMode::GpioOut);

        // gpio_f.write_pin(1,1);
        // gpio_f.write_pin(2,1);
        // gpio_f.write_pin(3,1);
        gpio_f.write_pins(0xC,0xC);
        
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

