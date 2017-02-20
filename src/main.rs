#![feature(lang_items,asm)]
#![no_std]
#![no_main]



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
        *RCGCGPIO |= RCGCGPIO_PORTF;
        *GPIO_PORT_F_GPIODIR |= 0x1<<3;
        *GPIO_PORT_F_GPIODEN |= 0x1<<3;
        
        loop {
            *GPIO_PORT_F_GPIODATA |= 0x1<<3;
            // *GPIO_PORT_F_GPIODATA &= !(0x1<<3);

        }


    }
    
    loop{}
}

// #[lang = "eh_personality"]
// extern "C" fn eh_personality() {
// }

#[lang = "panic_fmt"]
pub extern fn rust_begin_panic(_msg: core::fmt::Arguments,
                               _file: &'static str,
                               _line: u32) -> ! {
    loop {};
}


// extern {
//     fn _stack_top();
// }

#[link_section = ".vectors"]
// #[no_mangle]
pub static _VECTORS: [unsafe extern fn(); 2] = [
    // _stack_top,
    main,
    main,
];

