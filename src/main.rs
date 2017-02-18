#![feature(lang_items,asm)]
#![no_std]
#![no_main]

#[no_mangle]
pub extern fn main() {
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
