#![feature(lang_items,asm)]
#![no_std]
#![no_main]

#[no_mangle]
pub extern fn main() {
    loop{}
}

#[lang = "panic_fmt"]
pub extern fn rust_begin_panic(_msg: core::fmt::Arguments,
                               _file: &'static str,
                               _line: u32) -> ! {
    loop {};
}


