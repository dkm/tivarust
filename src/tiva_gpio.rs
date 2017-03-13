use driverlib::sysctl;
use driverlib::gpio;
use driverlib::memmap;
use core::ptr;

pub struct TivaGpio {
    pub sysctl_idx : u32,
    pub base_addr : u32,
    pub use_hpb : bool,
}

pub enum TivaGpioMode {
    GpioIn,
    GpioInPd,
    GpioInPu,
    GpioOut,
    GpioOD,
    GpioODPu,
    GpioHw,
}

impl TivaGpio {
    pub unsafe fn init(&self) {
        // bit-band access
        write_bitband!(sysctl::SYSCTL_RCGCGPIO, self.sysctl_idx, 1);

        // Use HPB instead of APB
        write_bitband!(sysctl::SYSCTL_GPIOHBCTL, self.sysctl_idx, 1);
    }

    pub unsafe fn init_pin(&self, pin:u8, mode : TivaGpioMode){
        match mode {
            GpioOut => {
                write_bitband!(self.base_addr + gpio::GPIO_O_DEN, pin, 1);
                write_bitband!(self.base_addr + gpio::GPIO_O_DIR, pin, 1);
            },
            GpioHw => {
                write_bitband!(self.base_addr + gpio::GPIO_O_DIR, pin, 0);
                write_bitband!(self.base_addr + gpio::GPIO_O_AFSEL, pin, 1);
            },
            _ => {
            },
        }
    }

    pub unsafe fn write_pin(&self, pin:u8, val:u8){
        hwreg!(self.base_addr + gpio::GPIO_O_DATA | (1<<(pin + 2)),
               u32,
               if val != 0 {1<<pin} else {0}
        );
    }

    pub unsafe fn write_pins(&self, pin_mask:u8, val:u8){
        hwreg!(self.base_addr + gpio::GPIO_O_DATA | ((pin_mask as u32)<<2),
               u32,
               val as u32
        );
    }

    pub unsafe fn read_pin(&self, pin:u8) -> u8 {
        return hwreg!(self.base_addr + gpio::GPIO_O_DATA | (1<<(pin + 2)),
               u32
        ) as u8;
    }

    pub unsafe fn read_pins(&self, pin_mask:u8)-> u8 {
        return hwreg!(self.base_addr + gpio::GPIO_O_DATA | ((pin_mask as u32)<<2),
               u32
        ) as u8;
    }
}
