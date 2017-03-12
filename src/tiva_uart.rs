use driverlib::sysctl;
use driverlib::pinmap;
use driverlib::memmap;
use driverlib::gpio;

use core::ptr;

struct TivaUartConf {
    sysctl_periph_uart : u32,
    sysctl_periph_gpio : u32,
    gpio_base_addr_ahb : u32,
    gpio_rx_pin : u32,
    gpio_tx_pin: u32,
    uart_base: u32,
}
    
const Uart0 : TivaUartConf = TivaUartConf {
    sysctl_periph_uart : sysctl::SYSCTL_PERIPH_UART0,
    sysctl_periph_gpio : sysctl::SYSCTL_PERIPH_GPIOA,
    gpio_base_addr_ahb : memmap::GPIO_PORTA_AHB_BASE,
    gpio_rx_pin : pinmap::GPIO_PA0_U0RX,
    gpio_tx_pin : pinmap::GPIO_PA1_U0TX,
    uart_base : memmap::UART0_BASE
};


macro_rules! config_gpio_pins {
    ($ahb_addr:expr, $pin :expr) => {
        let pmc_index_shift : u32 = ($pin >> 8) & 0xff;
        let pmc_pin_mask : u32 = $pin & 0xf;

        hwreg!($ahb_addr + gpio::GPIO_O_PCTL, u32,
               ((hwreg!($ahb_addr + gpio::GPIO_O_PCTL, u32) &
                 !(0xf << pmc_index_shift)) |
                (pmc_pin_mask << pmc_index_shift)));
    }
}

pub unsafe fn uart_init(uart: u8, baudrate: u32) {
    let uart_conf : TivaUartConf = match (uart) {
        0 => Uart0,
        _ => Uart0
    };

    sysctl::sys_ctl_peripheral_enable(uart_conf.sysctl_periph_uart);
    sysctl::sys_ctl_peripheral_enable(uart_conf.sysctl_periph_gpio);
    
    config_gpio_pins!(uart_conf.gpio_base_addr_ahb, uart_conf.gpio_rx_pin);
    config_gpio_pins!(uart_conf.gpio_base_addr_ahb, uart_conf.gpio_tx_pin);

    

}
