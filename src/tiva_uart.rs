use driverlib::sysctl;
use driverlib::pinmap;
use driverlib::memmap;
use driverlib::gpio;
use driverlib::uart;

use tiva_gpio;

use core::ptr;

pub struct TivaUartConf {
    sysctl_periph_uart : u32,
    sysctl_periph_gpio : u32,
    gpio : tiva_gpio::TivaGpio,
    gpio_base_addr_ahb : u32,

    gpio_rx_pin_i : u8,
    gpio_tx_pin_i: u8,

    // gpio_rx_pin : u32,
    // gpio_tx_pin: u32,
    uart_base: u32,
}
    
const Uart0 : TivaUartConf = TivaUartConf {
    sysctl_periph_uart : sysctl::SYSCTL_PERIPH_UART0,
    sysctl_periph_gpio : sysctl::SYSCTL_PERIPH_GPIOA,
    gpio : tiva_gpio::TivaGpio{
        sysctl_idx:0,
        base_addr: memmap::GPIO_PORTA_AHB_BASE,
        use_hpb:true
    },
    gpio_rx_pin_i : 0,
    gpio_tx_pin_i : 1,

    gpio_base_addr_ahb : memmap::GPIO_PORTA_AHB_BASE,
    // gpio_rx_pin : pinmap::GPIO_PA0_U0RX,
    // gpio_tx_pin : pinmap::GPIO_PA1_U0TX,
    uart_base : memmap::UART0_BASE
};


macro_rules! config_gpio_pins {
    ($uart_conf:expr, $pin_idx :expr) => {
        let pmc_index_shift : u32 = $pin_idx as u32 * 4; // ($pin >> 8) & 0xff;
        let pmc_pin_mask : u32 = 1 << $pin_idx; //$pin & 0xf;

        hwreg!($uart_conf.gpio_base_addr_ahb + gpio::GPIO_O_PCTL, u32,
               ((hwreg!($uart_conf.gpio_base_addr_ahb + gpio::GPIO_O_PCTL, u32) &
                 !(0xf << pmc_index_shift)) |
                (pmc_pin_mask << pmc_index_shift)));
    }
}

pub unsafe fn uart_write(uart_conf: &TivaUartConf, data: char){

    //
    // Wait until space is available.
    //
    while(hwreg!(uart_conf.uart_base + uart::UART_O_FR, u32) & uart::UART_FR_TXFF) != 0 {
    }

    //
    // Send the char.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_DR, u32, data as u32);    
}

unsafe fn uart_enable(uart_conf: &TivaUartConf) {
    //
    // Enable the FIFO.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_LCRH, u32,
           hwreg!(uart_conf.uart_base + uart::UART_O_LCRH, u32)| uart::UART_LCRH_FEN);

    //
    // Enable RX, TX, and the UART.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32,
           hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32)| (uart::UART_CTL_UARTEN | uart::UART_CTL_TXE |
                                                                 uart::UART_CTL_RXE));
}


unsafe fn uart_disable(uart_conf: &TivaUartConf){
    while (hwreg!(uart_conf.uart_base + uart::UART_O_FR, u32) & uart::UART_FR_BUSY) != 0
    {
    }

    //
    // Disable the FIFO.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_LCRH, u32,
           hwreg!(uart_conf.uart_base + uart::UART_O_LCRH, u32) & !(uart::UART_LCRH_FEN));

    //
    // Disable the UART.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32,
           hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32) & !(uart::UART_CTL_UARTEN | uart::UART_CTL_TXE |
                                                                   uart::UART_CTL_RXE));
}

unsafe fn uart_config_set_exp_clk(uart_conf: &TivaUartConf,
                                  mut baudrate: u32,
                                  uart_clock: u32,
                                  ui32Config: u32) {
    uart_disable(uart_conf);

    if (baudrate * 16) > uart_clock {
        hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32,
               hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32)| uart::UART_CTL_HSE);
        baudrate /= 2;
    } else {
        hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32,
               hwreg!(uart_conf.uart_base + uart::UART_O_CTL, u32) & !(uart::UART_CTL_HSE));

    }

    let ui32Div : u32 = (((uart_clock * 8) / baudrate) + 1) / 2;

    //
    // Set the baud rate.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_IBRD, u32, ui32Div / 64);
    hwreg!(uart_conf.uart_base + uart::UART_O_FBRD, u32, ui32Div % 64);

    //
    // Set parity, data length, and number of stop bits.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_LCRH, u32, ui32Config);

    //
    // Clear the flags register.
    //
    hwreg!(uart_conf.uart_base + uart::UART_O_FR, u32, 0);

    uart_enable(uart_conf);
}

pub unsafe fn uart_init(uart: u8, baudrate: u32) -> TivaUartConf {
    
    let uart_conf : TivaUartConf = match (uart) {
        0 => Uart0,
        _ => Uart0
    };

    sysctl::sys_ctl_peripheral_enable(uart_conf.sysctl_periph_uart);

    uart_conf.gpio.init();

//    sysctl::sys_ctl_peripheral_enable(uart_conf.sysctl_periph_gpio);
    
    // config_gpio_pins!(uart_conf.gpio_base_addr_ahb, uart_conf.gpio_rx_pin);
    // config_gpio_pins!(uart_conf.gpio_base_addr_ahb, uart_conf.gpio_tx_pin);
    config_gpio_pins!(uart_conf, uart_conf.gpio_rx_pin_i);
    config_gpio_pins!(uart_conf, uart_conf.gpio_tx_pin_i);

    uart_conf.gpio.init_pin(uart_conf.gpio_rx_pin_i, tiva_gpio::TivaGpioMode::GpioHw);
    uart_conf.gpio.init_pin(uart_conf.gpio_tx_pin_i, tiva_gpio::TivaGpioMode::GpioHw);

    return uart_conf;
}
