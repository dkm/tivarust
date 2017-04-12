# tivarust: Small pet project to learn Rust and low level ARM

## Setting up environment

Install rustup, rust nightly and xargo: https://github.com/japaric/xargo

## Building

```
$ PATH=$HOME/.cargo/bin:$PATH make all
```

## Debuging using openocd

### Running openocd & GDB

```
$ make debug
```

### Using python extension to inspect low level registers

```
(gdb) source gdb.py
(gdb) read-tiva
RCC : 0x01CE1540 @ 0x400FE060
  > Main oscillator ENABLED
  > Oscillator source: [MOSC] Main Oscillator
  > [XTAL] Crystal value 16.0 Mhz
  > [BYPASS] The system clock is the PLL output divided by SYSDIV
  > PLL is operating normally
  > [PWMDIV] PWM Unit Clock Divisor is 64
  > [USEPWMDIV] PWM Clock Divisor : The system clock divider is the source for PWM
  > [USESYSDIV] Enable System Clock Divider: system clock divider is used (forced when PLL is selected as the source)
  > [SYSDIV] System Clock Divisor: 3
  > [ACG] Auto Clock Gating: RCGCn registers are used
...
```


