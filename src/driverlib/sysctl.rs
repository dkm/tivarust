use core::ptr;

//*****************************************************************************
//
// hw_sysctl.h - Macros used when accessing the system control hardware.
//
// Copyright (c) 2005-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.0.12573 of the Tiva Firmware Development Package.
//
//*****************************************************************************


//*****************************************************************************
//
// The following are defines for the System Control register addresses.
//
//*****************************************************************************
pub const SYSCTL_DID0: u32 = 0x400FE000 ; // Device Identification 0
pub const SYSCTL_DID1: u32 = 0x400FE004 ; // Device Identification 1
pub const SYSCTL_DC0: u32 = 0x400FE008 ; // Device Capabilities 0
pub const SYSCTL_DC1: u32 = 0x400FE010 ; // Device Capabilities 1
pub const SYSCTL_DC2: u32 = 0x400FE014 ; // Device Capabilities 2
pub const SYSCTL_DC3: u32 = 0x400FE018 ; // Device Capabilities 3
pub const SYSCTL_DC4: u32 = 0x400FE01C ; // Device Capabilities 4
pub const SYSCTL_DC5: u32 = 0x400FE020 ; // Device Capabilities 5
pub const SYSCTL_DC6: u32 = 0x400FE024 ; // Device Capabilities 6
pub const SYSCTL_DC7: u32 = 0x400FE028 ; // Device Capabilities 7
pub const SYSCTL_DC8: u32 = 0x400FE02C ; // Device Capabilities 8
pub const SYSCTL_PBORCTL: u32 = 0x400FE030 ; // Brown-Out Reset Control
pub const SYSCTL_PTBOCTL: u32 = 0x400FE038 ; // Power-Temp Brown Out Control
pub const SYSCTL_SRCR0: u32 = 0x400FE040 ; // Software Reset Control 0
pub const SYSCTL_SRCR1: u32 = 0x400FE044 ; // Software Reset Control 1
pub const SYSCTL_SRCR2: u32 = 0x400FE048 ; // Software Reset Control 2
pub const SYSCTL_RIS: u32 = 0x400FE050 ; // Raw Interrupt Status
pub const SYSCTL_IMC: u32 = 0x400FE054 ; // Interrupt Mask Control
pub const SYSCTL_MISC: u32 = 0x400FE058 ; // Masked Interrupt Status and
// Clear
pub const SYSCTL_RESC: u32 = 0x400FE05C ; // Reset Cause
pub const SYSCTL_PWRTC: u32 = 0x400FE060 ; // Power-Temperature Cause
pub const SYSCTL_RCC: u32 = 0x400FE060 ; // Run-Mode Clock Configuration
pub const SYSCTL_NMIC: u32 = 0x400FE064 ; // NMI Cause Register
pub const SYSCTL_GPIOHBCTL: u32 = 0x400FE06C ; // GPIO High-Performance Bus
// Control
pub const SYSCTL_RCC2: u32 = 0x400FE070 ; // Run-Mode Clock Configuration 2
pub const SYSCTL_MOSCCTL: u32 = 0x400FE07C ; // Main Oscillator Control
pub const SYSCTL_RSCLKCFG: u32 = 0x400FE0B0 ; // Run and Sleep Mode Configuration
// Register
pub const SYSCTL_MEMTIM0: u32 = 0x400FE0C0 ; // Memory Timing Parameter Register
// 0 for Main Flash and EEPROM
pub const SYSCTL_RCGC0: u32 = 0x400FE100 ; // Run Mode Clock Gating Control
// Register 0
pub const SYSCTL_RCGC1: u32 = 0x400FE104 ; // Run Mode Clock Gating Control
// Register 1
pub const SYSCTL_RCGC2: u32 = 0x400FE108 ; // Run Mode Clock Gating Control
// Register 2
pub const SYSCTL_SCGC0: u32 = 0x400FE110 ; // Sleep Mode Clock Gating Control
// Register 0
pub const SYSCTL_SCGC1: u32 = 0x400FE114 ; // Sleep Mode Clock Gating Control
// Register 1
pub const SYSCTL_SCGC2: u32 = 0x400FE118 ; // Sleep Mode Clock Gating Control
// Register 2
pub const SYSCTL_DCGC0: u32 = 0x400FE120 ; // Deep Sleep Mode Clock Gating
// Control Register 0
pub const SYSCTL_DCGC1: u32 = 0x400FE124 ; // Deep-Sleep Mode Clock Gating
// Control Register 1
pub const SYSCTL_DCGC2: u32 = 0x400FE128 ; // Deep Sleep Mode Clock Gating
// Control Register 2
pub const SYSCTL_ALTCLKCFG: u32 = 0x400FE138 ; // Alternate Clock Configuration
pub const SYSCTL_DSLPCLKCFG: u32 = 0x400FE144 ; // Deep Sleep Clock Configuration
pub const SYSCTL_DSCLKCFG: u32 = 0x400FE144 ; // Deep Sleep Clock Configuration
// Register
pub const SYSCTL_DIVSCLK: u32 = 0x400FE148 ; // Divisor and Source Clock
// Configuration
pub const SYSCTL_SYSPROP: u32 = 0x400FE14C ; // System Properties
pub const SYSCTL_PIOSCCAL: u32 = 0x400FE150 ; // Precision Internal Oscillator
// Calibration
pub const SYSCTL_PIOSCSTAT: u32 = 0x400FE154 ; // Precision Internal Oscillator
// Statistics
pub const SYSCTL_PLLFREQ0: u32 = 0x400FE160 ; // PLL Frequency 0
pub const SYSCTL_PLLFREQ1: u32 = 0x400FE164 ; // PLL Frequency 1
pub const SYSCTL_PLLSTAT: u32 = 0x400FE168 ; // PLL Status
pub const SYSCTL_SLPPWRCFG: u32 = 0x400FE188 ; // Sleep Power Configuration
pub const SYSCTL_DSLPPWRCFG: u32 = 0x400FE18C ; // Deep-Sleep Power Configuration
pub const SYSCTL_DC9: u32 = 0x400FE190 ; // Device Capabilities 9
pub const SYSCTL_NVMSTAT: u32 = 0x400FE1A0 ; // Non-Volatile Memory Information
pub const SYSCTL_LDOSPCTL: u32 = 0x400FE1B4 ; // LDO Sleep Power Control
pub const SYSCTL_LDODPCTL: u32 = 0x400FE1BC ; // LDO Deep-Sleep Power Control
pub const SYSCTL_RESBEHAVCTL: u32 = 0x400FE1D8 ; // Reset Behavior Control Register
pub const SYSCTL_HSSR: u32 = 0x400FE1F4 ; // Hardware System Service Request
pub const SYSCTL_USBPDS: u32 = 0x400FE280 ; // USB Power Domain Status
pub const SYSCTL_USBMPC: u32 = 0x400FE284 ; // USB Memory Power Control
pub const SYSCTL_EMACPDS: u32 = 0x400FE288 ; // Ethernet MAC Power Domain Status
pub const SYSCTL_EMACMPC: u32 = 0x400FE28C ; // Ethernet MAC Memory Power
// Control
pub const SYSCTL_LCDMPC: u32 = 0x400FE294 ; // LCD Memory Power Control
pub const SYSCTL_PPWD: u32 = 0x400FE300 ; // Watchdog Timer Peripheral
// Present
pub const SYSCTL_PPTIMER: u32 = 0x400FE304 ; // 16/32-Bit General-Purpose Timer
// Peripheral Present
pub const SYSCTL_PPGPIO: u32 = 0x400FE308 ; // General-Purpose Input/Output
// Peripheral Present
pub const SYSCTL_PPDMA: u32 = 0x400FE30C ; // Micro Direct Memory Access
// Peripheral Present
pub const SYSCTL_PPEPI: u32 = 0x400FE310 ; // EPI Peripheral Present
pub const SYSCTL_PPHIB: u32 = 0x400FE314 ; // Hibernation Peripheral Present
pub const SYSCTL_PPUART: u32 = 0x400FE318 ; // Universal Asynchronous
// Receiver/Transmitter Peripheral
// Present
pub const SYSCTL_PPSSI: u32 = 0x400FE31C ; // Synchronous Serial Interface
// Peripheral Present
pub const SYSCTL_PPI2C: u32 = 0x400FE320 ; // Inter-Integrated Circuit
// Peripheral Present
pub const SYSCTL_PPUSB: u32 = 0x400FE328 ; // Universal Serial Bus Peripheral
// Present
pub const SYSCTL_PPEPHY: u32 = 0x400FE330 ; // Ethernet PHY Peripheral Present
pub const SYSCTL_PPCAN: u32 = 0x400FE334 ; // Controller Area Network
// Peripheral Present
pub const SYSCTL_PPADC: u32 = 0x400FE338 ; // Analog-to-Digital Converter
// Peripheral Present
pub const SYSCTL_PPACMP: u32 = 0x400FE33C ; // Analog Comparator Peripheral
// Present
pub const SYSCTL_PPPWM: u32 = 0x400FE340 ; // Pulse Width Modulator Peripheral
// Present
pub const SYSCTL_PPQEI: u32 = 0x400FE344 ; // Quadrature Encoder Interface
// Peripheral Present
pub const SYSCTL_PPLPC: u32 = 0x400FE348 ; // Low Pin Count Interface
// Peripheral Present
pub const SYSCTL_PPPECI: u32 = 0x400FE350 ; // Platform Environment Control
// Interface Peripheral Present
pub const SYSCTL_PPFAN: u32 = 0x400FE354 ; // Fan Control Peripheral Present
pub const SYSCTL_PPEEPROM: u32 = 0x400FE358 ; // EEPROM Peripheral Present
pub const SYSCTL_PPWTIMER: u32 = 0x400FE35C ; // 32/64-Bit Wide General-Purpose
// Timer Peripheral Present
pub const SYSCTL_PPRTS: u32 = 0x400FE370 ; // Remote Temperature Sensor
// Peripheral Present
pub const SYSCTL_PPCCM: u32 = 0x400FE374 ; // CRC and Cryptographic Modules
// Peripheral Present
pub const SYSCTL_PPLCD: u32 = 0x400FE390 ; // LCD Peripheral Present
pub const SYSCTL_PPOWIRE: u32 = 0x400FE398 ; // 1-Wire Peripheral Present
pub const SYSCTL_PPEMAC: u32 = 0x400FE39C ; // Ethernet MAC Peripheral Present
pub const SYSCTL_PPHIM: u32 = 0x400FE3A4 ; // Human Interface Master
// Peripheral Present
pub const SYSCTL_SRWD: u32 = 0x400FE500 ; // Watchdog Timer Software Reset
pub const SYSCTL_SRTIMER: u32 = 0x400FE504 ; // 16/32-Bit General-Purpose Timer
// Software Reset
pub const SYSCTL_SRGPIO: u32 = 0x400FE508 ; // General-Purpose Input/Output
// Software Reset
pub const SYSCTL_SRDMA: u32 = 0x400FE50C ; // Micro Direct Memory Access
// Software Reset
pub const SYSCTL_SREPI: u32 = 0x400FE510 ; // EPI Software Reset
pub const SYSCTL_SRHIB: u32 = 0x400FE514 ; // Hibernation Software Reset
pub const SYSCTL_SRUART: u32 = 0x400FE518 ; // Universal Asynchronous
// Receiver/Transmitter Software
// Reset
pub const SYSCTL_SRSSI: u32 = 0x400FE51C ; // Synchronous Serial Interface
// Software Reset
pub const SYSCTL_SRI2C: u32 = 0x400FE520 ; // Inter-Integrated Circuit
// Software Reset
pub const SYSCTL_SRUSB: u32 = 0x400FE528 ; // Universal Serial Bus Software
// Reset
pub const SYSCTL_SREPHY: u32 = 0x400FE530 ; // Ethernet PHY Software Reset
pub const SYSCTL_SRCAN: u32 = 0x400FE534 ; // Controller Area Network Software
// Reset
pub const SYSCTL_SRADC: u32 = 0x400FE538 ; // Analog-to-Digital Converter
// Software Reset
pub const SYSCTL_SRACMP: u32 = 0x400FE53C ; // Analog Comparator Software Reset
pub const SYSCTL_SRPWM: u32 = 0x400FE540 ; // Pulse Width Modulator Software
// Reset
pub const SYSCTL_SRQEI: u32 = 0x400FE544 ; // Quadrature Encoder Interface
// Software Reset
pub const SYSCTL_SREEPROM: u32 = 0x400FE558 ; // EEPROM Software Reset
pub const SYSCTL_SRWTIMER: u32 = 0x400FE55C ; // 32/64-Bit Wide General-Purpose
// Timer Software Reset
pub const SYSCTL_SRCCM: u32 = 0x400FE574 ; // CRC and Cryptographic Modules
// Software Reset
pub const SYSCTL_SRLCD: u32 = 0x400FE590 ; // LCD Controller Software Reset
pub const SYSCTL_SROWIRE: u32 = 0x400FE598 ; // 1-Wire Software Reset
pub const SYSCTL_SREMAC: u32 = 0x400FE59C ; // Ethernet MAC Software Reset
pub const SYSCTL_RCGCWD: u32 = 0x400FE600 ; // Watchdog Timer Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCTIMER: u32 = 0x400FE604 ; // 16/32-Bit General-Purpose Timer
// Run Mode Clock Gating Control
pub const SYSCTL_RCGCGPIO: u32 = 0x400FE608 ; // General-Purpose Input/Output Run
// Mode Clock Gating Control
pub const SYSCTL_RCGCDMA: u32 = 0x400FE60C ; // Micro Direct Memory Access Run
// Mode Clock Gating Control
pub const SYSCTL_RCGCEPI: u32 = 0x400FE610 ; // EPI Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCHIB: u32 = 0x400FE614 ; // Hibernation Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART: u32 = 0x400FE618 ; // Universal Asynchronous
// Receiver/Transmitter Run Mode
// Clock Gating Control
pub const SYSCTL_RCGCSSI: u32 = 0x400FE61C ; // Synchronous Serial Interface Run
// Mode Clock Gating Control
pub const SYSCTL_RCGCI2C: u32 = 0x400FE620 ; // Inter-Integrated Circuit Run
// Mode Clock Gating Control
pub const SYSCTL_RCGCUSB: u32 = 0x400FE628 ; // Universal Serial Bus Run Mode
// Clock Gating Control
pub const SYSCTL_RCGCEPHY: u32 = 0x400FE630 ; // Ethernet PHY Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCCAN: u32 = 0x400FE634 ; // Controller Area Network Run Mode
// Clock Gating Control
pub const SYSCTL_RCGCADC: u32 = 0x400FE638 ; // Analog-to-Digital Converter Run
// Mode Clock Gating Control
pub const SYSCTL_RCGCACMP: u32 = 0x400FE63C ; // Analog Comparator Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCPWM: u32 = 0x400FE640 ; // Pulse Width Modulator Run Mode
// Clock Gating Control
pub const SYSCTL_RCGCQEI: u32 = 0x400FE644 ; // Quadrature Encoder Interface Run
// Mode Clock Gating Control
pub const SYSCTL_RCGCEEPROM: u32 = 0x400FE658 ; // EEPROM Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCWTIMER: u32 = 0x400FE65C ; // 32/64-Bit Wide General-Purpose
// Timer Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCCCM: u32 = 0x400FE674 ; // CRC and Cryptographic Modules
// Run Mode Clock Gating Control
pub const SYSCTL_RCGCLCD: u32 = 0x400FE690 ; // LCD Controller Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCOWIRE: u32 = 0x400FE698 ; // 1-Wire Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCEMAC: u32 = 0x400FE69C ; // Ethernet MAC Run Mode Clock
// Gating Control
pub const SYSCTL_SCGCWD: u32 = 0x400FE700 ; // Watchdog Timer Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCTIMER: u32 = 0x400FE704 ; // 16/32-Bit General-Purpose Timer
// Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCGPIO: u32 = 0x400FE708 ; // General-Purpose Input/Output
// Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCDMA: u32 = 0x400FE70C ; // Micro Direct Memory Access Sleep
// Mode Clock Gating Control
pub const SYSCTL_SCGCEPI: u32 = 0x400FE710 ; // EPI Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCHIB: u32 = 0x400FE714 ; // Hibernation Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART: u32 = 0x400FE718 ; // Universal Asynchronous
// Receiver/Transmitter Sleep Mode
// Clock Gating Control
pub const SYSCTL_SCGCSSI: u32 = 0x400FE71C ; // Synchronous Serial Interface
// Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCI2C: u32 = 0x400FE720 ; // Inter-Integrated Circuit Sleep
// Mode Clock Gating Control
pub const SYSCTL_SCGCUSB: u32 = 0x400FE728 ; // Universal Serial Bus Sleep Mode
// Clock Gating Control
pub const SYSCTL_SCGCEPHY: u32 = 0x400FE730 ; // Ethernet PHY Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCCAN: u32 = 0x400FE734 ; // Controller Area Network Sleep
// Mode Clock Gating Control
pub const SYSCTL_SCGCADC: u32 = 0x400FE738 ; // Analog-to-Digital Converter
// Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCACMP: u32 = 0x400FE73C ; // Analog Comparator Sleep Mode
// Clock Gating Control
pub const SYSCTL_SCGCPWM: u32 = 0x400FE740 ; // Pulse Width Modulator Sleep Mode
// Clock Gating Control
pub const SYSCTL_SCGCQEI: u32 = 0x400FE744 ; // Quadrature Encoder Interface
// Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCEEPROM: u32 = 0x400FE758 ; // EEPROM Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCWTIMER: u32 = 0x400FE75C ; // 32/64-Bit Wide General-Purpose
// Timer Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCCCM: u32 = 0x400FE774 ; // CRC and Cryptographic Modules
// Sleep Mode Clock Gating Control
pub const SYSCTL_SCGCLCD: u32 = 0x400FE790 ; // LCD Controller Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCOWIRE: u32 = 0x400FE798 ; // 1-Wire Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCEMAC: u32 = 0x400FE79C ; // Ethernet MAC Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCWD: u32 = 0x400FE800 ; // Watchdog Timer Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCTIMER: u32 = 0x400FE804 ; // 16/32-Bit General-Purpose Timer
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCGPIO: u32 = 0x400FE808 ; // General-Purpose Input/Output
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCDMA: u32 = 0x400FE80C ; // Micro Direct Memory Access
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCEPI: u32 = 0x400FE810 ; // EPI Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCHIB: u32 = 0x400FE814 ; // Hibernation Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART: u32 = 0x400FE818 ; // Universal Asynchronous
// Receiver/Transmitter Deep-Sleep
// Mode Clock Gating Control
pub const SYSCTL_DCGCSSI: u32 = 0x400FE81C ; // Synchronous Serial Interface
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCI2C: u32 = 0x400FE820 ; // Inter-Integrated Circuit
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCUSB: u32 = 0x400FE828 ; // Universal Serial Bus Deep-Sleep
// Mode Clock Gating Control
pub const SYSCTL_DCGCEPHY: u32 = 0x400FE830 ; // Ethernet PHY Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCCAN: u32 = 0x400FE834 ; // Controller Area Network
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCADC: u32 = 0x400FE838 ; // Analog-to-Digital Converter
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCACMP: u32 = 0x400FE83C ; // Analog Comparator Deep-Sleep
// Mode Clock Gating Control
pub const SYSCTL_DCGCPWM: u32 = 0x400FE840 ; // Pulse Width Modulator Deep-Sleep
// Mode Clock Gating Control
pub const SYSCTL_DCGCQEI: u32 = 0x400FE844 ; // Quadrature Encoder Interface
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCEEPROM: u32 = 0x400FE858 ; // EEPROM Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCWTIMER: u32 = 0x400FE85C ; // 32/64-Bit Wide General-Purpose
// Timer Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCCCM: u32 = 0x400FE874 ; // CRC and Cryptographic Modules
// Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCLCD: u32 = 0x400FE890 ; // LCD Controller Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCOWIRE: u32 = 0x400FE898 ; // 1-Wire Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCEMAC: u32 = 0x400FE89C ; // Ethernet MAC Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_PCWD: u32 = 0x400FE900 ; // Watchdog Timer Power Control
pub const SYSCTL_PCTIMER: u32 = 0x400FE904 ; // 16/32-Bit General-Purpose Timer
// Power Control
pub const SYSCTL_PCGPIO: u32 = 0x400FE908 ; // General-Purpose Input/Output
// Power Control
pub const SYSCTL_PCDMA: u32 = 0x400FE90C ; // Micro Direct Memory Access Power
// Control
pub const SYSCTL_PCEPI: u32 = 0x400FE910 ; // External Peripheral Interface
// Power Control
pub const SYSCTL_PCHIB: u32 = 0x400FE914 ; // Hibernation Power Control
pub const SYSCTL_PCUART: u32 = 0x400FE918 ; // Universal Asynchronous
// Receiver/Transmitter Power
// Control
pub const SYSCTL_PCSSI: u32 = 0x400FE91C ; // Synchronous Serial Interface
// Power Control
pub const SYSCTL_PCI2C: u32 = 0x400FE920 ; // Inter-Integrated Circuit Power
// Control
pub const SYSCTL_PCUSB: u32 = 0x400FE928 ; // Universal Serial Bus Power
// Control
pub const SYSCTL_PCEPHY: u32 = 0x400FE930 ; // Ethernet PHY Power Control
pub const SYSCTL_PCCAN: u32 = 0x400FE934 ; // Controller Area Network Power
// Control
pub const SYSCTL_PCADC: u32 = 0x400FE938 ; // Analog-to-Digital Converter
// Power Control
pub const SYSCTL_PCACMP: u32 = 0x400FE93C ; // Analog Comparator Power Control
pub const SYSCTL_PCPWM: u32 = 0x400FE940 ; // Pulse Width Modulator Power
// Control
pub const SYSCTL_PCQEI: u32 = 0x400FE944 ; // Quadrature Encoder Interface
// Power Control
pub const SYSCTL_PCEEPROM: u32 = 0x400FE958 ; // EEPROM Power Control
pub const SYSCTL_PCCCM: u32 = 0x400FE974 ; // CRC and Cryptographic Modules
// Power Control
pub const SYSCTL_PCLCD: u32 = 0x400FE990 ; // LCD Controller Power Control
pub const SYSCTL_PCOWIRE: u32 = 0x400FE998 ; // 1-Wire Power Control
pub const SYSCTL_PCEMAC: u32 = 0x400FE99C ; // Ethernet MAC Power Control
pub const SYSCTL_PRWD: u32 = 0x400FEA00 ; // Watchdog Timer Peripheral Ready
pub const SYSCTL_PRTIMER: u32 = 0x400FEA04 ; // 16/32-Bit General-Purpose Timer
// Peripheral Ready
pub const SYSCTL_PRGPIO: u32 = 0x400FEA08 ; // General-Purpose Input/Output
// Peripheral Ready
pub const SYSCTL_PRDMA: u32 = 0x400FEA0C ; // Micro Direct Memory Access
// Peripheral Ready
pub const SYSCTL_PREPI: u32 = 0x400FEA10 ; // EPI Peripheral Ready
pub const SYSCTL_PRHIB: u32 = 0x400FEA14 ; // Hibernation Peripheral Ready
pub const SYSCTL_PRUART: u32 = 0x400FEA18 ; // Universal Asynchronous
// Receiver/Transmitter Peripheral
// Ready
pub const SYSCTL_PRSSI: u32 = 0x400FEA1C ; // Synchronous Serial Interface
// Peripheral Ready
pub const SYSCTL_PRI2C: u32 = 0x400FEA20 ; // Inter-Integrated Circuit
// Peripheral Ready
pub const SYSCTL_PRUSB: u32 = 0x400FEA28 ; // Universal Serial Bus Peripheral
// Ready
pub const SYSCTL_PREPHY: u32 = 0x400FEA30 ; // Ethernet PHY Peripheral Ready
pub const SYSCTL_PRCAN: u32 = 0x400FEA34 ; // Controller Area Network
// Peripheral Ready
pub const SYSCTL_PRADC: u32 = 0x400FEA38 ; // Analog-to-Digital Converter
// Peripheral Ready
pub const SYSCTL_PRACMP: u32 = 0x400FEA3C ; // Analog Comparator Peripheral
// Ready
pub const SYSCTL_PRPWM: u32 = 0x400FEA40 ; // Pulse Width Modulator Peripheral
// Ready
pub const SYSCTL_PRQEI: u32 = 0x400FEA44 ; // Quadrature Encoder Interface
// Peripheral Ready
pub const SYSCTL_PREEPROM: u32 = 0x400FEA58 ; // EEPROM Peripheral Ready
pub const SYSCTL_PRWTIMER: u32 = 0x400FEA5C ; // 32/64-Bit Wide General-Purpose
// Timer Peripheral Ready
pub const SYSCTL_PRCCM: u32 = 0x400FEA74 ; // CRC and Cryptographic Modules
// Peripheral Ready
pub const SYSCTL_PRLCD: u32 = 0x400FEA90 ; // LCD Controller Peripheral Ready
pub const SYSCTL_PROWIRE: u32 = 0x400FEA98 ; // 1-Wire Peripheral Ready
pub const SYSCTL_PREMAC: u32 = 0x400FEA9C ; // Ethernet MAC Peripheral Ready
pub const SYSCTL_CCMCGREQ: u32 = 0x44030204 ; // Cryptographic Modules Clock
// Gating Request

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID0 register.
//
//*****************************************************************************
pub const SYSCTL_DID0_VER_M: u32 = 0x70000000 ; // DID0 Version
pub const SYSCTL_DID0_VER_1: u32 = 0x10000000 ; // Second version of the DID0
// register format.
pub const SYSCTL_DID0_CLASS_M: u32 = 0x00FF0000 ; // Device Class
pub const  SYSCTL_DID0_CLASS_TM4C123 : u32 =     0x00050000  ; // Tiva TM4C123x and TM4E123x
// microcontrollers
pub const  SYSCTL_DID0_CLASS_TM4C129: u32 =    0x000A0000 ; // Tiva(TM) TM4C129-class
// microcontrollers
pub const SYSCTL_DID0_MAJ_M: u32 = 0x0000FF00 ; // Major Revision
pub const SYSCTL_DID0_MAJ_REVA: u32 = 0x00000000 ; // Revision A (initial device)
pub const SYSCTL_DID0_MAJ_REVB: u32 = 0x00000100 ; // Revision B (first base layer
// revision)
pub const SYSCTL_DID0_MAJ_REVC: u32 = 0x00000200 ; // Revision C (second base layer
// revision)
pub const SYSCTL_DID0_MIN_M: u32 = 0x000000FF ; // Minor Revision
pub const SYSCTL_DID0_MIN_0: u32 = 0x00000000 ; // Initial device, or a major
// revision update
pub const SYSCTL_DID0_MIN_1: u32 = 0x00000001 ; // First metal layer change
pub const SYSCTL_DID0_MIN_2: u32 = 0x00000002 ; // Second metal layer change

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DID1 register.
//
//*****************************************************************************
pub const SYSCTL_DID1_VER_M: u32 = 0xF0000000 ; // DID1 Version
pub const SYSCTL_DID1_VER_1: u32 = 0x10000000 ; // fury_ib
pub const SYSCTL_DID1_FAM_M: u32 = 0x0F000000 ; // Family
pub const SYSCTL_DID1_FAM_TIVA: u32 = 0x00000000 ; // Tiva family of microcontollers
pub const SYSCTL_DID1_PRTNO_M: u32 = 0x00FF0000 ; // Part Number
pub const  SYSCTL_DID1_PRTNO_TM4C1230C3PM: u32 =    0x00220000 ; // TM4C1230C3PM
pub const  SYSCTL_DID1_PRTNO_TM4C1230D5PM: u32 =    0x00230000 ; // TM4C1230D5PM
pub const  SYSCTL_DID1_PRTNO_TM4C1230E6PM: u32 =    0x00200000 ; // TM4C1230E6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1230H6PM: u32 =    0x00210000 ; // TM4C1230H6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1231C3PM: u32 =    0x00180000 ; // TM4C1231C3PM
pub const  SYSCTL_DID1_PRTNO_TM4C1231D5PM: u32 =    0x00190000 ; // TM4C1231D5PM
pub const  SYSCTL_DID1_PRTNO_TM4C1231D5PZ: u32 =    0x00360000 ; // TM4C1231D5PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1231E6PM: u32 =    0x00100000 ; // TM4C1231E6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1231E6PZ: u32 =    0x00300000 ; // TM4C1231E6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1231H6PGE: u32 =    0x00350000 ; // TM4C1231H6PGE
pub const  SYSCTL_DID1_PRTNO_TM4C1231H6PM: u32 =    0x00110000 ; // TM4C1231H6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1231H6PZ: u32 =    0x00310000 ; // TM4C1231H6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1232C3PM: u32 =    0x00080000 ; // TM4C1232C3PM
pub const  SYSCTL_DID1_PRTNO_TM4C1232D5PM: u32 =    0x00090000 ; // TM4C1232D5PM
pub const  SYSCTL_DID1_PRTNO_TM4C1232E6PM: u32 =    0x000A0000 ; // TM4C1232E6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1232H6PM: u32 =    0x000B0000 ; // TM4C1232H6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1233C3PM: u32 =    0x00010000 ; // TM4C1233C3PM
pub const  SYSCTL_DID1_PRTNO_TM4C1233D5PM: u32 =    0x00020000 ; // TM4C1233D5PM
pub const  SYSCTL_DID1_PRTNO_TM4C1233D5PZ: u32 =    0x00D00000 ; // TM4C1233D5PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1233E6PM: u32 =    0x00030000 ; // TM4C1233E6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1233E6PZ: u32 =    0x00D10000 ; // TM4C1233E6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1233H6PGE: u32 =    0x00D60000 ; // TM4C1233H6PGE
pub const  SYSCTL_DID1_PRTNO_TM4C1233H6PM: u32 =    0x00040000 ; // TM4C1233H6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1233H6PZ: u32 =    0x00D20000 ; // TM4C1233H6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1236D5PM: u32 =    0x00520000 ; // TM4C1236D5PM
pub const  SYSCTL_DID1_PRTNO_TM4C1236E6PM: u32 =    0x00500000 ; // TM4C1236E6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1236H6PM: u32 =    0x00510000 ; // TM4C1236H6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1237D5PM: u32 =    0x00480000 ; // TM4C1237D5PM
pub const  SYSCTL_DID1_PRTNO_TM4C1237D5PZ: u32 =    0x00660000 ; // TM4C1237D5PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1237E6PM: u32 =    0x00400000 ; // TM4C1237E6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1237E6PZ: u32 =    0x00600000 ; // TM4C1237E6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C1237H6PGE: u32 =    0x00650000 ; // TM4C1237H6PGE
pub const  SYSCTL_DID1_PRTNO_TM4C1237H6PM: u32 =    0x00410000 ; // TM4C1237H6PM
pub const  SYSCTL_DID1_PRTNO_TM4C1237H6PZ: u32 =    0x00610000 ; // TM4C1237H6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C123AE6PM: u32 =    0x00800000 ; // TM4C123AE6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123AH6PM: u32 =    0x00830000 ; // TM4C123AH6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123BE6PM: u32 =    0x00700000 ; // TM4C123BE6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123BE6PZ: u32 =    0x00C30000 ; // TM4C123BE6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C123BH6PGE: u32 =    0x00C60000 ; // TM4C123BH6PGE
pub const  SYSCTL_DID1_PRTNO_TM4C123BH6PM: u32 =    0x00730000 ; // TM4C123BH6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123BH6PZ: u32 =    0x00C40000 ; // TM4C123BH6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C123BH6ZRB: u32 =    0x00E90000 ; // TM4C123BH6ZRB
pub const  SYSCTL_DID1_PRTNO_TM4C123FE6PM: u32 =    0x00B00000 ; // TM4C123FE6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123FH6PM: u32 =    0x00B10000 ; // TM4C123FH6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123GE6PM: u32 =    0x00A00000 ; // TM4C123GE6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123GE6PZ: u32 =    0x00C00000 ; // TM4C123GE6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C123GH6PGE: u32 =    0x00C50000 ; // TM4C123GH6PGE
pub const  SYSCTL_DID1_PRTNO_TM4C123GH6PM: u32 =    0x00A10000 ; // TM4C123GH6PM
pub const  SYSCTL_DID1_PRTNO_TM4C123GH6PZ: u32 =    0x00C10000 ; // TM4C123GH6PZ
pub const  SYSCTL_DID1_PRTNO_TM4C123GH6ZRB: u32 =    0x00E30000 ; // TM4C123GH6ZRB
pub const  SYSCTL_DID1_PRTNO_TM4C1290NCPDT: u32 =    0x00190000 ; // TM4C1290NCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C1290NCZAD: u32 =    0x001B0000 ; // TM4C1290NCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C1292NCPDT: u32 =    0x001C0000 ; // TM4C1292NCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C1292NCZAD: u32 =    0x001E0000 ; // TM4C1292NCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C1294KCPDT: u32 =    0x00340000 ; // TM4C1294KCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C1294NCPDT: u32 =    0x001F0000 ; // TM4C1294NCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C1294NCZAD: u32 =    0x00210000 ; // TM4C1294NCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C1297NCZAD: u32 =    0x00220000 ; // TM4C1297NCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C1299KCZAD: u32 =    0x00360000 ; // TM4C1299KCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C1299NCZAD: u32 =    0x00230000 ; // TM4C1299NCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C129CNCPDT: u32 =    0x00240000 ; // TM4C129CNCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C129CNCZAD: u32 =    0x00260000 ; // TM4C129CNCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C129DNCPDT: u32 =    0x00270000 ; // TM4C129DNCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C129DNCZAD: u32 =    0x00290000 ; // TM4C129DNCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C129EKCPDT: u32 =    0x00350000 ; // TM4C129EKCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C129ENCPDT: u32 =    0x002D0000 ; // TM4C129ENCPDT
pub const  SYSCTL_DID1_PRTNO_TM4C129ENCZAD: u32 =    0x002F0000 ; // TM4C129ENCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C129LNCZAD: u32 =    0x00300000 ; // TM4C129LNCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C129XKCZAD: u32 =    0x00370000 ; // TM4C129XKCZAD
pub const  SYSCTL_DID1_PRTNO_TM4C129XNCZAD: u32 =    0x00320000 ; // TM4C129XNCZAD
pub const SYSCTL_DID1_PINCNT_M: u32 = 0x0000E000 ; // Package Pin Count
pub const SYSCTL_DID1_PINCNT_100: u32 = 0x00004000 ; // 100-pin LQFP package
pub const SYSCTL_DID1_PINCNT_64: u32 = 0x00006000 ; // 64-pin LQFP package
pub const SYSCTL_DID1_PINCNT_144: u32 = 0x00008000 ; // 144-pin LQFP package
pub const SYSCTL_DID1_PINCNT_157: u32 = 0x0000A000 ; // 157-pin BGA package
pub const SYSCTL_DID1_PINCNT_128: u32 = 0x0000C000 ; // 128-pin TQFP package
pub const SYSCTL_DID1_TEMP_M: u32 = 0x000000E0 ; // Temperature Range
pub const SYSCTL_DID1_TEMP_C: u32 = 0x00000000 ; // Commercial temperature range
pub const SYSCTL_DID1_TEMP_I: u32 = 0x00000020 ; // Industrial temperature range
pub const SYSCTL_DID1_TEMP_E: u32 = 0x00000040 ; // Extended temperature range
pub const SYSCTL_DID1_TEMP_IE: u32 = 0x00000060 ; // Available in both industrial
// temperature range (-40C to 85C)
// and extended temperature range
// (-40C to 105C) devices. See
pub const SYSCTL_DID1_PKG_M: u32 = 0x00000018 ; // Package Type
pub const SYSCTL_DID1_PKG_QFP: u32 = 0x00000008 ; // QFP package
pub const SYSCTL_DID1_PKG_BGA: u32 = 0x00000010 ; // BGA package
pub const SYSCTL_DID1_ROHS: u32 = 0x00000004 ; // RoHS-Compliance
pub const SYSCTL_DID1_QUAL_M: u32 = 0x00000003 ; // Qualification Status
pub const SYSCTL_DID1_QUAL_ES: u32 = 0x00000000 ; // Engineering Sample (unqualified)
pub const SYSCTL_DID1_QUAL_PP: u32 = 0x00000001 ; // Pilot Production (unqualified)
pub const SYSCTL_DID1_QUAL_FQ: u32 = 0x00000002 ; // Fully Qualified

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC0 register.
//
//*****************************************************************************
pub const SYSCTL_DC0_SRAMSZ_M: u32 = 0xFFFF0000 ; // SRAM Size
pub const SYSCTL_DC0_SRAMSZ_2KB: u32 = 0x00070000 ; // 2 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_4KB: u32 = 0x000F0000 ; // 4 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_6KB: u32 = 0x00170000 ; // 6 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_8KB: u32 = 0x001F0000 ; // 8 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_12KB: u32 = 0x002F0000 ; // 12 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_16KB: u32 = 0x003F0000 ; // 16 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_20KB: u32 = 0x004F0000 ; // 20 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_24KB: u32 = 0x005F0000 ; // 24 KB of SRAM
pub const SYSCTL_DC0_SRAMSZ_32KB: u32 = 0x007F0000 ; // 32 KB of SRAM
pub const SYSCTL_DC0_FLASHSZ_M: u32 = 0x0000FFFF ; // Flash Size
pub const SYSCTL_DC0_FLASHSZ_8KB: u32 = 0x00000003 ; // 8 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_16KB: u32 = 0x00000007 ; // 16 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_32KB: u32 = 0x0000000F ; // 32 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_64KB: u32 = 0x0000001F ; // 64 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_96KB: u32 = 0x0000002F ; // 96 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_128K: u32 = 0x0000003F ; // 128 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_192K: u32 = 0x0000005F ; // 192 KB of Flash
pub const SYSCTL_DC0_FLASHSZ_256K: u32 = 0x0000007F ; // 256 KB of Flash

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC1 register.
//
//*****************************************************************************
pub const SYSCTL_DC1_WDT1: u32 = 0x10000000 ; // Watchdog Timer1 Present
pub const SYSCTL_DC1_CAN1: u32 = 0x02000000 ; // CAN Module 1 Present
pub const SYSCTL_DC1_CAN0: u32 = 0x01000000 ; // CAN Module 0 Present
pub const SYSCTL_DC1_PWM1: u32 = 0x00200000 ; // PWM Module 1 Present
pub const SYSCTL_DC1_PWM0: u32 = 0x00100000 ; // PWM Module 0 Present
pub const SYSCTL_DC1_ADC1: u32 = 0x00020000 ; // ADC Module 1 Present
pub const SYSCTL_DC1_ADC0: u32 = 0x00010000 ; // ADC Module 0 Present
pub const SYSCTL_DC1_MINSYSDIV_M: u32 = 0x0000F000 ; // System Clock Divider
pub const SYSCTL_DC1_MINSYSDIV_80: u32 = 0x00001000 ; // Specifies an 80-MHz CPU clock
// with a PLL divider of 2.5
pub const SYSCTL_DC1_MINSYSDIV_66: u32 = 0x00002000 ; // Specifies a 66-MHz CPU clock
// with a PLL divider of 3
pub const SYSCTL_DC1_MINSYSDIV_50: u32 = 0x00003000 ; // Specifies a 50-MHz CPU clock
// with a PLL divider of 4
pub const SYSCTL_DC1_MINSYSDIV_40: u32 = 0x00004000 ; // Specifies a 40-MHz CPU clock
// with a PLL divider of 5
pub const SYSCTL_DC1_MINSYSDIV_25: u32 = 0x00007000 ; // Specifies a 25-MHz clock with a
// PLL divider of 8
pub const SYSCTL_DC1_MINSYSDIV_20: u32 = 0x00009000 ; // Specifies a 20-MHz clock with a
// PLL divider of 10
pub const SYSCTL_DC1_ADC1SPD_M: u32 = 0x00000C00 ; // Max ADC1 Speed
pub const SYSCTL_DC1_ADC1SPD_125K: u32 = 0x00000000 ; // 125K samples/second
pub const SYSCTL_DC1_ADC1SPD_250K: u32 = 0x00000400 ; // 250K samples/second
pub const SYSCTL_DC1_ADC1SPD_500K: u32 = 0x00000800 ; // 500K samples/second
pub const SYSCTL_DC1_ADC1SPD_1M: u32 = 0x00000C00 ; // 1M samples/second
pub const SYSCTL_DC1_ADC0SPD_M: u32 = 0x00000300 ; // Max ADC0 Speed
pub const SYSCTL_DC1_ADC0SPD_125K: u32 = 0x00000000 ; // 125K samples/second
pub const SYSCTL_DC1_ADC0SPD_250K: u32 = 0x00000100 ; // 250K samples/second
pub const SYSCTL_DC1_ADC0SPD_500K: u32 = 0x00000200 ; // 500K samples/second
pub const SYSCTL_DC1_ADC0SPD_1M: u32 = 0x00000300 ; // 1M samples/second
pub const SYSCTL_DC1_MPU: u32 = 0x00000080 ; // MPU Present
pub const SYSCTL_DC1_HIB: u32 = 0x00000040 ; // Hibernation Module Present
pub const SYSCTL_DC1_TEMP: u32 = 0x00000020 ; // Temp Sensor Present
pub const SYSCTL_DC1_PLL: u32 = 0x00000010 ; // PLL Present
pub const SYSCTL_DC1_WDT0: u32 = 0x00000008 ; // Watchdog Timer 0 Present
pub const SYSCTL_DC1_SWO: u32 = 0x00000004 ; // SWO Trace Port Present
pub const SYSCTL_DC1_SWD: u32 = 0x00000002 ; // SWD Present
pub const SYSCTL_DC1_JTAG: u32 = 0x00000001 ; // JTAG Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC2 register.
//
//*****************************************************************************
pub const SYSCTL_DC2_EPI0: u32 = 0x40000000 ; // EPI Module 0 Present
pub const SYSCTL_DC2_I2S0: u32 = 0x10000000 ; // I2S Module 0 Present
pub const SYSCTL_DC2_COMP2: u32 = 0x04000000 ; // Analog Comparator 2 Present
pub const SYSCTL_DC2_COMP1: u32 = 0x02000000 ; // Analog Comparator 1 Present
pub const SYSCTL_DC2_COMP0: u32 = 0x01000000 ; // Analog Comparator 0 Present
pub const SYSCTL_DC2_TIMER3: u32 = 0x00080000 ; // Timer Module 3 Present
pub const SYSCTL_DC2_TIMER2: u32 = 0x00040000 ; // Timer Module 2 Present
pub const SYSCTL_DC2_TIMER1: u32 = 0x00020000 ; // Timer Module 1 Present
pub const SYSCTL_DC2_TIMER0: u32 = 0x00010000 ; // Timer Module 0 Present
pub const SYSCTL_DC2_I2C1HS: u32 = 0x00008000 ; // I2C Module 1 Speed
pub const SYSCTL_DC2_I2C1: u32 = 0x00004000 ; // I2C Module 1 Present
pub const SYSCTL_DC2_I2C0HS: u32 = 0x00002000 ; // I2C Module 0 Speed
pub const SYSCTL_DC2_I2C0: u32 = 0x00001000 ; // I2C Module 0 Present
pub const SYSCTL_DC2_QEI1: u32 = 0x00000200 ; // QEI Module 1 Present
pub const SYSCTL_DC2_QEI0: u32 = 0x00000100 ; // QEI Module 0 Present
pub const SYSCTL_DC2_SSI1: u32 = 0x00000020 ; // SSI Module 1 Present
pub const SYSCTL_DC2_SSI0: u32 = 0x00000010 ; // SSI Module 0 Present
pub const SYSCTL_DC2_UART2: u32 = 0x00000004 ; // UART Module 2 Present
pub const SYSCTL_DC2_UART1: u32 = 0x00000002 ; // UART Module 1 Present
pub const SYSCTL_DC2_UART0: u32 = 0x00000001 ; // UART Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC3 register.
//
//*****************************************************************************
pub const SYSCTL_DC3_32KHZ: u32 = 0x80000000 ; // 32KHz Input Clock Available
pub const SYSCTL_DC3_CCP5: u32 = 0x20000000 ; // T2CCP1 Pin Present
pub const SYSCTL_DC3_CCP4: u32 = 0x10000000 ; // T2CCP0 Pin Present
pub const SYSCTL_DC3_CCP3: u32 = 0x08000000 ; // T1CCP1 Pin Present
pub const SYSCTL_DC3_CCP2: u32 = 0x04000000 ; // T1CCP0 Pin Present
pub const SYSCTL_DC3_CCP1: u32 = 0x02000000 ; // T0CCP1 Pin Present
pub const SYSCTL_DC3_CCP0: u32 = 0x01000000 ; // T0CCP0 Pin Present
pub const SYSCTL_DC3_ADC0AIN7: u32 = 0x00800000 ; // ADC Module 0 AIN7 Pin Present
pub const SYSCTL_DC3_ADC0AIN6: u32 = 0x00400000 ; // ADC Module 0 AIN6 Pin Present
pub const SYSCTL_DC3_ADC0AIN5: u32 = 0x00200000 ; // ADC Module 0 AIN5 Pin Present
pub const SYSCTL_DC3_ADC0AIN4: u32 = 0x00100000 ; // ADC Module 0 AIN4 Pin Present
pub const SYSCTL_DC3_ADC0AIN3: u32 = 0x00080000 ; // ADC Module 0 AIN3 Pin Present
pub const SYSCTL_DC3_ADC0AIN2: u32 = 0x00040000 ; // ADC Module 0 AIN2 Pin Present
pub const SYSCTL_DC3_ADC0AIN1: u32 = 0x00020000 ; // ADC Module 0 AIN1 Pin Present
pub const SYSCTL_DC3_ADC0AIN0: u32 = 0x00010000 ; // ADC Module 0 AIN0 Pin Present
pub const SYSCTL_DC3_PWMFAULT: u32 = 0x00008000 ; // PWM Fault Pin Present
pub const SYSCTL_DC3_C2O: u32 = 0x00004000 ; // C2o Pin Present
pub const SYSCTL_DC3_C2PLUS: u32 = 0x00002000 ; // C2+ Pin Present
pub const SYSCTL_DC3_C2MINUS: u32 = 0x00001000 ; // C2- Pin Present
pub const SYSCTL_DC3_C1O: u32 = 0x00000800 ; // C1o Pin Present
pub const SYSCTL_DC3_C1PLUS: u32 = 0x00000400 ; // C1+ Pin Present
pub const SYSCTL_DC3_C1MINUS: u32 = 0x00000200 ; // C1- Pin Present
pub const SYSCTL_DC3_C0O: u32 = 0x00000100 ; // C0o Pin Present
pub const SYSCTL_DC3_C0PLUS: u32 = 0x00000080 ; // C0+ Pin Present
pub const SYSCTL_DC3_C0MINUS: u32 = 0x00000040 ; // C0- Pin Present
pub const SYSCTL_DC3_PWM5: u32 = 0x00000020 ; // PWM5 Pin Present
pub const SYSCTL_DC3_PWM4: u32 = 0x00000010 ; // PWM4 Pin Present
pub const SYSCTL_DC3_PWM3: u32 = 0x00000008 ; // PWM3 Pin Present
pub const SYSCTL_DC3_PWM2: u32 = 0x00000004 ; // PWM2 Pin Present
pub const SYSCTL_DC3_PWM1: u32 = 0x00000002 ; // PWM1 Pin Present
pub const SYSCTL_DC3_PWM0: u32 = 0x00000001 ; // PWM0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC4 register.
//
//*****************************************************************************
pub const SYSCTL_DC4_EPHY0: u32 = 0x40000000 ; // Ethernet PHY Layer 0 Present
pub const SYSCTL_DC4_EMAC0: u32 = 0x10000000 ; // Ethernet MAC Layer 0 Present
pub const SYSCTL_DC4_E1588: u32 = 0x01000000 ; // 1588 Capable
pub const SYSCTL_DC4_PICAL: u32 = 0x00040000 ; // PIOSC Calibrate
pub const SYSCTL_DC4_CCP7: u32 = 0x00008000 ; // T3CCP1 Pin Present
pub const SYSCTL_DC4_CCP6: u32 = 0x00004000 ; // T3CCP0 Pin Present
pub const SYSCTL_DC4_UDMA: u32 = 0x00002000 ; // Micro-DMA Module Present
pub const SYSCTL_DC4_ROM: u32 = 0x00001000 ; // Internal Code ROM Present
pub const SYSCTL_DC4_GPIOJ: u32 = 0x00000100 ; // GPIO Port J Present
pub const SYSCTL_DC4_GPIOH: u32 = 0x00000080 ; // GPIO Port H Present
pub const SYSCTL_DC4_GPIOG: u32 = 0x00000040 ; // GPIO Port G Present
pub const SYSCTL_DC4_GPIOF: u32 = 0x00000020 ; // GPIO Port F Present
pub const SYSCTL_DC4_GPIOE: u32 = 0x00000010 ; // GPIO Port E Present
pub const SYSCTL_DC4_GPIOD: u32 = 0x00000008 ; // GPIO Port D Present
pub const SYSCTL_DC4_GPIOC: u32 = 0x00000004 ; // GPIO Port C Present
pub const SYSCTL_DC4_GPIOB: u32 = 0x00000002 ; // GPIO Port B Present
pub const SYSCTL_DC4_GPIOA: u32 = 0x00000001 ; // GPIO Port A Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC5 register.
//
//*****************************************************************************
pub const SYSCTL_DC5_PWMFAULT3: u32 = 0x08000000 ; // PWM Fault 3 Pin Present
pub const SYSCTL_DC5_PWMFAULT2: u32 = 0x04000000 ; // PWM Fault 2 Pin Present
pub const SYSCTL_DC5_PWMFAULT1: u32 = 0x02000000 ; // PWM Fault 1 Pin Present
pub const SYSCTL_DC5_PWMFAULT0: u32 = 0x01000000 ; // PWM Fault 0 Pin Present
pub const SYSCTL_DC5_PWMEFLT: u32 = 0x00200000 ; // PWM Extended Fault Active
pub const SYSCTL_DC5_PWMESYNC: u32 = 0x00100000 ; // PWM Extended SYNC Active
pub const SYSCTL_DC5_PWM7: u32 = 0x00000080 ; // PWM7 Pin Present
pub const SYSCTL_DC5_PWM6: u32 = 0x00000040 ; // PWM6 Pin Present
pub const SYSCTL_DC5_PWM5: u32 = 0x00000020 ; // PWM5 Pin Present
pub const SYSCTL_DC5_PWM4: u32 = 0x00000010 ; // PWM4 Pin Present
pub const SYSCTL_DC5_PWM3: u32 = 0x00000008 ; // PWM3 Pin Present
pub const SYSCTL_DC5_PWM2: u32 = 0x00000004 ; // PWM2 Pin Present
pub const SYSCTL_DC5_PWM1: u32 = 0x00000002 ; // PWM1 Pin Present
pub const SYSCTL_DC5_PWM0: u32 = 0x00000001 ; // PWM0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC6 register.
//
//*****************************************************************************
pub const SYSCTL_DC6_USB0PHY: u32 = 0x00000010 ; // USB Module 0 PHY Present
pub const SYSCTL_DC6_USB0_M: u32 = 0x00000003 ; // USB Module 0 Present
pub const SYSCTL_DC6_USB0_DEV: u32 = 0x00000001 ; // USB0 is Device Only
pub const SYSCTL_DC6_USB0_HOSTDEV: u32 = 0x00000002 ; // USB is Device or Host
pub const SYSCTL_DC6_USB0_OTG: u32 = 0x00000003 ; // USB0 is OTG

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC7 register.
//
//*****************************************************************************
pub const SYSCTL_DC7_DMACH30: u32 = 0x40000000 ; // DMA Channel 30
pub const SYSCTL_DC7_DMACH29: u32 = 0x20000000 ; // DMA Channel 29
pub const SYSCTL_DC7_DMACH28: u32 = 0x10000000 ; // DMA Channel 28
pub const SYSCTL_DC7_DMACH27: u32 = 0x08000000 ; // DMA Channel 27
pub const SYSCTL_DC7_DMACH26: u32 = 0x04000000 ; // DMA Channel 26
pub const SYSCTL_DC7_DMACH25: u32 = 0x02000000 ; // DMA Channel 25
pub const SYSCTL_DC7_DMACH24: u32 = 0x01000000 ; // DMA Channel 24
pub const SYSCTL_DC7_DMACH23: u32 = 0x00800000 ; // DMA Channel 23
pub const SYSCTL_DC7_DMACH22: u32 = 0x00400000 ; // DMA Channel 22
pub const SYSCTL_DC7_DMACH21: u32 = 0x00200000 ; // DMA Channel 21
pub const SYSCTL_DC7_DMACH20: u32 = 0x00100000 ; // DMA Channel 20
pub const SYSCTL_DC7_DMACH19: u32 = 0x00080000 ; // DMA Channel 19
pub const SYSCTL_DC7_DMACH18: u32 = 0x00040000 ; // DMA Channel 18
pub const SYSCTL_DC7_DMACH17: u32 = 0x00020000 ; // DMA Channel 17
pub const SYSCTL_DC7_DMACH16: u32 = 0x00010000 ; // DMA Channel 16
pub const SYSCTL_DC7_DMACH15: u32 = 0x00008000 ; // DMA Channel 15
pub const SYSCTL_DC7_DMACH14: u32 = 0x00004000 ; // DMA Channel 14
pub const SYSCTL_DC7_DMACH13: u32 = 0x00002000 ; // DMA Channel 13
pub const SYSCTL_DC7_DMACH12: u32 = 0x00001000 ; // DMA Channel 12
pub const SYSCTL_DC7_DMACH11: u32 = 0x00000800 ; // DMA Channel 11
pub const SYSCTL_DC7_DMACH10: u32 = 0x00000400 ; // DMA Channel 10
pub const SYSCTL_DC7_DMACH9: u32 = 0x00000200 ; // DMA Channel 9
pub const SYSCTL_DC7_DMACH8: u32 = 0x00000100 ; // DMA Channel 8
pub const SYSCTL_DC7_DMACH7: u32 = 0x00000080 ; // DMA Channel 7
pub const SYSCTL_DC7_DMACH6: u32 = 0x00000040 ; // DMA Channel 6
pub const SYSCTL_DC7_DMACH5: u32 = 0x00000020 ; // DMA Channel 5
pub const SYSCTL_DC7_DMACH4: u32 = 0x00000010 ; // DMA Channel 4
pub const SYSCTL_DC7_DMACH3: u32 = 0x00000008 ; // DMA Channel 3
pub const SYSCTL_DC7_DMACH2: u32 = 0x00000004 ; // DMA Channel 2
pub const SYSCTL_DC7_DMACH1: u32 = 0x00000002 ; // DMA Channel 1
pub const SYSCTL_DC7_DMACH0: u32 = 0x00000001 ; // DMA Channel 0

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC8 register.
//
//*****************************************************************************
pub const SYSCTL_DC8_ADC1AIN15: u32 = 0x80000000 ; // ADC Module 1 AIN15 Pin Present
pub const SYSCTL_DC8_ADC1AIN14: u32 = 0x40000000 ; // ADC Module 1 AIN14 Pin Present
pub const SYSCTL_DC8_ADC1AIN13: u32 = 0x20000000 ; // ADC Module 1 AIN13 Pin Present
pub const SYSCTL_DC8_ADC1AIN12: u32 = 0x10000000 ; // ADC Module 1 AIN12 Pin Present
pub const SYSCTL_DC8_ADC1AIN11: u32 = 0x08000000 ; // ADC Module 1 AIN11 Pin Present
pub const SYSCTL_DC8_ADC1AIN10: u32 = 0x04000000 ; // ADC Module 1 AIN10 Pin Present
pub const SYSCTL_DC8_ADC1AIN9: u32 = 0x02000000 ; // ADC Module 1 AIN9 Pin Present
pub const SYSCTL_DC8_ADC1AIN8: u32 = 0x01000000 ; // ADC Module 1 AIN8 Pin Present
pub const SYSCTL_DC8_ADC1AIN7: u32 = 0x00800000 ; // ADC Module 1 AIN7 Pin Present
pub const SYSCTL_DC8_ADC1AIN6: u32 = 0x00400000 ; // ADC Module 1 AIN6 Pin Present
pub const SYSCTL_DC8_ADC1AIN5: u32 = 0x00200000 ; // ADC Module 1 AIN5 Pin Present
pub const SYSCTL_DC8_ADC1AIN4: u32 = 0x00100000 ; // ADC Module 1 AIN4 Pin Present
pub const SYSCTL_DC8_ADC1AIN3: u32 = 0x00080000 ; // ADC Module 1 AIN3 Pin Present
pub const SYSCTL_DC8_ADC1AIN2: u32 = 0x00040000 ; // ADC Module 1 AIN2 Pin Present
pub const SYSCTL_DC8_ADC1AIN1: u32 = 0x00020000 ; // ADC Module 1 AIN1 Pin Present
pub const SYSCTL_DC8_ADC1AIN0: u32 = 0x00010000 ; // ADC Module 1 AIN0 Pin Present
pub const SYSCTL_DC8_ADC0AIN15: u32 = 0x00008000 ; // ADC Module 0 AIN15 Pin Present
pub const SYSCTL_DC8_ADC0AIN14: u32 = 0x00004000 ; // ADC Module 0 AIN14 Pin Present
pub const SYSCTL_DC8_ADC0AIN13: u32 = 0x00002000 ; // ADC Module 0 AIN13 Pin Present
pub const SYSCTL_DC8_ADC0AIN12: u32 = 0x00001000 ; // ADC Module 0 AIN12 Pin Present
pub const SYSCTL_DC8_ADC0AIN11: u32 = 0x00000800 ; // ADC Module 0 AIN11 Pin Present
pub const SYSCTL_DC8_ADC0AIN10: u32 = 0x00000400 ; // ADC Module 0 AIN10 Pin Present
pub const SYSCTL_DC8_ADC0AIN9: u32 = 0x00000200 ; // ADC Module 0 AIN9 Pin Present
pub const SYSCTL_DC8_ADC0AIN8: u32 = 0x00000100 ; // ADC Module 0 AIN8 Pin Present
pub const SYSCTL_DC8_ADC0AIN7: u32 = 0x00000080 ; // ADC Module 0 AIN7 Pin Present
pub const SYSCTL_DC8_ADC0AIN6: u32 = 0x00000040 ; // ADC Module 0 AIN6 Pin Present
pub const SYSCTL_DC8_ADC0AIN5: u32 = 0x00000020 ; // ADC Module 0 AIN5 Pin Present
pub const SYSCTL_DC8_ADC0AIN4: u32 = 0x00000010 ; // ADC Module 0 AIN4 Pin Present
pub const SYSCTL_DC8_ADC0AIN3: u32 = 0x00000008 ; // ADC Module 0 AIN3 Pin Present
pub const SYSCTL_DC8_ADC0AIN2: u32 = 0x00000004 ; // ADC Module 0 AIN2 Pin Present
pub const SYSCTL_DC8_ADC0AIN1: u32 = 0x00000002 ; // ADC Module 0 AIN1 Pin Present
pub const SYSCTL_DC8_ADC0AIN0: u32 = 0x00000001 ; // ADC Module 0 AIN0 Pin Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PBORCTL register.
//
//*****************************************************************************
pub const SYSCTL_PBORCTL_BOR0: u32 = 0x00000004 ; // VDD under BOR0 Event Action
pub const SYSCTL_PBORCTL_BOR1: u32 = 0x00000002 ; // VDD under BOR1 Event Action

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PTBOCTL register.
//
//*****************************************************************************
pub const  SYSCTL_PTBOCTL_VDDA_UBOR_M: u32 =    0x00000300 ; // VDDA under BOR Event Action
pub const  SYSCTL_PTBOCTL_VDDA_UBOR_NONE: u32 =    0x00000000 ; // No Action
pub const  SYSCTL_PTBOCTL_VDDA_UBOR_SYSINT: u32 =    0x00000100 ; // System control interrupt
pub const  SYSCTL_PTBOCTL_VDDA_UBOR_NMI: u32 =    0x00000200 ; // NMI
pub const  SYSCTL_PTBOCTL_VDDA_UBOR_RST: u32 =    0x00000300 ; // Reset
pub const  SYSCTL_PTBOCTL_VDD_UBOR_M: u32 =    0x00000003 ; // VDD (VDDS) under BOR Event
// Action
pub const  SYSCTL_PTBOCTL_VDD_UBOR_NONE: u32 =    0x00000000 ; // No Action
pub const  SYSCTL_PTBOCTL_VDD_UBOR_SYSINT: u32 =    0x00000001 ; // System control interrupt
pub const  SYSCTL_PTBOCTL_VDD_UBOR_NMI: u32 =    0x00000002 ; // NMI
pub const  SYSCTL_PTBOCTL_VDD_UBOR_RST: u32 =    0x00000003 ; // Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR0 register.
//
//*****************************************************************************
pub const SYSCTL_SRCR0_WDT1: u32 = 0x10000000 ; // WDT1 Reset Control
pub const SYSCTL_SRCR0_CAN1: u32 = 0x02000000 ; // CAN1 Reset Control
pub const SYSCTL_SRCR0_CAN0: u32 = 0x01000000 ; // CAN0 Reset Control
pub const SYSCTL_SRCR0_PWM0: u32 = 0x00100000 ; // PWM Reset Control
pub const SYSCTL_SRCR0_ADC1: u32 = 0x00020000 ; // ADC1 Reset Control
pub const SYSCTL_SRCR0_ADC0: u32 = 0x00010000 ; // ADC0 Reset Control
pub const SYSCTL_SRCR0_HIB: u32 = 0x00000040 ; // HIB Reset Control
pub const SYSCTL_SRCR0_WDT0: u32 = 0x00000008 ; // WDT0 Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR1 register.
//
//*****************************************************************************
pub const SYSCTL_SRCR1_COMP2: u32 = 0x04000000 ; // Analog Comp 2 Reset Control
pub const SYSCTL_SRCR1_COMP1: u32 = 0x02000000 ; // Analog Comp 1 Reset Control
pub const SYSCTL_SRCR1_COMP0: u32 = 0x01000000 ; // Analog Comp 0 Reset Control
pub const SYSCTL_SRCR1_TIMER3: u32 = 0x00080000 ; // Timer 3 Reset Control
pub const SYSCTL_SRCR1_TIMER2: u32 = 0x00040000 ; // Timer 2 Reset Control
pub const SYSCTL_SRCR1_TIMER1: u32 = 0x00020000 ; // Timer 1 Reset Control
pub const SYSCTL_SRCR1_TIMER0: u32 = 0x00010000 ; // Timer 0 Reset Control
pub const SYSCTL_SRCR1_I2C1: u32 = 0x00004000 ; // I2C1 Reset Control
pub const SYSCTL_SRCR1_I2C0: u32 = 0x00001000 ; // I2C0 Reset Control
pub const SYSCTL_SRCR1_QEI1: u32 = 0x00000200 ; // QEI1 Reset Control
pub const SYSCTL_SRCR1_QEI0: u32 = 0x00000100 ; // QEI0 Reset Control
pub const SYSCTL_SRCR1_SSI1: u32 = 0x00000020 ; // SSI1 Reset Control
pub const SYSCTL_SRCR1_SSI0: u32 = 0x00000010 ; // SSI0 Reset Control
pub const SYSCTL_SRCR1_UART2: u32 = 0x00000004 ; // UART2 Reset Control
pub const SYSCTL_SRCR1_UART1: u32 = 0x00000002 ; // UART1 Reset Control
pub const SYSCTL_SRCR1_UART0: u32 = 0x00000001 ; // UART0 Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCR2 register.
//
//*****************************************************************************
pub const SYSCTL_SRCR2_USB0: u32 = 0x00010000 ; // USB0 Reset Control
pub const SYSCTL_SRCR2_UDMA: u32 = 0x00002000 ; // Micro-DMA Reset Control
pub const SYSCTL_SRCR2_GPIOJ: u32 = 0x00000100 ; // Port J Reset Control
pub const SYSCTL_SRCR2_GPIOH: u32 = 0x00000080 ; // Port H Reset Control
pub const SYSCTL_SRCR2_GPIOG: u32 = 0x00000040 ; // Port G Reset Control
pub const SYSCTL_SRCR2_GPIOF: u32 = 0x00000020 ; // Port F Reset Control
pub const SYSCTL_SRCR2_GPIOE: u32 = 0x00000010 ; // Port E Reset Control
pub const SYSCTL_SRCR2_GPIOD: u32 = 0x00000008 ; // Port D Reset Control
pub const SYSCTL_SRCR2_GPIOC: u32 = 0x00000004 ; // Port C Reset Control
pub const SYSCTL_SRCR2_GPIOB: u32 = 0x00000002 ; // Port B Reset Control
pub const SYSCTL_SRCR2_GPIOA: u32 = 0x00000001 ; // Port A Reset Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RIS register.
//
//*****************************************************************************
pub const SYSCTL_RIS_BOR0RIS: u32 = 0x00000800 ; // VDD under BOR0 Raw Interrupt
// Status
pub const SYSCTL_RIS_VDDARIS: u32 = 0x00000400 ; // VDDA Power OK Event Raw
// Interrupt Status
pub const SYSCTL_RIS_MOSCPUPRIS: u32 = 0x00000100 ; // MOSC Power Up Raw Interrupt
// Status
pub const SYSCTL_RIS_USBPLLLRIS: u32 = 0x00000080 ; // USB PLL Lock Raw Interrupt
// Status
pub const SYSCTL_RIS_PLLLRIS: u32 = 0x00000040 ; // PLL Lock Raw Interrupt Status
pub const SYSCTL_RIS_MOFRIS: u32 = 0x00000008 ; // Main Oscillator Failure Raw
// Interrupt Status
pub const SYSCTL_RIS_BOR1RIS: u32 = 0x00000002 ; // VDD under BOR1 Raw Interrupt
// Status
pub const SYSCTL_RIS_BORRIS: u32 = 0x00000002 ; // Brown-Out Reset Raw Interrupt
// Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_IMC register.
//
//*****************************************************************************
pub const SYSCTL_IMC_BOR0IM: u32 = 0x00000800 ; // VDD under BOR0 Interrupt Mask
pub const SYSCTL_IMC_VDDAIM: u32 = 0x00000400 ; // VDDA Power OK Interrupt Mask
pub const SYSCTL_IMC_MOSCPUPIM: u32 = 0x00000100 ; // MOSC Power Up Interrupt Mask
pub const SYSCTL_IMC_USBPLLLIM: u32 = 0x00000080 ; // USB PLL Lock Interrupt Mask
pub const SYSCTL_IMC_PLLLIM: u32 = 0x00000040 ; // PLL Lock Interrupt Mask
pub const SYSCTL_IMC_MOFIM: u32 = 0x00000008 ; // Main Oscillator Failure
// Interrupt Mask
pub const SYSCTL_IMC_BORIM: u32 = 0x00000002 ; // Brown-Out Reset Interrupt Mask
pub const SYSCTL_IMC_BOR1IM: u32 = 0x00000002 ; // VDD under BOR1 Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MISC register.
//
//*****************************************************************************
pub const SYSCTL_MISC_BOR0MIS: u32 = 0x00000800 ; // VDD under BOR0 Masked Interrupt
// Status
pub const SYSCTL_MISC_VDDAMIS: u32 = 0x00000400 ; // VDDA Power OK Masked Interrupt
// Status
pub const SYSCTL_MISC_MOSCPUPMIS: u32 = 0x00000100 ; // MOSC Power Up Masked Interrupt
// Status
pub const SYSCTL_MISC_USBPLLLMIS: u32 = 0x00000080 ; // USB PLL Lock Masked Interrupt
// Status
pub const SYSCTL_MISC_PLLLMIS: u32 = 0x00000040 ; // PLL Lock Masked Interrupt Status
pub const SYSCTL_MISC_MOFMIS: u32 = 0x00000008 ; // Main Oscillator Failure Masked
// Interrupt Status
pub const SYSCTL_MISC_BORMIS: u32 = 0x00000002 ; // BOR Masked Interrupt Status
pub const SYSCTL_MISC_BOR1MIS: u32 = 0x00000002 ; // VDD under BOR1 Masked Interrupt
// Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESC register.
//
//*****************************************************************************
pub const SYSCTL_RESC_MOSCFAIL: u32 = 0x00010000 ; // MOSC Failure Reset
pub const SYSCTL_RESC_HSSR: u32 = 0x00001000 ; // HSSR Reset
pub const SYSCTL_RESC_HIB: u32 = 0x00000040 ; // HIB Reset
pub const SYSCTL_RESC_WDT1: u32 = 0x00000020 ; // Watchdog Timer 1 Reset
pub const SYSCTL_RESC_SW: u32 = 0x00000010 ; // Software Reset
pub const SYSCTL_RESC_WDT0: u32 = 0x00000008 ; // Watchdog Timer 0 Reset
pub const SYSCTL_RESC_BOR: u32 = 0x00000004 ; // Brown-Out Reset
pub const SYSCTL_RESC_POR: u32 = 0x00000002 ; // Power-On Reset
pub const SYSCTL_RESC_EXT: u32 = 0x00000001 ; // External Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PWRTC register.
//
//*****************************************************************************
pub const SYSCTL_PWRTC_VDDA_UBOR: u32 = 0x00000010 ; // VDDA Under BOR Status
pub const SYSCTL_PWRTC_VDD_UBOR: u32 = 0x00000001 ; // VDD Under BOR Status

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC register.
//
//*****************************************************************************
pub const SYSCTL_RCC_ACG: u32 = 0x08000000 ; // Auto Clock Gating
pub const SYSCTL_RCC_SYSDIV_M: u32 = 0x07800000 ; // System Clock Divisor
pub const SYSCTL_RCC_USESYSDIV: u32 = 0x00400000 ; // Enable System Clock Divider
pub const SYSCTL_RCC_USEPWMDIV: u32 = 0x00100000 ; // Enable PWM Clock Divisor
pub const SYSCTL_RCC_PWMDIV_M: u32 = 0x000E0000 ; // PWM Unit Clock Divisor
pub const SYSCTL_RCC_PWMDIV_2: u32 = 0x00000000 ; // PWM clock /2
pub const SYSCTL_RCC_PWMDIV_4: u32 = 0x00020000 ; // PWM clock /4
pub const SYSCTL_RCC_PWMDIV_8: u32 = 0x00040000 ; // PWM clock /8
pub const SYSCTL_RCC_PWMDIV_16: u32 = 0x00060000 ; // PWM clock /16
pub const SYSCTL_RCC_PWMDIV_32: u32 = 0x00080000 ; // PWM clock /32
pub const SYSCTL_RCC_PWMDIV_64: u32 = 0x000A0000 ; // PWM clock /64
pub const SYSCTL_RCC_PWRDN: u32 = 0x00002000 ; // PLL Power Down
pub const SYSCTL_RCC_BYPASS: u32 = 0x00000800 ; // PLL Bypass
pub const SYSCTL_RCC_XTAL_M: u32 = 0x000007C0 ; // Crystal Value
pub const SYSCTL_RCC_XTAL_4MHZ: u32 = 0x00000180 ; // 4 MHz
pub const SYSCTL_RCC_XTAL_4_09MHZ: u32 = 0x000001C0 ; // 4.096 MHz
pub const SYSCTL_RCC_XTAL_4_91MHZ: u32 = 0x00000200 ; // 4.9152 MHz
pub const SYSCTL_RCC_XTAL_5MHZ: u32 = 0x00000240 ; // 5 MHz
pub const SYSCTL_RCC_XTAL_5_12MHZ: u32 = 0x00000280 ; // 5.12 MHz
pub const SYSCTL_RCC_XTAL_6MHZ: u32 = 0x000002C0 ; // 6 MHz
pub const SYSCTL_RCC_XTAL_6_14MHZ: u32 = 0x00000300 ; // 6.144 MHz
pub const SYSCTL_RCC_XTAL_7_37MHZ: u32 = 0x00000340 ; // 7.3728 MHz
pub const SYSCTL_RCC_XTAL_8MHZ: u32 = 0x00000380 ; // 8 MHz
pub const SYSCTL_RCC_XTAL_8_19MHZ: u32 = 0x000003C0 ; // 8.192 MHz
pub const SYSCTL_RCC_XTAL_10MHZ: u32 = 0x00000400 ; // 10 MHz
pub const SYSCTL_RCC_XTAL_12MHZ: u32 = 0x00000440 ; // 12 MHz
pub const SYSCTL_RCC_XTAL_12_2MHZ: u32 = 0x00000480 ; // 12.288 MHz
pub const SYSCTL_RCC_XTAL_13_5MHZ: u32 = 0x000004C0 ; // 13.56 MHz
pub const SYSCTL_RCC_XTAL_14_3MHZ: u32 = 0x00000500 ; // 14.31818 MHz
pub const SYSCTL_RCC_XTAL_16MHZ: u32 = 0x00000540 ; // 16 MHz
pub const SYSCTL_RCC_XTAL_16_3MHZ: u32 = 0x00000580 ; // 16.384 MHz
pub const SYSCTL_RCC_XTAL_18MHZ: u32 = 0x000005C0 ; // 18.0 MHz (USB)
pub const SYSCTL_RCC_XTAL_20MHZ: u32 = 0x00000600 ; // 20.0 MHz (USB)
pub const SYSCTL_RCC_XTAL_24MHZ: u32 = 0x00000640 ; // 24.0 MHz (USB)
pub const SYSCTL_RCC_XTAL_25MHZ: u32 = 0x00000680 ; // 25.0 MHz (USB)
pub const SYSCTL_RCC_OSCSRC_M: u32 = 0x00000030 ; // Oscillator Source
pub const SYSCTL_RCC_OSCSRC_MAIN: u32 = 0x00000000 ; // MOSC
pub const SYSCTL_RCC_OSCSRC_INT: u32 = 0x00000010 ; // IOSC
pub const SYSCTL_RCC_OSCSRC_INT4: u32 = 0x00000020 ; // IOSC/4
pub const SYSCTL_RCC_OSCSRC_30: u32 = 0x00000030 ; // LFIOSC
pub const SYSCTL_RCC_MOSCDIS: u32 = 0x00000001 ; // Main Oscillator Disable
pub const SYSCTL_RCC_SYSDIV_S: u32 =     23;
pub const SYSCTL_RCC_XTAL_S: u32 = 6          ; // Shift to the XTAL field

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_NMIC register.
//
//*****************************************************************************
pub const SYSCTL_NMIC_MOSCFAIL: u32 = 0x00010000 ; // MOSC Failure NMI
pub const SYSCTL_NMIC_TAMPER: u32 = 0x00000200 ; // Tamper Event NMI
pub const SYSCTL_NMIC_WDT1: u32 = 0x00000020 ; // Watch Dog Timer (WDT) 1 NMI
pub const SYSCTL_NMIC_WDT0: u32 = 0x00000008 ; // Watch Dog Timer (WDT) 0 NMI
pub const SYSCTL_NMIC_POWER: u32 = 0x00000004 ; // Power/Brown Out Event NMI
pub const SYSCTL_NMIC_EXTERNAL: u32 = 0x00000001 ; // External Pin NMI

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_GPIOHBCTL
// register.
//
//*****************************************************************************
pub const SYSCTL_GPIOHBCTL_PORTJ: u32 = 0x00000100 ; // Port J Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTH: u32 = 0x00000080 ; // Port H Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTG: u32 = 0x00000040 ; // Port G Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTF: u32 = 0x00000020 ; // Port F Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTE: u32 = 0x00000010 ; // Port E Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTD: u32 = 0x00000008 ; // Port D Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTC: u32 = 0x00000004 ; // Port C Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTB: u32 = 0x00000002 ; // Port B Advanced High-Performance
// Bus
pub const SYSCTL_GPIOHBCTL_PORTA: u32 = 0x00000001 ; // Port A Advanced High-Performance
// Bus

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCC2 register.
//
//*****************************************************************************
pub const SYSCTL_RCC2_USERCC2: u32 = 0x80000000 ; // Use RCC2
pub const SYSCTL_RCC2_DIV400: u32 = 0x40000000 ; // Divide PLL as 400 MHz vs. 200
// MHz
pub const SYSCTL_RCC2_SYSDIV2_M: u32 = 0x1F800000 ; // System Clock Divisor 2
pub const SYSCTL_RCC2_SYSDIV2LSB: u32 = 0x00400000 ; // Additional LSB for SYSDIV2
pub const SYSCTL_RCC2_USBPWRDN: u32 = 0x00004000 ; // Power-Down USB PLL
pub const SYSCTL_RCC2_PWRDN2: u32 = 0x00002000 ; // Power-Down PLL 2
pub const SYSCTL_RCC2_BYPASS2: u32 = 0x00000800 ; // PLL Bypass 2
pub const SYSCTL_RCC2_OSCSRC2_M: u32 = 0x00000070 ; // Oscillator Source 2
pub const SYSCTL_RCC2_OSCSRC2_MO: u32 = 0x00000000 ; // MOSC
pub const SYSCTL_RCC2_OSCSRC2_IO: u32 = 0x00000010 ; // PIOSC
pub const SYSCTL_RCC2_OSCSRC2_IO4: u32 = 0x00000020 ; // PIOSC/4
pub const SYSCTL_RCC2_OSCSRC2_30: u32 = 0x00000030 ; // LFIOSC
pub const SYSCTL_RCC2_OSCSRC2_32: u32 = 0x00000070 ; // 32.768 kHz
pub const SYSCTL_RCC2_SYSDIV2_S: u32 =       2;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MOSCCTL register.
//
//*****************************************************************************
pub const SYSCTL_MOSCCTL_OSCRNG: u32 = 0x00000010 ; // Oscillator Range
pub const SYSCTL_MOSCCTL_PWRDN: u32 = 0x00000008 ; // Power Down
pub const SYSCTL_MOSCCTL_NOXTAL: u32 = 0x00000004 ; // No Crystal Connected
pub const SYSCTL_MOSCCTL_MOSCIM: u32 = 0x00000002 ; // MOSC Failure Action
pub const SYSCTL_MOSCCTL_CVAL: u32 = 0x00000001 ; // Clock Validation for MOSC

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RSCLKCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_RSCLKCFG_MEMTIMU: u32 = 0x80000000 ; // Memory Timing Register Update
pub const SYSCTL_RSCLKCFG_NEWFREQ: u32 = 0x40000000 ; // New PLLFREQ Accept
pub const SYSCTL_RSCLKCFG_ACG: u32 = 0x20000000 ; // Auto Clock Gating
pub const SYSCTL_RSCLKCFG_USEPLL: u32 = 0x10000000 ; // Use PLL
pub const SYSCTL_RSCLKCFG_PLLSRC_M: u32 =    0x0F000000;  // PLL Source
pub const  SYSCTL_RSCLKCFG_PLLSRC_PIOSC: u32 = 0x00000000;  // PIOSC is PLL input clock source
pub const  SYSCTL_RSCLKCFG_PLLSRC_MOSC: u32 = 0x03000000;  // MOSC is the PLL input clock
// source
pub const  SYSCTL_RSCLKCFG_OSCSRC_M: u32 =     0x00F00000;  // Oscillator Source
pub const  SYSCTL_RSCLKCFG_OSCSRC_PIOSC: u32 =  0x00000000;  // PIOSC is oscillator source
pub const  SYSCTL_RSCLKCFG_OSCSRC_LFIOSC: u32 = 0x00200000;  // LFIOSC is oscillator source
pub const  SYSCTL_RSCLKCFG_OSCSRC_MOSC: u32 =     0x00300000;  // MOSC is oscillator source
pub const  SYSCTL_RSCLKCFG_OSCSRC_RTC: u32 =     0x00400000;  // Hibernation Module RTC
// Oscillator (RTCOSC)
pub const  SYSCTL_RSCLKCFG_OSYSDIV_M : u32 =  0x000FFC00 ; // Oscillator System Clock Divisor
pub const  SYSCTL_RSCLKCFG_PSYSDIV_M : u32 =  0x000003FF;  // PLL System Clock Divisor
pub const  SYSCTL_RSCLKCFG_OSYSDIV_S : u32 =  10;
pub const  SYSCTL_RSCLKCFG_PSYSDIV_S : u32 = 0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_MEMTIM0 register.
//
//*****************************************************************************
pub const SYSCTL_MEMTIM0_EBCHT_M: u32 = 0x03C00000 ; // EEPROM Clock High Time
pub const  SYSCTL_MEMTIM0_EBCHT_0_5: u32 =    0x00000000 ; // 1/2 system clock period
pub const SYSCTL_MEMTIM0_EBCHT_1: u32 = 0x00400000 ; // 1 system clock period
pub const  SYSCTL_MEMTIM0_EBCHT_1_5: u32 =    0x00800000 ; // 1.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_2: u32 = 0x00C00000 ; // 2 system clock periods
pub const  SYSCTL_MEMTIM0_EBCHT_2_5: u32 =    0x01000000 ; // 2.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_3: u32 = 0x01400000 ; // 3 system clock periods
pub const  SYSCTL_MEMTIM0_EBCHT_3_5: u32 =    0x01800000 ; // 3.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCHT_4: u32 = 0x01C00000 ; // 4 system clock periods
pub const  SYSCTL_MEMTIM0_EBCHT_4_5: u32 =    0x02000000 ; // 4.5 system clock periods
pub const SYSCTL_MEMTIM0_EBCE: u32 = 0x00200000 ; // EEPROM Bank Clock Edge
pub const SYSCTL_MEMTIM0_MB1: u32 = 0x00100010 ; // Must be one
pub const SYSCTL_MEMTIM0_EWS_M: u32 = 0x000F0000 ; // EEPROM Wait States
pub const SYSCTL_MEMTIM0_FBCHT_M: u32 = 0x000003C0 ; // Flash Bank Clock High Time
pub const  SYSCTL_MEMTIM0_FBCHT_0_5: u32 =    0x00000000 ; // 1/2 system clock period
pub const SYSCTL_MEMTIM0_FBCHT_1: u32 = 0x00000040 ; // 1 system clock period
pub const  SYSCTL_MEMTIM0_FBCHT_1_5: u32 =    0x00000080 ; // 1.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_2: u32 = 0x000000C0 ; // 2 system clock periods
pub const  SYSCTL_MEMTIM0_FBCHT_2_5: u32 =    0x00000100 ; // 2.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_3: u32 = 0x00000140 ; // 3 system clock periods
pub const  SYSCTL_MEMTIM0_FBCHT_3_5: u32 =    0x00000180 ; // 3.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCHT_4: u32 = 0x000001C0 ; // 4 system clock periods
pub const  SYSCTL_MEMTIM0_FBCHT_4_5: u32 =    0x00000200 ; // 4.5 system clock periods
pub const SYSCTL_MEMTIM0_FBCE: u32 = 0x00000020 ; // Flash Bank Clock Edge
pub const SYSCTL_MEMTIM0_FWS_M: u32 = 0x0000000F ; // Flash Wait State
pub const  SYSCTL_MEMTIM0_EWS_S: u32 =        16;
pub const  SYSCTL_MEMTIM0_FWS_S: u32 =        0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC0 register.
//
//*****************************************************************************
pub const SYSCTL_RCGC0_WDT1: u32 = 0x10000000 ; // WDT1 Clock Gating Control
pub const SYSCTL_RCGC0_CAN1: u32 = 0x02000000 ; // CAN1 Clock Gating Control
pub const SYSCTL_RCGC0_CAN0: u32 = 0x01000000 ; // CAN0 Clock Gating Control
pub const SYSCTL_RCGC0_PWM0: u32 = 0x00100000 ; // PWM Clock Gating Control
pub const SYSCTL_RCGC0_ADC1: u32 = 0x00020000 ; // ADC1 Clock Gating Control
pub const SYSCTL_RCGC0_ADC0: u32 = 0x00010000 ; // ADC0 Clock Gating Control
pub const SYSCTL_RCGC0_ADC1SPD_M: u32 = 0x00000C00 ; // ADC1 Sample Speed
pub const  SYSCTL_RCGC0_ADC1SPD_125K: u32 =    0x00000000 ; // 125K samples/second
pub const  SYSCTL_RCGC0_ADC1SPD_250K: u32 =    0x00000400 ; // 250K samples/second
pub const  SYSCTL_RCGC0_ADC1SPD_500K: u32 =    0x00000800 ; // 500K samples/second
pub const SYSCTL_RCGC0_ADC1SPD_1M: u32 = 0x00000C00 ; // 1M samples/second
pub const SYSCTL_RCGC0_ADC0SPD_M: u32 = 0x00000300 ; // ADC0 Sample Speed
pub const  SYSCTL_RCGC0_ADC0SPD_125K: u32 =    0x00000000 ; // 125K samples/second
pub const  SYSCTL_RCGC0_ADC0SPD_250K: u32 =    0x00000100 ; // 250K samples/second
pub const  SYSCTL_RCGC0_ADC0SPD_500K: u32 =    0x00000200 ; // 500K samples/second
pub const SYSCTL_RCGC0_ADC0SPD_1M: u32 = 0x00000300 ; // 1M samples/second
pub const SYSCTL_RCGC0_HIB: u32 = 0x00000040 ; // HIB Clock Gating Control
pub const SYSCTL_RCGC0_WDT0: u32 = 0x00000008 ; // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC1 register.
//
//*****************************************************************************
pub const SYSCTL_RCGC1_COMP2: u32 = 0x04000000 ; // Analog Comparator 2 Clock Gating
pub const SYSCTL_RCGC1_COMP1: u32 = 0x02000000 ; // Analog Comparator 1 Clock Gating
pub const SYSCTL_RCGC1_COMP0: u32 = 0x01000000 ; // Analog Comparator 0 Clock Gating
pub const SYSCTL_RCGC1_TIMER3: u32 = 0x00080000 ; // Timer 3 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER2: u32 = 0x00040000 ; // Timer 2 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER1: u32 = 0x00020000 ; // Timer 1 Clock Gating Control
pub const SYSCTL_RCGC1_TIMER0: u32 = 0x00010000 ; // Timer 0 Clock Gating Control
pub const SYSCTL_RCGC1_I2C1: u32 = 0x00004000 ; // I2C1 Clock Gating Control
pub const SYSCTL_RCGC1_I2C0: u32 = 0x00001000 ; // I2C0 Clock Gating Control
pub const SYSCTL_RCGC1_QEI1: u32 = 0x00000200 ; // QEI1 Clock Gating Control
pub const SYSCTL_RCGC1_QEI0: u32 = 0x00000100 ; // QEI0 Clock Gating Control
pub const SYSCTL_RCGC1_SSI1: u32 = 0x00000020 ; // SSI1 Clock Gating Control
pub const SYSCTL_RCGC1_SSI0: u32 = 0x00000010 ; // SSI0 Clock Gating Control
pub const SYSCTL_RCGC1_UART2: u32 = 0x00000004 ; // UART2 Clock Gating Control
pub const SYSCTL_RCGC1_UART1: u32 = 0x00000002 ; // UART1 Clock Gating Control
pub const SYSCTL_RCGC1_UART0: u32 = 0x00000001 ; // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGC2 register.
//
//*****************************************************************************
pub const SYSCTL_RCGC2_USB0: u32 = 0x00010000 ; // USB0 Clock Gating Control
pub const SYSCTL_RCGC2_UDMA: u32 = 0x00002000 ; // Micro-DMA Clock Gating Control
pub const SYSCTL_RCGC2_GPIOJ: u32 = 0x00000100 ; // Port J Clock Gating Control
pub const SYSCTL_RCGC2_GPIOH: u32 = 0x00000080 ; // Port H Clock Gating Control
pub const SYSCTL_RCGC2_GPIOG: u32 = 0x00000040 ; // Port G Clock Gating Control
pub const SYSCTL_RCGC2_GPIOF: u32 = 0x00000020 ; // Port F Clock Gating Control
pub const SYSCTL_RCGC2_GPIOE: u32 = 0x00000010 ; // Port E Clock Gating Control
pub const SYSCTL_RCGC2_GPIOD: u32 = 0x00000008 ; // Port D Clock Gating Control
pub const SYSCTL_RCGC2_GPIOC: u32 = 0x00000004 ; // Port C Clock Gating Control
pub const SYSCTL_RCGC2_GPIOB: u32 = 0x00000002 ; // Port B Clock Gating Control
pub const SYSCTL_RCGC2_GPIOA: u32 = 0x00000001 ; // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC0 register.
//
//*****************************************************************************
pub const SYSCTL_SCGC0_WDT1: u32 = 0x10000000 ; // WDT1 Clock Gating Control
pub const SYSCTL_SCGC0_CAN1: u32 = 0x02000000 ; // CAN1 Clock Gating Control
pub const SYSCTL_SCGC0_CAN0: u32 = 0x01000000 ; // CAN0 Clock Gating Control
pub const SYSCTL_SCGC0_PWM0: u32 = 0x00100000 ; // PWM Clock Gating Control
pub const SYSCTL_SCGC0_ADC1: u32 = 0x00020000 ; // ADC1 Clock Gating Control
pub const SYSCTL_SCGC0_ADC0: u32 = 0x00010000 ; // ADC0 Clock Gating Control
pub const SYSCTL_SCGC0_ADCSPD_M: u32 = 0x00000F00 ; // ADC Sample Speed
pub const SYSCTL_SCGC0_HIB: u32 = 0x00000040 ; // HIB Clock Gating Control
pub const SYSCTL_SCGC0_WDT0: u32 = 0x00000008 ; // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC1 register.
//
//*****************************************************************************
pub const SYSCTL_SCGC1_COMP2: u32 = 0x04000000 ; // Analog Comparator 2 Clock Gating
pub const SYSCTL_SCGC1_COMP1: u32 = 0x02000000 ; // Analog Comparator 1 Clock Gating
pub const SYSCTL_SCGC1_COMP0: u32 = 0x01000000 ; // Analog Comparator 0 Clock Gating
pub const SYSCTL_SCGC1_TIMER3: u32 = 0x00080000 ; // Timer 3 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER2: u32 = 0x00040000 ; // Timer 2 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER1: u32 = 0x00020000 ; // Timer 1 Clock Gating Control
pub const SYSCTL_SCGC1_TIMER0: u32 = 0x00010000 ; // Timer 0 Clock Gating Control
pub const SYSCTL_SCGC1_I2C1: u32 = 0x00004000 ; // I2C1 Clock Gating Control
pub const SYSCTL_SCGC1_I2C0: u32 = 0x00001000 ; // I2C0 Clock Gating Control
pub const SYSCTL_SCGC1_QEI1: u32 = 0x00000200 ; // QEI1 Clock Gating Control
pub const SYSCTL_SCGC1_QEI0: u32 = 0x00000100 ; // QEI0 Clock Gating Control
pub const SYSCTL_SCGC1_SSI1: u32 = 0x00000020 ; // SSI1 Clock Gating Control
pub const SYSCTL_SCGC1_SSI0: u32 = 0x00000010 ; // SSI0 Clock Gating Control
pub const SYSCTL_SCGC1_UART2: u32 = 0x00000004 ; // UART2 Clock Gating Control
pub const SYSCTL_SCGC1_UART1: u32 = 0x00000002 ; // UART1 Clock Gating Control
pub const SYSCTL_SCGC1_UART0: u32 = 0x00000001 ; // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGC2 register.
//
//*****************************************************************************
pub const SYSCTL_SCGC2_USB0: u32 = 0x00010000 ; // USB0 Clock Gating Control
pub const SYSCTL_SCGC2_UDMA: u32 = 0x00002000 ; // Micro-DMA Clock Gating Control
pub const SYSCTL_SCGC2_GPIOJ: u32 = 0x00000100 ; // Port J Clock Gating Control
pub const SYSCTL_SCGC2_GPIOH: u32 = 0x00000080 ; // Port H Clock Gating Control
pub const SYSCTL_SCGC2_GPIOG: u32 = 0x00000040 ; // Port G Clock Gating Control
pub const SYSCTL_SCGC2_GPIOF: u32 = 0x00000020 ; // Port F Clock Gating Control
pub const SYSCTL_SCGC2_GPIOE: u32 = 0x00000010 ; // Port E Clock Gating Control
pub const SYSCTL_SCGC2_GPIOD: u32 = 0x00000008 ; // Port D Clock Gating Control
pub const SYSCTL_SCGC2_GPIOC: u32 = 0x00000004 ; // Port C Clock Gating Control
pub const SYSCTL_SCGC2_GPIOB: u32 = 0x00000002 ; // Port B Clock Gating Control
pub const SYSCTL_SCGC2_GPIOA: u32 = 0x00000001 ; // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC0 register.
//
//*****************************************************************************
pub const SYSCTL_DCGC0_WDT1: u32 = 0x10000000 ; // WDT1 Clock Gating Control
pub const SYSCTL_DCGC0_CAN1: u32 = 0x02000000 ; // CAN1 Clock Gating Control
pub const SYSCTL_DCGC0_CAN0: u32 = 0x01000000 ; // CAN0 Clock Gating Control
pub const SYSCTL_DCGC0_PWM0: u32 = 0x00100000 ; // PWM Clock Gating Control
pub const SYSCTL_DCGC0_ADC1: u32 = 0x00020000 ; // ADC1 Clock Gating Control
pub const SYSCTL_DCGC0_ADC0: u32 = 0x00010000 ; // ADC0 Clock Gating Control
pub const SYSCTL_DCGC0_HIB: u32 = 0x00000040 ; // HIB Clock Gating Control
pub const SYSCTL_DCGC0_WDT0: u32 = 0x00000008 ; // WDT0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC1 register.
//
//*****************************************************************************
pub const SYSCTL_DCGC1_COMP2: u32 = 0x04000000 ; // Analog Comparator 2 Clock Gating
pub const SYSCTL_DCGC1_COMP1: u32 = 0x02000000 ; // Analog Comparator 1 Clock Gating
pub const SYSCTL_DCGC1_COMP0: u32 = 0x01000000 ; // Analog Comparator 0 Clock Gating
pub const SYSCTL_DCGC1_TIMER3: u32 = 0x00080000 ; // Timer 3 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER2: u32 = 0x00040000 ; // Timer 2 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER1: u32 = 0x00020000 ; // Timer 1 Clock Gating Control
pub const SYSCTL_DCGC1_TIMER0: u32 = 0x00010000 ; // Timer 0 Clock Gating Control
pub const SYSCTL_DCGC1_I2C1: u32 = 0x00004000 ; // I2C1 Clock Gating Control
pub const SYSCTL_DCGC1_I2C0: u32 = 0x00001000 ; // I2C0 Clock Gating Control
pub const SYSCTL_DCGC1_QEI1: u32 = 0x00000200 ; // QEI1 Clock Gating Control
pub const SYSCTL_DCGC1_QEI0: u32 = 0x00000100 ; // QEI0 Clock Gating Control
pub const SYSCTL_DCGC1_SSI1: u32 = 0x00000020 ; // SSI1 Clock Gating Control
pub const SYSCTL_DCGC1_SSI0: u32 = 0x00000010 ; // SSI0 Clock Gating Control
pub const SYSCTL_DCGC1_UART2: u32 = 0x00000004 ; // UART2 Clock Gating Control
pub const SYSCTL_DCGC1_UART1: u32 = 0x00000002 ; // UART1 Clock Gating Control
pub const SYSCTL_DCGC1_UART0: u32 = 0x00000001 ; // UART0 Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGC2 register.
//
//*****************************************************************************
pub const SYSCTL_DCGC2_USB0: u32 = 0x00010000 ; // USB0 Clock Gating Control
pub const SYSCTL_DCGC2_UDMA: u32 = 0x00002000 ; // Micro-DMA Clock Gating Control
pub const SYSCTL_DCGC2_GPIOJ: u32 = 0x00000100 ; // Port J Clock Gating Control
pub const SYSCTL_DCGC2_GPIOH: u32 = 0x00000080 ; // Port H Clock Gating Control
pub const SYSCTL_DCGC2_GPIOG: u32 = 0x00000040 ; // Port G Clock Gating Control
pub const SYSCTL_DCGC2_GPIOF: u32 = 0x00000020 ; // Port F Clock Gating Control
pub const SYSCTL_DCGC2_GPIOE: u32 = 0x00000010 ; // Port E Clock Gating Control
pub const SYSCTL_DCGC2_GPIOD: u32 = 0x00000008 ; // Port D Clock Gating Control
pub const SYSCTL_DCGC2_GPIOC: u32 = 0x00000004 ; // Port C Clock Gating Control
pub const SYSCTL_DCGC2_GPIOB: u32 = 0x00000002 ; // Port B Clock Gating Control
pub const SYSCTL_DCGC2_GPIOA: u32 = 0x00000001 ; // Port A Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_ALTCLKCFG
// register.
//
//*****************************************************************************
pub const  SYSCTL_ALTCLKCFG_ALTCLK_M: u32 =    0x0000000F ; // Alternate Clock Source
pub const  SYSCTL_ALTCLKCFG_ALTCLK_PIOSC: u32 =    0x00000000 ; // PIOSC
pub const  SYSCTL_ALTCLKCFG_ALTCLK_RTCOSC: u32 =    0x00000003 ; // Hibernation Module Real-time
// clock output (RTCOSC)
pub const  SYSCTL_ALTCLKCFG_ALTCLK_LFIOSC: u32 =    0x00000004 ; // Low-frequency internal
// oscillator (LFIOSC)

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPCLKCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_DSLPCLKCFG_D_M: u32 = 0x1F800000 ; // Divider Field Override
pub const SYSCTL_DSLPCLKCFG_O_M: u32 = 0x00000070 ; // Clock Source
pub const SYSCTL_DSLPCLKCFG_O_IGN: u32 = 0x00000000 ; // MOSC
pub const SYSCTL_DSLPCLKCFG_O_IO: u32 = 0x00000010 ; // PIOSC
pub const SYSCTL_DSLPCLKCFG_O_30: u32 = 0x00000030 ; // LFIOSC
pub const SYSCTL_DSLPCLKCFG_O_32: u32 = 0x00000070 ; // 32.768 kHz
pub const  SYSCTL_DSLPCLKCFG_PIOSCPD: u32 =    0x00000002 ; // PIOSC Power Down Request
pub const SYSCTL_DSLPCLKCFG_D_S: u32 =       2;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSCLKCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_DSCLKCFG_PIOSCPD: u32 = 0x80000000 ; // PIOSC Power Down
pub const SYSCTL_DSCLKCFG_MOSCDPD: u32 = 0x40000000 ; // MOSC Disable Power Down
pub const  SYSCTL_DSCLKCFG_DSOSCSRC_M: u32 =    0x00F00000 ; // Deep Sleep Oscillator Source
pub const  SYSCTL_DSCLKCFG_DSOSCSRC_PIOSC: u32 =    0x00000000 ; // PIOSC
pub const  SYSCTL_DSCLKCFG_DSOSCSRC_LFIOSC: u32 =    0x00200000 ; // LFIOSC
pub const  SYSCTL_DSCLKCFG_DSOSCSRC_MOSC: u32 =    0x00300000 ; // MOSC
pub const  SYSCTL_DSCLKCFG_DSOSCSRC_RTC: u32 =    0x00400000 ; // Hibernation Module RTCOSC
pub const  SYSCTL_DSCLKCFG_DSSYSDIV_M: u32 =    0x000003FF ; // Deep Sleep Clock Divisor
pub const SYSCTL_DSCLKCFG_DSSYSDIV_S: u32 =    0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DIVSCLK register.
//
//*****************************************************************************
pub const SYSCTL_DIVSCLK_EN: u32 = 0x80000000 ; // DIVSCLK Enable
pub const SYSCTL_DIVSCLK_SRC_M: u32 = 0x00030000 ; // Clock Source
pub const  SYSCTL_DIVSCLK_SRC_SYSCLK: u32 =    0x00000000 ; // System Clock
pub const  SYSCTL_DIVSCLK_SRC_PIOSC: u32 =    0x00010000 ; // PIOSC
pub const SYSCTL_DIVSCLK_SRC_MOSC: u32 = 0x00020000 ; // MOSC
pub const SYSCTL_DIVSCLK_DIV_M: u32 = 0x000000FF ; // Divisor Value
pub const  SYSCTL_DIVSCLK_DIV_S: u32 =        0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SYSPROP register.
//
//*****************************************************************************
pub const SYSCTL_SYSPROP_FPU: u32 = 0x00000001 ; // FPU Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCCAL
// register.
//
//*****************************************************************************
pub const SYSCTL_PIOSCCAL_UTEN: u32 = 0x80000000 ; // Use User Trim Value
pub const SYSCTL_PIOSCCAL_CAL: u32 = 0x00000200 ; // Start Calibration
pub const SYSCTL_PIOSCCAL_UPDATE: u32 = 0x00000100 ; // Update Trim
pub const SYSCTL_PIOSCCAL_UT_M: u32 = 0x0000007F ; // User Trim Value
pub const  SYSCTL_PIOSCCAL_UT_S: u32 =        0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PIOSCSTAT
// register.
//
//*****************************************************************************
pub const SYSCTL_PIOSCSTAT_DT_M: u32 = 0x007F0000 ; // Default Trim Value
pub const SYSCTL_PIOSCSTAT_CR_M: u32 = 0x00000300 ; // Calibration Result
pub const SYSCTL_PIOSCSTAT_CRNONE: u32 = 0x00000000 ; // Calibration has not been
// attempted
pub const SYSCTL_PIOSCSTAT_CRPASS: u32 = 0x00000100 ; // The last calibration operation
// completed to meet 1% accuracy
pub const SYSCTL_PIOSCSTAT_CRFAIL: u32 = 0x00000200 ; // The last calibration operation
// failed to meet 1% accuracy
pub const SYSCTL_PIOSCSTAT_CT_M: u32 = 0x0000007F ; // Calibration Trim Value
pub const  SYSCTL_PIOSCSTAT_DT_S: u32 =       16;
pub const  SYSCTL_PIOSCSTAT_CT_S: u32 =       0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ0
// register.
//
//*****************************************************************************
pub const SYSCTL_PLLFREQ0_PLLPWR: u32 = 0x00800000 ; // PLL Power
pub const SYSCTL_PLLFREQ0_MFRAC_M: u32 = 0x000FFC00 ; // PLL M Fractional Value
pub const SYSCTL_PLLFREQ0_MINT_M: u32 = 0x000003FF ; // PLL M Integer Value
pub const  SYSCTL_PLLFREQ0_MFRAC_S: u32 =     10;
pub const  SYSCTL_PLLFREQ0_MINT_S: u32 =      0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLFREQ1
// register.
//
//*****************************************************************************
pub const SYSCTL_PLLFREQ1_Q_M: u32 = 0x00001F00 ; // PLL Q Value
pub const SYSCTL_PLLFREQ1_N_M: u32 = 0x0000001F ; // PLL N Value
pub const  SYSCTL_PLLFREQ1_Q_S: u32 =         8;
pub const  SYSCTL_PLLFREQ1_N_S: u32 =         0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PLLSTAT register.
//
//*****************************************************************************
pub const SYSCTL_PLLSTAT_LOCK: u32 = 0x00000001 ; // PLL Lock

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SLPPWRCFG
// register.
//
//*****************************************************************************
pub const  SYSCTL_SLPPWRCFG_FLASHPM_M: u32 =    0x00000030 ; // Flash Power Modes
pub const  SYSCTL_SLPPWRCFG_FLASHPM_NRM: u32 =    0x00000000 ; // Active Mode
pub const  SYSCTL_SLPPWRCFG_FLASHPM_SLP: u32 =    0x00000020 ; // Low Power Mode
pub const  SYSCTL_SLPPWRCFG_SRAMPM_M: u32 =    0x00000003 ; // SRAM Power Modes
pub const  SYSCTL_SLPPWRCFG_SRAMPM_NRM: u32 =    0x00000000 ; // Active Mode
pub const  SYSCTL_SLPPWRCFG_SRAMPM_SBY: u32 =    0x00000001 ; // Standby Mode
pub const  SYSCTL_SLPPWRCFG_SRAMPM_LP: u32 =    0x00000003 ; // Low Power Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DSLPPWRCFG
// register.
//
//*****************************************************************************
pub const SYSCTL_DSLPPWRCFG_LDOSM: u32 = 0x00000200 ; // LDO Sleep Mode
pub const SYSCTL_DSLPPWRCFG_TSPD: u32 = 0x00000100 ; // Temperature Sense Power Down
pub const  SYSCTL_DSLPPWRCFG_FLASHPM_M: u32 =    0x00000030 ; // Flash Power Modes
pub const  SYSCTL_DSLPPWRCFG_FLASHPM_NRM: u32 =    0x00000000 ; // Active Mode
pub const  SYSCTL_DSLPPWRCFG_FLASHPM_SLP: u32 =    0x00000020 ; // Low Power Mode
pub const  SYSCTL_DSLPPWRCFG_SRAMPM_M: u32 =    0x00000003 ; // SRAM Power Modes
pub const  SYSCTL_DSLPPWRCFG_SRAMPM_NRM: u32 =    0x00000000 ; // Active Mode
pub const  SYSCTL_DSLPPWRCFG_SRAMPM_SBY: u32 =    0x00000001 ; // Standby Mode
pub const  SYSCTL_DSLPPWRCFG_SRAMPM_LP: u32 =    0x00000003 ; // Low Power Mode

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DC9 register.
//
//*****************************************************************************
pub const SYSCTL_DC9_ADC1DC7: u32 = 0x00800000 ; // ADC1 DC7 Present
pub const SYSCTL_DC9_ADC1DC6: u32 = 0x00400000 ; // ADC1 DC6 Present
pub const SYSCTL_DC9_ADC1DC5: u32 = 0x00200000 ; // ADC1 DC5 Present
pub const SYSCTL_DC9_ADC1DC4: u32 = 0x00100000 ; // ADC1 DC4 Present
pub const SYSCTL_DC9_ADC1DC3: u32 = 0x00080000 ; // ADC1 DC3 Present
pub const SYSCTL_DC9_ADC1DC2: u32 = 0x00040000 ; // ADC1 DC2 Present
pub const SYSCTL_DC9_ADC1DC1: u32 = 0x00020000 ; // ADC1 DC1 Present
pub const SYSCTL_DC9_ADC1DC0: u32 = 0x00010000 ; // ADC1 DC0 Present
pub const SYSCTL_DC9_ADC0DC7: u32 = 0x00000080 ; // ADC0 DC7 Present
pub const SYSCTL_DC9_ADC0DC6: u32 = 0x00000040 ; // ADC0 DC6 Present
pub const SYSCTL_DC9_ADC0DC5: u32 = 0x00000020 ; // ADC0 DC5 Present
pub const SYSCTL_DC9_ADC0DC4: u32 = 0x00000010 ; // ADC0 DC4 Present
pub const SYSCTL_DC9_ADC0DC3: u32 = 0x00000008 ; // ADC0 DC3 Present
pub const SYSCTL_DC9_ADC0DC2: u32 = 0x00000004 ; // ADC0 DC2 Present
pub const SYSCTL_DC9_ADC0DC1: u32 = 0x00000002 ; // ADC0 DC1 Present
pub const SYSCTL_DC9_ADC0DC0: u32 = 0x00000001 ; // ADC0 DC0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_NVMSTAT register.
//
//*****************************************************************************
pub const SYSCTL_NVMSTAT_FWB: u32 = 0x00000001 ; // 32 Word Flash Write Buffer
// Available

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDOSPCTL
// register.
//
//*****************************************************************************
pub const SYSCTL_LDOSPCTL_VADJEN: u32 = 0x80000000 ; // Voltage Adjust Enable
pub const SYSCTL_LDOSPCTL_VLDO_M: u32 = 0x000000FF ; // LDO Output Voltage
pub const  SYSCTL_LDOSPCTL_VLDO_0_90V: u32 =    0x00000012 ; // 0.90 V
pub const  SYSCTL_LDOSPCTL_VLDO_0_95V: u32 =    0x00000013 ; // 0.95 V
pub const  SYSCTL_LDOSPCTL_VLDO_1_00V: u32 =    0x00000014 ; // 1.00 V
pub const  SYSCTL_LDOSPCTL_VLDO_1_05V: u32 =    0x00000015 ; // 1.05 V
pub const  SYSCTL_LDOSPCTL_VLDO_1_10V: u32 =    0x00000016 ; // 1.10 V
pub const  SYSCTL_LDOSPCTL_VLDO_1_15V: u32 =    0x00000017 ; // 1.15 V
pub const  SYSCTL_LDOSPCTL_VLDO_1_20V: u32 =    0x00000018 ; // 1.20 V

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LDODPCTL
// register.
//
//*****************************************************************************
pub const SYSCTL_LDODPCTL_VADJEN: u32 = 0x80000000 ; // Voltage Adjust Enable
pub const SYSCTL_LDODPCTL_VLDO_M: u32 = 0x000000FF ; // LDO Output Voltage
pub const  SYSCTL_LDODPCTL_VLDO_0_90: u32 =    0x00000012 ; // 0.90 V
pub const  SYSCTL_LDODPCTL_VLDO_0_95V: u32 =    0x00000013 ; // 0.95 V
pub const  SYSCTL_LDODPCTL_VLDO_1_00V: u32 =    0x00000014 ; // 1.00 V
pub const  SYSCTL_LDODPCTL_VLDO_1_05V: u32 =    0x00000015 ; // 1.05 V
pub const  SYSCTL_LDODPCTL_VLDO_1_10V: u32 =    0x00000016 ; // 1.10 V
pub const  SYSCTL_LDODPCTL_VLDO_1_15V: u32 =    0x00000017 ; // 1.15 V
pub const  SYSCTL_LDODPCTL_VLDO_1_20V: u32 =    0x00000018 ; // 1.20 V
pub const  SYSCTL_LDODPCTL_VLDO_1_25V: u32 =    0x00000019 ; // 1.25 V
pub const  SYSCTL_LDODPCTL_VLDO_1_30V: u32 =    0x0000001A ; // 1.30 V
pub const  SYSCTL_LDODPCTL_VLDO_1_35V: u32 =    0x0000001B ; // 1.35 V

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RESBEHAVCTL
// register.
//
//*****************************************************************************
pub const  SYSCTL_RESBEHAVCTL_WDOG1_M: u32 =    0x000000C0 ; // Watchdog 1 Reset Operation
pub const  SYSCTL_RESBEHAVCTL_WDOG1_SYSRST: u32 =    0x00000080 ; // Watchdog 1 issues a system
// reset. The application starts
// within 10 us
pub const  SYSCTL_RESBEHAVCTL_WDOG1_POR: u32 =    0x000000C0 ; // Watchdog 1 issues a simulated
// POR sequence. Application starts
// less than 500 us after
// deassertion (Default)
pub const  SYSCTL_RESBEHAVCTL_WDOG0_M: u32 =    0x00000030 ; // Watchdog 0 Reset Operation
pub const  SYSCTL_RESBEHAVCTL_WDOG0_SYSRST: u32 =    0x00000020 ; // Watchdog 0 issues a system
// reset. The application starts
// within 10 us
pub const  SYSCTL_RESBEHAVCTL_WDOG0_POR: u32 =    0x00000030 ; // Watchdog 0 issues a simulated
// POR sequence. Application starts
// less than 500 us after
// deassertion (Default)
pub const  SYSCTL_RESBEHAVCTL_BOR_M: u32 =    0x0000000C ; // BOR Reset operation
pub const  SYSCTL_RESBEHAVCTL_BOR_SYSRST: u32 =    0x00000008 ; // Brown Out Reset issues system
// reset. The application starts
// within 10 us
pub const  SYSCTL_RESBEHAVCTL_BOR_POR: u32 =    0x0000000C ; // Brown Out Reset issues a
// simulated POR sequence. The
// application starts less than 500
// us after deassertion (Default)
pub const  SYSCTL_RESBEHAVCTL_EXTRES_M: u32 =    0x00000003 ; // External RST Pin Operation
pub const  SYSCTL_RESBEHAVCTL_EXTRES_SYSRST: u32 =    0x00000002 ; // External RST assertion issues a
// system reset. The application
// starts within 10 us
pub const  SYSCTL_RESBEHAVCTL_EXTRES_POR: u32 =    0x00000003 ; // External RST assertion issues a
// simulated POR sequence.
// Application starts less than 500
// us after deassertion (Default)

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_HSSR register.
//
//*****************************************************************************
pub const SYSCTL_HSSR_KEY_M: u32 = 0xFF000000 ; // Write Key
pub const SYSCTL_HSSR_CDOFF_M: u32 = 0x00FFFFFF ; // Command Descriptor Pointer
pub const  SYSCTL_HSSR_KEY_S: u32 =           24;
pub const  SYSCTL_HSSR_CDOFF_S: u32 =         0;

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_USBPDS register.
//
//*****************************************************************************
pub const SYSCTL_USBPDS_MEMSTAT_M: u32 = 0x0000000C ; // Memory Array Power Status
pub const  SYSCTL_USBPDS_MEMSTAT_OFF: u32 =    0x00000000 ; // Array OFF
pub const  SYSCTL_USBPDS_MEMSTAT_RETAIN: u32 =    0x00000004 ; // SRAM Retention
pub const  SYSCTL_USBPDS_MEMSTAT_ON: u32 =    0x0000000C ; // Array On
pub const SYSCTL_USBPDS_PWRSTAT_M: u32 = 0x00000003 ; // Power Domain Status
pub const  SYSCTL_USBPDS_PWRSTAT_OFF: u32 =    0x00000000 ; // OFF
pub const  SYSCTL_USBPDS_PWRSTAT_ON: u32 =    0x00000003 ; // ON

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_USBMPC register.
//
//*****************************************************************************
pub const SYSCTL_USBMPC_PWRCTL_M: u32 = 0x00000003 ; // Memory Array Power Control
pub const  SYSCTL_USBMPC_PWRCTL_OFF: u32 =    0x00000000 ; // Array OFF
pub const  SYSCTL_USBMPC_PWRCTL_RETAIN: u32 =    0x00000001 ; // SRAM Retention
pub const SYSCTL_USBMPC_PWRCTL_ON: u32 = 0x00000003 ; // Array On

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_EMACPDS register.
//
//*****************************************************************************
pub const  SYSCTL_EMACPDS_MEMSTAT_M: u32 =    0x0000000C ; // Memory Array Power Status
pub const  SYSCTL_EMACPDS_MEMSTAT_OFF: u32 =    0x00000000 ; // Array OFF
pub const  SYSCTL_EMACPDS_MEMSTAT_ON: u32 =    0x0000000C ; // Array On
pub const  SYSCTL_EMACPDS_PWRSTAT_M: u32 =    0x00000003 ; // Power Domain Status
pub const  SYSCTL_EMACPDS_PWRSTAT_OFF: u32 =    0x00000000 ; // OFF
pub const  SYSCTL_EMACPDS_PWRSTAT_ON: u32 =    0x00000003 ; // ON

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_EMACMPC register.
//
//*****************************************************************************
pub const SYSCTL_EMACMPC_PWRCTL_M: u32 = 0x00000003 ; // Memory Array Power Control
pub const  SYSCTL_EMACMPC_PWRCTL_OFF: u32 =    0x00000000 ; // Array OFF
pub const  SYSCTL_EMACMPC_PWRCTL_ON: u32 =    0x00000003 ; // Array On

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_LCDMPC register.
//
//*****************************************************************************
pub const SYSCTL_LCDMPC_PWRCTL_M: u32 = 0x00000003 ; // Memory Array Power Control
pub const  SYSCTL_LCDMPC_PWRCTL_OFF: u32 =    0x00000000 ; // Array OFF
pub const SYSCTL_LCDMPC_PWRCTL_ON: u32 = 0x00000003 ; // Array On

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWD register.
//
//*****************************************************************************
pub const SYSCTL_PPWD_P1: u32 = 0x00000002 ; // Watchdog Timer 1 Present
pub const SYSCTL_PPWD_P0: u32 = 0x00000001 ; // Watchdog Timer 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPTIMER register.
//
//*****************************************************************************
pub const SYSCTL_PPTIMER_P7: u32 = 0x00000080 ; // 16/32-Bit General-Purpose Timer
// 7 Present
pub const SYSCTL_PPTIMER_P6: u32 = 0x00000040 ; // 16/32-Bit General-Purpose Timer
// 6 Present
pub const SYSCTL_PPTIMER_P5: u32 = 0x00000020 ; // 16/32-Bit General-Purpose Timer
// 5 Present
pub const SYSCTL_PPTIMER_P4: u32 = 0x00000010 ; // 16/32-Bit General-Purpose Timer
// 4 Present
pub const SYSCTL_PPTIMER_P3: u32 = 0x00000008 ; // 16/32-Bit General-Purpose Timer
// 3 Present
pub const SYSCTL_PPTIMER_P2: u32 = 0x00000004 ; // 16/32-Bit General-Purpose Timer
// 2 Present
pub const SYSCTL_PPTIMER_P1: u32 = 0x00000002 ; // 16/32-Bit General-Purpose Timer
// 1 Present
pub const SYSCTL_PPTIMER_P0: u32 = 0x00000001 ; // 16/32-Bit General-Purpose Timer
// 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPGPIO register.
//
//*****************************************************************************
pub const SYSCTL_PPGPIO_P17: u32 = 0x00020000 ; // GPIO Port T Present
pub const SYSCTL_PPGPIO_P16: u32 = 0x00010000 ; // GPIO Port S Present
pub const SYSCTL_PPGPIO_P15: u32 = 0x00008000 ; // GPIO Port R Present
pub const SYSCTL_PPGPIO_P14: u32 = 0x00004000 ; // GPIO Port Q Present
pub const SYSCTL_PPGPIO_P13: u32 = 0x00002000 ; // GPIO Port P Present
pub const SYSCTL_PPGPIO_P12: u32 = 0x00001000 ; // GPIO Port N Present
pub const SYSCTL_PPGPIO_P11: u32 = 0x00000800 ; // GPIO Port M Present
pub const SYSCTL_PPGPIO_P10: u32 = 0x00000400 ; // GPIO Port L Present
pub const SYSCTL_PPGPIO_P9: u32 = 0x00000200 ; // GPIO Port K Present
pub const SYSCTL_PPGPIO_P8: u32 = 0x00000100 ; // GPIO Port J Present
pub const SYSCTL_PPGPIO_P7: u32 = 0x00000080 ; // GPIO Port H Present
pub const SYSCTL_PPGPIO_P6: u32 = 0x00000040 ; // GPIO Port G Present
pub const SYSCTL_PPGPIO_P5: u32 = 0x00000020 ; // GPIO Port F Present
pub const SYSCTL_PPGPIO_P4: u32 = 0x00000010 ; // GPIO Port E Present
pub const SYSCTL_PPGPIO_P3: u32 = 0x00000008 ; // GPIO Port D Present
pub const SYSCTL_PPGPIO_P2: u32 = 0x00000004 ; // GPIO Port C Present
pub const SYSCTL_PPGPIO_P1: u32 = 0x00000002 ; // GPIO Port B Present
pub const SYSCTL_PPGPIO_P0: u32 = 0x00000001 ; // GPIO Port A Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPDMA register.
//
//*****************************************************************************
pub const SYSCTL_PPDMA_P0: u32 = 0x00000001 ; // uDMA Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEPI register.
//
//*****************************************************************************
pub const SYSCTL_PPEPI_P0: u32 = 0x00000001 ; // EPI Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPHIB register.
//
//*****************************************************************************
pub const SYSCTL_PPHIB_P0: u32 = 0x00000001 ; // Hibernation Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUART register.
//
//*****************************************************************************
pub const SYSCTL_PPUART_P7: u32 = 0x00000080 ; // UART Module 7 Present
pub const SYSCTL_PPUART_P6: u32 = 0x00000040 ; // UART Module 6 Present
pub const SYSCTL_PPUART_P5: u32 = 0x00000020 ; // UART Module 5 Present
pub const SYSCTL_PPUART_P4: u32 = 0x00000010 ; // UART Module 4 Present
pub const SYSCTL_PPUART_P3: u32 = 0x00000008 ; // UART Module 3 Present
pub const SYSCTL_PPUART_P2: u32 = 0x00000004 ; // UART Module 2 Present
pub const SYSCTL_PPUART_P1: u32 = 0x00000002 ; // UART Module 1 Present
pub const SYSCTL_PPUART_P0: u32 = 0x00000001 ; // UART Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPSSI register.
//
//*****************************************************************************
pub const SYSCTL_PPSSI_P3: u32 = 0x00000008 ; // SSI Module 3 Present
pub const SYSCTL_PPSSI_P2: u32 = 0x00000004 ; // SSI Module 2 Present
pub const SYSCTL_PPSSI_P1: u32 = 0x00000002 ; // SSI Module 1 Present
pub const SYSCTL_PPSSI_P0: u32 = 0x00000001 ; // SSI Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPI2C register.
//
//*****************************************************************************
pub const SYSCTL_PPI2C_P9: u32 = 0x00000200 ; // I2C Module 9 Present
pub const SYSCTL_PPI2C_P8: u32 = 0x00000100 ; // I2C Module 8 Present
pub const SYSCTL_PPI2C_P7: u32 = 0x00000080 ; // I2C Module 7 Present
pub const SYSCTL_PPI2C_P6: u32 = 0x00000040 ; // I2C Module 6 Present
pub const SYSCTL_PPI2C_P5: u32 = 0x00000020 ; // I2C Module 5 Present
pub const SYSCTL_PPI2C_P4: u32 = 0x00000010 ; // I2C Module 4 Present
pub const SYSCTL_PPI2C_P3: u32 = 0x00000008 ; // I2C Module 3 Present
pub const SYSCTL_PPI2C_P2: u32 = 0x00000004 ; // I2C Module 2 Present
pub const SYSCTL_PPI2C_P1: u32 = 0x00000002 ; // I2C Module 1 Present
pub const SYSCTL_PPI2C_P0: u32 = 0x00000001 ; // I2C Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPUSB register.
//
//*****************************************************************************
pub const SYSCTL_PPUSB_P0: u32 = 0x00000001 ; // USB Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEPHY register.
//
//*****************************************************************************
pub const SYSCTL_PPEPHY_P0: u32 = 0x00000001 ; // Ethernet PHY Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPCAN register.
//
//*****************************************************************************
pub const SYSCTL_PPCAN_P1: u32 = 0x00000002 ; // CAN Module 1 Present
pub const SYSCTL_PPCAN_P0: u32 = 0x00000001 ; // CAN Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPADC register.
//
//*****************************************************************************
pub const SYSCTL_PPADC_P1: u32 = 0x00000002 ; // ADC Module 1 Present
pub const SYSCTL_PPADC_P0: u32 = 0x00000001 ; // ADC Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPACMP register.
//
//*****************************************************************************
pub const SYSCTL_PPACMP_P0: u32 = 0x00000001 ; // Analog Comparator Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPPWM register.
//
//*****************************************************************************
pub const SYSCTL_PPPWM_P1: u32 = 0x00000002 ; // PWM Module 1 Present
pub const SYSCTL_PPPWM_P0: u32 = 0x00000001 ; // PWM Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPQEI register.
//
//*****************************************************************************
pub const SYSCTL_PPQEI_P1: u32 = 0x00000002 ; // QEI Module 1 Present
pub const SYSCTL_PPQEI_P0: u32 = 0x00000001 ; // QEI Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPLPC register.
//
//*****************************************************************************
pub const SYSCTL_PPLPC_P0: u32 = 0x00000001 ; // LPC Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPPECI register.
//
//*****************************************************************************
pub const SYSCTL_PPPECI_P0: u32 = 0x00000001 ; // PECI Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPFAN register.
//
//*****************************************************************************
pub const SYSCTL_PPFAN_P0: u32 = 0x00000001 ; // FAN Module 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_PPEEPROM_P0: u32 = 0x00000001 ; // EEPROM Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_PPWTIMER_P5: u32 = 0x00000020 ; // 32/64-Bit Wide General-Purpose
// Timer 5 Present
pub const SYSCTL_PPWTIMER_P4: u32 = 0x00000010 ; // 32/64-Bit Wide General-Purpose
// Timer 4 Present
pub const SYSCTL_PPWTIMER_P3: u32 = 0x00000008 ; // 32/64-Bit Wide General-Purpose
// Timer 3 Present
pub const SYSCTL_PPWTIMER_P2: u32 = 0x00000004 ; // 32/64-Bit Wide General-Purpose
// Timer 2 Present
pub const SYSCTL_PPWTIMER_P1: u32 = 0x00000002 ; // 32/64-Bit Wide General-Purpose
// Timer 1 Present
pub const SYSCTL_PPWTIMER_P0: u32 = 0x00000001 ; // 32/64-Bit Wide General-Purpose
// Timer 0 Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPRTS register.
//
//*****************************************************************************
pub const SYSCTL_PPRTS_P0: u32 = 0x00000001 ; // RTS Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPCCM register.
//
//*****************************************************************************
pub const SYSCTL_PPCCM_P0: u32 = 0x00000001 ; // CRC and Cryptographic Modules
// Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPLCD register.
//
//*****************************************************************************
pub const SYSCTL_PPLCD_P0: u32 = 0x00000001 ; // LCD Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPOWIRE register.
//
//*****************************************************************************
pub const SYSCTL_PPOWIRE_P0: u32 = 0x00000001 ; // 1-Wire Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPEMAC register.
//
//*****************************************************************************
pub const SYSCTL_PPEMAC_P0: u32 = 0x00000001 ; // Ethernet Controller Module
// Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PPHIM register.
//
//*****************************************************************************
pub const SYSCTL_PPHIM_P0: u32 = 0x00000001 ; // HIM Module Present

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWD register.
//
//*****************************************************************************
pub const SYSCTL_SRWD_R1: u32 = 0x00000002 ; // Watchdog Timer 1 Software Reset
pub const SYSCTL_SRWD_R0: u32 = 0x00000001 ; // Watchdog Timer 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRTIMER register.
//
//*****************************************************************************
pub const SYSCTL_SRTIMER_R7: u32 = 0x00000080 ; // 16/32-Bit General-Purpose Timer
// 7 Software Reset
pub const SYSCTL_SRTIMER_R6: u32 = 0x00000040 ; // 16/32-Bit General-Purpose Timer
// 6 Software Reset
pub const SYSCTL_SRTIMER_R5: u32 = 0x00000020 ; // 16/32-Bit General-Purpose Timer
// 5 Software Reset
pub const SYSCTL_SRTIMER_R4: u32 = 0x00000010 ; // 16/32-Bit General-Purpose Timer
// 4 Software Reset
pub const SYSCTL_SRTIMER_R3: u32 = 0x00000008 ; // 16/32-Bit General-Purpose Timer
// 3 Software Reset
pub const SYSCTL_SRTIMER_R2: u32 = 0x00000004 ; // 16/32-Bit General-Purpose Timer
// 2 Software Reset
pub const SYSCTL_SRTIMER_R1: u32 = 0x00000002 ; // 16/32-Bit General-Purpose Timer
// 1 Software Reset
pub const SYSCTL_SRTIMER_R0: u32 = 0x00000001 ; // 16/32-Bit General-Purpose Timer
// 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRGPIO register.
//
//*****************************************************************************
pub const SYSCTL_SRGPIO_R17: u32 = 0x00020000 ; // GPIO Port T Software Reset
pub const SYSCTL_SRGPIO_R16: u32 = 0x00010000 ; // GPIO Port S Software Reset
pub const SYSCTL_SRGPIO_R15: u32 = 0x00008000 ; // GPIO Port R Software Reset
pub const SYSCTL_SRGPIO_R14: u32 = 0x00004000 ; // GPIO Port Q Software Reset
pub const SYSCTL_SRGPIO_R13: u32 = 0x00002000 ; // GPIO Port P Software Reset
pub const SYSCTL_SRGPIO_R12: u32 = 0x00001000 ; // GPIO Port N Software Reset
pub const SYSCTL_SRGPIO_R11: u32 = 0x00000800 ; // GPIO Port M Software Reset
pub const SYSCTL_SRGPIO_R10: u32 = 0x00000400 ; // GPIO Port L Software Reset
pub const SYSCTL_SRGPIO_R9: u32 = 0x00000200 ; // GPIO Port K Software Reset
pub const SYSCTL_SRGPIO_R8: u32 = 0x00000100 ; // GPIO Port J Software Reset
pub const SYSCTL_SRGPIO_R7: u32 = 0x00000080 ; // GPIO Port H Software Reset
pub const SYSCTL_SRGPIO_R6: u32 = 0x00000040 ; // GPIO Port G Software Reset
pub const SYSCTL_SRGPIO_R5: u32 = 0x00000020 ; // GPIO Port F Software Reset
pub const SYSCTL_SRGPIO_R4: u32 = 0x00000010 ; // GPIO Port E Software Reset
pub const SYSCTL_SRGPIO_R3: u32 = 0x00000008 ; // GPIO Port D Software Reset
pub const SYSCTL_SRGPIO_R2: u32 = 0x00000004 ; // GPIO Port C Software Reset
pub const SYSCTL_SRGPIO_R1: u32 = 0x00000002 ; // GPIO Port B Software Reset
pub const SYSCTL_SRGPIO_R0: u32 = 0x00000001 ; // GPIO Port A Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRDMA register.
//
//*****************************************************************************
pub const SYSCTL_SRDMA_R0: u32 = 0x00000001 ; // uDMA Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREPI register.
//
//*****************************************************************************
pub const SYSCTL_SREPI_R0: u32 = 0x00000001 ; // EPI Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRHIB register.
//
//*****************************************************************************
pub const SYSCTL_SRHIB_R0: u32 = 0x00000001 ; // Hibernation Module Software
// Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUART register.
//
//*****************************************************************************
pub const SYSCTL_SRUART_R7: u32 = 0x00000080 ; // UART Module 7 Software Reset
pub const SYSCTL_SRUART_R6: u32 = 0x00000040 ; // UART Module 6 Software Reset
pub const SYSCTL_SRUART_R5: u32 = 0x00000020 ; // UART Module 5 Software Reset
pub const SYSCTL_SRUART_R4: u32 = 0x00000010 ; // UART Module 4 Software Reset
pub const SYSCTL_SRUART_R3: u32 = 0x00000008 ; // UART Module 3 Software Reset
pub const SYSCTL_SRUART_R2: u32 = 0x00000004 ; // UART Module 2 Software Reset
pub const SYSCTL_SRUART_R1: u32 = 0x00000002 ; // UART Module 1 Software Reset
pub const SYSCTL_SRUART_R0: u32 = 0x00000001 ; // UART Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRSSI register.
//
//*****************************************************************************
pub const SYSCTL_SRSSI_R3: u32 = 0x00000008 ; // SSI Module 3 Software Reset
pub const SYSCTL_SRSSI_R2: u32 = 0x00000004 ; // SSI Module 2 Software Reset
pub const SYSCTL_SRSSI_R1: u32 = 0x00000002 ; // SSI Module 1 Software Reset
pub const SYSCTL_SRSSI_R0: u32 = 0x00000001 ; // SSI Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRI2C register.
//
//*****************************************************************************
pub const SYSCTL_SRI2C_R9: u32 = 0x00000200 ; // I2C Module 9 Software Reset
pub const SYSCTL_SRI2C_R8: u32 = 0x00000100 ; // I2C Module 8 Software Reset
pub const SYSCTL_SRI2C_R7: u32 = 0x00000080 ; // I2C Module 7 Software Reset
pub const SYSCTL_SRI2C_R6: u32 = 0x00000040 ; // I2C Module 6 Software Reset
pub const SYSCTL_SRI2C_R5: u32 = 0x00000020 ; // I2C Module 5 Software Reset
pub const SYSCTL_SRI2C_R4: u32 = 0x00000010 ; // I2C Module 4 Software Reset
pub const SYSCTL_SRI2C_R3: u32 = 0x00000008 ; // I2C Module 3 Software Reset
pub const SYSCTL_SRI2C_R2: u32 = 0x00000004 ; // I2C Module 2 Software Reset
pub const SYSCTL_SRI2C_R1: u32 = 0x00000002 ; // I2C Module 1 Software Reset
pub const SYSCTL_SRI2C_R0: u32 = 0x00000001 ; // I2C Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRUSB register.
//
//*****************************************************************************
pub const SYSCTL_SRUSB_R0: u32 = 0x00000001 ; // USB Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREPHY register.
//
//*****************************************************************************
pub const SYSCTL_SREPHY_R0: u32 = 0x00000001 ; // Ethernet PHY Module Software
// Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCAN register.
//
//*****************************************************************************
pub const SYSCTL_SRCAN_R1: u32 = 0x00000002 ; // CAN Module 1 Software Reset
pub const SYSCTL_SRCAN_R0: u32 = 0x00000001 ; // CAN Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRADC register.
//
//*****************************************************************************
pub const SYSCTL_SRADC_R1: u32 = 0x00000002 ; // ADC Module 1 Software Reset
pub const SYSCTL_SRADC_R0: u32 = 0x00000001 ; // ADC Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRACMP register.
//
//*****************************************************************************
pub const SYSCTL_SRACMP_R0: u32 = 0x00000001 ; // Analog Comparator Module 0
// Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRPWM register.
//
//*****************************************************************************
pub const SYSCTL_SRPWM_R1: u32 = 0x00000002 ; // PWM Module 1 Software Reset
pub const SYSCTL_SRPWM_R0: u32 = 0x00000001 ; // PWM Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRQEI register.
//
//*****************************************************************************
pub const SYSCTL_SRQEI_R1: u32 = 0x00000002 ; // QEI Module 1 Software Reset
pub const SYSCTL_SRQEI_R0: u32 = 0x00000001 ; // QEI Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_SREEPROM_R0: u32 = 0x00000001 ; // EEPROM Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_SRWTIMER_R5: u32 = 0x00000020 ; // 32/64-Bit Wide General-Purpose
// Timer 5 Software Reset
pub const SYSCTL_SRWTIMER_R4: u32 = 0x00000010 ; // 32/64-Bit Wide General-Purpose
// Timer 4 Software Reset
pub const SYSCTL_SRWTIMER_R3: u32 = 0x00000008 ; // 32/64-Bit Wide General-Purpose
// Timer 3 Software Reset
pub const SYSCTL_SRWTIMER_R2: u32 = 0x00000004 ; // 32/64-Bit Wide General-Purpose
// Timer 2 Software Reset
pub const SYSCTL_SRWTIMER_R1: u32 = 0x00000002 ; // 32/64-Bit Wide General-Purpose
// Timer 1 Software Reset
pub const SYSCTL_SRWTIMER_R0: u32 = 0x00000001 ; // 32/64-Bit Wide General-Purpose
// Timer 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRCCM register.
//
//*****************************************************************************
pub const SYSCTL_SRCCM_R0: u32 = 0x00000001 ; // CRC and Cryptographic Modules
// Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SRLCD register.
//
//*****************************************************************************
pub const SYSCTL_SRLCD_R0: u32 = 0x00000001 ; // LCD Module 0 Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SROWIRE register.
//
//*****************************************************************************
pub const SYSCTL_SROWIRE_R0: u32 = 0x00000001 ; // 1-Wire Module Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SREMAC register.
//
//*****************************************************************************
pub const SYSCTL_SREMAC_R0: u32 = 0x00000001 ; // Ethernet Controller MAC Module 0
// Software Reset

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWD register.
//
//*****************************************************************************
pub const SYSCTL_RCGCWD_R1: u32 = 0x00000002 ; // Watchdog Timer 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCWD_R0: u32 = 0x00000001 ; // Watchdog Timer 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCTIMER_R7: u32 = 0x00000080 ; // 16/32-Bit General-Purpose Timer
// 7 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R6: u32 = 0x00000040 ; // 16/32-Bit General-Purpose Timer
// 6 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R5: u32 = 0x00000020 ; // 16/32-Bit General-Purpose Timer
// 5 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R4: u32 = 0x00000010 ; // 16/32-Bit General-Purpose Timer
// 4 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R3: u32 = 0x00000008 ; // 16/32-Bit General-Purpose Timer
// 3 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R2: u32 = 0x00000004 ; // 16/32-Bit General-Purpose Timer
// 2 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R1: u32 = 0x00000002 ; // 16/32-Bit General-Purpose Timer
// 1 Run Mode Clock Gating Control
pub const SYSCTL_RCGCTIMER_R0: u32 = 0x00000001 ; // 16/32-Bit General-Purpose Timer
// 0 Run Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCGPIO
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCGPIO_R17: u32 = 0x00020000 ; // GPIO Port T Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R16: u32 = 0x00010000 ; // GPIO Port S Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R15: u32 = 0x00008000 ; // GPIO Port R Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R14: u32 = 0x00004000 ; // GPIO Port Q Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R13: u32 = 0x00002000 ; // GPIO Port P Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R12: u32 = 0x00001000 ; // GPIO Port N Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R11: u32 = 0x00000800 ; // GPIO Port M Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R10: u32 = 0x00000400 ; // GPIO Port L Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R9: u32 = 0x00000200 ; // GPIO Port K Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R8: u32 = 0x00000100 ; // GPIO Port J Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R7: u32 = 0x00000080 ; // GPIO Port H Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R6: u32 = 0x00000040 ; // GPIO Port G Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R5: u32 = 0x00000020 ; // GPIO Port F Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R4: u32 = 0x00000010 ; // GPIO Port E Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R3: u32 = 0x00000008 ; // GPIO Port D Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R2: u32 = 0x00000004 ; // GPIO Port C Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R1: u32 = 0x00000002 ; // GPIO Port B Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCGPIO_R0: u32 = 0x00000001 ; // GPIO Port A Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCDMA register.
//
//*****************************************************************************
pub const SYSCTL_RCGCDMA_R0: u32 = 0x00000001 ; // uDMA Module Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEPI register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEPI_R0: u32 = 0x00000001 ; // EPI Module Run Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCHIB register.
//
//*****************************************************************************
pub const SYSCTL_RCGCHIB_R0: u32 = 0x00000001 ; // Hibernation Module Run Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUART
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCUART_R7: u32 = 0x00000080 ; // UART Module 7 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART_R6: u32 = 0x00000040 ; // UART Module 6 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART_R5: u32 = 0x00000020 ; // UART Module 5 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART_R4: u32 = 0x00000010 ; // UART Module 4 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART_R3: u32 = 0x00000008 ; // UART Module 3 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART_R2: u32 = 0x00000004 ; // UART Module 2 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART_R1: u32 = 0x00000002 ; // UART Module 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCUART_R0: u32 = 0x00000001 ; // UART Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCSSI register.
//
//*****************************************************************************
pub const SYSCTL_RCGCSSI_R3: u32 = 0x00000008 ; // SSI Module 3 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCSSI_R2: u32 = 0x00000004 ; // SSI Module 2 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCSSI_R1: u32 = 0x00000002 ; // SSI Module 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCSSI_R0: u32 = 0x00000001 ; // SSI Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCI2C register.
//
//*****************************************************************************
pub const SYSCTL_RCGCI2C_R9: u32 = 0x00000200 ; // I2C Module 9 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R8: u32 = 0x00000100 ; // I2C Module 8 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R7: u32 = 0x00000080 ; // I2C Module 7 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R6: u32 = 0x00000040 ; // I2C Module 6 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R5: u32 = 0x00000020 ; // I2C Module 5 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R4: u32 = 0x00000010 ; // I2C Module 4 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R3: u32 = 0x00000008 ; // I2C Module 3 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R2: u32 = 0x00000004 ; // I2C Module 2 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R1: u32 = 0x00000002 ; // I2C Module 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCI2C_R0: u32 = 0x00000001 ; // I2C Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCUSB register.
//
//*****************************************************************************
pub const SYSCTL_RCGCUSB_R0: u32 = 0x00000001 ; // USB Module Run Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEPHY
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEPHY_R0: u32 = 0x00000001 ; // Ethernet PHY Module Run Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCCAN register.
//
//*****************************************************************************
pub const SYSCTL_RCGCCAN_R1: u32 = 0x00000002 ; // CAN Module 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCCAN_R0: u32 = 0x00000001 ; // CAN Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCADC register.
//
//*****************************************************************************
pub const SYSCTL_RCGCADC_R1: u32 = 0x00000002 ; // ADC Module 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCADC_R0: u32 = 0x00000001 ; // ADC Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCACMP
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCACMP_R0: u32 = 0x00000001 ; // Analog Comparator Module 0 Run
// Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCPWM register.
//
//*****************************************************************************
pub const SYSCTL_RCGCPWM_R1: u32 = 0x00000002 ; // PWM Module 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCPWM_R0: u32 = 0x00000001 ; // PWM Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCQEI register.
//
//*****************************************************************************
pub const SYSCTL_RCGCQEI_R1: u32 = 0x00000002 ; // QEI Module 1 Run Mode Clock
// Gating Control
pub const SYSCTL_RCGCQEI_R0: u32 = 0x00000001 ; // QEI Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEEPROM_R0: u32 = 0x00000001 ; // EEPROM Module Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCWTIMER_R5: u32 = 0x00000020 ; // 32/64-Bit Wide General-Purpose
// Timer 5 Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCWTIMER_R4: u32 = 0x00000010 ; // 32/64-Bit Wide General-Purpose
// Timer 4 Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCWTIMER_R3: u32 = 0x00000008 ; // 32/64-Bit Wide General-Purpose
// Timer 3 Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCWTIMER_R2: u32 = 0x00000004 ; // 32/64-Bit Wide General-Purpose
// Timer 2 Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCWTIMER_R1: u32 = 0x00000002 ; // 32/64-Bit Wide General-Purpose
// Timer 1 Run Mode Clock Gating
// Control
pub const SYSCTL_RCGCWTIMER_R0: u32 = 0x00000001 ; // 32/64-Bit Wide General-Purpose
// Timer 0 Run Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_RCGCCCM_R0: u32 = 0x00000001 ; // CRC and Cryptographic Modules
// Run Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCLCD register.
//
//*****************************************************************************
pub const SYSCTL_RCGCLCD_R0: u32 = 0x00000001 ; // LCD Controller Module 0 Run Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCOWIRE
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCOWIRE_R0: u32 = 0x00000001 ; // 1-Wire Module 0 Run Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_RCGCEMAC
// register.
//
//*****************************************************************************
pub const SYSCTL_RCGCEMAC_R0: u32 = 0x00000001 ; // Ethernet MAC Module 0 Run Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWD register.
//
//*****************************************************************************
pub const SYSCTL_SCGCWD_S1: u32 = 0x00000002 ; // Watchdog Timer 1 Sleep Mode
// Clock Gating Control
pub const SYSCTL_SCGCWD_S0: u32 = 0x00000001 ; // Watchdog Timer 0 Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCTIMER_S7: u32 = 0x00000080 ; // 16/32-Bit General-Purpose Timer
// 7 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCTIMER_S6: u32 = 0x00000040 ; // 16/32-Bit General-Purpose Timer
// 6 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCTIMER_S5: u32 = 0x00000020 ; // 16/32-Bit General-Purpose Timer
// 5 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCTIMER_S4: u32 = 0x00000010 ; // 16/32-Bit General-Purpose Timer
// 4 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCTIMER_S3: u32 = 0x00000008 ; // 16/32-Bit General-Purpose Timer
// 3 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCTIMER_S2: u32 = 0x00000004 ; // 16/32-Bit General-Purpose Timer
// 2 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCTIMER_S1: u32 = 0x00000002 ; // 16/32-Bit General-Purpose Timer
// 1 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCTIMER_S0: u32 = 0x00000001 ; // 16/32-Bit General-Purpose Timer
// 0 Sleep Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCGPIO
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCGPIO_S17: u32 = 0x00020000 ; // GPIO Port T Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S16: u32 = 0x00010000 ; // GPIO Port S Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S15: u32 = 0x00008000 ; // GPIO Port R Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S14: u32 = 0x00004000 ; // GPIO Port Q Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S13: u32 = 0x00002000 ; // GPIO Port P Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S12: u32 = 0x00001000 ; // GPIO Port N Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S11: u32 = 0x00000800 ; // GPIO Port M Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S10: u32 = 0x00000400 ; // GPIO Port L Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S9: u32 = 0x00000200 ; // GPIO Port K Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S8: u32 = 0x00000100 ; // GPIO Port J Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S7: u32 = 0x00000080 ; // GPIO Port H Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S6: u32 = 0x00000040 ; // GPIO Port G Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S5: u32 = 0x00000020 ; // GPIO Port F Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S4: u32 = 0x00000010 ; // GPIO Port E Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S3: u32 = 0x00000008 ; // GPIO Port D Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S2: u32 = 0x00000004 ; // GPIO Port C Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S1: u32 = 0x00000002 ; // GPIO Port B Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCGPIO_S0: u32 = 0x00000001 ; // GPIO Port A Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCDMA register.
//
//*****************************************************************************
pub const SYSCTL_SCGCDMA_S0: u32 = 0x00000001 ; // uDMA Module Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEPI register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEPI_S0: u32 = 0x00000001 ; // EPI Module Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCHIB register.
//
//*****************************************************************************
pub const SYSCTL_SCGCHIB_S0: u32 = 0x00000001 ; // Hibernation Module Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUART
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCUART_S7: u32 = 0x00000080 ; // UART Module 7 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART_S6: u32 = 0x00000040 ; // UART Module 6 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART_S5: u32 = 0x00000020 ; // UART Module 5 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART_S4: u32 = 0x00000010 ; // UART Module 4 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART_S3: u32 = 0x00000008 ; // UART Module 3 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART_S2: u32 = 0x00000004 ; // UART Module 2 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART_S1: u32 = 0x00000002 ; // UART Module 1 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCUART_S0: u32 = 0x00000001 ; // UART Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCSSI register.
//
//*****************************************************************************
pub const SYSCTL_SCGCSSI_S3: u32 = 0x00000008 ; // SSI Module 3 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCSSI_S2: u32 = 0x00000004 ; // SSI Module 2 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCSSI_S1: u32 = 0x00000002 ; // SSI Module 1 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCSSI_S0: u32 = 0x00000001 ; // SSI Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCI2C register.
//
//*****************************************************************************
pub const SYSCTL_SCGCI2C_S9: u32 = 0x00000200 ; // I2C Module 9 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S8: u32 = 0x00000100 ; // I2C Module 8 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S7: u32 = 0x00000080 ; // I2C Module 7 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S6: u32 = 0x00000040 ; // I2C Module 6 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S5: u32 = 0x00000020 ; // I2C Module 5 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S4: u32 = 0x00000010 ; // I2C Module 4 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S3: u32 = 0x00000008 ; // I2C Module 3 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S2: u32 = 0x00000004 ; // I2C Module 2 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S1: u32 = 0x00000002 ; // I2C Module 1 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCI2C_S0: u32 = 0x00000001 ; // I2C Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCUSB register.
//
//*****************************************************************************
pub const SYSCTL_SCGCUSB_S0: u32 = 0x00000001 ; // USB Module Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEPHY
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEPHY_S0: u32 = 0x00000001 ; // PHY Module Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCCAN register.
//
//*****************************************************************************
pub const SYSCTL_SCGCCAN_S1: u32 = 0x00000002 ; // CAN Module 1 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCCAN_S0: u32 = 0x00000001 ; // CAN Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCADC register.
//
//*****************************************************************************
pub const SYSCTL_SCGCADC_S1: u32 = 0x00000002 ; // ADC Module 1 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCADC_S0: u32 = 0x00000001 ; // ADC Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCACMP
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCACMP_S0: u32 = 0x00000001 ; // Analog Comparator Module 0 Sleep
// Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCPWM register.
//
//*****************************************************************************
pub const SYSCTL_SCGCPWM_S1: u32 = 0x00000002 ; // PWM Module 1 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCPWM_S0: u32 = 0x00000001 ; // PWM Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCQEI register.
//
//*****************************************************************************
pub const SYSCTL_SCGCQEI_S1: u32 = 0x00000002 ; // QEI Module 1 Sleep Mode Clock
// Gating Control
pub const SYSCTL_SCGCQEI_S0: u32 = 0x00000001 ; // QEI Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEEPROM_S0: u32 = 0x00000001 ; // EEPROM Module Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCWTIMER_S5: u32 = 0x00000020 ; // 32/64-Bit Wide General-Purpose
// Timer 5 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCWTIMER_S4: u32 = 0x00000010 ; // 32/64-Bit Wide General-Purpose
// Timer 4 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCWTIMER_S3: u32 = 0x00000008 ; // 32/64-Bit Wide General-Purpose
// Timer 3 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCWTIMER_S2: u32 = 0x00000004 ; // 32/64-Bit Wide General-Purpose
// Timer 2 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCWTIMER_S1: u32 = 0x00000002 ; // 32/64-Bit Wide General-Purpose
// Timer 1 Sleep Mode Clock Gating
// Control
pub const SYSCTL_SCGCWTIMER_S0: u32 = 0x00000001 ; // 32/64-Bit Wide General-Purpose
// Timer 0 Sleep Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_SCGCCCM_S0: u32 = 0x00000001 ; // CRC and Cryptographic Modules
// Sleep Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCLCD register.
//
//*****************************************************************************
pub const SYSCTL_SCGCLCD_S0: u32 = 0x00000001 ; // LCD Controller Module 0 Sleep
// Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCOWIRE
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCOWIRE_S0: u32 = 0x00000001 ; // 1-Wire Module 0 Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_SCGCEMAC
// register.
//
//*****************************************************************************
pub const SYSCTL_SCGCEMAC_S0: u32 = 0x00000001 ; // Ethernet MAC Module 0 Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWD register.
//
//*****************************************************************************
pub const SYSCTL_DCGCWD_D1: u32 = 0x00000002 ; // Watchdog Timer 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCWD_D0: u32 = 0x00000001 ; // Watchdog Timer 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCTIMER_D7: u32 = 0x00000080 ; // 16/32-Bit General-Purpose Timer
// 7 Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCTIMER_D6: u32 = 0x00000040 ; // 16/32-Bit General-Purpose Timer
// 6 Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCTIMER_D5: u32 = 0x00000020 ; // 16/32-Bit General-Purpose Timer
// 5 Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCTIMER_D4: u32 = 0x00000010 ; // 16/32-Bit General-Purpose Timer
// 4 Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCTIMER_D3: u32 = 0x00000008 ; // 16/32-Bit General-Purpose Timer
// 3 Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCTIMER_D2: u32 = 0x00000004 ; // 16/32-Bit General-Purpose Timer
// 2 Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCTIMER_D1: u32 = 0x00000002 ; // 16/32-Bit General-Purpose Timer
// 1 Deep-Sleep Mode Clock Gating
// Control
pub const SYSCTL_DCGCTIMER_D0: u32 = 0x00000001 ; // 16/32-Bit General-Purpose Timer
// 0 Deep-Sleep Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCGPIO
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCGPIO_D17: u32 = 0x00020000 ; // GPIO Port T Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D16: u32 = 0x00010000 ; // GPIO Port S Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D15: u32 = 0x00008000 ; // GPIO Port R Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D14: u32 = 0x00004000 ; // GPIO Port Q Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D13: u32 = 0x00002000 ; // GPIO Port P Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D12: u32 = 0x00001000 ; // GPIO Port N Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D11: u32 = 0x00000800 ; // GPIO Port M Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D10: u32 = 0x00000400 ; // GPIO Port L Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D9: u32 = 0x00000200 ; // GPIO Port K Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D8: u32 = 0x00000100 ; // GPIO Port J Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D7: u32 = 0x00000080 ; // GPIO Port H Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D6: u32 = 0x00000040 ; // GPIO Port G Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D5: u32 = 0x00000020 ; // GPIO Port F Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D4: u32 = 0x00000010 ; // GPIO Port E Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D3: u32 = 0x00000008 ; // GPIO Port D Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D2: u32 = 0x00000004 ; // GPIO Port C Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D1: u32 = 0x00000002 ; // GPIO Port B Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCGPIO_D0: u32 = 0x00000001 ; // GPIO Port A Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCDMA register.
//
//*****************************************************************************
pub const SYSCTL_DCGCDMA_D0: u32 = 0x00000001 ; // uDMA Module Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEPI register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEPI_D0: u32 = 0x00000001 ; // EPI Module Deep-Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCHIB register.
//
//*****************************************************************************
pub const SYSCTL_DCGCHIB_D0: u32 = 0x00000001 ; // Hibernation Module Deep-Sleep
// Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUART
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCUART_D7: u32 = 0x00000080 ; // UART Module 7 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART_D6: u32 = 0x00000040 ; // UART Module 6 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART_D5: u32 = 0x00000020 ; // UART Module 5 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART_D4: u32 = 0x00000010 ; // UART Module 4 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART_D3: u32 = 0x00000008 ; // UART Module 3 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART_D2: u32 = 0x00000004 ; // UART Module 2 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART_D1: u32 = 0x00000002 ; // UART Module 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCUART_D0: u32 = 0x00000001 ; // UART Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCSSI register.
//
//*****************************************************************************
pub const SYSCTL_DCGCSSI_D3: u32 = 0x00000008 ; // SSI Module 3 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCSSI_D2: u32 = 0x00000004 ; // SSI Module 2 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCSSI_D1: u32 = 0x00000002 ; // SSI Module 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCSSI_D0: u32 = 0x00000001 ; // SSI Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCI2C register.
//
//*****************************************************************************
pub const SYSCTL_DCGCI2C_D9: u32 = 0x00000200 ; // I2C Module 9 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D8: u32 = 0x00000100 ; // I2C Module 8 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D7: u32 = 0x00000080 ; // I2C Module 7 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D6: u32 = 0x00000040 ; // I2C Module 6 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D5: u32 = 0x00000020 ; // I2C Module 5 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D4: u32 = 0x00000010 ; // I2C Module 4 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D3: u32 = 0x00000008 ; // I2C Module 3 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D2: u32 = 0x00000004 ; // I2C Module 2 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D1: u32 = 0x00000002 ; // I2C Module 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCI2C_D0: u32 = 0x00000001 ; // I2C Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCUSB register.
//
//*****************************************************************************
pub const SYSCTL_DCGCUSB_D0: u32 = 0x00000001 ; // USB Module Deep-Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEPHY
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEPHY_D0: u32 = 0x00000001 ; // PHY Module Deep-Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCCAN register.
//
//*****************************************************************************
pub const SYSCTL_DCGCCAN_D1: u32 = 0x00000002 ; // CAN Module 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCCAN_D0: u32 = 0x00000001 ; // CAN Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCADC register.
//
//*****************************************************************************
pub const SYSCTL_DCGCADC_D1: u32 = 0x00000002 ; // ADC Module 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCADC_D0: u32 = 0x00000001 ; // ADC Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCACMP
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCACMP_D0: u32 = 0x00000001 ; // Analog Comparator Module 0
// Deep-Sleep Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCPWM register.
//
//*****************************************************************************
pub const SYSCTL_DCGCPWM_D1: u32 = 0x00000002 ; // PWM Module 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCPWM_D0: u32 = 0x00000001 ; // PWM Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCQEI register.
//
//*****************************************************************************
pub const SYSCTL_DCGCQEI_D1: u32 = 0x00000002 ; // QEI Module 1 Deep-Sleep Mode
// Clock Gating Control
pub const SYSCTL_DCGCQEI_D0: u32 = 0x00000001 ; // QEI Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEEPROM_D0: u32 = 0x00000001 ; // EEPROM Module Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCWTIMER_D5: u32 = 0x00000020 ; // 32/64-Bit Wide General-Purpose
// Timer 5 Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCWTIMER_D4: u32 = 0x00000010 ; // 32/64-Bit Wide General-Purpose
// Timer 4 Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCWTIMER_D3: u32 = 0x00000008 ; // 32/64-Bit Wide General-Purpose
// Timer 3 Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCWTIMER_D2: u32 = 0x00000004 ; // 32/64-Bit Wide General-Purpose
// Timer 2 Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCWTIMER_D1: u32 = 0x00000002 ; // 32/64-Bit Wide General-Purpose
// Timer 1 Deep-Sleep Mode Clock
// Gating Control
pub const SYSCTL_DCGCWTIMER_D0: u32 = 0x00000001 ; // 32/64-Bit Wide General-Purpose
// Timer 0 Deep-Sleep Mode Clock
// Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCCCM register.
//
//*****************************************************************************
pub const SYSCTL_DCGCCCM_D0: u32 = 0x00000001 ; // CRC and Cryptographic Modules
// Deep-Sleep Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCLCD register.
//
//*****************************************************************************
pub const SYSCTL_DCGCLCD_D0: u32 = 0x00000001 ; // LCD Controller Module 0
// Deep-Sleep Mode Clock Gating
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCOWIRE
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCOWIRE_D0: u32 = 0x00000001 ; // 1-Wire Module 0 Deep-Sleep Mode
// Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_DCGCEMAC
// register.
//
//*****************************************************************************
pub const SYSCTL_DCGCEMAC_D0: u32 = 0x00000001 ; // Ethernet MAC Module 0 Deep-Sleep
// Mode Clock Gating Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCWD register.
//
//*****************************************************************************
pub const SYSCTL_PCWD_P1: u32 = 0x00000002 ; // Watchdog Timer 1 Power Control
pub const SYSCTL_PCWD_P0: u32 = 0x00000001 ; // Watchdog Timer 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCTIMER register.
//
//*****************************************************************************
pub const SYSCTL_PCTIMER_P7: u32 = 0x00000080 ; // General-Purpose Timer 7 Power
// Control
pub const SYSCTL_PCTIMER_P6: u32 = 0x00000040 ; // General-Purpose Timer 6 Power
// Control
pub const SYSCTL_PCTIMER_P5: u32 = 0x00000020 ; // General-Purpose Timer 5 Power
// Control
pub const SYSCTL_PCTIMER_P4: u32 = 0x00000010 ; // General-Purpose Timer 4 Power
// Control
pub const SYSCTL_PCTIMER_P3: u32 = 0x00000008 ; // General-Purpose Timer 3 Power
// Control
pub const SYSCTL_PCTIMER_P2: u32 = 0x00000004 ; // General-Purpose Timer 2 Power
// Control
pub const SYSCTL_PCTIMER_P1: u32 = 0x00000002 ; // General-Purpose Timer 1 Power
// Control
pub const SYSCTL_PCTIMER_P0: u32 = 0x00000001 ; // General-Purpose Timer 0 Power
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCGPIO register.
//
//*****************************************************************************
pub const SYSCTL_PCGPIO_P17: u32 = 0x00020000 ; // GPIO Port T Power Control
pub const SYSCTL_PCGPIO_P16: u32 = 0x00010000 ; // GPIO Port S Power Control
pub const SYSCTL_PCGPIO_P15: u32 = 0x00008000 ; // GPIO Port R Power Control
pub const SYSCTL_PCGPIO_P14: u32 = 0x00004000 ; // GPIO Port Q Power Control
pub const SYSCTL_PCGPIO_P13: u32 = 0x00002000 ; // GPIO Port P Power Control
pub const SYSCTL_PCGPIO_P12: u32 = 0x00001000 ; // GPIO Port N Power Control
pub const SYSCTL_PCGPIO_P11: u32 = 0x00000800 ; // GPIO Port M Power Control
pub const SYSCTL_PCGPIO_P10: u32 = 0x00000400 ; // GPIO Port L Power Control
pub const SYSCTL_PCGPIO_P9: u32 = 0x00000200 ; // GPIO Port K Power Control
pub const SYSCTL_PCGPIO_P8: u32 = 0x00000100 ; // GPIO Port J Power Control
pub const SYSCTL_PCGPIO_P7: u32 = 0x00000080 ; // GPIO Port H Power Control
pub const SYSCTL_PCGPIO_P6: u32 = 0x00000040 ; // GPIO Port G Power Control
pub const SYSCTL_PCGPIO_P5: u32 = 0x00000020 ; // GPIO Port F Power Control
pub const SYSCTL_PCGPIO_P4: u32 = 0x00000010 ; // GPIO Port E Power Control
pub const SYSCTL_PCGPIO_P3: u32 = 0x00000008 ; // GPIO Port D Power Control
pub const SYSCTL_PCGPIO_P2: u32 = 0x00000004 ; // GPIO Port C Power Control
pub const SYSCTL_PCGPIO_P1: u32 = 0x00000002 ; // GPIO Port B Power Control
pub const SYSCTL_PCGPIO_P0: u32 = 0x00000001 ; // GPIO Port A Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCDMA register.
//
//*****************************************************************************
pub const SYSCTL_PCDMA_P0: u32 = 0x00000001 ; // uDMA Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEPI register.
//
//*****************************************************************************
pub const SYSCTL_PCEPI_P0: u32 = 0x00000001 ; // EPI Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCHIB register.
//
//*****************************************************************************
pub const SYSCTL_PCHIB_P0: u32 = 0x00000001 ; // Hibernation Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCUART register.
//
//*****************************************************************************
pub const SYSCTL_PCUART_P7: u32 = 0x00000080 ; // UART Module 7 Power Control
pub const SYSCTL_PCUART_P6: u32 = 0x00000040 ; // UART Module 6 Power Control
pub const SYSCTL_PCUART_P5: u32 = 0x00000020 ; // UART Module 5 Power Control
pub const SYSCTL_PCUART_P4: u32 = 0x00000010 ; // UART Module 4 Power Control
pub const SYSCTL_PCUART_P3: u32 = 0x00000008 ; // UART Module 3 Power Control
pub const SYSCTL_PCUART_P2: u32 = 0x00000004 ; // UART Module 2 Power Control
pub const SYSCTL_PCUART_P1: u32 = 0x00000002 ; // UART Module 1 Power Control
pub const SYSCTL_PCUART_P0: u32 = 0x00000001 ; // UART Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCSSI register.
//
//*****************************************************************************
pub const SYSCTL_PCSSI_P3: u32 = 0x00000008 ; // SSI Module 3 Power Control
pub const SYSCTL_PCSSI_P2: u32 = 0x00000004 ; // SSI Module 2 Power Control
pub const SYSCTL_PCSSI_P1: u32 = 0x00000002 ; // SSI Module 1 Power Control
pub const SYSCTL_PCSSI_P0: u32 = 0x00000001 ; // SSI Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCI2C register.
//
//*****************************************************************************
pub const SYSCTL_PCI2C_P9: u32 = 0x00000200 ; // I2C Module 9 Power Control
pub const SYSCTL_PCI2C_P8: u32 = 0x00000100 ; // I2C Module 8 Power Control
pub const SYSCTL_PCI2C_P7: u32 = 0x00000080 ; // I2C Module 7 Power Control
pub const SYSCTL_PCI2C_P6: u32 = 0x00000040 ; // I2C Module 6 Power Control
pub const SYSCTL_PCI2C_P5: u32 = 0x00000020 ; // I2C Module 5 Power Control
pub const SYSCTL_PCI2C_P4: u32 = 0x00000010 ; // I2C Module 4 Power Control
pub const SYSCTL_PCI2C_P3: u32 = 0x00000008 ; // I2C Module 3 Power Control
pub const SYSCTL_PCI2C_P2: u32 = 0x00000004 ; // I2C Module 2 Power Control
pub const SYSCTL_PCI2C_P1: u32 = 0x00000002 ; // I2C Module 1 Power Control
pub const SYSCTL_PCI2C_P0: u32 = 0x00000001 ; // I2C Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCUSB register.
//
//*****************************************************************************
pub const SYSCTL_PCUSB_P0: u32 = 0x00000001 ; // USB Module Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEPHY register.
//
//*****************************************************************************
pub const SYSCTL_PCEPHY_P0: u32 = 0x00000001 ; // Ethernet PHY Module Power
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCCAN register.
//
//*****************************************************************************
pub const SYSCTL_PCCAN_P1: u32 = 0x00000002 ; // CAN Module 1 Power Control
pub const SYSCTL_PCCAN_P0: u32 = 0x00000001 ; // CAN Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCADC register.
//
//*****************************************************************************
pub const SYSCTL_PCADC_P1: u32 = 0x00000002 ; // ADC Module 1 Power Control
pub const SYSCTL_PCADC_P0: u32 = 0x00000001 ; // ADC Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCACMP register.
//
//*****************************************************************************
pub const SYSCTL_PCACMP_P0: u32 = 0x00000001 ; // Analog Comparator Module 0 Power
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCPWM register.
//
//*****************************************************************************
pub const SYSCTL_PCPWM_P0: u32 = 0x00000001 ; // PWM Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCQEI register.
//
//*****************************************************************************
pub const SYSCTL_PCQEI_P0: u32 = 0x00000001 ; // QEI Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_PCEEPROM_P0: u32 = 0x00000001 ; // EEPROM Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCCCM register.
//
//*****************************************************************************
pub const SYSCTL_PCCCM_P0: u32 = 0x00000001 ; // CRC and Cryptographic Modules
// Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCLCD register.
//
//*****************************************************************************
pub const SYSCTL_PCLCD_P0: u32 = 0x00000001 ; // LCD Controller Module 0 Power
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCOWIRE register.
//
//*****************************************************************************
pub const SYSCTL_PCOWIRE_P0: u32 = 0x00000001 ; // 1-Wire Module 0 Power Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PCEMAC register.
//
//*****************************************************************************
pub const SYSCTL_PCEMAC_P0: u32 = 0x00000001 ; // Ethernet MAC Module 0 Power
// Control

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWD register.
//
//*****************************************************************************
pub const SYSCTL_PRWD_R1: u32 = 0x00000002 ; // Watchdog Timer 1 Peripheral
// Ready
pub const SYSCTL_PRWD_R0: u32 = 0x00000001 ; // Watchdog Timer 0 Peripheral
// Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRTIMER register.
//
//*****************************************************************************
pub const SYSCTL_PRTIMER_R7: u32 = 0x00000080 ; // 16/32-Bit General-Purpose Timer
// 7 Peripheral Ready
pub const SYSCTL_PRTIMER_R6: u32 = 0x00000040 ; // 16/32-Bit General-Purpose Timer
// 6 Peripheral Ready
pub const SYSCTL_PRTIMER_R5: u32 = 0x00000020 ; // 16/32-Bit General-Purpose Timer
// 5 Peripheral Ready
pub const SYSCTL_PRTIMER_R4: u32 = 0x00000010 ; // 16/32-Bit General-Purpose Timer
// 4 Peripheral Ready
pub const SYSCTL_PRTIMER_R3: u32 = 0x00000008 ; // 16/32-Bit General-Purpose Timer
// 3 Peripheral Ready
pub const SYSCTL_PRTIMER_R2: u32 = 0x00000004 ; // 16/32-Bit General-Purpose Timer
// 2 Peripheral Ready
pub const SYSCTL_PRTIMER_R1: u32 = 0x00000002 ; // 16/32-Bit General-Purpose Timer
// 1 Peripheral Ready
pub const SYSCTL_PRTIMER_R0: u32 = 0x00000001 ; // 16/32-Bit General-Purpose Timer
// 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRGPIO register.
//
//*****************************************************************************
pub const SYSCTL_PRGPIO_R17: u32 = 0x00020000 ; // GPIO Port T Peripheral Ready
pub const SYSCTL_PRGPIO_R16: u32 = 0x00010000 ; // GPIO Port S Peripheral Ready
pub const SYSCTL_PRGPIO_R15: u32 = 0x00008000 ; // GPIO Port R Peripheral Ready
pub const SYSCTL_PRGPIO_R14: u32 = 0x00004000 ; // GPIO Port Q Peripheral Ready
pub const SYSCTL_PRGPIO_R13: u32 = 0x00002000 ; // GPIO Port P Peripheral Ready
pub const SYSCTL_PRGPIO_R12: u32 = 0x00001000 ; // GPIO Port N Peripheral Ready
pub const SYSCTL_PRGPIO_R11: u32 = 0x00000800 ; // GPIO Port M Peripheral Ready
pub const SYSCTL_PRGPIO_R10: u32 = 0x00000400 ; // GPIO Port L Peripheral Ready
pub const SYSCTL_PRGPIO_R9: u32 = 0x00000200 ; // GPIO Port K Peripheral Ready
pub const SYSCTL_PRGPIO_R8: u32 = 0x00000100 ; // GPIO Port J Peripheral Ready
pub const SYSCTL_PRGPIO_R7: u32 = 0x00000080 ; // GPIO Port H Peripheral Ready
pub const SYSCTL_PRGPIO_R6: u32 = 0x00000040 ; // GPIO Port G Peripheral Ready
pub const SYSCTL_PRGPIO_R5: u32 = 0x00000020 ; // GPIO Port F Peripheral Ready
pub const SYSCTL_PRGPIO_R4: u32 = 0x00000010 ; // GPIO Port E Peripheral Ready
pub const SYSCTL_PRGPIO_R3: u32 = 0x00000008 ; // GPIO Port D Peripheral Ready
pub const SYSCTL_PRGPIO_R2: u32 = 0x00000004 ; // GPIO Port C Peripheral Ready
pub const SYSCTL_PRGPIO_R1: u32 = 0x00000002 ; // GPIO Port B Peripheral Ready
pub const SYSCTL_PRGPIO_R0: u32 = 0x00000001 ; // GPIO Port A Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRDMA register.
//
//*****************************************************************************
pub const SYSCTL_PRDMA_R0: u32 = 0x00000001 ; // uDMA Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREPI register.
//
//*****************************************************************************
pub const SYSCTL_PREPI_R0: u32 = 0x00000001 ; // EPI Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRHIB register.
//
//*****************************************************************************
pub const SYSCTL_PRHIB_R0: u32 = 0x00000001 ; // Hibernation Module Peripheral
// Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUART register.
//
//*****************************************************************************
pub const SYSCTL_PRUART_R7: u32 = 0x00000080 ; // UART Module 7 Peripheral Ready
pub const SYSCTL_PRUART_R6: u32 = 0x00000040 ; // UART Module 6 Peripheral Ready
pub const SYSCTL_PRUART_R5: u32 = 0x00000020 ; // UART Module 5 Peripheral Ready
pub const SYSCTL_PRUART_R4: u32 = 0x00000010 ; // UART Module 4 Peripheral Ready
pub const SYSCTL_PRUART_R3: u32 = 0x00000008 ; // UART Module 3 Peripheral Ready
pub const SYSCTL_PRUART_R2: u32 = 0x00000004 ; // UART Module 2 Peripheral Ready
pub const SYSCTL_PRUART_R1: u32 = 0x00000002 ; // UART Module 1 Peripheral Ready
pub const SYSCTL_PRUART_R0: u32 = 0x00000001 ; // UART Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRSSI register.
//
//*****************************************************************************
pub const SYSCTL_PRSSI_R3: u32 = 0x00000008 ; // SSI Module 3 Peripheral Ready
pub const SYSCTL_PRSSI_R2: u32 = 0x00000004 ; // SSI Module 2 Peripheral Ready
pub const SYSCTL_PRSSI_R1: u32 = 0x00000002 ; // SSI Module 1 Peripheral Ready
pub const SYSCTL_PRSSI_R0: u32 = 0x00000001 ; // SSI Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRI2C register.
//
//*****************************************************************************
pub const SYSCTL_PRI2C_R9: u32 = 0x00000200 ; // I2C Module 9 Peripheral Ready
pub const SYSCTL_PRI2C_R8: u32 = 0x00000100 ; // I2C Module 8 Peripheral Ready
pub const SYSCTL_PRI2C_R7: u32 = 0x00000080 ; // I2C Module 7 Peripheral Ready
pub const SYSCTL_PRI2C_R6: u32 = 0x00000040 ; // I2C Module 6 Peripheral Ready
pub const SYSCTL_PRI2C_R5: u32 = 0x00000020 ; // I2C Module 5 Peripheral Ready
pub const SYSCTL_PRI2C_R4: u32 = 0x00000010 ; // I2C Module 4 Peripheral Ready
pub const SYSCTL_PRI2C_R3: u32 = 0x00000008 ; // I2C Module 3 Peripheral Ready
pub const SYSCTL_PRI2C_R2: u32 = 0x00000004 ; // I2C Module 2 Peripheral Ready
pub const SYSCTL_PRI2C_R1: u32 = 0x00000002 ; // I2C Module 1 Peripheral Ready
pub const SYSCTL_PRI2C_R0: u32 = 0x00000001 ; // I2C Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRUSB register.
//
//*****************************************************************************
pub const SYSCTL_PRUSB_R0: u32 = 0x00000001 ; // USB Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREPHY register.
//
//*****************************************************************************
pub const SYSCTL_PREPHY_R0: u32 = 0x00000001 ; // Ethernet PHY Module Peripheral
// Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRCAN register.
//
//*****************************************************************************
pub const SYSCTL_PRCAN_R1: u32 = 0x00000002 ; // CAN Module 1 Peripheral Ready
pub const SYSCTL_PRCAN_R0: u32 = 0x00000001 ; // CAN Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRADC register.
//
//*****************************************************************************
pub const SYSCTL_PRADC_R1: u32 = 0x00000002 ; // ADC Module 1 Peripheral Ready
pub const SYSCTL_PRADC_R0: u32 = 0x00000001 ; // ADC Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRACMP register.
//
//*****************************************************************************
pub const SYSCTL_PRACMP_R0: u32 = 0x00000001 ; // Analog Comparator Module 0
// Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRPWM register.
//
//*****************************************************************************
pub const SYSCTL_PRPWM_R1: u32 = 0x00000002 ; // PWM Module 1 Peripheral Ready
pub const SYSCTL_PRPWM_R0: u32 = 0x00000001 ; // PWM Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRQEI register.
//
//*****************************************************************************
pub const SYSCTL_PRQEI_R1: u32 = 0x00000002 ; // QEI Module 1 Peripheral Ready
pub const SYSCTL_PRQEI_R0: u32 = 0x00000001 ; // QEI Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREEPROM
// register.
//
//*****************************************************************************
pub const SYSCTL_PREEPROM_R0: u32 = 0x00000001 ; // EEPROM Module Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRWTIMER
// register.
//
//*****************************************************************************
pub const SYSCTL_PRWTIMER_R5: u32 = 0x00000020 ; // 32/64-Bit Wide General-Purpose
// Timer 5 Peripheral Ready
pub const SYSCTL_PRWTIMER_R4: u32 = 0x00000010 ; // 32/64-Bit Wide General-Purpose
// Timer 4 Peripheral Ready
pub const SYSCTL_PRWTIMER_R3: u32 = 0x00000008 ; // 32/64-Bit Wide General-Purpose
// Timer 3 Peripheral Ready
pub const SYSCTL_PRWTIMER_R2: u32 = 0x00000004 ; // 32/64-Bit Wide General-Purpose
// Timer 2 Peripheral Ready
pub const SYSCTL_PRWTIMER_R1: u32 = 0x00000002 ; // 32/64-Bit Wide General-Purpose
// Timer 1 Peripheral Ready
pub const SYSCTL_PRWTIMER_R0: u32 = 0x00000001 ; // 32/64-Bit Wide General-Purpose
// Timer 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRCCM register.
//
//*****************************************************************************
pub const SYSCTL_PRCCM_R0: u32 = 0x00000001 ; // CRC and Cryptographic Modules
// Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PRLCD register.
//
//*****************************************************************************
pub const SYSCTL_PRLCD_R0: u32 = 0x00000001 ; // LCD Controller Module 0
// Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PROWIRE register.
//
//*****************************************************************************
pub const SYSCTL_PROWIRE_R0: u32 = 0x00000001 ; // 1-Wire Module 0 Peripheral Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_PREMAC register.
//
//*****************************************************************************
pub const SYSCTL_PREMAC_R0: u32 = 0x00000001 ; // Ethernet MAC Module 0 Peripheral
// Ready

//*****************************************************************************
//
// The following are defines for the bit fields in the SYSCTL_CCMCGREQ
// register.
//
//*****************************************************************************
pub const SYSCTL_CCMCGREQ_DESCFG: u32 = 0x00000004 ; // DES Clock Gating Request
pub const SYSCTL_CCMCGREQ_AESCFG: u32 = 0x00000002 ; // AES Clock Gating Request
pub const SYSCTL_CCMCGREQ_SHACFG: u32 = 0x00000001 ; // SHA/MD5 Clock Gating Request

// //*****************************************************************************
// //
// // The following definitions are deprecated.
// //
// //*****************************************************************************
// #ifndef DEPRECATED

// //*****************************************************************************
// //
// // The following are deprecated defines for the bit fields in the SYSCTL_DID0
// // register.
// //
// //*****************************************************************************
// #define SYSCTL_DID0_CLASS_BLIZZARD                                            \
// 0x00050000  // Tiva(TM) C Series TM4C123-class
// // microcontrollers
//     #define SYSCTL_DID0_CLASS_SNOWFLAKE                                           \
// 0x000A0000  // Tiva(TM) C Series TM4C129-class
// // microcontrollers

// //*****************************************************************************
// //
// // The following are deprecated defines for the bit fields in the SYSCTL_PWRTC
// // register.
// //
// //*****************************************************************************
//     pub const SYSCTL_PWRTC_VDDA_UBOR0: u32 = 0x00000010 ; // VDDA Under BOR0 Status
// pub const SYSCTL_PWRTC_VDD_UBOR0: u32 = 0x00000001 ; // VDD Under BOR0 Status


// from driverlib/sysctl.h
//*****************************************************************************
//
// sysctl.h - Prototypes for the system control driver.
//
// Copyright (c) 2005-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.0.12573 of the Tiva Peripheral Driver Library.
//
//*****************************************************************************

//*****************************************************************************
//
// The following are values that can be passed to the
// SysCtlPeripheralPresent(), SysCtlPeripheralEnable(),
// SysCtlPeripheralDisable(), and SysCtlPeripheralReset() APIs as the
// ui32Peripheral parameter.  The peripherals in the fourth group (upper nibble
// is 3) can only be used with the SysCtlPeripheralPresent() API.
//
//*****************************************************************************
pub const  SYSCTL_PERIPH_ADC0      :u32 = 0xf0003800 ; // ADC
pub const  SYSCTL_PERIPH_ADC1      :u32 = 0xf0003801 ; // ADC 1
pub const  SYSCTL_PERIPH_CAN0      :u32 = 0xf0003400 ; // CAN 0
pub const  SYSCTL_PERIPH_CAN1      :u32 = 0xf0003401 ; // CAN 1
pub const  SYSCTL_PERIPH_COMP0     :u32 = 0xf0003c00 ; // Analog Comparator Module 0
pub const  SYSCTL_PERIPH_EMAC0     :u32 = 0xf0009c00 ; // Ethernet MAC0
pub const  SYSCTL_PERIPH_EPHY0     :u32 = 0xf0003000 ; // Ethernet PHY0
pub const  SYSCTL_PERIPH_EPI0      :u32 = 0xf0001000 ; // EPI0
pub const  SYSCTL_PERIPH_GPIOA     :u32 = 0xf0000800 ; // GPIO A
pub const  SYSCTL_PERIPH_GPIOB     :u32 = 0xf0000801 ; // GPIO B
pub const  SYSCTL_PERIPH_GPIOC     :u32 = 0xf0000802 ; // GPIO C
pub const  SYSCTL_PERIPH_GPIOD     :u32 = 0xf0000803 ; // GPIO D
pub const  SYSCTL_PERIPH_GPIOE     :u32 = 0xf0000804 ; // GPIO E
pub const  SYSCTL_PERIPH_GPIOF     :u32 = 0xf0000805 ; // GPIO F
pub const  SYSCTL_PERIPH_GPIOG     :u32 = 0xf0000806 ; // GPIO G
pub const  SYSCTL_PERIPH_GPIOH     :u32 = 0xf0000807 ; // GPIO H
pub const  SYSCTL_PERIPH_GPIOJ     :u32 = 0xf0000808 ; // GPIO J
pub const  SYSCTL_PERIPH_HIBERNATE :u32 = 0xf0001400 ; // Hibernation module
pub const  SYSCTL_PERIPH_CCM0      :u32 = 0xf0007400 ; // CCM 0
pub const  SYSCTL_PERIPH_EEPROM0   :u32 = 0xf0005800 ; // EEPROM 0
pub const  SYSCTL_PERIPH_FAN0      :u32 = 0xf0005400 ; // FAN 0
pub const  SYSCTL_PERIPH_FAN1      :u32 = 0xf0005401 ; // FAN 1
pub const  SYSCTL_PERIPH_GPIOK     :u32 = 0xf0000809 ; // GPIO K
pub const  SYSCTL_PERIPH_GPIOL     :u32 = 0xf000080a ; // GPIO L
pub const  SYSCTL_PERIPH_GPIOM     :u32 = 0xf000080b ; // GPIO M
pub const  SYSCTL_PERIPH_GPION     :u32 = 0xf000080c ; // GPIO N
pub const  SYSCTL_PERIPH_GPIOP     :u32 = 0xf000080d ; // GPIO P
pub const  SYSCTL_PERIPH_GPIOQ     :u32 = 0xf000080e ; // GPIO Q
pub const  SYSCTL_PERIPH_GPIOR     :u32 = 0xf000080f ; // GPIO R
pub const  SYSCTL_PERIPH_GPIOS     :u32 = 0xf0000810 ; // GPIO S
pub const  SYSCTL_PERIPH_GPIOT     :u32 = 0xf0000811 ; // GPIO T
pub const  SYSCTL_PERIPH_I2C0      :u32 = 0xf0002000 ; // I2C 0
pub const  SYSCTL_PERIPH_I2C1      :u32 = 0xf0002001 ; // I2C 1
pub const  SYSCTL_PERIPH_I2C2      :u32 = 0xf0002002 ; // I2C 2
pub const  SYSCTL_PERIPH_I2C3      :u32 = 0xf0002003 ; // I2C 3
pub const  SYSCTL_PERIPH_I2C4      :u32 = 0xf0002004 ; // I2C 4
pub const  SYSCTL_PERIPH_I2C5      :u32 = 0xf0002005 ; // I2C 5
pub const  SYSCTL_PERIPH_I2C6      :u32 = 0xf0002006 ; // I2C 6
pub const  SYSCTL_PERIPH_I2C7      :u32 = 0xf0002007 ; // I2C 7
pub const  SYSCTL_PERIPH_I2C8      :u32 = 0xf0002008 ; // I2C 8
pub const  SYSCTL_PERIPH_I2C9      :u32 = 0xf0002009 ; // I2C 9
pub const  SYSCTL_PERIPH_LCD0      :u32 = 0xf0009000 ; // LCD 0
pub const  SYSCTL_PERIPH_ONEWIRE0  :u32 = 0xf0009800 ; // One Wire 0
pub const  SYSCTL_PERIPH_PWM0      :u32 = 0xf0004000 ; // PWM 0
pub const  SYSCTL_PERIPH_PWM1      :u32 = 0xf0004001 ; // PWM 1
pub const  SYSCTL_PERIPH_QEI0      :u32 = 0xf0004400 ; // QEI 0
pub const  SYSCTL_PERIPH_QEI1      :u32 = 0xf0004401 ; // QEI 1
pub const  SYSCTL_PERIPH_SSI0      :u32 = 0xf0001c00 ; // SSI 0
pub const  SYSCTL_PERIPH_SSI1      :u32 = 0xf0001c01 ; // SSI 1
pub const  SYSCTL_PERIPH_SSI2      :u32 = 0xf0001c02 ; // SSI 2
pub const  SYSCTL_PERIPH_SSI3      :u32 = 0xf0001c03 ; // SSI 3
pub const  SYSCTL_PERIPH_TIMER0    :u32 = 0xf0000400 ; // Timer 0
pub const  SYSCTL_PERIPH_TIMER1    :u32 = 0xf0000401 ; // Timer 1
pub const  SYSCTL_PERIPH_TIMER2    :u32 = 0xf0000402 ; // Timer 2
pub const  SYSCTL_PERIPH_TIMER3    :u32 = 0xf0000403 ; // Timer 3
pub const  SYSCTL_PERIPH_TIMER4    :u32 = 0xf0000404 ; // Timer 4
pub const  SYSCTL_PERIPH_TIMER5    :u32 = 0xf0000405 ; // Timer 5
pub const  SYSCTL_PERIPH_TIMER6    :u32 = 0xf0000406 ; // Timer 6
pub const  SYSCTL_PERIPH_TIMER7    :u32 = 0xf0000407 ; // Timer 7
pub const  SYSCTL_PERIPH_UART0     :u32 = 0xf0001800 ; // UART 0
pub const  SYSCTL_PERIPH_UART1     :u32 = 0xf0001801 ; // UART 1
pub const  SYSCTL_PERIPH_UART2     :u32 = 0xf0001802 ; // UART 2
pub const  SYSCTL_PERIPH_UART3     :u32 = 0xf0001803 ; // UART 3
pub const  SYSCTL_PERIPH_UART4     :u32 = 0xf0001804 ; // UART 4
pub const  SYSCTL_PERIPH_UART5     :u32 = 0xf0001805 ; // UART 5
pub const  SYSCTL_PERIPH_UART6     :u32 = 0xf0001806 ; // UART 6
pub const  SYSCTL_PERIPH_UART7     :u32 = 0xf0001807 ; // UART 7
pub const  SYSCTL_PERIPH_UDMA      :u32 = 0xf0000c00 ; // uDMA
pub const  SYSCTL_PERIPH_USB0      :u32 = 0xf0002800 ; // USB 0
pub const  SYSCTL_PERIPH_WDOG0     :u32 = 0xf0000000 ; // Watchdog 0
pub const  SYSCTL_PERIPH_WDOG1     :u32 = 0xf0000001 ; // Watchdog 1
pub const  SYSCTL_PERIPH_WTIMER0   :u32 = 0xf0005c00 ; // Wide Timer 0
pub const  SYSCTL_PERIPH_WTIMER1   :u32 = 0xf0005c01 ; // Wide Timer 1
pub const  SYSCTL_PERIPH_WTIMER2   :u32 = 0xf0005c02 ; // Wide Timer 2
pub const  SYSCTL_PERIPH_WTIMER3   :u32 = 0xf0005c03 ; // Wide Timer 3
pub const  SYSCTL_PERIPH_WTIMER4   :u32 = 0xf0005c04 ; // Wide Timer 4
pub const  SYSCTL_PERIPH_WTIMER5   :u32 = 0xf0005c05 ; // Wide Timer 5

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlLDOSleepSet() and
// SysCtlLDODeepSleepSet() APIs as the ui32Voltage value, or returned by the
// SysCtlLDOSleepGet() and SysCtlLDODeepSleepGet() APIs.
//
//*****************************************************************************
pub const  SYSCTL_LDO_0_90V        :u32 = 0x80000012 ; // LDO output of 0.90V
pub const  SYSCTL_LDO_0_95V        :u32 = 0x80000013 ; // LDO output of 0.95V
pub const  SYSCTL_LDO_1_00V        :u32 = 0x80000014 ; // LDO output of 1.00V
pub const  SYSCTL_LDO_1_05V        :u32 = 0x80000015 ; // LDO output of 1.05V
pub const  SYSCTL_LDO_1_10V        :u32 = 0x80000016 ; // LDO output of 1.10V
pub const  SYSCTL_LDO_1_15V        :u32 = 0x80000017 ; // LDO output of 1.15V
pub const  SYSCTL_LDO_1_20V        :u32 = 0x80000018 ; // LDO output of 1.20V

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlIntEnable(),
// SysCtlIntDisable(), and SysCtlIntClear() APIs, or returned in the bit mask
// by the SysCtlIntStatus() API.
//
//*****************************************************************************
pub const  SYSCTL_INT_BOR0         :u32 = 0x00000800 ; // VDD under BOR0
pub const  SYSCTL_INT_VDDA_OK      :u32 = 0x00000400 ; // VDDA Power OK
pub const  SYSCTL_INT_MOSC_PUP     :u32 = 0x00000100 ; // MOSC power-up interrupt
pub const  SYSCTL_INT_USBPLL_LOCK  :u32 = 0x00000080 ; // USB PLL lock interrupt
pub const  SYSCTL_INT_PLL_LOCK     :u32 = 0x00000040 ; // PLL lock interrupt
pub const  SYSCTL_INT_MOSC_FAIL    :u32 = 0x00000008 ; // Main oscillator failure int
pub const  SYSCTL_INT_BOR1         :u32 = 0x00000002 ; // VDD under BOR1
pub const  SYSCTL_INT_BOR          :u32 = 0x00000002 ; // Brown out interrupt

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlResetCauseClear()
// API or returned by the SysCtlResetCauseGet() API.
//
//*****************************************************************************
pub const  SYSCTL_CAUSE_HSRVREQ    :u32 = 0x00001000 ; // Hardware System Service Request
pub const  SYSCTL_CAUSE_HIB        :u32 = 0x00000040 ; // Hibernate reset
pub const  SYSCTL_CAUSE_WDOG1      :u32 = 0x00000020 ; // Watchdog 1 reset
pub const  SYSCTL_CAUSE_SW         :u32 = 0x00000010 ; // Software reset
pub const  SYSCTL_CAUSE_WDOG0      :u32 = 0x00000008 ; // Watchdog 0 reset
pub const  SYSCTL_CAUSE_BOR        :u32 = 0x00000004 ; // Brown-out reset
pub const  SYSCTL_CAUSE_POR        :u32 = 0x00000002 ; // Power on reset
pub const  SYSCTL_CAUSE_EXT        :u32 = 0x00000001 ; // External reset

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlBrownOutConfigSet()
// API as the ui32Config parameter.
//
//*****************************************************************************
pub const  SYSCTL_BOR_RESET        :u32 = 0x00000002 ; // Reset instead of interrupting
pub const  SYSCTL_BOR_RESAMPLE     :u32 = 0x00000001 ; // Resample BOR before asserting

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlPWMClockSet() API
// as the ui32Config parameter, and can be returned by the SysCtlPWMClockGet()
// API.
//
//*****************************************************************************
pub const  SYSCTL_PWMDIV_1         :u32 = 0x00000000 ; // PWM clock is processor clock /1
pub const  SYSCTL_PWMDIV_2         :u32 = 0x00100000 ; // PWM clock is processor clock /2
pub const  SYSCTL_PWMDIV_4         :u32 = 0x00120000 ; // PWM clock is processor clock /4
pub const  SYSCTL_PWMDIV_8         :u32 = 0x00140000 ; // PWM clock is processor clock /8
pub const  SYSCTL_PWMDIV_16        :u32 = 0x00160000 ; // PWM clock is processor clock /16
pub const  SYSCTL_PWMDIV_32        :u32 = 0x00180000 ; // PWM clock is processor clock /32
pub const  SYSCTL_PWMDIV_64        :u32 = 0x001A0000 ; // PWM clock is processor clock /64

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlClockSet() API as
// the ui32Config parameter.
//
//*****************************************************************************
pub const  SYSCTL_SYSDIV_1         :u32 = 0x07800000 ; // Processor clock is osc/pll /1
pub const  SYSCTL_SYSDIV_2         :u32 = 0x00C00000 ; // Processor clock is osc/pll /2
pub const  SYSCTL_SYSDIV_3         :u32 = 0x01400000 ; // Processor clock is osc/pll /3
pub const  SYSCTL_SYSDIV_4         :u32 = 0x01C00000 ; // Processor clock is osc/pll /4
pub const  SYSCTL_SYSDIV_5         :u32 = 0x02400000 ; // Processor clock is osc/pll /5
pub const  SYSCTL_SYSDIV_6         :u32 = 0x02C00000 ; // Processor clock is osc/pll /6
pub const  SYSCTL_SYSDIV_7         :u32 = 0x03400000 ; // Processor clock is osc/pll /7
pub const  SYSCTL_SYSDIV_8         :u32 = 0x03C00000 ; // Processor clock is osc/pll /8
pub const  SYSCTL_SYSDIV_9         :u32 = 0x04400000 ; // Processor clock is osc/pll /9
pub const  SYSCTL_SYSDIV_10        :u32 = 0x04C00000 ; // Processor clock is osc/pll /10
pub const  SYSCTL_SYSDIV_11        :u32 = 0x05400000 ; // Processor clock is osc/pll /11
pub const  SYSCTL_SYSDIV_12        :u32 = 0x05C00000 ; // Processor clock is osc/pll /12
pub const  SYSCTL_SYSDIV_13        :u32 = 0x06400000 ; // Processor clock is osc/pll /13
pub const  SYSCTL_SYSDIV_14        :u32 = 0x06C00000 ; // Processor clock is osc/pll /14
pub const  SYSCTL_SYSDIV_15        :u32 = 0x07400000 ; // Processor clock is osc/pll /15
pub const  SYSCTL_SYSDIV_16        :u32 = 0x07C00000 ; // Processor clock is osc/pll /16
pub const  SYSCTL_SYSDIV_17        :u32 = 0x88400000 ; // Processor clock is osc/pll /17
pub const  SYSCTL_SYSDIV_18        :u32 = 0x88C00000 ; // Processor clock is osc/pll /18
pub const  SYSCTL_SYSDIV_19        :u32 = 0x89400000 ; // Processor clock is osc/pll /19
pub const  SYSCTL_SYSDIV_20        :u32 = 0x89C00000 ; // Processor clock is osc/pll /20
pub const  SYSCTL_SYSDIV_21        :u32 = 0x8A400000 ; // Processor clock is osc/pll /21
pub const  SYSCTL_SYSDIV_22        :u32 = 0x8AC00000 ; // Processor clock is osc/pll /22
pub const  SYSCTL_SYSDIV_23        :u32 = 0x8B400000 ; // Processor clock is osc/pll /23
pub const  SYSCTL_SYSDIV_24        :u32 = 0x8BC00000 ; // Processor clock is osc/pll /24
pub const  SYSCTL_SYSDIV_25        :u32 = 0x8C400000 ; // Processor clock is osc/pll /25
pub const  SYSCTL_SYSDIV_26        :u32 = 0x8CC00000 ; // Processor clock is osc/pll /26
pub const  SYSCTL_SYSDIV_27        :u32 = 0x8D400000 ; // Processor clock is osc/pll /27
pub const  SYSCTL_SYSDIV_28        :u32 = 0x8DC00000 ; // Processor clock is osc/pll /28
pub const  SYSCTL_SYSDIV_29        :u32 = 0x8E400000 ; // Processor clock is osc/pll /29
pub const  SYSCTL_SYSDIV_30        :u32 = 0x8EC00000 ; // Processor clock is osc/pll /30
pub const  SYSCTL_SYSDIV_31        :u32 = 0x8F400000 ; // Processor clock is osc/pll /31
pub const  SYSCTL_SYSDIV_32        :u32 = 0x8FC00000 ; // Processor clock is osc/pll /32
pub const  SYSCTL_SYSDIV_33        :u32 = 0x90400000 ; // Processor clock is osc/pll /33
pub const  SYSCTL_SYSDIV_34        :u32 = 0x90C00000 ; // Processor clock is osc/pll /34
pub const  SYSCTL_SYSDIV_35        :u32 = 0x91400000 ; // Processor clock is osc/pll /35
pub const  SYSCTL_SYSDIV_36        :u32 = 0x91C00000 ; // Processor clock is osc/pll /36
pub const  SYSCTL_SYSDIV_37        :u32 = 0x92400000 ; // Processor clock is osc/pll /37
pub const  SYSCTL_SYSDIV_38        :u32 = 0x92C00000 ; // Processor clock is osc/pll /38
pub const  SYSCTL_SYSDIV_39        :u32 = 0x93400000 ; // Processor clock is osc/pll /39
pub const  SYSCTL_SYSDIV_40        :u32 = 0x93C00000 ; // Processor clock is osc/pll /40
pub const  SYSCTL_SYSDIV_41        :u32 = 0x94400000 ; // Processor clock is osc/pll /41
pub const  SYSCTL_SYSDIV_42        :u32 = 0x94C00000 ; // Processor clock is osc/pll /42
pub const  SYSCTL_SYSDIV_43        :u32 = 0x95400000 ; // Processor clock is osc/pll /43
pub const  SYSCTL_SYSDIV_44        :u32 = 0x95C00000 ; // Processor clock is osc/pll /44
pub const  SYSCTL_SYSDIV_45        :u32 = 0x96400000 ; // Processor clock is osc/pll /45
pub const  SYSCTL_SYSDIV_46        :u32 = 0x96C00000 ; // Processor clock is osc/pll /46
pub const  SYSCTL_SYSDIV_47        :u32 = 0x97400000 ; // Processor clock is osc/pll /47
pub const  SYSCTL_SYSDIV_48        :u32 = 0x97C00000 ; // Processor clock is osc/pll /48
pub const  SYSCTL_SYSDIV_49        :u32 = 0x98400000 ; // Processor clock is osc/pll /49
pub const  SYSCTL_SYSDIV_50        :u32 = 0x98C00000 ; // Processor clock is osc/pll /50
pub const  SYSCTL_SYSDIV_51        :u32 = 0x99400000 ; // Processor clock is osc/pll /51
pub const  SYSCTL_SYSDIV_52        :u32 = 0x99C00000 ; // Processor clock is osc/pll /52
pub const  SYSCTL_SYSDIV_53        :u32 = 0x9A400000 ; // Processor clock is osc/pll /53
pub const  SYSCTL_SYSDIV_54        :u32 = 0x9AC00000 ; // Processor clock is osc/pll /54
pub const  SYSCTL_SYSDIV_55        :u32 = 0x9B400000 ; // Processor clock is osc/pll /55
pub const  SYSCTL_SYSDIV_56        :u32 = 0x9BC00000 ; // Processor clock is osc/pll /56
pub const  SYSCTL_SYSDIV_57        :u32 = 0x9C400000 ; // Processor clock is osc/pll /57
pub const  SYSCTL_SYSDIV_58        :u32 = 0x9CC00000 ; // Processor clock is osc/pll /58
pub const  SYSCTL_SYSDIV_59        :u32 = 0x9D400000 ; // Processor clock is osc/pll /59
pub const  SYSCTL_SYSDIV_60        :u32 = 0x9DC00000 ; // Processor clock is osc/pll /60
pub const  SYSCTL_SYSDIV_61        :u32 = 0x9E400000 ; // Processor clock is osc/pll /61
pub const  SYSCTL_SYSDIV_62        :u32 = 0x9EC00000 ; // Processor clock is osc/pll /62
pub const  SYSCTL_SYSDIV_63        :u32 = 0x9F400000 ; // Processor clock is osc/pll /63
pub const  SYSCTL_SYSDIV_64        :u32 = 0x9FC00000 ; // Processor clock is osc/pll /64
pub const  SYSCTL_SYSDIV_2_5       :u32 = 0xC1000000 ; // Processor clock is pll / 2.5
pub const  SYSCTL_SYSDIV_3_5       :u32 = 0xC1800000 ; // Processor clock is pll / 3.5
pub const  SYSCTL_SYSDIV_4_5       :u32 = 0xC2000000 ; // Processor clock is pll / 4.5
pub const  SYSCTL_SYSDIV_5_5       :u32 = 0xC2800000 ; // Processor clock is pll / 5.5
pub const  SYSCTL_SYSDIV_6_5       :u32 = 0xC3000000 ; // Processor clock is pll / 6.5
pub const  SYSCTL_SYSDIV_7_5       :u32 = 0xC3800000 ; // Processor clock is pll / 7.5
pub const  SYSCTL_SYSDIV_8_5       :u32 = 0xC4000000 ; // Processor clock is pll / 8.5
pub const  SYSCTL_SYSDIV_9_5       :u32 = 0xC4800000 ; // Processor clock is pll / 9.5
pub const  SYSCTL_SYSDIV_10_5      :u32 = 0xC5000000 ; // Processor clock is pll / 10.5
pub const  SYSCTL_SYSDIV_11_5      :u32 = 0xC5800000 ; // Processor clock is pll / 11.5
pub const  SYSCTL_SYSDIV_12_5      :u32 = 0xC6000000 ; // Processor clock is pll / 12.5
pub const  SYSCTL_SYSDIV_13_5      :u32 = 0xC6800000 ; // Processor clock is pll / 13.5
pub const  SYSCTL_SYSDIV_14_5      :u32 = 0xC7000000 ; // Processor clock is pll / 14.5
pub const  SYSCTL_SYSDIV_15_5      :u32 = 0xC7800000 ; // Processor clock is pll / 15.5
pub const  SYSCTL_SYSDIV_16_5      :u32 = 0xC8000000 ; // Processor clock is pll / 16.5
pub const  SYSCTL_SYSDIV_17_5      :u32 = 0xC8800000 ; // Processor clock is pll / 17.5
pub const  SYSCTL_SYSDIV_18_5      :u32 = 0xC9000000 ; // Processor clock is pll / 18.5
pub const  SYSCTL_SYSDIV_19_5      :u32 = 0xC9800000 ; // Processor clock is pll / 19.5
pub const  SYSCTL_SYSDIV_20_5      :u32 = 0xCA000000 ; // Processor clock is pll / 20.5
pub const  SYSCTL_SYSDIV_21_5      :u32 = 0xCA800000 ; // Processor clock is pll / 21.5
pub const  SYSCTL_SYSDIV_22_5      :u32 = 0xCB000000 ; // Processor clock is pll / 22.5
pub const  SYSCTL_SYSDIV_23_5      :u32 = 0xCB800000 ; // Processor clock is pll / 23.5
pub const  SYSCTL_SYSDIV_24_5      :u32 = 0xCC000000 ; // Processor clock is pll / 24.5
pub const  SYSCTL_SYSDIV_25_5      :u32 = 0xCC800000 ; // Processor clock is pll / 25.5
pub const  SYSCTL_SYSDIV_26_5      :u32 = 0xCD000000 ; // Processor clock is pll / 26.5
pub const  SYSCTL_SYSDIV_27_5      :u32 = 0xCD800000 ; // Processor clock is pll / 27.5
pub const  SYSCTL_SYSDIV_28_5      :u32 = 0xCE000000 ; // Processor clock is pll / 28.5
pub const  SYSCTL_SYSDIV_29_5      :u32 = 0xCE800000 ; // Processor clock is pll / 29.5
pub const  SYSCTL_SYSDIV_30_5      :u32 = 0xCF000000 ; // Processor clock is pll / 30.5
pub const  SYSCTL_SYSDIV_31_5      :u32 = 0xCF800000 ; // Processor clock is pll / 31.5
pub const  SYSCTL_SYSDIV_32_5      :u32 = 0xD0000000 ; // Processor clock is pll / 32.5
pub const  SYSCTL_SYSDIV_33_5      :u32 = 0xD0800000 ; // Processor clock is pll / 33.5
pub const  SYSCTL_SYSDIV_34_5      :u32 = 0xD1000000 ; // Processor clock is pll / 34.5
pub const  SYSCTL_SYSDIV_35_5      :u32 = 0xD1800000 ; // Processor clock is pll / 35.5
pub const  SYSCTL_SYSDIV_36_5      :u32 = 0xD2000000 ; // Processor clock is pll / 36.5
pub const  SYSCTL_SYSDIV_37_5      :u32 = 0xD2800000 ; // Processor clock is pll / 37.5
pub const  SYSCTL_SYSDIV_38_5      :u32 = 0xD3000000 ; // Processor clock is pll / 38.5
pub const  SYSCTL_SYSDIV_39_5      :u32 = 0xD3800000 ; // Processor clock is pll / 39.5
pub const  SYSCTL_SYSDIV_40_5      :u32 = 0xD4000000 ; // Processor clock is pll / 40.5
pub const  SYSCTL_SYSDIV_41_5      :u32 = 0xD4800000 ; // Processor clock is pll / 41.5
pub const  SYSCTL_SYSDIV_42_5      :u32 = 0xD5000000 ; // Processor clock is pll / 42.5
pub const  SYSCTL_SYSDIV_43_5      :u32 = 0xD5800000 ; // Processor clock is pll / 43.5
pub const  SYSCTL_SYSDIV_44_5      :u32 = 0xD6000000 ; // Processor clock is pll / 44.5
pub const  SYSCTL_SYSDIV_45_5      :u32 = 0xD6800000 ; // Processor clock is pll / 45.5
pub const  SYSCTL_SYSDIV_46_5      :u32 = 0xD7000000 ; // Processor clock is pll / 46.5
pub const  SYSCTL_SYSDIV_47_5      :u32 = 0xD7800000 ; // Processor clock is pll / 47.5
pub const  SYSCTL_SYSDIV_48_5      :u32 = 0xD8000000 ; // Processor clock is pll / 48.5
pub const  SYSCTL_SYSDIV_49_5      :u32 = 0xD8800000 ; // Processor clock is pll / 49.5
pub const  SYSCTL_SYSDIV_50_5      :u32 = 0xD9000000 ; // Processor clock is pll / 50.5
pub const  SYSCTL_SYSDIV_51_5      :u32 = 0xD9800000 ; // Processor clock is pll / 51.5
pub const  SYSCTL_SYSDIV_52_5      :u32 = 0xDA000000 ; // Processor clock is pll / 52.5
pub const  SYSCTL_SYSDIV_53_5      :u32 = 0xDA800000 ; // Processor clock is pll / 53.5
pub const  SYSCTL_SYSDIV_54_5      :u32 = 0xDB000000 ; // Processor clock is pll / 54.5
pub const  SYSCTL_SYSDIV_55_5      :u32 = 0xDB800000 ; // Processor clock is pll / 55.5
pub const  SYSCTL_SYSDIV_56_5      :u32 = 0xDC000000 ; // Processor clock is pll / 56.5
pub const  SYSCTL_SYSDIV_57_5      :u32 = 0xDC800000 ; // Processor clock is pll / 57.5
pub const  SYSCTL_SYSDIV_58_5      :u32 = 0xDD000000 ; // Processor clock is pll / 58.5
pub const  SYSCTL_SYSDIV_59_5      :u32 = 0xDD800000 ; // Processor clock is pll / 59.5
pub const  SYSCTL_SYSDIV_60_5      :u32 = 0xDE000000 ; // Processor clock is pll / 60.5
pub const  SYSCTL_SYSDIV_61_5      :u32 = 0xDE800000 ; // Processor clock is pll / 61.5
pub const  SYSCTL_SYSDIV_62_5      :u32 = 0xDF000000 ; // Processor clock is pll / 62.5
pub const  SYSCTL_SYSDIV_63_5      :u32 = 0xDF800000 ; // Processor clock is pll / 63.5
pub const  SYSCTL_CFG_VCO_480      :u32 = 0xF1000000 ; // VCO is 480 MHz
pub const  SYSCTL_CFG_VCO_320      :u32 = 0xF0000000 ; // VCO is 320 MHz
pub const  SYSCTL_USE_PLL          :u32 = 0x00000000 ; // System clock is the PLL clock
pub const  SYSCTL_USE_OSC          :u32 = 0x00003800 ; // System clock is the osc clock
pub const  SYSCTL_XTAL_1MHZ        :u32 = 0x00000000 ; // External crystal is 1MHz
pub const  SYSCTL_XTAL_1_84MHZ     :u32 = 0x00000040 ; // External crystal is 1.8432MHz
pub const  SYSCTL_XTAL_2MHZ        :u32 = 0x00000080 ; // External crystal is 2MHz
pub const  SYSCTL_XTAL_2_45MHZ     :u32 = 0x000000C0 ; // External crystal is 2.4576MHz
pub const  SYSCTL_XTAL_3_57MHZ     :u32 = 0x00000100 ; // External crystal is 3.579545MHz
pub const  SYSCTL_XTAL_3_68MHZ     :u32 = 0x00000140 ; // External crystal is 3.6864MHz
pub const  SYSCTL_XTAL_4MHZ        :u32 = 0x00000180 ; // External crystal is 4MHz
pub const  SYSCTL_XTAL_4_09MHZ     :u32 = 0x000001C0 ; // External crystal is 4.096MHz
pub const  SYSCTL_XTAL_4_91MHZ     :u32 = 0x00000200 ; // External crystal is 4.9152MHz
pub const  SYSCTL_XTAL_5MHZ        :u32 = 0x00000240 ; // External crystal is 5MHz
pub const  SYSCTL_XTAL_5_12MHZ     :u32 = 0x00000280 ; // External crystal is 5.12MHz
pub const  SYSCTL_XTAL_6MHZ        :u32 = 0x000002C0 ; // External crystal is 6MHz
pub const  SYSCTL_XTAL_6_14MHZ     :u32 = 0x00000300 ; // External crystal is 6.144MHz
pub const  SYSCTL_XTAL_7_37MHZ     :u32 = 0x00000340 ; // External crystal is 7.3728MHz
pub const  SYSCTL_XTAL_8MHZ        :u32 = 0x00000380 ; // External crystal is 8MHz
pub const  SYSCTL_XTAL_8_19MHZ     :u32 = 0x000003C0 ; // External crystal is 8.192MHz
pub const  SYSCTL_XTAL_10MHZ       :u32 = 0x00000400 ; // External crystal is 10 MHz
pub const  SYSCTL_XTAL_12MHZ       :u32 = 0x00000440 ; // External crystal is 12 MHz
pub const  SYSCTL_XTAL_12_2MHZ     :u32 = 0x00000480 ; // External crystal is 12.288 MHz
pub const  SYSCTL_XTAL_13_5MHZ     :u32 = 0x000004C0 ; // External crystal is 13.56 MHz
pub const  SYSCTL_XTAL_14_3MHZ     :u32 = 0x00000500 ; // External crystal is 14.31818 MHz
pub const  SYSCTL_XTAL_16MHZ       :u32 = 0x00000540 ; // External crystal is 16 MHz
pub const  SYSCTL_XTAL_16_3MHZ     :u32 = 0x00000580 ; // External crystal is 16.384 MHz
pub const  SYSCTL_XTAL_18MHZ       :u32 = 0x000005C0 ; // External crystal is 18.0 MHz
pub const  SYSCTL_XTAL_20MHZ       :u32 = 0x00000600 ; // External crystal is 20.0 MHz
pub const  SYSCTL_XTAL_24MHZ       :u32 = 0x00000640 ; // External crystal is 24.0 MHz
pub const  SYSCTL_XTAL_25MHZ       :u32 = 0x00000680 ; // External crystal is 25.0 MHz
pub const  SYSCTL_OSC_MAIN         :u32 = 0x00000000 ; // Osc source is main osc
pub const  SYSCTL_OSC_INT          :u32 = 0x00000010 ; // Osc source is int. osc
pub const  SYSCTL_OSC_INT4         :u32 = 0x00000020 ; // Osc source is int. osc /4
pub const  SYSCTL_OSC_INT30        :u32 = 0x00000030 ; // Osc source is int. 30 KHz
pub const  SYSCTL_OSC_EXT32        :u32 = 0x80000038 ; // Osc source is ext. 32 KHz
pub const  SYSCTL_INT_OSC_DIS      :u32 = 0x00000002 ; // Disable internal oscillator
pub const  SYSCTL_MAIN_OSC_DIS     :u32 = 0x00000001 ; // Disable main oscillator

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlDeepSleepClockSet()
// API as the ui32Config parameter.
//
//*****************************************************************************
pub const  SYSCTL_DSLP_DIV_1       :u32 = 0x00000000 ; // Deep-sleep clock is osc /1
pub const  SYSCTL_DSLP_DIV_2       :u32 = 0x00800000 ; // Deep-sleep clock is osc /2
pub const  SYSCTL_DSLP_DIV_3       :u32 = 0x01000000 ; // Deep-sleep clock is osc /3
pub const  SYSCTL_DSLP_DIV_4       :u32 = 0x01800000 ; // Deep-sleep clock is osc /4
pub const  SYSCTL_DSLP_DIV_5       :u32 = 0x02000000 ; // Deep-sleep clock is osc /5
pub const  SYSCTL_DSLP_DIV_6       :u32 = 0x02800000 ; // Deep-sleep clock is osc /6
pub const  SYSCTL_DSLP_DIV_7       :u32 = 0x03000000 ; // Deep-sleep clock is osc /7
pub const  SYSCTL_DSLP_DIV_8       :u32 = 0x03800000 ; // Deep-sleep clock is osc /8
pub const  SYSCTL_DSLP_DIV_9       :u32 = 0x04000000 ; // Deep-sleep clock is osc /9
pub const  SYSCTL_DSLP_DIV_10      :u32 = 0x04800000 ; // Deep-sleep clock is osc /10
pub const  SYSCTL_DSLP_DIV_11      :u32 = 0x05000000 ; // Deep-sleep clock is osc /11
pub const  SYSCTL_DSLP_DIV_12      :u32 = 0x05800000 ; // Deep-sleep clock is osc /12
pub const  SYSCTL_DSLP_DIV_13      :u32 = 0x06000000 ; // Deep-sleep clock is osc /13
pub const  SYSCTL_DSLP_DIV_14      :u32 = 0x06800000 ; // Deep-sleep clock is osc /14
pub const  SYSCTL_DSLP_DIV_15      :u32 = 0x07000000 ; // Deep-sleep clock is osc /15
pub const  SYSCTL_DSLP_DIV_16      :u32 = 0x07800000 ; // Deep-sleep clock is osc /16
pub const  SYSCTL_DSLP_DIV_17      :u32 = 0x08000000 ; // Deep-sleep clock is osc /17
pub const  SYSCTL_DSLP_DIV_18      :u32 = 0x08800000 ; // Deep-sleep clock is osc /18
pub const  SYSCTL_DSLP_DIV_19      :u32 = 0x09000000 ; // Deep-sleep clock is osc /19
pub const  SYSCTL_DSLP_DIV_20      :u32 = 0x09800000 ; // Deep-sleep clock is osc /20
pub const  SYSCTL_DSLP_DIV_21      :u32 = 0x0A000000 ; // Deep-sleep clock is osc /21
pub const  SYSCTL_DSLP_DIV_22      :u32 = 0x0A800000 ; // Deep-sleep clock is osc /22
pub const  SYSCTL_DSLP_DIV_23      :u32 = 0x0B000000 ; // Deep-sleep clock is osc /23
pub const  SYSCTL_DSLP_DIV_24      :u32 = 0x0B800000 ; // Deep-sleep clock is osc /24
pub const  SYSCTL_DSLP_DIV_25      :u32 = 0x0C000000 ; // Deep-sleep clock is osc /25
pub const  SYSCTL_DSLP_DIV_26      :u32 = 0x0C800000 ; // Deep-sleep clock is osc /26
pub const  SYSCTL_DSLP_DIV_27      :u32 = 0x0D000000 ; // Deep-sleep clock is osc /27
pub const  SYSCTL_DSLP_DIV_28      :u32 = 0x0D800000 ; // Deep-sleep clock is osc /28
pub const  SYSCTL_DSLP_DIV_29      :u32 = 0x0E000000 ; // Deep-sleep clock is osc /29
pub const  SYSCTL_DSLP_DIV_30      :u32 = 0x0E800000 ; // Deep-sleep clock is osc /30
pub const  SYSCTL_DSLP_DIV_31      :u32 = 0x0F000000 ; // Deep-sleep clock is osc /31
pub const  SYSCTL_DSLP_DIV_32      :u32 = 0x0F800000 ; // Deep-sleep clock is osc /32
pub const  SYSCTL_DSLP_DIV_33      :u32 = 0x10000000 ; // Deep-sleep clock is osc /33
pub const  SYSCTL_DSLP_DIV_34      :u32 = 0x10800000 ; // Deep-sleep clock is osc /34
pub const  SYSCTL_DSLP_DIV_35      :u32 = 0x11000000 ; // Deep-sleep clock is osc /35
pub const  SYSCTL_DSLP_DIV_36      :u32 = 0x11800000 ; // Deep-sleep clock is osc /36
pub const  SYSCTL_DSLP_DIV_37      :u32 = 0x12000000 ; // Deep-sleep clock is osc /37
pub const  SYSCTL_DSLP_DIV_38      :u32 = 0x12800000 ; // Deep-sleep clock is osc /38
pub const  SYSCTL_DSLP_DIV_39      :u32 = 0x13000000 ; // Deep-sleep clock is osc /39
pub const  SYSCTL_DSLP_DIV_40      :u32 = 0x13800000 ; // Deep-sleep clock is osc /40
pub const  SYSCTL_DSLP_DIV_41      :u32 = 0x14000000 ; // Deep-sleep clock is osc /41
pub const  SYSCTL_DSLP_DIV_42      :u32 = 0x14800000 ; // Deep-sleep clock is osc /42
pub const  SYSCTL_DSLP_DIV_43      :u32 = 0x15000000 ; // Deep-sleep clock is osc /43
pub const  SYSCTL_DSLP_DIV_44      :u32 = 0x15800000 ; // Deep-sleep clock is osc /44
pub const  SYSCTL_DSLP_DIV_45      :u32 = 0x16000000 ; // Deep-sleep clock is osc /45
pub const  SYSCTL_DSLP_DIV_46      :u32 = 0x16800000 ; // Deep-sleep clock is osc /46
pub const  SYSCTL_DSLP_DIV_47      :u32 = 0x17000000 ; // Deep-sleep clock is osc /47
pub const  SYSCTL_DSLP_DIV_48      :u32 = 0x17800000 ; // Deep-sleep clock is osc /48
pub const  SYSCTL_DSLP_DIV_49      :u32 = 0x18000000 ; // Deep-sleep clock is osc /49
pub const  SYSCTL_DSLP_DIV_50      :u32 = 0x18800000 ; // Deep-sleep clock is osc /50
pub const  SYSCTL_DSLP_DIV_51      :u32 = 0x19000000 ; // Deep-sleep clock is osc /51
pub const  SYSCTL_DSLP_DIV_52      :u32 = 0x19800000 ; // Deep-sleep clock is osc /52
pub const  SYSCTL_DSLP_DIV_53      :u32 = 0x1A000000 ; // Deep-sleep clock is osc /53
pub const  SYSCTL_DSLP_DIV_54      :u32 = 0x1A800000 ; // Deep-sleep clock is osc /54
pub const  SYSCTL_DSLP_DIV_55      :u32 = 0x1B000000 ; // Deep-sleep clock is osc /55
pub const  SYSCTL_DSLP_DIV_56      :u32 = 0x1B800000 ; // Deep-sleep clock is osc /56
pub const  SYSCTL_DSLP_DIV_57      :u32 = 0x1C000000 ; // Deep-sleep clock is osc /57
pub const  SYSCTL_DSLP_DIV_58      :u32 = 0x1C800000 ; // Deep-sleep clock is osc /58
pub const  SYSCTL_DSLP_DIV_59      :u32 = 0x1D000000 ; // Deep-sleep clock is osc /59
pub const  SYSCTL_DSLP_DIV_60      :u32 = 0x1D800000 ; // Deep-sleep clock is osc /60
pub const  SYSCTL_DSLP_DIV_61      :u32 = 0x1E000000 ; // Deep-sleep clock is osc /61
pub const  SYSCTL_DSLP_DIV_62      :u32 = 0x1E800000 ; // Deep-sleep clock is osc /62
pub const  SYSCTL_DSLP_DIV_63      :u32 = 0x1F000000 ; // Deep-sleep clock is osc /63
pub const  SYSCTL_DSLP_DIV_64      :u32 = 0x1F800000 ; // Deep-sleep clock is osc /64
pub const  SYSCTL_DSLP_OSC_MAIN    :u32 = 0x00000000 ; // Osc source is main osc
pub const  SYSCTL_DSLP_OSC_INT     :u32 = 0x00000010 ; // Osc source is int. osc
pub const  SYSCTL_DSLP_OSC_INT30   :u32 = 0x00000030 ; // Osc source is int. 30 KHz
pub const  SYSCTL_DSLP_OSC_EXT32   :u32 = 0x00000070 ; // Osc source is ext. 32 KHz
pub const  SYSCTL_DSLP_PIOSC_PD    :u32 = 0x00000002 ; // Power down PIOSC in deep-sleep
pub const  SYSCTL_DSLP_MOSC_PD     :u32 = 0x40000000 ; // Power down MOSC in deep-sleep

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlPIOSCCalibrate()
// API as the ui32Type parameter.
//
//*****************************************************************************
pub const  SYSCTL_PIOSC_CAL_AUTO   :u32 = 0x00000200 ; // Automatic calibration
pub const  SYSCTL_PIOSC_CAL_FACT   :u32 = 0x00000100 ; // Factory calibration
pub const  SYSCTL_PIOSC_CAL_USER   :u32 = 0x80000100 ; // User-supplied calibration

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlMOSCConfigSet() API
// as the ui32Config parameter.
//
//*****************************************************************************
pub const  SYSCTL_MOSC_VALIDATE    :u32 = 0x00000001 ; // Enable MOSC validation
pub const  SYSCTL_MOSC_INTERRUPT   :u32 = 0x00000002 ; // Generate interrupt on MOSC fail
pub const  SYSCTL_MOSC_NO_XTAL     :u32 = 0x00000004 ; // No crystal is attached to MOSC
pub const  SYSCTL_MOSC_PWR_DIS     :u32 = 0x00000008 ; // Power down the MOSC.
pub const  SYSCTL_MOSC_LOWFREQ     :u32 = 0x00000000 ; // MOSC is less than 10MHz
pub const  SYSCTL_MOSC_HIGHFREQ    :u32 = 0x00000010 ; // MOSC is greater than 10MHz
pub const  SYSCTL_MOSC_SESRC       :u32 = 0x00000020 ; // Singled ended oscillator source.

//*****************************************************************************
//
// The following are values that can be passed to the SysCtlSleepPowerSet() and
// SysCtlDeepSleepPowerSet() APIs as the ui32Config parameter.
//
//*****************************************************************************
pub const  SYSCTL_LDO_SLEEP        :u32 = 0x00000200 ; // LDO in sleep mode
// (Deep Sleep Only)
pub const  SYSCTL_TEMP_LOW_POWER   :u32 = 0x00000100 ; // Temp sensor in low power mode
// (Deep Sleep Only)
pub const  SYSCTL_FLASH_NORMAL     :u32 = 0x00000000 ; // Flash in normal mode
pub const  SYSCTL_FLASH_LOW_POWER  :u32 = 0x00000020 ; // Flash in low power mode
pub const  SYSCTL_SRAM_NORMAL      :u32 = 0x00000000 ; // SRAM in normal mode
pub const  SYSCTL_SRAM_STANDBY     :u32 = 0x00000001 ; // SRAM in standby mode
pub const  SYSCTL_SRAM_LOW_POWER   :u32 = 0x00000003 ; // SRAM in low power mode

//*****************************************************************************
//
// Defines for the SysCtlResetBehaviorSet() and SysCtlResetBehaviorGet() APIs.
//
//*****************************************************************************
pub const  SYSCTL_ONRST_WDOG0_POR  :u32 = 0x00000030;
pub const  SYSCTL_ONRST_WDOG0_SYS  :u32 = 0x00000020;
pub const  SYSCTL_ONRST_WDOG1_POR  :u32 = 0x000000C0;
pub const  SYSCTL_ONRST_WDOG1_SYS  :u32 = 0x00000080;
pub const  SYSCTL_ONRST_BOR_POR    :u32 = 0x0000000C;
pub const  SYSCTL_ONRST_BOR_SYS    :u32 = 0x00000008;
pub const  SYSCTL_ONRST_EXT_POR    :u32 = 0x00000003;
pub const  SYSCTL_ONRST_EXT_SYS    :u32 = 0x00000002;

//*****************************************************************************
//
// Values used with the SysCtlVoltageEventConfig() API.
//
//*****************************************************************************
pub const  SYSCTL_VEVENT_VDDABO_NONE: u32 =                                    0x00000000;
pub const  SYSCTL_VEVENT_VDDABO_INT: u32 =    0x00000100;
pub const SYSCTL_VEVENT_VDDABO_NMI: u32 =    0x00000200;
pub const  SYSCTL_VEVENT_VDDABO_RST: u32 =    0x00000300;
pub const  SYSCTL_VEVENT_VDDBO_NONE: u32 =    0x00000000;
pub const SYSCTL_VEVENT_VDDBO_INT: u32 =     0x00000001;
pub const  SYSCTL_VEVENT_VDDBO_NMI : u32 =    0x00000002;
pub const  SYSCTL_VEVENT_VDDBO_RST : u32 =    0x00000003;

//*****************************************************************************
//
// Values used with the SysCtlVoltageEventStatus() and
// SysCtlVoltageEventClear() APIs.
//
//*****************************************************************************
pub const  SYSCTL_VESTAT_VDDBOR    : u32 =    0x00000040;
pub const  SYSCTL_VESTAT_VDDABOR   : u32 =    0x00000010;

//*****************************************************************************
//
// Values used with the SysCtlNMIStatus() API.
//
//*****************************************************************************
pub const  SYSCTL_NMI_MOSCFAIL     : u32 =    0x00010000;
pub const  SYSCTL_NMI_TAMPER       : u32 =    0x00000200;
pub const  SYSCTL_NMI_WDT1         : u32 =    0x00000020;
pub const  SYSCTL_NMI_WDT0         : u32 =    0x00000008;
pub const  SYSCTL_NMI_POWER        : u32 =    0x00000004;
pub const  SYSCTL_NMI_EXTERNAL     : u32 =    0x00000001;

//*****************************************************************************
//
// The defines for the SysCtlClockOutConfig() API.
//
//*****************************************************************************
pub const  SYSCTL_CLKOUT_EN        : u32 =    0x80000000;
pub const  SYSCTL_CLKOUT_DIS       : u32 =    0x00000000;
pub const  SYSCTL_CLKOUT_SYSCLK    : u32 =    0x00000000;
pub const  SYSCTL_CLKOUT_PIOSC     : u32 =    0x00010000;
pub const  SYSCTL_CLKOUT_MOSC      : u32 =    0x00020000;

//*****************************************************************************
//
// The following defines are used with the SysCtlAltClkConfig() function.
//
//*****************************************************************************
pub const  SYSCTL_ALTCLK_PIOSC     : u32 =    0x00000000;
pub const  SYSCTL_ALTCLK_RTCOSC    : u32 =    0x00000003;
pub const  SYSCTL_ALTCLK_LFIOSC    : u32 =    0x00000004;


//*****************************************************************************
//
// sysctl.c - Driver for the system controller.
//
// Copyright (c) 2005-2014 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
// 
//   Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the  
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// 
// This is part of revision 2.1.0.12573 of the Tiva Peripheral Driver Library.
//
//*****************************************************************************



//*****************************************************************************
//
// The base addresses of the various peripheral control registers.
//
//*****************************************************************************
pub const SYSCTL_PPBASE :u32 = 0x400fe300;
pub const SYSCTL_SRBASE :u32 = 0x400fe500;
pub const SYSCTL_RCGCBASE :u32 = 0x400fe600;
pub const SYSCTL_SCGCBASE :u32 = 0x400fe700;
pub const SYSCTL_DCGCBASE :u32 = 0x400fe800;
pub const SYSCTL_PCBASE :u32 = 0x400fe900;
pub const SYSCTL_PRBASE :u32 = 0x400fea00;


pub unsafe fn cpu_clock_init(clock: u32) {
    let mut ui32Delay;
    let ui32Config : u32 = match(clock) {
        80 => {SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN},
        50 => {SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN},
        40 => {SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN},
        16 => {SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN},
        1  => {SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_XTAL_1MHZ | SYSCTL_OSC_MAIN},
        _  => {SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN},
    };

    //
    // Get the current value of the RCC and RCC2 registers.
    //

    let mut ui32RCC = hwreg!(SYSCTL_RCC, u32);
    let mut ui32RCC2 = hwreg!(SYSCTL_RCC2, u32);

    //
    // Bypass the PLL and system clock dividers for now.
    //
    ui32RCC |= SYSCTL_RCC_BYPASS;
    ui32RCC &= !(SYSCTL_RCC_USESYSDIV);
    ui32RCC2 |= SYSCTL_RCC2_BYPASS2;

    //
    // Write the new RCC value.
    //
    hwreg!(SYSCTL_RCC, u32, ui32RCC);
    hwreg!(SYSCTL_RCC2, u32, ui32RCC2);

    //
    // See if the oscillator needs to be enabled.
    //
    if(((ui32RCC & SYSCTL_RCC_MOSCDIS) != 0) && (!(ui32Config & SYSCTL_MAIN_OSC_DIS) != 0))
    {
        //
        // Make sure that the required oscillators are enabled.  For now, the
        // previously enabled oscillators must be enabled along with the newly
        // requested oscillators.
        //
        ui32RCC &= (!SYSCTL_RCC_MOSCDIS | (ui32Config & SYSCTL_MAIN_OSC_DIS));

        //
        // Clear the MOSC power up raw interrupt status to be sure it is not
        // set when waiting below.
        //
        hwreg!(SYSCTL_MISC, u32, SYSCTL_MISC_MOSCPUPMIS);

        //
        // Write the new RCC value.
        //
        hwreg!(SYSCTL_RCC, u32, ui32RCC);

        //
        // Timeout using the legacy delay value.
        //
        ui32Delay = 524288;

        while((hwreg!(SYSCTL_RIS, u32) & SYSCTL_RIS_MOSCPUPRIS) == 0)
        {
            ui32Delay = ui32Delay - 1;

            if(ui32Delay == 0)
            {
                break;
            }
        }

        //
        // If the main oscillator failed to start up then do not switch to
        // it and return.
        //
        if(ui32Delay == 0)
        {
            return;
        }
    }
    
    //
    // Set the new crystal value and oscillator source.  Because the OSCSRC2
    // field in RCC2 overlaps the XTAL field in RCC, the OSCSRC field has a
    // special encoding within ui32Config to avoid the overlap.
    //
    ui32RCC &= !(SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);
    ui32RCC |= ui32Config & (SYSCTL_RCC_XTAL_M | SYSCTL_RCC_OSCSRC_M);
    ui32RCC2 &= !(SYSCTL_RCC2_USERCC2 | SYSCTL_RCC2_OSCSRC2_M);
    ui32RCC2 |= ui32Config & (SYSCTL_RCC2_USERCC2 | SYSCTL_RCC_OSCSRC_M);
    ui32RCC2 |= (ui32Config & 0x00000008) << 3;

    
    //
    // Write the new RCC value.
    //
    hwreg!(SYSCTL_RCC, u32, ui32RCC);
    hwreg!(SYSCTL_RCC2, u32, ui32RCC2);

    //
    // Set the PLL configuration.
    //
    ui32RCC &= !SYSCTL_RCC_PWRDN;
    ui32RCC |= ui32Config & SYSCTL_RCC_PWRDN;
    ui32RCC2 &= !SYSCTL_RCC2_PWRDN2;
    ui32RCC2 |= ui32Config & SYSCTL_RCC2_PWRDN2;

    //
    // Clear the PLL lock interrupt.
    //
    hwreg!(SYSCTL_MISC, u32, SYSCTL_MISC_PLLLMIS);

    //
    // Write the new RCC value.
    //
    if((ui32RCC2 & SYSCTL_RCC2_USERCC2) != 0)
    {
        hwreg!(SYSCTL_RCC2, u32, ui32RCC2);
        hwreg!(SYSCTL_RCC, u32, ui32RCC);
    }
    else
    {
        hwreg!(SYSCTL_RCC, u32, ui32RCC);
        hwreg!(SYSCTL_RCC2, u32, ui32RCC2);
    }

    //
    // Set the requested system divider and disable the appropriate
    // oscillators.  This value is not written immediately.
    //
    ui32RCC &= !(SYSCTL_RCC_SYSDIV_M | SYSCTL_RCC_USESYSDIV |
                 SYSCTL_RCC_MOSCDIS);
    ui32RCC |= ui32Config & (SYSCTL_RCC_SYSDIV_M | SYSCTL_RCC_USESYSDIV |
                             SYSCTL_RCC_MOSCDIS);
    ui32RCC2 &= !(SYSCTL_RCC2_SYSDIV2_M);
    ui32RCC2 |= ui32Config & SYSCTL_RCC2_SYSDIV2_M;
    if(ui32Config & SYSCTL_RCC2_DIV400) != 0
    {
        ui32RCC |= SYSCTL_RCC_USESYSDIV;
        ui32RCC2 &= !(SYSCTL_RCC_USESYSDIV);
        ui32RCC2 |= ui32Config & (SYSCTL_RCC2_DIV400 | SYSCTL_RCC2_SYSDIV2LSB);
    }
    else
    {
        ui32RCC2 &= !(SYSCTL_RCC2_DIV400);
    }

    //
    // See if the PLL output is being used to clock the system.
    //
    if((ui32Config & SYSCTL_RCC_BYPASS) == 0)
    {
        //
        // Wait until the PLL has locked.
        //
        for ui32Delay in 0..32768
        {
            if((hwreg!(SYSCTL_PLLSTAT, u32) & SYSCTL_PLLSTAT_LOCK) != 0)
            {
                break;
            }
        }

        //
        // Enable use of the PLL.
        //
        ui32RCC &= !(SYSCTL_RCC_BYPASS);
        ui32RCC2 &= !(SYSCTL_RCC2_BYPASS2);
    }

    //
    // Write the final RCC value.
    //
    hwreg!(SYSCTL_RCC, u32, ui32RCC);
    hwreg!(SYSCTL_RCC2, u32, ui32RCC2);

    //
    // Delay for a little bit so that the system divider takes effect.
    //
    // SysCtlDelay(16);
}

pub unsafe fn sys_ctl_peripheral_enable(periph_id : u32) {
        write_bitband!((SYSCTL_RCGCBASE + ((periph_id & 0xff00) >> 8)),(periph_id & 0xff),1);
}



static g_pui32Xtals : [u32; 27] = [
    1000000,
    1843200,
    2000000,
    2457600,
    3579545,
    3686400,
    4000000,
    4096000,
    4915200,
    5000000,
    5120000,
    6000000,
    6144000,
    7372800,
    8000000,
    8192000,
    10000000,
    12000000,
    12288000,
    13560000,
    14318180,
    16000000,
    16384000,
    18000000,
    20000000,
    24000000,
    25000000
];

pub unsafe fn sys_ctl_clock_get() -> u32 {

    //
    // Read RCC and RCC2.
    //
    let mut ui32RCC = hwreg!(SYSCTL_RCC, u32);
    let mut ui32RCC2 = hwreg!(SYSCTL_RCC2, u32);

    //
    // Get the base clock rate.
    //
    let mut ui32Clk : u32;
    let mut ui32Max : u32;

    let match_val : u32 = match (ui32RCC2 & SYSCTL_RCC2_USERCC2) != 0 {
        true => {ui32RCC2 & SYSCTL_RCC2_OSCSRC2_M},
        false => {ui32RCC & SYSCTL_RCC_OSCSRC_M}};

    match match_val {
        //
        // The main oscillator is the clock source.  Determine its rate from
        // the crystal setting field.
        //
        SYSCTL_RCC_OSCSRC_MAIN => {
            ui32Clk = g_pui32Xtals[((ui32RCC & SYSCTL_RCC_XTAL_M) >>
                                   SYSCTL_RCC_XTAL_S) as usize];
        },

        //
        // The internal oscillator is the source clock.
        //
        SYSCTL_RCC_OSCSRC_INT => {
            //
            // The internal oscillator on all devices is 16 MHz.
            //
            ui32Clk = 16000000;
        },

        //
        // The internal oscillator divided by four is the source clock.
        //
        SYSCTL_RCC_OSCSRC_INT4 =>{
            //
            // The internal oscillator on all devices is 16 MHz.
            //
            ui32Clk = 16000000 / 4;
        }

        //
        // The internal 30-KHz oscillator is the source clock.
        //
        SYSCTL_RCC_OSCSRC_30 => {
            //
            // The internal 30-KHz oscillator has an accuracy of +/- 30%.
            //
            ui32Clk = 30000;
        },

        //
        // The 32.768-KHz clock from the hibernate module is the source clock.
        //
        SYSCTL_RCC2_OSCSRC2_32 => {
            ui32Clk = 32768;
        },

        //
        // An unknown setting, so return a zero clock (that is, an unknown
        // clock rate).
        //
        _ => {
            return(0);
        }
    }

    //
    // Default the maximum frequency to the maximum 32-bit unsigned value.
    //
    let mut ui32Max :u32 = 0xffffffff;

    //
    // See if the PLL is being used.
    //
    if(((ui32RCC2 & SYSCTL_RCC2_USERCC2) != 0 &&
        (ui32RCC2 & SYSCTL_RCC2_BYPASS2) == 0) ||
       ((ui32RCC2 & SYSCTL_RCC2_USERCC2) == 0 && (ui32RCC & SYSCTL_RCC_BYPASS) == 0))
    {
        //
        // Read the two PLL frequency registers.  The formula for a
        // TM4C123 device is "(xtal * m) / ((q + 1) * (n + 1))".
        //
        let ui32PLL = hwreg!(SYSCTL_PLLFREQ0, u32);
        let ui32PLL1 = hwreg!(SYSCTL_PLLFREQ1, u32);

        //
        // Divide the input clock by the dividers.
        //
        ui32Clk /= ((((ui32PLL1 & SYSCTL_PLLFREQ1_Q_M) >>
                      SYSCTL_PLLFREQ1_Q_S) + 1) *
                    (((ui32PLL1 & SYSCTL_PLLFREQ1_N_M) >>
                      SYSCTL_PLLFREQ1_N_S) + 1) * 2);

        //
        // Multiply the clock by the multiplier, which is split into an
        // integer part and a fractional part.
        //
        ui32Clk = ((ui32Clk * ((ui32PLL & SYSCTL_PLLFREQ0_MINT_M) >>
                               SYSCTL_PLLFREQ0_MINT_S)) +
                   ((ui32Clk * ((ui32PLL & SYSCTL_PLLFREQ0_MFRAC_M) >>
                                SYSCTL_PLLFREQ0_MFRAC_S)) >> 10));

        //
        // Force the system divider to be enabled.  It is always used when
        // using the PLL, but in some cases it does not read as being enabled.
        //
        ui32RCC |= SYSCTL_RCC_USESYSDIV;

        //
        // Calculate the maximum system frequency.
        //
        match (hwreg!(SYSCTL_DC1, u32) & SYSCTL_DC1_MINSYSDIV_M){
            SYSCTL_DC1_MINSYSDIV_80 => {
                ui32Max = 80000000;
            },
            SYSCTL_DC1_MINSYSDIV_66 => {
                ui32Max = 66666666;
            },
            SYSCTL_DC1_MINSYSDIV_50 => {
                ui32Max = 50000000;
            },
            SYSCTL_DC1_MINSYSDIV_40 => {
                ui32Max = 40000000;
            },
            SYSCTL_DC1_MINSYSDIV_25 => {
                ui32Max = 25000000;
            },
            SYSCTL_DC1_MINSYSDIV_20 => {
                ui32Max = 20000000;
            },
            _ => {
            }
        }
    }

    //
    // See if the system divider is being used.
    //
    if(ui32RCC & SYSCTL_RCC_USESYSDIV) != 0
    {
        //
        // Adjust the clock rate by the system clock divider.
        //
        if(ui32RCC2 & SYSCTL_RCC2_USERCC2) != 0
        {
            if((ui32RCC2 & SYSCTL_RCC2_DIV400)!= 0 &&
               (((ui32RCC2 & SYSCTL_RCC2_USERCC2) != 0 &&
                 (ui32RCC2 & SYSCTL_RCC2_BYPASS2) == 0) ||
                ((ui32RCC2 & SYSCTL_RCC2_USERCC2) == 0 &&
                 (ui32RCC & SYSCTL_RCC_BYPASS) == 0)))

            {
                ui32Clk = ((ui32Clk * 2) / (((ui32RCC2 &
                                              (SYSCTL_RCC2_SYSDIV2_M |
                                               SYSCTL_RCC2_SYSDIV2LSB)) >>
                                             (SYSCTL_RCC2_SYSDIV2_S - 1)) +
                                            1));
            }
            else
            {
                ui32Clk /= (((ui32RCC2 & SYSCTL_RCC2_SYSDIV2_M) >>
                             SYSCTL_RCC2_SYSDIV2_S) + 1);
            }
        }
        else
        {
            ui32Clk /= (((ui32RCC & SYSCTL_RCC_SYSDIV_M) >>
                         SYSCTL_RCC_SYSDIV_S) + 1);
        }
    }

    //
    // Limit the maximum clock to the maximum clock frequency.
    //
    if(ui32Max < ui32Clk)
    {
        ui32Clk = ui32Max;
    }

    //
    // Return the computed clock rate.
    //
    return(ui32Clk);
}
