//*****************************************************************************
//
// hw_uart.h - Macros and defines used when accessing the UART hardware.
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
// The following are defines for the UART register offsets.
//
//*****************************************************************************
pub const UART_O_DR :u32 = 0x00000000;  // UART Data
pub const UART_O_RSR :u32 = 0x00000004;  // UART Receive Status/Error Clear
pub const UART_O_ECR :u32 = 0x00000004;  // UART Receive Status/Error Clear
pub const UART_O_FR :u32 = 0x00000018;  // UART Flag
pub const UART_O_ILPR :u32 = 0x00000020;  // UART IrDA Low-Power Register
pub const UART_O_IBRD :u32 = 0x00000024;  // UART Integer Baud-Rate Divisor
pub const UART_O_FBRD :u32 = 0x00000028;  // UART Fractional Baud-Rate
                                            // Divisor
pub const UART_O_LCRH :u32 = 0x0000002C;  // UART Line Control
pub const UART_O_CTL :u32 = 0x00000030;  // UART Control
pub const UART_O_IFLS :u32 = 0x00000034;  // UART Interrupt FIFO Level Select
pub const UART_O_IM :u32 = 0x00000038;  // UART Interrupt Mask
pub const UART_O_RIS :u32 = 0x0000003C;  // UART Raw Interrupt Status
pub const UART_O_MIS :u32 = 0x00000040;  // UART Masked Interrupt Status
pub const UART_O_ICR :u32 = 0x00000044;  // UART Interrupt Clear
pub const UART_O_DMACTL :u32 = 0x00000048;  // UART DMA Control
pub const UART_O_9BITADDR :u32 = 0x000000A4;  // UART 9-Bit Self Address
pub const UART_O_9BITAMASK :u32 = 0x000000A8;  // UART 9-Bit Self Address Mask
pub const UART_O_PP :u32 = 0x00000FC0;  // UART Peripheral Properties
pub const UART_O_CC :u32 = 0x00000FC8;  // UART Clock Configuration

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DR register.
//
//*****************************************************************************
pub const UART_DR_OE :u32 = 0x00000800;  // UART Overrun Error
pub const UART_DR_BE :u32 = 0x00000400;  // UART Break Error
pub const UART_DR_PE :u32 = 0x00000200;  // UART Parity Error
pub const UART_DR_FE :u32 = 0x00000100;  // UART Framing Error
pub const UART_DR_DATA_M :u32 = 0x000000FF;  // Data Transmitted or Received
pub const UART_DR_DATA_S :u32 =         0;

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RSR register.
//
//*****************************************************************************
pub const UART_RSR_OE :u32 = 0x00000008;  // UART Overrun Error
pub const UART_RSR_BE :u32 = 0x00000004;  // UART Break Error
pub const UART_RSR_PE :u32 = 0x00000002;  // UART Parity Error
pub const UART_RSR_FE :u32 = 0x00000001;  // UART Framing Error

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ECR register.
//
//*****************************************************************************
pub const UART_ECR_DATA_M :u32 = 0x000000FF;  // Error Clear
pub const UART_ECR_DATA_S : u32 =         0;

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FR register.
//
//*****************************************************************************
pub const UART_FR_RI :u32 = 0x00000100;  // Ring Indicator
pub const UART_FR_TXFE :u32 = 0x00000080;  // UART Transmit FIFO Empty
pub const UART_FR_RXFF :u32 = 0x00000040;  // UART Receive FIFO Full
pub const UART_FR_TXFF :u32 = 0x00000020;  // UART Transmit FIFO Full
pub const UART_FR_RXFE :u32 = 0x00000010;  // UART Receive FIFO Empty
pub const UART_FR_BUSY :u32 = 0x00000008;  // UART Busy
pub const UART_FR_DCD :u32 = 0x00000004;  // Data Carrier Detect
pub const UART_FR_DSR :u32 = 0x00000002;  // Data Set Ready
pub const UART_FR_CTS :u32 = 0x00000001;  // Clear To Send

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ILPR register.
//
//*****************************************************************************
pub const UART_ILPR_ILPDVSR_M :u32 = 0x000000FF;  // IrDA Low-Power Divisor
pub const UART_ILPR_ILPDVSR_S : u32 =     0;

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IBRD register.
//
//*****************************************************************************
pub const UART_IBRD_DIVINT_M :u32 = 0x0000FFFF;  // Integer Baud-Rate Divisor
pub const UART_IBRD_DIVINT_S : u32 =      0;

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_FBRD register.
//
//*****************************************************************************
pub const UART_FBRD_DIVFRAC_M :u32 = 0x0000003F;  // Fractional Baud-Rate Divisor
pub const UART_FBRD_DIVFRAC_S : u32 =     0;

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_LCRH register.
//
//*****************************************************************************
pub const UART_LCRH_SPS :u32 = 0x00000080;  // UART Stick Parity Select
pub const UART_LCRH_WLEN_M :u32 = 0x00000060;  // UART Word Length
pub const UART_LCRH_WLEN_5 :u32 = 0x00000000;  // 5 bits (default)
pub const UART_LCRH_WLEN_6 :u32 = 0x00000020;  // 6 bits
pub const UART_LCRH_WLEN_7 :u32 = 0x00000040;  // 7 bits
pub const UART_LCRH_WLEN_8 :u32 = 0x00000060;  // 8 bits
pub const UART_LCRH_FEN :u32 = 0x00000010;  // UART Enable FIFOs
pub const UART_LCRH_STP2 :u32 = 0x00000008;  // UART Two Stop Bits Select
pub const UART_LCRH_EPS :u32 = 0x00000004;  // UART Even Parity Select
pub const UART_LCRH_PEN :u32 = 0x00000002;  // UART Parity Enable
pub const UART_LCRH_BRK :u32 = 0x00000001;  // UART Send Break

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CTL register.
//
//*****************************************************************************
pub const UART_CTL_CTSEN :u32 = 0x00008000;  // Enable Clear To Send
pub const UART_CTL_RTSEN :u32 = 0x00004000;  // Enable Request to Send
pub const UART_CTL_RTS :u32 = 0x00000800;  // Request to Send
pub const UART_CTL_DTR :u32 = 0x00000400;  // Data Terminal Ready
pub const UART_CTL_RXE :u32 = 0x00000200;  // UART Receive Enable
pub const UART_CTL_TXE :u32 = 0x00000100;  // UART Transmit Enable
pub const UART_CTL_LBE :u32 = 0x00000080;  // UART Loop Back Enable
pub const UART_CTL_HSE :u32 = 0x00000020;  // High-Speed Enable
pub const UART_CTL_EOT :u32 = 0x00000010;  // End of Transmission
pub const UART_CTL_SMART :u32 = 0x00000008;  // ISO 7816 Smart Card Support
pub const UART_CTL_SIRLP :u32 = 0x00000004;  // UART SIR Low-Power Mode
pub const UART_CTL_SIREN :u32 = 0x00000002;  // UART SIR Enable
pub const UART_CTL_UARTEN :u32 = 0x00000001;  // UART Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IFLS register.
//
//*****************************************************************************
pub const UART_IFLS_RX_M :u32 = 0x00000038;  // UART Receive Interrupt FIFO
                                            // Level Select
pub const UART_IFLS_RX1_8 :u32 = 0x00000000;  // RX FIFO >= 1/8 full
pub const UART_IFLS_RX2_8 :u32 = 0x00000008;  // RX FIFO >= 1/4 full
pub const UART_IFLS_RX4_8 :u32 = 0x00000010;  // RX FIFO >= 1/2 full (default)
pub const UART_IFLS_RX6_8 :u32 = 0x00000018;  // RX FIFO >= 3/4 full
pub const UART_IFLS_RX7_8 :u32 = 0x00000020;  // RX FIFO >= 7/8 full
pub const UART_IFLS_TX_M :u32 = 0x00000007;  // UART Transmit Interrupt FIFO
                                            // Level Select
pub const UART_IFLS_TX1_8 :u32 = 0x00000000;  // TX FIFO <= 1/8 full
pub const UART_IFLS_TX2_8 :u32 = 0x00000001;  // TX FIFO <= 1/4 full
pub const UART_IFLS_TX4_8 :u32 = 0x00000002;  // TX FIFO <= 1/2 full (default)
pub const UART_IFLS_TX6_8 :u32 = 0x00000003;  // TX FIFO <= 3/4 full
pub const UART_IFLS_TX7_8 :u32 = 0x00000004;  // TX FIFO <= 7/8 full

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_IM register.
//
//*****************************************************************************
pub const UART_IM_DMATXIM :u32 = 0x00020000;  // Transmit DMA Interrupt Mask
pub const UART_IM_DMARXIM :u32 = 0x00010000;  // Receive DMA Interrupt Mask
pub const UART_IM_9BITIM :u32 = 0x00001000;  // 9-Bit Mode Interrupt Mask
pub const UART_IM_EOTIM :u32 = 0x00000800;  // End of Transmission Interrupt
                                            // Mask
pub const UART_IM_OEIM :u32 = 0x00000400;  // UART Overrun Error Interrupt
                                            // Mask
pub const UART_IM_BEIM :u32 = 0x00000200;  // UART Break Error Interrupt Mask
pub const UART_IM_PEIM :u32 = 0x00000100;  // UART Parity Error Interrupt Mask
pub const UART_IM_FEIM :u32 = 0x00000080;  // UART Framing Error Interrupt
                                            // Mask
pub const UART_IM_RTIM :u32 = 0x00000040;  // UART Receive Time-Out Interrupt
                                            // Mask
pub const UART_IM_TXIM :u32 = 0x00000020;  // UART Transmit Interrupt Mask
pub const UART_IM_RXIM :u32 = 0x00000010;  // UART Receive Interrupt Mask
pub const UART_IM_DSRMIM :u32 = 0x00000008;  // UART Data Set Ready Modem
                                            // Interrupt Mask
pub const UART_IM_DCDMIM :u32 = 0x00000004;  // UART Data Carrier Detect Modem
                                            // Interrupt Mask
pub const UART_IM_CTSMIM :u32 = 0x00000002;  // UART Clear to Send Modem
                                            // Interrupt Mask
pub const UART_IM_RIMIM :u32 = 0x00000001;  // UART Ring Indicator Modem
                                            // Interrupt Mask

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_RIS register.
//
//*****************************************************************************
pub const UART_RIS_DMATXRIS :u32 = 0x00020000;  // Transmit DMA Raw Interrupt
                                            // Status
pub const UART_RIS_DMARXRIS :u32 = 0x00010000;  // Receive DMA Raw Interrupt Status
pub const UART_RIS_9BITRIS :u32 = 0x00001000;  // 9-Bit Mode Raw Interrupt Status
pub const UART_RIS_EOTRIS :u32 = 0x00000800;  // End of Transmission Raw
                                            // Interrupt Status
pub const UART_RIS_OERIS :u32 = 0x00000400;  // UART Overrun Error Raw Interrupt
                                            // Status
pub const UART_RIS_BERIS :u32 = 0x00000200;  // UART Break Error Raw Interrupt
                                            // Status
pub const UART_RIS_PERIS :u32 = 0x00000100;  // UART Parity Error Raw Interrupt
                                            // Status
pub const UART_RIS_FERIS :u32 = 0x00000080;  // UART Framing Error Raw Interrupt
                                            // Status
pub const UART_RIS_RTRIS :u32 = 0x00000040;  // UART Receive Time-Out Raw
                                            // Interrupt Status
pub const UART_RIS_TXRIS :u32 = 0x00000020;  // UART Transmit Raw Interrupt
                                            // Status
pub const UART_RIS_RXRIS :u32 = 0x00000010;  // UART Receive Raw Interrupt
                                            // Status
pub const UART_RIS_DSRRIS :u32 = 0x00000008;  // UART Data Set Ready Modem Raw
                                            // Interrupt Status
pub const UART_RIS_DCDRIS :u32 = 0x00000004;  // UART Data Carrier Detect Modem
                                            // Raw Interrupt Status
pub const UART_RIS_CTSRIS :u32 = 0x00000002;  // UART Clear to Send Modem Raw
                                            // Interrupt Status
pub const UART_RIS_RIRIS :u32 = 0x00000001;  // UART Ring Indicator Modem Raw
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_MIS register.
//
//*****************************************************************************
pub const UART_MIS_DMATXMIS :u32 = 0x00020000;  // Transmit DMA Masked Interrupt
                                            // Status
pub const UART_MIS_DMARXMIS :u32 = 0x00010000;  // Receive DMA Masked Interrupt
                                            // Status
pub const UART_MIS_9BITMIS :u32 = 0x00001000;  // 9-Bit Mode Masked Interrupt
                                            // Status
pub const UART_MIS_EOTMIS :u32 = 0x00000800;  // End of Transmission Masked
                                            // Interrupt Status
pub const UART_MIS_OEMIS :u32 = 0x00000400;  // UART Overrun Error Masked
                                            // Interrupt Status
pub const UART_MIS_BEMIS :u32 = 0x00000200;  // UART Break Error Masked
                                            // Interrupt Status
pub const UART_MIS_PEMIS :u32 = 0x00000100;  // UART Parity Error Masked
                                            // Interrupt Status
pub const UART_MIS_FEMIS :u32 = 0x00000080;  // UART Framing Error Masked
                                            // Interrupt Status
pub const UART_MIS_RTMIS :u32 = 0x00000040;  // UART Receive Time-Out Masked
                                            // Interrupt Status
pub const UART_MIS_TXMIS :u32 = 0x00000020;  // UART Transmit Masked Interrupt
                                            // Status
pub const UART_MIS_RXMIS :u32 = 0x00000010;  // UART Receive Masked Interrupt
                                            // Status
pub const UART_MIS_DSRMIS :u32 = 0x00000008;  // UART Data Set Ready Modem Masked
                                            // Interrupt Status
pub const UART_MIS_DCDMIS :u32 = 0x00000004;  // UART Data Carrier Detect Modem
                                            // Masked Interrupt Status
pub const UART_MIS_CTSMIS :u32 = 0x00000002;  // UART Clear to Send Modem Masked
                                            // Interrupt Status
pub const UART_MIS_RIMIS :u32 = 0x00000001;  // UART Ring Indicator Modem Masked
                                            // Interrupt Status

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_ICR register.
//
//*****************************************************************************
pub const UART_ICR_DMATXIC :u32 = 0x00020000;  // Transmit DMA Interrupt Clear
pub const UART_ICR_DMARXIC :u32 = 0x00010000;  // Receive DMA Interrupt Clear
pub const UART_ICR_9BITIC :u32 = 0x00001000;  // 9-Bit Mode Interrupt Clear
pub const UART_ICR_EOTIC :u32 = 0x00000800;  // End of Transmission Interrupt
                                            // Clear
pub const UART_ICR_OEIC :u32 = 0x00000400;  // Overrun Error Interrupt Clear
pub const UART_ICR_BEIC :u32 = 0x00000200;  // Break Error Interrupt Clear
pub const UART_ICR_PEIC :u32 = 0x00000100;  // Parity Error Interrupt Clear
pub const UART_ICR_FEIC :u32 = 0x00000080;  // Framing Error Interrupt Clear
pub const UART_ICR_RTIC :u32 = 0x00000040;  // Receive Time-Out Interrupt Clear
pub const UART_ICR_TXIC :u32 = 0x00000020;  // Transmit Interrupt Clear
pub const UART_ICR_RXIC :u32 = 0x00000010;  // Receive Interrupt Clear
pub const UART_ICR_DSRMIC :u32 = 0x00000008;  // UART Data Set Ready Modem
                                            // Interrupt Clear
pub const UART_ICR_DCDMIC :u32 = 0x00000004;  // UART Data Carrier Detect Modem
                                            // Interrupt Clear
pub const UART_ICR_CTSMIC :u32 = 0x00000002;  // UART Clear to Send Modem
                                            // Interrupt Clear
pub const UART_ICR_RIMIC :u32 = 0x00000001;  // UART Ring Indicator Modem
                                            // Interrupt Clear

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_DMACTL register.
//
//*****************************************************************************
pub const UART_DMACTL_DMAERR :u32 = 0x00000004;  // DMA on Error
pub const UART_DMACTL_TXDMAE :u32 = 0x00000002;  // Transmit DMA Enable
pub const UART_DMACTL_RXDMAE :u32 = 0x00000001;  // Receive DMA Enable

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITADDR
// register.
//
//*****************************************************************************
pub const UART_9BITADDR_9BITEN :u32 = 0x00008000;  // Enable 9-Bit Mode
pub const UART_9BITADDR_ADDR_M :u32 = 0x000000FF;  // Self Address for 9-Bit Mode
pub const UART_9BITADDR_ADDR_S : u32 =    0;

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_9BITAMASK
// register.
//
//*****************************************************************************
pub const UART_9BITAMASK_MASK_M :u32 = 0x000000FF;  // Self Address Mask for 9-Bit Mode
pub const UART_9BITAMASK_MASK_S : u32 =   0;

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_PP register.
//
//*****************************************************************************
pub const UART_PP_MSE :u32 = 0x00000008;  // Modem Support Extended
pub const UART_PP_MS :u32 = 0x00000004;  // Modem Support
pub const UART_PP_NB :u32 = 0x00000002;  // 9-Bit Support
pub const UART_PP_SC :u32 = 0x00000001;  // Smart Card Support

//*****************************************************************************
//
// The following are defines for the bit fields in the UART_O_CC register.
//
//*****************************************************************************
pub const UART_CC_CS_M :u32 = 0x0000000F;  // UART Baud Clock Source
pub const UART_CC_CS_SYSCLK :u32 = 0x00000000;  // System clock (based on clock
                                            // source and divisor factor)
pub const UART_CC_CS_PIOSC :u32 = 0x00000005;  // PIOSC


//*****************************************************************************
//
// uart.h - Defines and Macros for the UART.
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
// Values that can be passed to UARTIntEnable, UARTIntDisable, and UARTIntClear
// as the ui32IntFlags parameter, and returned from UARTIntStatus.
//
//*****************************************************************************
pub const UART_INT_DMATX :u32 = 0x20000;     // DMA TX interrupt
pub const UART_INT_DMARX :u32 = 0x10000;     // DMA RX interrupt
pub const UART_INT_9BIT :u32 = 0x1000;      // 9-bit address match interrupt
pub const UART_INT_OE :u32 = 0x400;       // Overrun Error Interrupt Mask
pub const UART_INT_BE :u32 = 0x200;       // Break Error Interrupt Mask
pub const UART_INT_PE :u32 = 0x100;       // Parity Error Interrupt Mask
pub const UART_INT_FE :u32 = 0x080;       // Framing Error Interrupt Mask
pub const UART_INT_RT :u32 = 0x040;       // Receive Timeout Interrupt Mask
pub const UART_INT_TX :u32 = 0x020;       // Transmit Interrupt Mask
pub const UART_INT_RX :u32 = 0x010;       // Receive Interrupt Mask
pub const UART_INT_DSR :u32 = 0x008;       // DSR Modem Interrupt Mask
pub const UART_INT_DCD :u32 = 0x004;       // DCD Modem Interrupt Mask
pub const UART_INT_CTS :u32 = 0x002;       // CTS Modem Interrupt Mask
pub const UART_INT_RI :u32 = 0x001;       // RI Modem Interrupt Mask

//*****************************************************************************
//
// Values that can be passed to UARTConfigSetExpClk as the ui32Config parameter
// and returned by UARTConfigGetExpClk in the pui32Config parameter.
// Additionally, the UART_CONFIG_PAR_* subset can be passed to
// UARTParityModeSet as the ui32Parity parameter, and are returned by
// UARTParityModeGet.
//
//*****************************************************************************
pub const UART_CONFIG_WLEN_MASK :u32 = 0x00000060;  // Mask for extracting word length
pub const UART_CONFIG_WLEN_8 :u32 = 0x00000060;  // 8 bit data
pub const UART_CONFIG_WLEN_7 :u32 = 0x00000040;  // 7 bit data
pub const UART_CONFIG_WLEN_6 :u32 = 0x00000020;  // 6 bit data
pub const UART_CONFIG_WLEN_5 :u32 = 0x00000000;  // 5 bit data
pub const UART_CONFIG_STOP_MASK :u32 = 0x00000008;  // Mask for extracting stop bits
pub const UART_CONFIG_STOP_ONE :u32 = 0x00000000;  // One stop bit
pub const UART_CONFIG_STOP_TWO :u32 = 0x00000008;  // Two stop bits
pub const UART_CONFIG_PAR_MASK :u32 = 0x00000086;  // Mask for extracting parity
pub const UART_CONFIG_PAR_NONE :u32 = 0x00000000;  // No parity
pub const UART_CONFIG_PAR_EVEN :u32 = 0x00000006;  // Even parity
pub const UART_CONFIG_PAR_ODD :u32 = 0x00000002;  // Odd parity
pub const UART_CONFIG_PAR_ONE :u32 = 0x00000082;  // Parity bit is one
pub const UART_CONFIG_PAR_ZERO :u32 = 0x00000086;  // Parity bit is zero

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ui32TxLevel parameter
// and returned by UARTFIFOLevelGet in the pui32TxLevel.
//
//*****************************************************************************
pub const UART_FIFO_TX1_8 :u32 = 0x00000000;  // Transmit interrupt at 1/8 Full
pub const UART_FIFO_TX2_8 :u32 = 0x00000001;  // Transmit interrupt at 1/4 Full
pub const UART_FIFO_TX4_8 :u32 = 0x00000002;  // Transmit interrupt at 1/2 Full
pub const UART_FIFO_TX6_8 :u32 = 0x00000003;  // Transmit interrupt at 3/4 Full
pub const UART_FIFO_TX7_8 :u32 = 0x00000004;  // Transmit interrupt at 7/8 Full

//*****************************************************************************
//
// Values that can be passed to UARTFIFOLevelSet as the ui32RxLevel parameter
// and returned by UARTFIFOLevelGet in the pui32RxLevel.
//
//*****************************************************************************
pub const UART_FIFO_RX1_8 :u32 = 0x00000000;  // Receive interrupt at 1/8 Full
pub const UART_FIFO_RX2_8 :u32 = 0x00000008;  // Receive interrupt at 1/4 Full
pub const UART_FIFO_RX4_8 :u32 = 0x00000010;  // Receive interrupt at 1/2 Full
pub const UART_FIFO_RX6_8 :u32 = 0x00000018;  // Receive interrupt at 3/4 Full
pub const UART_FIFO_RX7_8 :u32 = 0x00000020;  // Receive interrupt at 7/8 Full

//*****************************************************************************
//
// Values that can be passed to UARTDMAEnable() and UARTDMADisable().
//
//*****************************************************************************
pub const UART_DMA_ERR_RXSTOP :u32 = 0x00000004;  // Stop DMA receive if UART error
pub const UART_DMA_TX :u32 = 0x00000002;  // Enable DMA for transmit
pub const UART_DMA_RX :u32 = 0x00000001;  // Enable DMA for receive

//*****************************************************************************
//
// Values returned from UARTRxErrorGet().
//
//*****************************************************************************
pub const UART_RXERROR_OVERRUN :u32 = 0x00000008;
pub const UART_RXERROR_BREAK :u32 = 0x00000004;
pub const UART_RXERROR_PARITY :u32 = 0x00000002;
pub const UART_RXERROR_FRAMING :u32 = 0x00000001;

//*****************************************************************************
//
// Values that can be passed to UARTHandshakeOutputsSet() or returned from
// UARTHandshakeOutputGet().
//
//*****************************************************************************
pub const UART_OUTPUT_RTS :u32 = 0x00000800;
pub const UART_OUTPUT_DTR :u32 = 0x00000400;

//*****************************************************************************
//
// Values that can be returned from UARTHandshakeInputsGet().
//
//*****************************************************************************
pub const UART_INPUT_RI :u32 = 0x00000100;
pub const UART_INPUT_DCD :u32 = 0x00000004;
pub const UART_INPUT_DSR :u32 = 0x00000002;
pub const UART_INPUT_CTS :u32 = 0x00000001;

//*****************************************************************************
//
// Values that can be passed to UARTFlowControl() or returned from
// UARTFlowControlGet().
//
//*****************************************************************************
pub const UART_FLOWCONTROL_TX :u32 = 0x00008000;
pub const UART_FLOWCONTROL_RX :u32 = 0x00004000;
pub const UART_FLOWCONTROL_NONE :u32 = 0x00000000;

//*****************************************************************************
//
// Values that can be passed to UARTTxIntModeSet() or returned from
// UARTTxIntModeGet().
//
//*****************************************************************************
pub const UART_TXINT_MODE_FIFO :u32 = 0x00000000;
pub const UART_TXINT_MODE_EOT :u32 = 0x00000010;

//*****************************************************************************
//
// Values that can be passed to UARTClockSourceSet() or returned from
// UARTClockSourceGet().
//
//*****************************************************************************
pub const UART_CLOCK_SYSTEM :u32 = 0x00000000;
pub const UART_CLOCK_PIOSC :u32 = 0x00000005;
