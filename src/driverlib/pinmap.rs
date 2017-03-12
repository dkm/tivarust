//*****************************************************************************
//
// pin_map.h - Mapping of peripherals to pins for all parts.
//
// Copyright (c) 2007-2014 Texas Instruments Incorporated.  All rights reserved.
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
// TM4C123GH6PM Port/Pin Mapping Definitions
//
//*****************************************************************************

pub const GPIO_PA0_U0RX :u32 = 0x00000001;
pub const GPIO_PA0_CAN1RX :u32 = 0x00000008;

pub const GPIO_PA1_U0TX :u32 = 0x00000401;
pub const GPIO_PA1_CAN1TX :u32 = 0x00000408;

pub const GPIO_PA2_SSI0CLK :u32 = 0x00000802;

pub const GPIO_PA3_SSI0FSS :u32 = 0x00000C02;

pub const GPIO_PA4_SSI0RX :u32 = 0x00001002;

pub const GPIO_PA5_SSI0TX :u32 = 0x00001402;

pub const GPIO_PA6_I2C1SCL :u32 = 0x00001803;
pub const GPIO_PA6_M1PWM2 :u32 = 0x00001805;

pub const GPIO_PA7_I2C1SDA :u32 = 0x00001C03;
pub const GPIO_PA7_M1PWM3 :u32 = 0x00001C05;

pub const GPIO_PB0_U1RX :u32 = 0x00010001;
pub const GPIO_PB0_T2CCP0 :u32 = 0x00010007;

pub const GPIO_PB1_U1TX :u32 = 0x00010401;
pub const GPIO_PB1_T2CCP1 :u32 = 0x00010407;

pub const GPIO_PB2_I2C0SCL :u32 = 0x00010803;
pub const GPIO_PB2_T3CCP0 :u32 = 0x00010807;

pub const GPIO_PB3_I2C0SDA :u32 = 0x00010C03;
pub const GPIO_PB3_T3CCP1 :u32 = 0x00010C07;

pub const GPIO_PB4_SSI2CLK :u32 = 0x00011002;
pub const GPIO_PB4_M0PWM2 :u32 = 0x00011004;
pub const GPIO_PB4_T1CCP0 :u32 = 0x00011007;
pub const GPIO_PB4_CAN0RX :u32 = 0x00011008;

pub const GPIO_PB5_SSI2FSS :u32 = 0x00011402;
pub const GPIO_PB5_M0PWM3 :u32 = 0x00011404;
pub const GPIO_PB5_T1CCP1 :u32 = 0x00011407;
pub const GPIO_PB5_CAN0TX :u32 = 0x00011408;

pub const GPIO_PB6_SSI2RX :u32 = 0x00011802;
pub const GPIO_PB6_M0PWM0 :u32 = 0x00011804;
pub const GPIO_PB6_T0CCP0 :u32 = 0x00011807;

pub const GPIO_PB7_SSI2TX :u32 = 0x00011C02;
pub const GPIO_PB7_M0PWM1 :u32 = 0x00011C04;
pub const GPIO_PB7_T0CCP1 :u32 = 0x00011C07;

pub const GPIO_PC0_TCK :u32 = 0x00020001;
pub const GPIO_PC0_SWCLK :u32 = 0x00020001;
pub const GPIO_PC0_T4CCP0 :u32 = 0x00020007;

pub const GPIO_PC1_TMS :u32 = 0x00020401;
pub const GPIO_PC1_SWDIO :u32 = 0x00020401;
pub const GPIO_PC1_T4CCP1 :u32 = 0x00020407;

pub const GPIO_PC2_TDI :u32 = 0x00020801;
pub const GPIO_PC2_T5CCP0 :u32 = 0x00020807;

pub const GPIO_PC3_SWO :u32 = 0x00020C01;
pub const GPIO_PC3_TDO :u32 = 0x00020C01;
pub const GPIO_PC3_T5CCP1 :u32 = 0x00020C07;

pub const GPIO_PC4_U4RX :u32 = 0x00021001;
pub const GPIO_PC4_U1RX :u32 = 0x00021002;
pub const GPIO_PC4_M0PWM6 :u32 = 0x00021004;
pub const GPIO_PC4_IDX1 :u32 = 0x00021006;
pub const GPIO_PC4_WT0CCP0 :u32 = 0x00021007;
pub const GPIO_PC4_U1RTS :u32 = 0x00021008;

pub const GPIO_PC5_U4TX :u32 = 0x00021401;
pub const GPIO_PC5_U1TX :u32 = 0x00021402;
pub const GPIO_PC5_M0PWM7 :u32 = 0x00021404;
pub const GPIO_PC5_PHA1 :u32 = 0x00021406;
pub const GPIO_PC5_WT0CCP1 :u32 = 0x00021407;
pub const GPIO_PC5_U1CTS :u32 = 0x00021408;

pub const GPIO_PC6_U3RX :u32 = 0x00021801;
pub const GPIO_PC6_PHB1 :u32 = 0x00021806;
pub const GPIO_PC6_WT1CCP0 :u32 = 0x00021807;
pub const GPIO_PC6_USB0EPEN :u32 = 0x00021808;

pub const GPIO_PC7_U3TX :u32 = 0x00021C01;
pub const GPIO_PC7_WT1CCP1 :u32 = 0x00021C07;
pub const GPIO_PC7_USB0PFLT :u32 = 0x00021C08;

pub const GPIO_PD0_SSI3CLK :u32 = 0x00030001;
pub const GPIO_PD0_SSI1CLK :u32 = 0x00030002;
pub const GPIO_PD0_I2C3SCL :u32 = 0x00030003;
pub const GPIO_PD0_M0PWM6 :u32 = 0x00030004;
pub const GPIO_PD0_M1PWM0 :u32 = 0x00030005;
pub const GPIO_PD0_WT2CCP0 :u32 = 0x00030007;

pub const GPIO_PD1_SSI3FSS :u32 = 0x00030401;
pub const GPIO_PD1_SSI1FSS :u32 = 0x00030402;
pub const GPIO_PD1_I2C3SDA :u32 = 0x00030403;
pub const GPIO_PD1_M0PWM7 :u32 = 0x00030404;
pub const GPIO_PD1_M1PWM1 :u32 = 0x00030405;
pub const GPIO_PD1_WT2CCP1 :u32 = 0x00030407;

pub const GPIO_PD2_SSI3RX :u32 = 0x00030801;
pub const GPIO_PD2_SSI1RX :u32 = 0x00030802;
pub const GPIO_PD2_M0FAULT0 :u32 = 0x00030804;
pub const GPIO_PD2_WT3CCP0 :u32 = 0x00030807;
pub const GPIO_PD2_USB0EPEN :u32 = 0x00030808;

pub const GPIO_PD3_SSI3TX :u32 = 0x00030C01;
pub const GPIO_PD3_SSI1TX :u32 = 0x00030C02;
pub const GPIO_PD3_IDX0 :u32 = 0x00030C06;
pub const GPIO_PD3_WT3CCP1 :u32 = 0x00030C07;
pub const GPIO_PD3_USB0PFLT :u32 = 0x00030C08;

pub const GPIO_PD4_U6RX :u32 = 0x00031001;
pub const GPIO_PD4_WT4CCP0 :u32 = 0x00031007;

pub const GPIO_PD5_U6TX :u32 = 0x00031401;
pub const GPIO_PD5_WT4CCP1 :u32 = 0x00031407;

pub const GPIO_PD6_U2RX :u32 = 0x00031801;
pub const GPIO_PD6_M0FAULT0 :u32 = 0x00031804;
pub const GPIO_PD6_PHA0 :u32 = 0x00031806;
pub const GPIO_PD6_WT5CCP0 :u32 = 0x00031807;

pub const GPIO_PD7_U2TX :u32 = 0x00031C01;
pub const GPIO_PD7_PHB0 :u32 = 0x00031C06;
pub const GPIO_PD7_WT5CCP1 :u32 = 0x00031C07;
pub const GPIO_PD7_NMI :u32 = 0x00031C08;

pub const GPIO_PE0_U7RX :u32 = 0x00040001;

pub const GPIO_PE1_U7TX :u32 = 0x00040401;

pub const GPIO_PE4_U5RX :u32 = 0x00041001;
pub const GPIO_PE4_I2C2SCL :u32 = 0x00041003;
pub const GPIO_PE4_M0PWM4 :u32 = 0x00041004;
pub const GPIO_PE4_M1PWM2 :u32 = 0x00041005;
pub const GPIO_PE4_CAN0RX :u32 = 0x00041008;

pub const GPIO_PE5_U5TX :u32 = 0x00041401;
pub const GPIO_PE5_I2C2SDA :u32 = 0x00041403;
pub const GPIO_PE5_M0PWM5 :u32 = 0x00041404;
pub const GPIO_PE5_M1PWM3 :u32 = 0x00041405;
pub const GPIO_PE5_CAN0TX :u32 = 0x00041408;

pub const GPIO_PF0_U1RTS :u32 = 0x00050001;
pub const GPIO_PF0_SSI1RX :u32 = 0x00050002;
pub const GPIO_PF0_CAN0RX :u32 = 0x00050003;
pub const GPIO_PF0_M1PWM4 :u32 = 0x00050005;
pub const GPIO_PF0_PHA0 :u32 = 0x00050006;
pub const GPIO_PF0_T0CCP0 :u32 = 0x00050007;
pub const GPIO_PF0_NMI :u32 = 0x00050008;
pub const GPIO_PF0_C0O :u32 = 0x00050009;

pub const GPIO_PF1_U1CTS :u32 = 0x00050401;
pub const GPIO_PF1_SSI1TX :u32 = 0x00050402;
pub const GPIO_PF1_M1PWM5 :u32 = 0x00050405;
pub const GPIO_PF1_PHB0 :u32 = 0x00050406;
pub const GPIO_PF1_T0CCP1 :u32 = 0x00050407;
pub const GPIO_PF1_C1O :u32 = 0x00050409;
pub const GPIO_PF1_TRD1 :u32 = 0x0005040E;

pub const GPIO_PF2_SSI1CLK :u32 = 0x00050802;
pub const GPIO_PF2_M0FAULT0 :u32 = 0x00050804;
pub const GPIO_PF2_M1PWM6 :u32 = 0x00050805;
pub const GPIO_PF2_T1CCP0 :u32 = 0x00050807;
pub const GPIO_PF2_TRD0 :u32 = 0x0005080E;

pub const GPIO_PF3_SSI1FSS :u32 = 0x00050C02;
pub const GPIO_PF3_CAN0TX :u32 = 0x00050C03;
pub const GPIO_PF3_M1PWM7 :u32 = 0x00050C05;
pub const GPIO_PF3_T1CCP1 :u32 = 0x00050C07;
pub const GPIO_PF3_TRCLK :u32 = 0x00050C0E;

pub const GPIO_PF4_M1FAULT0 :u32 = 0x00051005;
pub const GPIO_PF4_IDX0 :u32 = 0x00051006;
pub const GPIO_PF4_T2CCP0 :u32 = 0x00051007;
pub const GPIO_PF4_USB0EPEN :u32 = 0x00051008;
