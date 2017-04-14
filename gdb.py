#!/usr/bin/env python3
import binascii
import struct
import re
import argparse

class GpioHook:
    def __init__(self):
        pass

    def call(self, reg, val):
        if reg[0].startswith('GPIOPCTL_'):
            print "  > ",
            mask = 0xF
            pm = []
            for i in range(8):
                v = (val & (mask << (4*i)))>>4*i
                pm.append(str(v))
            print ', '.join(['pm[{}]={}'.format(idx, pmval) for idx,pmval in enumerate(pm)])

        elif reg[0].startswith('GPIOAFSEL_'):
            print "  > ",
            mask = 0xFF
            val = val & mask
            afsel = []
            for i in range(8):
                v = (val & (1<<i)) != 0
                afsel.append(str(v))

            print ', '.join(['AF[{}]={}'.format(idx, afval) for idx,afval in enumerate(afsel)])

        elif reg[0].startswith('GPIOSLR_'):
            print "  > ",
            mask = 0xFF
            val = val & mask
            srvals = []
            for i in range(8):
                v = (val & (1<<i)) != 0
                srvals.append(str(v))

            print ', '.join(['SlewRate[{}]={}'.format(idx, srval) for idx,srval in enumerate(srvals)])

        elif reg[0].startswith('GPIODR2R_'):
            print "  > ",
            mask = 0xFF
            val = val & mask
            d2vals = []
            for i in range(8):
                v = (val & (1<<i)) != 0
                d2vals.append(str(v))

            print ', '.join(['2mA[{}]={}'.format(idx, d2val) for idx,d2val in enumerate(d2vals)])

        elif reg[0].startswith('GPIODR4R_'):
            print "  > ",
            mask = 0xFF
            val = val & mask
            d4vals = []
            for i in range(8):
                v = (val & (1<<i)) != 0
                d4vals.append(str(v))

            print ', '.join(['4mA[{}]={}'.format(idx, d4val) for idx,d4val in enumerate(d4vals)])

        elif reg[0].startswith('GPIODR8R_'):
            print "  > ",
            mask = 0xFF
            val = val & mask
            d8vals = []
            for i in range(8):
                v = (val & (1<<i)) != 0
                d8vals.append(str(v))

            print ', '.join(['8mA[{}]={}'.format(idx, d8val) for idx,d8val in enumerate(d8vals)])

        elif reg[0].startswith('GPIODIR_'):
            print '  > ',
            mask = 0xFF
            val = val & mask
            dirvals = []
            for i in range(8):
                v = (val & (1<<i)) != 0
                dirvals.append('OUT' if v else 'IN')

            print ', '.join(['Dir[{}]={}'.format(idx, dirval) for idx,dirval in enumerate(dirvals)])
            
class RCC2Hook:
    def call(self, reg, val):
        usercc2 = (val &(1<<32))!=0
        print '  > [USERCC2] Use RCC2: {}'.format(usercc2)
        if not usercc2:
            return

class RCCHook:
    def call(self, reg, val):
        print '  > Main oscillator {}'.format('DISABLED'if val & 0x1 else 'ENABLED')
        oscsrc = (val & (0x3<<4))>>4
        oscvals = {0:'[MOSC] Main Oscillator',
                   1:'[PIOSC] Precision internal oscillator',
                   2:'[PIOSC/4] Precision internal oscillator/4',
                   3:'[LFIOSC] Low-freq internal oscillator/4',
                   }
        print '  > Oscillator source: {}'.format(oscvals[oscsrc])
        xtalvals = {
            0x6:('4', 'Reserved'),
            0x7:('4.096', 'Reserved'),
            0x8:('4.9152', 'Reserved'),
            0x9:('5','5'),
            0xA:('5.12', '5.12'),
            0xB:('6', '6'),
            0xC:('6.144', '6.144'),
            0xD:('7.3728', '7.3728'),
            0xE:('8', '8'),
            0xF:('8.192', '8.192'),
            0x10:('10.0', '10.0'),
            0x11:('12.0', '12.0'),
            0x12:('12.288', '12.288'),
            0x13:('13.56', '13.56'),
            0x14:('14.31818', '14.31818'),
            0x15:('16.0', '16.0'),
            0x16:('16.364', '16.364'),
            0x17:('18.0', '18.0'),
            0x18:('20.0', '20.0'),
            0x19:('24.0', '24.0'),
            0x1A:('25.0', '25.0'),
            }
        xtalval = (val & (0x1F<<6))>>6
        print '  > [XTAL] Crystal value {} Mhz'.format(xtalvals[xtalval][0])

        pllbypass = (val & (1<<11)) != 0
        print '  > [BYPASS] The system clock is {} divided by SYSDIV'.format('derived from the OSC' if pllbypass else 'the PLL output')

        pllpwrdown = (val & (1<<13)) != 0
        print '  > PLL is {}'.format('powered down' if pllpwrdown else 'operating normally')

        pwmdiv = (val & (0x7<<17))>>17
        print '  > [PWMDIV] PWM Unit Clock Divisor is {}'.format(2**(pwmdiv+1) if pwmdiv < 5 else '64')

        pwmclockdiv = (val & (1<<20)) != 0
        print '  > [USEPWMDIV] PWM Clock Divisor : {} is the source for PWM'.format('The PWM clock divider' if pwmclockdiv else 'The system clock divider')

        usesysdiv = (val & (1<<22)) != 0
        print '  > [USESYSDIV] Enable System Clock Divider: {}'.format('system clock divider is used (forced when PLL is selected as the source)' if usesysdiv else 'The system clock is used undivided')

        sysdiv = (val & (0xF<<23))>>23
        print '  > [SYSDIV] System Clock Divisor: {}'.format(sysdiv+1)
        if not pllbypass:
            print '    > {}'.format({
                0x0 : 'reserved',
                0x1 : 'reserved',
                0x2 : '66.67 Mhz',
                0x3 : '50 Mhz',
                0x4 : '40 Mhz',
                0x5 : '33.33 Mhz',
                0x6 : '28.57 Mhz',
                0x7 : '25 Mhz',
                0x8 : '22.22 Mhz',
                0x9 : '20 Mhz',
                0xa : '18.18 Mhz',
                0xb : '16.67 Mhz',
                0xc : '15.38 Mhz',
                0xd : '14.29 Mhz',
                0xe : '13.33 Mhz',
                0xf : '12.5 Mhz',
            }[sysdiv])

        acg = (val & (1<<27)) != 0
        print '  > [ACG] Auto Clock Gating: {}'.format('SCGCn/DCGCn registers are used' if acg else 'RCGCn registers are used')
class UartHook:
    def __init__(self, baudrate = 115200):
        self.ibrd = None
        self.fbrd = None
        self.baudrate = baudrate

    def call(self, reg, val):
        if reg[0].startswith('UARTIBRD_'):
            self.ibrd = val
        elif reg[0].startswith('UARTFBRD_'):
            self.fbrd = val
            print "  > ",
            print "UART clock is {} for baudrate {}".format(
                ((2 * (64 * self.ibrd + self.fbrd)-1) * self.baudrate)/8,
                self.baudrate)
        elif reg[0].startswith('UARTLCHR_'):
            print '  > BRK: {}'.format('send' if (val & 0x1) else 'normal' )
            print '  > Parity [PEN]: {}'.format('enabled' if (val & 0x2) else 'disabled')
            print '  > Even parity select [EPS]: {}'.format('enabled' if (val & 0x4) else 'disabled')
            print '  > 2 stop bits [STP2]: {}'.format('enabled' if (val & (1<<3)) else 'disabled')
            print '  > Enable FIFO [FEN]: {}'.format('enabled' if (val & (1<<4)) else 'disabled')
            wlen = (val & (0x3<<5))>>5
            print '  > Word len [WLEN]: {} bits'.format(wlen + 5)
            print '  > Stick Parity Select [SPS]: {}'.format('checked...' if (val & (1<<7)) else 'disabled')
            
        elif reg[0].startswith('UARTCTL_'):
            print '  > UART Enable [UARTEN]: {}'.format((val & 0x1) != 0)
            print '  > UART Sir Enable [SIREN]: {}'.format((val & 0x2) != 0)
            print '  > UART Sir LowPower Mode [SIRLP]: {}'.format('enabled' if ((val & (1<<2)) != 0) else 'disabled')
            print '  > ISO 7816 support [SMART]: {}'.format('smart mode' if ((val & (0x1<<3)) != 0) else 'normal')
            print '  > End of Transmission [EOT]: TXRIS set when {}'.format('all transmitted' if (val & (1<<4)) else 'UARTIFLS conditions...' )
            print '  > High Speed enable [HSE]: sys clock divided by {}'.format(8 if (val&(1<<5)) else 16)
            print '  > Loop Back Enable [LBE]: {}'.format('enabled' if (val & (1<<7)) else 'disabled')
            print '  > UART Transmit Enable [TXE]: {}'.format('enabled' if (val & (1<<8)) else 'disabled')
            print '  > UART Receive Enable [RXE]: {}'.format('enabled' if (val & (1<<9)) else 'disabled')
            rtsen = (val & (1<<14)) != 0
            
            print '  > Request to send [RTS]: {}'.format('set' if (val & (1<<9)) and not rtsen else 'clear' if (val & (1<<9)) and not rtsen else 'discarded' )
            print '  > Enable Request to Send [RTSEN]: Hardware flow contrlol is {}'.format('enable' if rtsen else 'disabled')
            print '  > Enable Clear to Send [CTSEN]: CTS hardware flow is {}'.format('enable' if (val & (1<<15)) else 'disabled')

class GpioHbctlHook:
    def call(self, reg, val):
        print "  > Use HPB : ",
        mask = 0xFF
        val = val & mask
        hpb_vals = []
        ports = ['PORTA', 'PORTB', 'PORTC', 'PORTD',
                 'PORTE', 'PORTF']
        for i in range(8):
            v = (val & (1<<i)) != 0
            hpb_vals.append(str(v))

        print ', '.join(['{}: {}'.format(prt, hpb_val) for prt,hpb_val in zip(ports, hpb_vals)])
            
                                                          
parser = argparse.ArgumentParser(description='Parse command for reading Tiva status.')
parser.add_argument("-r", '--register', action='append', help='a regexp for matching some registers')
parser.add_argument('--use-apb', action='store_true', help='use legacy APB instead of AHB for GPIO registers')

class ReadTiva (gdb.Command):
  """Read something."""

  def __init__ (self):
    super (ReadTiva, self).__init__ ("read-tiva", gdb.COMMAND_USER)

  def single_read(self, reg):
    name = reg[0]
    addr = reg[1]
    size = reg[2]
    hook = reg[3]

    inf = gdb.selected_inferior ()

    m = inf.read_memory (addr, size)
    val = struct.unpack('i', m)[0]

    print "{} : 0x{:08X} @ 0x{:08X}".format(name, val, addr)

    if hook:
        hook.call(reg,val)

  def invoke (self, arg, from_tty):
    args = gdb.string_to_argv(arg)
    parsed_args = parser.parse_args(args)

    uart0hook = UartHook()
    gpiohook = GpioHook()
    rcchook = RCCHook()
    rcc2hook = RCC2Hook()
    gpiohbctlhook = GpioHbctlHook()

    all_regs = [
        ('RCC', 0x400FE060, 4, rcchook),
        ('RCC2', 0x400FE070, 4, rcc2hook),
        ('RCGCUART', 0x400FE618, 4, None),
        ('RCGCGPIO', 0x400FE608, 4, None),
        ('UARTIBRD_0', 0x4000C024, 4, uart0hook),
        ('UARTFBRD_0', 0x4000C028, 4, uart0hook),
        ('UARTLCHR_0', 0x4000C02C, 4, uart0hook),
        ('UARTCTL_0', 0x4000C030, 4, uart0hook),
        ('GPIOHBCTL', 0x400FE06C, 4, gpiohbctlhook),
    ]
    if parsed_args.use_apb:
        all_regs += [
            ('GPIODIR_PORTA', 0x40004400, 4, gpiohook),
            ('GPIOAFSEL_PORTA', 0x40004420, 4, gpiohook),
            ('GPIOPCTL_PORTA', 0x4000452C, 4, gpiohook),
            ('GPIOSLR_PORTA', 0x40004518, 4, gpiohook),
            ('GPIODR2R_PORTA', 0x40004500, 4, gpiohook),
            ('GPIODR4R_PORTA', 0x40004504, 4, gpiohook),
            ('GPIODR8R_PORTA', 0x40004508, 4, gpiohook),

            ('GPIODIR_PORTB', 0x40005400, 4, gpiohook),
            ('GPIOAFSEL_PORTB', 0x40005420, 4, gpiohook),
            ('GPIOPCTL_PORTB', 0x4000552C, 4, gpiohook),
            ('GPIOSLR_PORTB', 0x40005518, 4, gpiohook),
            ('GPIODR2R_PORTB', 0x40005500, 4, gpiohook),
            ('GPIODR4R_PORTB', 0x40005504, 4, gpiohook),
            ('GPIODR8R_PORTB', 0x40005508, 4, gpiohook),

            ('GPIODIR_PORTC', 0x40006400, 4, gpiohook),
            ('GPIOAFSEL_PORTC', 0x40006420, 4, gpiohook),
            ('GPIOPCTL_PORTC', 0x4000652C, 4, gpiohook),
            ('GPIOSLR_PORTC', 0x40006518, 4, gpiohook),
            ('GPIODR2R_PORTC', 0x40006500, 4, gpiohook),
            ('GPIODR4R_PORTC', 0x40006504, 4, gpiohook),
            ('GPIODR8R_PORTC', 0x40006508, 4, gpiohook),

            ('GPIODIR_PORTD', 0x40007400, 4, gpiohook),
            ('GPIOAFSEL_PORTD', 0x40007420, 4, gpiohook),
            ('GPIOPCTL_PORTD', 0x4000752C, 4, gpiohook),
            ('GPIOSLR_PORTD', 0x40007518, 4, gpiohook),
            ('GPIODR2R_PORTD', 0x40007500, 4, gpiohook),
            ('GPIODR4R_PORTD', 0x40007504, 4, gpiohook),
            ('GPIODR8R_PORTD', 0x40007508, 4, gpiohook),

            ('GPIODIR_PORTE', 0x40024400, 4, gpiohook),
            ('GPIOAFSEL_PORTE', 0x40024420, 4, gpiohook),
            ('GPIOPCTL_PORTE', 0x4002452C, 4, gpiohook),
            ('GPIOSLR_PORTE', 0x40024518, 4, gpiohook),
            ('GPIODR2R_PORTE', 0x40024500, 4, gpiohook),
            ('GPIODR4R_PORTE', 0x40024504, 4, gpiohook),
            ('GPIODR8R_PORTE', 0x40024508, 4, gpiohook),


            ('GPIODIR_PORTF', 0x40025400, 4, gpiohook),
            ('GPIOAFSEL_PORTF', 0x40025420, 4, gpiohook),
            ('GPIOPCTL_PORTF', 0x4002552C, 4, gpiohook),
            ('GPIOSLR_PORTF', 0x40025518, 4, gpiohook),
            ('GPIODR2R_PORTF', 0x40025500, 4, gpiohook),
            ('GPIODR4R_PORTF', 0x40025504, 4, gpiohook),
            ('GPIODR8R_PORTF', 0x40025508, 4, gpiohook),

        ]
    else:
        all_regs += [
            ('GPIODIR_PORTA', 0x40058400, 4, gpiohook),
            ('GPIOAFSEL_PORTA', 0x40058420, 4, gpiohook),
            ('GPIOPCTL_PORTA', 0x4005852C, 4, gpiohook),
            ('GPIOSLR_PORTA', 0x40058518, 4, gpiohook),
            ('GPIODR2R_PORTA', 0x40058500, 4, gpiohook),
            ('GPIODR4R_PORTA', 0x40058504, 4, gpiohook),
            ('GPIODR8R_PORTA', 0x40058508, 4, gpiohook),

            ('GPIODIR_PORTB', 0x40059400, 4, gpiohook),
            ('GPIOAFSEL_PORTB', 0x40059420, 4, gpiohook),
            ('GPIOPCTL_PORTB', 0x4005952C, 4, gpiohook),
            ('GPIOSLR_PORTB', 0x40059518, 4, gpiohook),
            ('GPIODR2R_PORTB', 0x40059500, 4, gpiohook),
            ('GPIODR4R_PORTB', 0x40059504, 4, gpiohook),
            ('GPIODR8R_PORTB', 0x40059508, 4, gpiohook),

            ('GPIODIR_PORTC', 0x4005A400, 4, gpiohook),
            ('GPIOAFSEL_PORTC', 0x4005A420, 4, gpiohook),
            ('GPIOPCTL_PORTC', 0x4005A52C, 4, gpiohook),
            ('GPIOSLR_PORTC', 0x4005A518, 4, gpiohook),
            ('GPIODR2R_PORTC', 0x4005A500, 4, gpiohook),
            ('GPIODR4R_PORTC', 0x4005A504, 4, gpiohook),
            ('GPIODR8R_PORTC', 0x4005A508, 4, gpiohook),

            ('GPIODIR_PORTD', 0x4005B400, 4, gpiohook),
            ('GPIOAFSEL_PORTD', 0x4005B420, 4, gpiohook),
            ('GPIOPCTL_PORTD', 0x4005B52C, 4, gpiohook),
            ('GPIOSLR_PORTD', 0x4005B518, 4, gpiohook),
            ('GPIODR2R_PORTD', 0x4005B500, 4, gpiohook),
            ('GPIODR4R_PORTD', 0x4005B504, 4, gpiohook),
            ('GPIODR8R_PORTD', 0x4005B508, 4, gpiohook),

            ('GPIODIR_PORTE', 0x4005C400, 4, gpiohook),
            ('GPIOAFSEL_PORTE', 0x4005C420, 4, gpiohook),
            ('GPIOPCTL_PORTE', 0x4005C52C, 4, gpiohook),
            ('GPIOSLR_PORTE', 0x4005C518, 4, gpiohook),
            ('GPIODR2R_PORTE', 0x4005C500, 4, gpiohook),
            ('GPIODR4R_PORTE', 0x4005C504, 4, gpiohook),
            ('GPIODR8R_PORTE', 0x4005C508, 4, gpiohook),

            ('GPIODIR_PORTF', 0x4005D400, 4, gpiohook),
            ('GPIOAFSEL_PORTF', 0x4005D420, 4, gpiohook),
            ('GPIOPCTL_PORTF', 0x4005D52C, 4, gpiohook),
            ('GPIOSLR_PORTF', 0x4005D518, 4, gpiohook),
            ('GPIODR2R_PORTF', 0x4005D500, 4, gpiohook),
            ('GPIODR4R_PORTF', 0x4005D504, 4, gpiohook),
            ('GPIODR8R_PORTF', 0x4005D508, 4, gpiohook),

        ]

    reg_to_print = all_regs
    
    if parsed_args.register:
        reg_to_print = []

        for regex in [re.compile(x) for x in parsed_args.register]:
            filtered = [x for x in all_regs]
            reg_to_print += filter(lambda x: regex.match(x[0]), all_regs)

    for reg in reg_to_print:
        self.single_read(reg)

ReadTiva ()
