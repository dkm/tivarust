/*
* Memory definition:
* FLASH:	start point 0x00,		lenght 0x40000.
* SRAM:		start point 0x20000000	length 0x8000.
* STACK:	start point 0x20007FFF	lenght 0x0.
*/
MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
    STACK (rwx) : ORIGIN = 0x20007FFF , LENGTH = 0x00000000
}

SECTIONS
{	
    .vectors 0x0 :
    {
      KEEP(*(.vectors))
    } >FLASH

    
    .text 0x400 : 
    {
    	_text_start = .;
        *(.text.*)
        *(.rodata.*)
        _end_text = .;
    }>FLASH
    
    .data : 
    {
    	_data_start = .;
        *(.data.*)
        *(vtable)
        _data_end = .;
    }>RAM AT >FLASH
    _data_load = LOADADDR(.data);

    .bss : 
    {
    	_bss_start = .;
        *(.bss.*)
        *(COMMON)
        _bss_end = .;
    }>RAM
    

    .stack : 
    {
	_stack_top = .;
    }>STACK
}
