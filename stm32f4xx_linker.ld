ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

PROVIDE(_stack_top = ORIGIN(SRAM) + LENGTH(SRAM));

SECTIONS
{
    .vector :
    {
        . = ALIGN(4);
        KEEP(*(.vector))
        . = ALIGN(4);
    } > FLASH

    .text : 
    {
        . = ALIGN(4);
        _stext = .;
        *(.text)
        *(.text.*)
        *(.rodata)
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } > FLASH

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data .data.*)
        . = ALIGN(4);
        _edata = .;
    } > SRAM AT> FLASH

    .bss (NOLOAD) :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss .bss.*)
        . = ALIGN(4);
        _ebss = .;
    } > SRAM
}