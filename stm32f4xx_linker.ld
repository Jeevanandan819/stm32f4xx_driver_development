/*=============================================================================
 * File: stm32f4xx_linker.ld
 * Description: Linker script for STM32 MCU, defining memory sections and layout.
 * Author: [Jeevanandan Sandan]
 * Date: [29-05-2025]
 *
 * Notes:
 * - Defines Flash and RAM memory regions.
 * - Specifies entry point and section allocations.
 * - Custom modifications for FreeRTOS integration.
 *=============================================================================
 */


MEMORY
{
    FLASH (rx) : ORIGIN=0x08000000, LENGTH=512K
    SRAM (rwx) : ORIGIN=0x20000000, LENGTH=128K
}

ENTRY(Reset_Handler)

_Min_Heap_Size = 20K; /* required amount of heap */
_Min_Stack_Size = 10K; /* required amount of stack */

_estack = ORIGIN(SRAM) + LENGTH(SRAM);

SECTIONS
{
    .isr_vector :
    {
        . = ALIGN(4);
        KEEP(*(.isr_vector))
        . = ALIGN(4);
    } >FLASH

    .text :
    {
        . = ALIGN(4);
        *(.text)
        *(.text*)
        KEEP (*(.init))
        KEEP (*(.fini))
        *(.rodata)         
        *(.rodata*)
        . = ALIGN(4);
        _etext = .;
    } >FLASH

    .data :
    {
        . = ALIGN(4);
        _sdata = .;
        *(.data)
        *(.data*)
        . = ALIGN(4);
        _edata = .;
    } >SRAM AT> FLASH

    _sidata = LOADADDR(.data);

    .bss :
    {
        . = ALIGN(4);
        _sbss = .;
        *(.bss)
        *(.bss*)
        . = ALIGN(4);
        _ebss = .;
    } >SRAM

    .user_stack_heap :
    {
        . = ALIGN(8);
        . = . + _Min_Stack_Size;
        . = . + _Min_Heap_Size;
        . = ALIGN(8);
    } >SRAM
}