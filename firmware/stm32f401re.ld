/* STM32F401RE: 512 KB flash, 96 KB SRAM. (F411RE has 128 KB SRAM; 96 KB is safe for both.) */
ENTRY(Reset_Handler)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 96K
}

__stack_top = ORIGIN(RAM) + LENGTH(RAM);   /* stack grows down from the end of RAM */

SECTIONS
{
    .isr_vector : { KEEP(*(.isr_vector)) } > FLASH

    .text : {
        *(.text*)
        *(.rodata*)
        . = ALIGN(4);
    } > FLASH

    .ARM.exidx : { *(.ARM.exidx*) } > FLASH

    /* .data: initialized globals. Stored in flash at __data_load_start, copied to RAM at boot. */
    __data_load_start = LOADADDR(.data);
    .data : {
        . = ALIGN(4);
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM AT > FLASH

    /* .bss: zero-initialized globals. Cleared at boot. */
    .bss (NOLOAD) : {
        . = ALIGN(4);
        __bss_start = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    /* keep 4 KB free for the stack */
    ASSERT(__bss_end + 4096 <= __stack_top, "RAM overflow: not enough room for the stack")
}
