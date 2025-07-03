ENTRY(Reset_Handler)

/* Stack pointer khai báo trước SECTIONS để tránh lỗi undefined `_estack` */
_estack = ORIGIN(RAM) + LENGTH(RAM);

MEMORY 
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 64K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}

SECTIONS
{
    .text : {
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
        *(.ARM.exidx*)   /* Cần có để tránh lỗi chồng với .data */
        KEEP(*(.init))
        KEEP(*(.fini))
        _etext = .;       /* Kết thúc phần text, dùng để copy .data */
    } > FLASH

    .data : AT(_etext) {
        _sdata = .;
        *(.data*)
        _edata = .;
    } > RAM

    .bss : {
        _sbss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
    } > RAM

    /* Dùng cho syscall (_sbrk) */
    _end = .;

    /* Địa chỉ copy .data từ FLASH (dùng trong startup) */
    _sidata = LOADADDR(.data);
}
