/*
 * Linker script para GD32F450VET6 (LQFP100)
 *  - Flash: 512 KB  (variante "VE")
 *  - RAM:   192 KB en 0x20000000 (SRAM0+SRAM1+SRAM2) + 64 KB TCM en 0x10000000
 *    (192 + 64 = 256 KB, coincide con los 256 KB de SRAM listados para la
 *    variante VE en el datasheet). El TCM solo es accesible por el bus de
 *    datos del core, no lo uses para buffers que acceda DMA/EXMC.
 */

ENTRY(Reset_Handler)

_estack = 0x10010000; /* CAMBIADO: el bootloader exige que el SP inicial de la app este en el
    rango 0x10000000-0x1001FFFF (TCM RAM), verificado con precision analizando su chequeo
    de validez ((SP<<3)>>20 == 0x800). Usar la SRAM principal (0x2003xxxx) da "APP not
    Programmed" aunque el resto este bien. */
_Min_Heap_Size  = 0x400;
_Min_Stack_Size = 0x800;

MEMORY
{
  RAM (xrw)   : ORIGIN = 0x20000000, LENGTH = 192K
  FLASH (rx)  : ORIGIN = 0x08020000, LENGTH = 384K  /* CAMBIADO: la app empieza en 0x08020000,
                  justo tras los 128KB del bootloader (confirmado por analisis del binario:
                  el bootloader salta ahi tras verificar un checksum). LENGTH=384K = 512K-128K. */
  TCMRAM (rw) : ORIGIN = 0x10000000, LENGTH = 64K
}

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
    *(.glue_7)
    *(.glue_7t)
    *(.eh_frame)
    KEEP (*(.init))
    KEEP (*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata :
  {
    . = ALIGN(4);
    *(.rodata)
    *(.rodata*)
    . = ALIGN(4);
  } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM : {
    __exidx_start = .;
    *(.ARM.exidx*)
    __exidx_end = .;
  } >FLASH

  .preinit_array :
  {
    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array*))
    PROVIDE_HIDDEN (__preinit_array_end = .);
  } >FLASH
  .init_array :
  {
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array*))
    PROVIDE_HIDDEN (__init_array_end = .);
  } >FLASH
  .fini_array :
  {
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(SORT(.fini_array.*)))
    KEEP (*(.fini_array*))
    PROVIDE_HIDDEN (__fini_array_end = .);
  } >FLASH

  _sidata = LOADADDR(.data);

  .data :
  {
    . = ALIGN(4);
    _sdata = .;
    *(.data)
    *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss :
  {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss)
    *(.bss*)
    *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  _sitcmram = LOADADDR(.tcmram);
  .tcmram :
  {
    . = ALIGN(4);
    _stcmram = .;
    *(.tcmram)
    *(.tcmram*)
    . = ALIGN(4);
    _etcmram = .;
  } >TCMRAM AT> FLASH

  ._user_heap_stack :
  {
    . = ALIGN(8);
    PROVIDE ( end = . );
    PROVIDE ( _end = . );
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >TCMRAM

  /DISCARD/ :
  {
    libc.a ( * )
    libm.a ( * )
    libgcc.a ( * )
  }

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
