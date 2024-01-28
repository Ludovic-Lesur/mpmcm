/*
 * mpmcm_hw1-0.ld
 *
 *  Created on: 6 mar. 2023
 *      Author: ARM
 */

__STACK_SIZE = 0x00000400;
__HEAP_SIZE  = 0x00000C00;

MEMORY
{
  FLASH 	(rx)  : ORIGIN = 0x08000000, LENGTH = 126K
  CCMSRAM	(rwx) : ORIGIN = 0x10000000, LENGTH = 10K
  SRAM1   	(rwx) : ORIGIN = 0x20000000, LENGTH = 16K
  SRAM2		(rwx) : ORIGIN = 0x20014000, LENGTH = 6K
}

ENTRY(Reset_Handler)

SECTIONS
{
  .text :
  {
    KEEP(*(.vectors))
   	KEEP(*(.ResetHandler))
	KEEP(*(.DefaultHandler))
    *(.text*)
    *(.rodata*)

    KEEP(*(.eh_frame*))
  } > FLASH

  .ARM.extab :
  {
    *(.ARM.extab* .gnu.linkonce.armextab.*)
  } > FLASH

  __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } > FLASH
  __exidx_end = .;

  .copy.table :
  {
    . = ALIGN(4);
    __copy_table_start__ = .;
    LONG(LOADADDR(.data_sram1));
    LONG(    ADDR(.data_sram1));
	LONG(  SIZEOF(.data_sram1) / 4);
	LONG(LOADADDR(.data_sram2));
	LONG(    ADDR(.data_sram2));
	LONG(  SIZEOF(.data_sram2) / 4);
	LONG(LOADADDR(.data_ccmsram));
	LONG(    ADDR(.data_ccmsram));
	LONG(  SIZEOF(.data_ccmsram) / 4);
    __copy_table_end__ = .;
  } > FLASH

  .zero.table :
  {
    . = ALIGN(4);
    __zero_table_start__ = .;
    LONG(    ADDR(.bss_sram1));
	LONG(  SIZEOF(.bss_sram1) / 4);
	LONG(    ADDR(.bss_sram2));
	LONG(  SIZEOF(.bss_sram2) / 4);
	LONG(    ADDR(.bss_ccmsram));
	LONG(  SIZEOF(.bss_ccmsram) / 4);
    __zero_table_end__ = .;
  } > FLASH
  
  __etext = .;

  .data_sram1 : ALIGN(4)
  {
  	. = ALIGN(4);
  	__data_sram1_start__ = .;
    *(vtable)
    *(.data_sram1)
    *(.data_sram1.*)
    . = ALIGN(4);
    __data_sram1_end__ = .;
  } > SRAM1 AT>FLASH

  .data_sram2 : ALIGN(4)
  {
    . = ALIGN(4);
    __data_sram2_start__ = .;
    *(.data_sram2)
    *(.data_sram2.*)
    . = ALIGN(4);
    __data_sram2_end__ = .;
  } > SRAM2 AT>FLASH
  
   .data_ccmsram : ALIGN(4)
  {
    . = ALIGN(4);
    __data_ccmsram_start__ = .;
    *(.data_ccmsram)
    *(.data_ccmsram.*)
    . = ALIGN(4);
    __data_ccmsram_end__ = .;
  } > CCMSRAM AT>FLASH

  .bss_sram1 :
  {
    . = ALIGN(4);
    __bss_start__ = .;
    *(.bss)
    *(.bss.*)
    *(COMMON)
    . = ALIGN(4);
    __bss_end__ = .;
  } > SRAM1 

  .bss_sram2 :
  {
    . = ALIGN(4);
    *(.bss_sram2)
    *(.bss_sram2.*)
    . = ALIGN(4);
  } > SRAM2 
  
  .bss_ccmsram :
  {
    . = ALIGN(4);
    *(.bss_ccmsram)
    *(.bss_ccmsram.*)
    . = ALIGN(4);
  } > CCMSRAM 

  .heap (COPY) :
  {
    . = ALIGN(8);
    __end__ = .;
    PROVIDE(end = .);
    . = . + __HEAP_SIZE;
    . = ALIGN(8);
    __HeapLimit = .;
  } > SRAM1

  .stack (ORIGIN(SRAM1) + LENGTH(SRAM1) - __STACK_SIZE) (COPY) :
  {
    . = ALIGN(8);
    __StackLimit = .;
    . = . + __STACK_SIZE;
    . = ALIGN(8);
    __StackTop = .;
  } > SRAM1
  PROVIDE(__stack = __StackTop);

  /* Check if data + heap + stack exceeds RAM limit */
  ASSERT(__StackLimit >= __HeapLimit, "region RAM overflowed with stack")
}
