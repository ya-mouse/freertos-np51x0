MEMORY 
{
	flash	: ORIGIN = 0, LENGTH = 512K
	ram	: ORIGIN = 0x10000000, LENGTH = 64M
}

/* define beginning and ending of stack */
__stack_end__ = ORIGIN(ram)+LENGTH(ram) - 4;

SECTIONS 
{
	. = 0;
	startup : { *(.startup)} >flash

	prog : 
	{
		*(.text)
		*(.rodata)
		*(.rodata*)
		*(.glue_7)
		*(.glue_7t)
	} >flash

	.ARM.exidx :
	{
		__exidx_start = .;
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
		__exidx_end = .;
	} >flash

	. = ALIGN(4);
	__end_of_text__ = .;

	.data : AT ( __end_of_text__ ) 
	{
		__data_beg__ = .;
		*(.data)
		__data_end__ = .;
	} >ram

	. = ALIGN(16);
	.netbuf(NOLOAD) :
	{
		*(.nbuf) 
	} >ram

	.bss : 
	{
		__bss_beg__ = .;
		*(.bss)
	} >ram

	. = ALIGN(4);
	.eh_frame :
	{
		 KEEP (*(.eh_frame))
	} > ram
	
	/* Align here to ensure that the .bss section occupies space up to
	_end.  Align after .bss to ensure correct alignment even if the
	.bss section disappears because there are no input sections.  */
	. = ALIGN(32 / 8);
}
	. = ALIGN(32 / 8);
	_end = .;
	_bss_end__ = . ; __bss_end__ = . ; __end__ = . ;
	PROVIDE (end = .);


