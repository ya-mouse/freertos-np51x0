	/* Sample initialization file */

	.extern main
	.extern exit

	.text
	.code 32


	.align  0

	.extern __bss_beg__
	.extern __bss_end__
	.extern __stack_end__
	.extern __data_beg__
	.extern __data_end__
	.extern __data+beg_src__

	.global start
	.global endless_loop

	/* Stack Sizes */
	.set	UND_STACK_SIZE, 0x00000004
	.set	ABT_STACK_SIZE, 0x00000004
	.set	FIQ_STACK_SIZE, 0x00000004
	.set	IRQ_STACK_SIZE, 0X00000400
	.set	SVC_STACK_SIZE, 0x00000400

	/* Standard definitions of Mode bits and Interrupt (I & F) flags in PSRs */
	.set	MODE_USR, 0x10            /* User Mode */
	.set	MODE_FIQ, 0x11            /* FIQ Mode */
	.set	MODE_IRQ, 0x12            /* IRQ Mode */
	.set	MODE_SVC, 0x13            /* Supervisor Mode */
	.set	MODE_ABT, 0x17            /* Abort Mode */
	.set	MODE_UND, 0x1B            /* Undefined Mode */
	.set	MODE_SYS, 0x1F            /* System Mode */

	.equ	I_BIT, 0x80               /* when I bit is set, IRQ is disabled */
	.equ	F_BIT, 0x40               /* when F bit is set, FIQ is disabled */


start:
_start:
_mainCRTStartup:

	/* Setup a stack for each mode - note that this only sets up a usable stack
	for system/user, SWI and IRQ modes.   Also each mode is setup with
	interrupts initially disabled. */
	ldr	r0, .LC6
	msr	CPSR_c, #MODE_UND|I_BIT|F_BIT /* Undefined Instruction Mode */
	mov	sp, r0
	sub	r0, r0, #UND_STACK_SIZE
	msr	CPSR_c, #MODE_ABT|I_BIT|F_BIT /* Abort Mode */
	mov	sp, r0
	sub	r0, r0, #ABT_STACK_SIZE
	msr	CPSR_c, #MODE_FIQ|I_BIT|F_BIT /* FIQ Mode */
	mov	sp, r0
	sub	r0, r0, #FIQ_STACK_SIZE
	msr	CPSR_c, #MODE_IRQ|I_BIT|F_BIT /* IRQ Mode */
	mov	sp, r0
	sub	r0, r0, #IRQ_STACK_SIZE
	msr	CPSR_c, #MODE_SVC|I_BIT|F_BIT /* Supervisor Mode */
	mov	sp, r0
	sub	r0, r0, #SVC_STACK_SIZE
	msr	CPSR_c, #MODE_SYS|I_BIT|F_BIT /* System Mode */
	mov	sp, r0

	/* We want to start in supervisor mode.  Operation will switch to system
	mode when the first task starts. */
	msr   CPSR_c, #MODE_SVC|I_BIT|F_BIT

	/* Clear BSS. */

	mov     a2, #0		/* Fill value */
	mov	fp, a2		/* Null frame pointer */
	mov	r7, a2		/* Null frame pointer for Thumb */

	ldr	r1, .LC1	/* Start of memory block */
	ldr	r3, .LC2	/* End of memory block */
	subs	r3, r3, r1      /* Length of block */
	beq	.end_clear_loop
	mov	r2, #0

.clear_loop:
	strb	r2, [r1], #1
	subs	r3, r3, #1
	bgt	.clear_loop

.end_clear_loop:

	/* Initialise data. */

	ldr	r1, .LC3	/* Start of data section */
	ldr	r2, .LC4	/* End of text section (LMA address to put data) */
	cmp	r1, r2
	beq	.end_set_loop	/* This code run at RAM */
	ldr	r3, .LC5	/* End of data section */
	subs	r3, r3, r1	/* Length of block */
	beq	.end_set_loop

.set_loop:
	ldrb	r4, [r2], #1
	strb	r4, [r1], #1
	subs	r3, r3, #1
	bgt	.set_loop

.end_set_loop:

	mov	r0, #0          /* no arguments  */
	mov	r1, #0          /* no argv either */

	bl	main

endless_loop:
	b	endless_loop


	.align 0

	.LC1:
	.word   __bss_beg__
	.LC2:
	.word   __bss_end__
	.LC3:
	.word   __data_beg__
	.LC4:
	.word	__end_of_text__
	.LC5:
	.word   __data_end__
	.LC6:
	.word   __stack_end__

	/* Setup vector table.  Note that undf, pabt, dabt, fiq just execute
	a null loop. */

.section .startup,"ax"
         .code 32
         .align 0

	b	_start			/* reset - _start		*/
	ldr	pc, _undf		/* undefined - _undf		*/
	ldr	pc, _swi		/* SWI - _swi			*/
	ldr	pc, _pabt		/* program abort - _pabt	*/
	ldr	pc, _dabt		/* data abort - _dabt		*/
	nop				/* reserved			*/
	ldr	pc, _irq		/* IRQ - _irq			*/
	ldr	pc, _fiq		/* FIQ - _fiq			*/

_undf:  .word __undf                    /* undefined			*/
_swi:   .word vPortYieldProcessor       /* SWI				*/
_pabt:  .word __pabt                    /* program abort		*/
_dabt:  .word __dabt                    /* data abort			*/
_irq:   .word __irq                     /* IRQ				*/
_fiq:   .word __fiq                     /* FIQ				*/

__undf:	b	.			/* undefined			*/
__pabt:	b	.			/* program abort		*/
__dabt:	b	.			/* data abort			*/
__fiq:	b	.			/* FIQ				*/

	.extern VICVectISR
	.extern ulCriticalNesting
	.extern pxCurrentTCB

.LC7:	.word	VICVectISR

__irq:
	/* Push r0 as we are going to use the register. */
	stmdb	sp!, {r0}

	ldr	r0, =0x98800014
	ldr	r0, [r0]
	cmp	r0, #0
	beq	2f

	/* Set r0 to point to the task stack pointer. */
	stmdb	sp,{sp}^
	nop
	sub	sp, sp, #4
	ldmia	sp!,{r0}

	/* Push the return address onto the stack. */
	stmdb	r0!, {lr}

	/* Now we have saved lr we can use it instead of r0. */
	mov	lr, r0

	/* Pop r0 so we can save it onto the system mode stack. */
	ldmia	sp!, {r0}

	/* Push all the system mode registers onto the task stack. */
	stmdb	lr,{r0-lr}^
	nop
	sub	lr, lr, #60

	/* Push the SPSR onto the task stack. */
	mrs	r0, spsr
	stmdb	lr!, {r0}

	ldr	r0, =ulCriticalNesting
	ldr	r0, [r0]
	stmdb	lr!, {r0}

	/* Store the new top of stack for the task. */
	ldr	r0, =pxCurrentTCB
	ldr	r0, [r0]
	str	lr, [r0]

	ldr	r0, =0x98800014
	ldr	r1, [r0]
	cmp	r1, #0
	beq	2f

	movne	r0, #0
#if __ARM_ARCH__ >= 5
	/*clz	r2, r1 */
	/*rsb	r2, r2, #31 */
	/*add	r0, r0, r2 */
#else
	movs	r2, r1, lsl #16
	movne	r1, r2
	addeq	r0, r0, #16

	movs	r2, r1, lsl #8
	movne	r1, r2
	addeq	r0, r0, #8

	movs	r2, r1, lsl #4
	movne	r1, r2
	addeq	r0, r0, #4

	movs	r2, r1, lsl #2
	movne	r1, r2
	addeq	r0, r0, #2

	movs	r2, r1, lsl #1
	addeqs	r0, r0, #1
#endif
	ldr	r1, .LC7
	ldr	pc, [r1, r0, lsl #2]
2:
	ldmia	sp!,{r0}
	movs	pc, lr
