@ Integer division on the MM32SPIN05's hardware divider (HWDIV, 8 cycles) instead of libgcc's shift and
@ subtract routines for the Cortex-M0 (~40-100 cycles, 726 bytes). These definitions take the place of
@ libgcc's (the compiler calls __aeabi_idiv / __aeabi_uidiv for every '/' and '%').
@
@ The divider is one set of registers shared by the main loop and the adc isr: each division runs with
@ the interrupts masked (PRIMASK saved and restored, ~10 cycles), and sets the signed / unsigned mode
@ itself. Division by zero returns 0 like libgcc's (the divider gives -1 / +1). Results checked against
@ C semantics on the chip (truncation towards zero, remainder with the dividend's sign, INT_MIN / -1 =
@ INT_MIN). The clock is enabled first thing in Reset_Handler (RCC AHBENR bit 26).
@ Returns: r0 = quotient, r1 = remainder (__aeabi_idivmod / __aeabi_uidivmod).

	.syntax unified
	.cpu cortex-m0
	.thumb

	.equ HWDIV, 0x40030000        @ DVDR +0, DVSR +4 (the write starts it), QUOTR +8, RMDR +12, CR +20

	.section .text.hwdiv, "ax", %progbits
	.balign 4

	.global __aeabi_idiv, __aeabi_idivmod, __divsi3
	.type __aeabi_idiv, %function
	.type __aeabi_idivmod, %function
	.type __divsi3, %function
	.thumb_func
__aeabi_idiv:
	.thumb_func
__aeabi_idivmod:
	.thumb_func
__divsi3:
	movs r3, #0                   @ CR: signed
	b 1f

	.global __aeabi_uidiv, __aeabi_uidivmod, __udivsi3
	.type __aeabi_uidiv, %function
	.type __aeabi_uidivmod, %function
	.type __udivsi3, %function
	.thumb_func
__aeabi_uidiv:
	.thumb_func
__aeabi_uidivmod:
	.thumb_func
__udivsi3:
	movs r3, #1                   @ CR: unsigned
1:	cmp r1, #0
	beq 2f
	ldr r2, =HWDIV
	mrs r12, primask
	cpsid i
	str r3, [r2, #20]
	str r0, [r2, #0]
	str r1, [r2, #4]
	ldr r0, [r2, #8]              @ waits for the result
	ldr r1, [r2, #12]
	msr primask, r12
	bx lr
2:	movs r1, r0                   @ x / 0 = 0, remainder x
	movs r0, #0
	bx lr
	.size __aeabi_idiv, . - __aeabi_idiv
	.size __aeabi_idivmod, . - __aeabi_idivmod
	.size __divsi3, . - __divsi3
	.size __aeabi_uidiv, . - __aeabi_uidiv
	.size __aeabi_uidivmod, . - __aeabi_uidivmod
	.size __udivsi3, . - __udivsi3
	.ltorg
