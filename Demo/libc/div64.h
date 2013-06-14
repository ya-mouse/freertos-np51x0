#ifndef _ASM_GENERIC_DIV64_H
#define _ASM_GENERIC_DIV64_H
/*
 * Copyright (C) 2003 Bernardo Innocenti <bernie@develer.com>
 * Based on former asm-ppc/div64.h and asm-m68knommu/div64.h
 *
 * The semantics of do_div() are:
 *
 * portULONG32_t do_div(portULONGLONG *n, portULONG32_t base)
 * {
 *	portULONG32_t remainder = *n % base;
 *	*n = *n / base;
 *	return remainder;
 * }
 *
 * NOTE: macro parameter n is evaluated multiple times,
 *       beware of side effects!
 */

extern portULONG __div64_32(portULONGLONG *dividend, portULONG divisor);

/* The unnecessary pointer compare is there
 * to check for type safety (n must be 64bit)
 */
# define do_div(n,base) ({				\
	portULONG __base = (base);			\
	portULONG __rem;					\
	(void)(((typeof((n)) *)0) == ((portULONGLONG *)0));	\
	if (((n) >> 32) == 0) {			\
		__rem = (portULONG)(n) % __base;		\
		(n) = (portULONG)(n) / __base;		\
	} else						\
		__rem = __div64_32(&(n), __base);	\
	__rem;						\
 })

/* Wrapper for do_div(). Doesn't modify dividend and returns
 * the result, not reminder.
 */
static inline portULONGLONG lldiv(portULONGLONG dividend, portULONG divisor)
{
	portULONGLONG __res = dividend;
	do_div(__res, divisor);
	return(__res);
}

#endif /* _ASM_GENERIC_DIV64_H */
