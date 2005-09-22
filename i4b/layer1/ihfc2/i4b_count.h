/* Counter file
 *
 * Including this file will setup a 10-bit, 1-unit-incremental
 * counter, which starts at zero. The current binary value can be
 * retrieved by referring to COUNT() in your program.
 */
#ifndef _I4B_COUNT_H_
#define _I4B_COUNT_H_
#define COUNT(x...) _COUNT(__9,__8,__7,__6,__5,__4,__3,__2,__1,__0,x)
#define _COUNT(x...) __COUNT(x)
#define __COUNT(a,b,c,d,e,f,g,h,i,j,k) k##_##a##b##c##d##e##f##g##h##i##j
#define __0 0
#define __1 0
#define __2 0
#define __3 0
#define __4 0
#define __5 0
#define __6 0
#define __7 0
#define __8 0
#define __9 0
#else
#if     __0 == 0
#undef  __0
#define __0 1
#else
#undef  __0
#define __0 0
#if     __1 == 0
#undef  __1
#define __1 1
#else
#undef  __1
#define __1 0
#if     __2 == 0
#undef  __2
#define __2 1
#else
#undef  __2
#define __2 0
#if     __3 == 0
#undef  __3
#define __3 1
#else
#undef  __3
#define __3 0
#if     __4 == 0
#undef  __4
#define __4 1
#else
#undef  __4
#define __4 0
#if     __5 == 0
#undef  __5
#define __5 1
#else
#undef  __5
#define __5 0
#if     __6 == 0
#undef  __6
#define __6 1
#else
#undef  __6
#define __6 0
#if     __7 == 0
#undef  __7
#define __7 1
#else
#undef  __7
#define __7 0
#if     __8 == 0
#undef  __8
#define __8 1
#else
#undef  __8
#define __8 0
#if     __9 == 0
#undef  __9
#define __9 1
#else
#undef  __9
#define __9 0
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif
#endif /* _I4B_COUNT_H_ */
