#ifndef _MACRO_H_
#define _MACRO_H_

/* NOTE: if you are using an old compiler
 * you might have to parse the files through
 * ``cpp'' first; Macros should be passed two
 * times by the compiler ...
 */

/*
 * MACRO LOGIC SUPPORT
 */

#undef YES
#undef NO
#undef COMMA

#define COMMA(args...) ,
#define YES(args...) args
#define NO(args...)

/* (NOT(YES) == NO) and (NOT(NO) == YES) */
#define NOT(arg) _NOT(YES arg(() NO))
#define          _NOT(args...) args

#define PASS(args...) args

/* test macro for YES == 1 or NO == 0 */
#define EVAL(macro) (0 macro(+1))

#if 0
/*
 * LOG2()
 * NOTE: input limit == (1 << 32)
 * NOTE: result is rounded up
 */
#define LOG2(arg)				\
(REP(32,_LOG2,arg) /* out of range: */32)

#define _LOG2(arg,no)				\
(arg <= (1<<(no))) ? (no) :
#endif

/*
 * ITEM()
 * NOTE: input limit == 16 items
 */
#define ITEM(n,args...)				\
        _ITEM(n,args,REP(G,COMMA))		\
	 /* check if ``n'' is a number: */	\
		IF_NUMBER(n)(/* nothing */)

#define _ITEM(args...)				\
	__ITEM(args)

#define __ITEM(n, /* removing by insertion: */	\
	       _0,_1,_2,_3,_4,_5,_6,_7,		\
	       _8,_9,_A,_B,_C,_D,_E,_F,args...)	\
	___ITEM(REP(n,COMMA)			\
		_F,_E,_D,_C,_B,_A,_9,_8,	\
		_7,_6,_5,_4,_3,_2,_1,_0)

#define ___ITEM(args...)			\
	____ITEM(args)

#define ____ITEM(_0,_1,_2,_3,_4,_5,_6,_7,	\
		 _8,_9,_A,_B,_C,_D,_E,_F,args...) _F

#define ITEM0(args...) _ITEM0(args, dummy)
#define                _ITEM0(a,args...) a

#define ITEM1(args...) _ITEM1(args, dummy)
#define                _ITEM1(a,b,args...) b

#define ITEM2(args...) _ITEM2(args, dummy)
#define                _ITEM2(a,b,c,args...) c

#define ITEM3(args...) _ITEM3(args, dummy)
#define                _ITEM3(a,b,c,d,args...) d

#define ITEM4(args...) _ITEM4(args, dummy)
#define                _ITEM4(a,b,c,d,e,args...) e

#define ITEM5(args...) _ITEM5(args, dummy)
#define                _ITEM5(a,b,c,d,e,f,args...) f

#define ITEM6(args...) _ITEM6(args, dummy)
#define                _ITEM6(a,b,c,d,e,f,g,args...) g

#define ITEM7(args...) _ITEM7(args, dummy)
#define                _ITEM7(a,b,c,d,e,f,g,h,args...) h

/*---------------------------------------------------------------------------*
 * : macro for generation of unique labels
 *---------------------------------------------------------------------------*/
#define UNIQUE(x) _UNIQUE( x , COUNT /* temporary */ (__LINE__))
#define           _UNIQUE(args...) __UNIQUE(args)
#define                            __UNIQUE(a,b) a##_##b

/*---------------------------------------------------------------------------*
 * : macro for concatenation of names
 *---------------------------------------------------------------------------*/
#undef CONCAT
#define CONCAT(args...) _CONCAT(args)
#define _CONCAT(a,b) a##b

/*---------------------------------------------------------------------------*
 * : macro for stripping an argument
 *---------------------------------------------------------------------------*/
#define STRIP(arg) CONCAT(,CONCAT(arg,))

/*---------------------------------------------------------------------------*
 * : macro for converting an argument to a string
 *---------------------------------------------------------------------------*/
#define STRING(x...) _STRING(x)
#define              _STRING(args...) #args

/*---------------------------------------------------------------------------*
 * : macro for checking if the argument is a number
 *
 * NOTE: this macro should only return YES when the
 * number ``x'' is supported by REP() macro!
 *---------------------------------------------------------------------------*/
#define IF_NUMBER(x) _IF_NUMBER_##x

#define _IF_NUMBER_  NO  /* on all other non-numbers the
			  * macro should not expand
			  */
#define _IF_NUMBER_0 YES
#define _IF_NUMBER_1 YES
#define _IF_NUMBER_2 YES
#define _IF_NUMBER_3 YES
#define _IF_NUMBER_4 YES
#define _IF_NUMBER_5 YES
#define _IF_NUMBER_6 YES
#define _IF_NUMBER_7 YES
#define _IF_NUMBER_8 YES
#define _IF_NUMBER_9 YES
#define _IF_NUMBER_A YES
#define _IF_NUMBER_B YES
#define _IF_NUMBER_C YES
#define _IF_NUMBER_D YES
#define _IF_NUMBER_E YES
#define _IF_NUMBER_F YES
#define _IF_NUMBER_16  YES
#define _IF_NUMBER_32  YES
#define _IF_NUMBER_64  YES
#define _IF_NUMBER_128 YES
#define _IF_NUMBER_256 YES
#define _IF_NUMBER_0x10  YES
#define _IF_NUMBER_0x20  YES
#define _IF_NUMBER_0x40  YES
#define _IF_NUMBER_0x80  YES
#define _IF_NUMBER_0x100 YES

/*---------------------------------------------------------------------------*
 * : macro for repeating another macro ``n'' times
 *
 * TODO: have this macro in the compiler
 * NOTE: please update ``IF_NUMBER()'' macro when changing ``REP()'' macro
 *---------------------------------------------------------------------------*/
#define REP(args...) /* expand macros in arguments first */ \
        _REP(args)
#define _REP(count,macro,args...) _##count(macro,args)

    /* expands to 
     *
     * macro(args, 0) 
     * macro(args, 1) 
     * macro(args, 2) 
     *     .
     *	   .
     *	   .
     * macro(args, count-1) 
     */

#define _0(m,a...)
#define _1(m,a...) _0(m,a) m(a,0)
#define _2(m,a...) _1(m,a) m(a,1)
#define _3(m,a...) _2(m,a) m(a,2)
#define _4(m,a...) _3(m,a) m(a,3)
#define _5(m,a...) _4(m,a) m(a,4)
#define _6(m,a...) _5(m,a) m(a,5)
#define _7(m,a...) _6(m,a) m(a,6)
#define _8(m,a...) _7(m,a) m(a,7)
#define _9(m,a...) _8(m,a) m(a,8)
#define _A(m,a...) _9(m,a) m(a,9)
#define _B(m,a...) _A(m,a) m(a,A)
#define _C(m,a...) _B(m,a) m(a,B)
#define _D(m,a...) _C(m,a) m(a,C)
#define _E(m,a...) _D(m,a) m(a,D)
#define _F(m,a...) _E(m,a) m(a,E)
#define _G(m,a...) m(a##0)m(a##1)m(a##2)m(a##3)m(a##4)m(a##5)m(a##6)m(a##7) \
		   m(a##8)m(a##9)m(a##A)m(a##B)m(a##C)m(a##D)m(a##E)m(a##F)

#define   _0x0(a...)
#define  _0x10(a...) _G(a,0x0)
#define  _0x20(a...) _G(a,0x0)_G(a,0x1)
#define  _0x40(a...) _G(a,0x0)_G(a,0x1)_G(a,0x2)_G(a,0x3)
#define  _0x80(a...) _G(a,0x0)_G(a,0x1)_G(a,0x2)_G(a,0x3)_G(a,0x4)_G(a,0x5)_G(a,0x6)_G(a,0x7)
#define _0x100(a...) _G(a,0x0)_G(a,0x1)_G(a,0x2)_G(a,0x3)_G(a,0x4)_G(a,0x5)_G(a,0x6)_G(a,0x7) \
		     _G(a,0x8)_G(a,0x9)_G(a,0xA)_G(a,0xB)_G(a,0xC)_G(a,0xD)_G(a,0xE)_G(a,0xF)

#define  _16  _0x10
#define  _32  _0x20
#define  _64  _0x40
#define _128  _0x80
#define _256 _0x100

/*---------------------------------------------------------------------------*
 * : macro for duplicating an argument
 *---------------------------------------------------------------------------*/
#define DUP(x) x x

#endif /* _MACRO_H_ */
