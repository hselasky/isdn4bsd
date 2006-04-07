/*-
 * Copyright (c) 2002-2005 Thomas Wintergerst. All rights reserved.
 *
 * Copyright (c) 2005 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 *	capi20.h - definition of CAPI messages
 *	--------------------------------------
 *
 * See "http://www.capi.org/" for a more detailed description
 * of the messages.
 *
 * This file uses alot of macros which can be expanded with
 * "cpp capi20.h | sed -e 'y/;/\n/' | more".
 *
 * All structures which name ends with *ENCODED 
 * use little endian byte order.
 *
 * All structures which name ends with *DECODED
 * use host byte order.
 *
 * $FreeBSD: $
 *
 */

#ifndef __CAPI20_H
#define __CAPI20_H

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/endian.h>

#ifndef __packed
#define __packed __attribute__((__packed__))
#endif

#ifdef __cplusplus
extern "C" {
#endif

#ifndef __UA_TYPES_H__
#define __UA_TYPES_H__

/* the following structures are 
 * used to force the compiler to
 * generate un-aligned memory 
 * access code on processors that
 * do not support un-aligned
 * memory accesses:
 */

struct void_p {
  void *data;
} __packed;

struct u_int16_p {
  u_int16_t data;
} __packed;

struct u_int32_p {
  u_int32_t data;
} __packed;

struct u_int64_p {
  u_int64_t data;
} __packed;

typedef struct void_p    void_p_t;
typedef struct u_int16_p u_int16_p_t;
typedef struct u_int32_p u_int32_p_t;
typedef struct u_int64_p u_int64_p_t;
#endif

/* global defines */

#define CAPI_STACK_VERSION     206
#define CAPI_APPLICATION_MAX   0x80 /* units */
#define CAPI_DEVICE_NAME       "/dev/capi20"
#define CAPI_SHLIB_BASENAME    "libcapi20.so"

/*---------------------------------------------------------------------------*
 *	macros
 *---------------------------------------------------------------------------*/
#ifndef NOT
#define NOT(arg) _NOT(YES arg(() NO))
#define          _NOT(args...) args
#endif

#ifndef YES
#define YES(args...) args
#define NO(args...)
#endif

#ifndef END
#define END(args...) args
#endif

#if ((1-YES(1)) || (1-NOT(NO)(1)) NO(||1))
#error "macros are not expanded correctly"
#endif

#ifndef ADD_BYTES
#define ADD_BYTES(ptr,size) ((void *)(((u_int8_t *)(ptr)) + (size)))
#endif

/* information element types 
 * for internal use:
 */
#define IE_END                  0
#define IE_BYTE                 1
#define IE_WORD                 2
#define IE_DWORD                3
#define IE_QWORD                4
#define IE_STRUCT               5 /* excludes length field */
#define IE_STRUCT_CAPI          6 /* includes length field */
#define IE_STRUCT_DECODED       7
#define IE_STRUCT_DECODED_EMPTY 8
#define IE_BYTE_ARRAY           9
#define IE_MAX                  10 /* exclusive */

enum { 
  CAPI_COMPOSE = IE_STRUCT_DECODED,
  CAPI_DEFAULT = IE_STRUCT_DECODED_EMPTY
};

struct capi_struct {
  u_int16_t len;
  u_int8_t * ppbyte[0]; /* casting hack */
  void * ptr;
} __packed;

#define IE_BYTE_COMPILE_M(b,w,d,q,s,a) b
#define IE_BYTE_COMPILE_1(name, def, decode)	\
  decode(u_int8_t name##_BYTE;)			\
  u_int8_t name;

#define IE_WORD_COMPILE_M(b,w,d,q,s,a) w
#define IE_WORD_COMPILE_1(name, def, decode)	\
  decode(u_int8_t name##_WORD;)			\
  u_int16_t name;

#define IE_DWORD_COMPILE_M(b,w,d,q,s,a) d
#define IE_DWORD_COMPILE_1(name, def, decode)	\
  decode(u_int8_t name##_DWORD;)		\
  u_int32_t name;

#define IE_QWORD_COMPILE_M(b,w,d,q,s,a) q
#define IE_QWORD_COMPILE_1(name, def, decode)	\
  decode(u_int8_t name##_QWORD;)		\
  u_int64_t name;

#define IE_STRUCT_COMPILE_M(b,w,d,q,s,a) s
#define IE_STRUCT_COMPILE_1(name, def, decode)	\
  decode(u_int8_t name##_STRUCT;		\
	 struct capi_struct name;)		\
  NOT(decode)(/* structure length is set to	\
	       * zero. Use the *DECODED		\
	       * structure if length is non-   	\
	       * zero.				\
	       */				\
	      u_int8_t name##_Null;)

#define IE_BYTE_ARRAY_COMPILE_M(b,w,d,q,s,a) a
#define IE_BYTE_ARRAY_COMPILE_1(name, def, decode)	\
  decode(u_int8_t name##_BYTE_ARRAY;			\
         u_int16_t name##_BYTE_ARRAY_LENGTH; )		\
   u_int8_t name [def];

#define CAPI_MAKE_DECODED_FIELD(n, type, field, def) \
  IE_##type##_COMPILE_1(field, def, YES)

#define CAPI_MAKE_ENCODED_FIELD(n, type, field, def) \
  IE_##type##_COMPILE_1(field, def, NO)

#define CAPI_MAKE_STRUCT(name)			\
  struct name##_DECODED {			\
    name(CAPI_MAKE_DECODED_FIELD,)()		\
    u_int8_t name##_end;			\
  } __packed;					\
  struct name##_ENCODED {			\
    name(CAPI_MAKE_ENCODED_FIELD,)()		\
  } __packed;

#define CAPI_MAKE_DEF_1(n,ENUM,value)		\
  enum { CAPI_##ENUM = (value) };		\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_REQ)		\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_CONF)		\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_IND)		\
  CAPI_MAKE_STRUCT(CAPI_##ENUM##_RESP)

#define CAPI_MAKE_DEF_2(n,ENUM,value)		\
  CAPI_##ENUM##_INDEX,

#define CAPI_MAKE_DEF_3(n, ENUM) \
  CAPI_MAKE_STRUCT(CAPI_##ENUM)

#define CAPI_MAKE_UNION_1(n,ENUM,value)		\
  struct CAPI_##ENUM##_REQ_##n ENUM##_REQ;	\
  struct CAPI_##ENUM##_CONF_##n ENUM##_CONF;	\
  struct CAPI_##ENUM##_IND_##n ENUM##_IND;	\
  struct CAPI_##ENUM##_RESP_##n ENUM##_RESP;

#define CAPI_MAKE_UNION_2(n, ENUM) \
  struct CAPI_##ENUM##_##n ENUM;

#define CAPI_DEBUG_OFFSET_1(field) \
  (((u_int8_t *)&(((struct capi_message_decoded *)0)->field)) - ((u_int8_t *)0))

/* this macro is a copy of the
 * next macro and is used to
 * allow macro recursion
 */
#define CAPI_MAKE_DEBUG_3(n, type, field, def)				\
  IE_##type##_COMPILE_M							\
  (									\
     YES({ IE_BYTE, 1, CAPI_DEBUG_OFFSET_1(n.field), #field },)		\
   ,									\
     YES({ IE_WORD, 2, CAPI_DEBUG_OFFSET_1(n.field), #field },)		\
   ,									\
     YES({ IE_DWORD, 4, CAPI_DEBUG_OFFSET_1(n.field), #field },)	\
   ,									\
     YES({ IE_QWORD, 8, CAPI_DEBUG_OFFSET_1(n.field), #field },)	\
   ,									\
     CAPI_##def(NO,)							\
     (									\
        CAPI_##def(CAPI_MAKE_DEBUG_3, def)()				\
     )									\
     NOT(CAPI_##def(NO,))						\
     (									\
       { IE_STRUCT, sizeof(void *),					\
	   CAPI_DEBUG_OFFSET_1(n.field.ptr), #field ".ptr" },		\
     )									\
   ,									\
      YES({ IE_BYTE_ARRAY, sizeof(u_int8_t [def]),			\
	      CAPI_DEBUG_OFFSET_1(n.field), #field "[" #def "]"},)	\
  )

/* this macro is used to make the debug
 * table in "libcapi.c"
 */
#define CAPI_MAKE_DEBUG_2(n, type, field, def)				\
  IE_##type##_COMPILE_M							\
  (									\
     YES({ IE_BYTE, 1, CAPI_DEBUG_OFFSET_1(n.field), #field },)		\
   ,									\
     YES({ IE_WORD, 2, CAPI_DEBUG_OFFSET_1(n.field), #field },)		\
   ,									\
     YES({ IE_DWORD, 4, CAPI_DEBUG_OFFSET_1(n.field), #field },)	\
   ,									\
     YES({ IE_QWORD, 8, CAPI_DEBUG_OFFSET_1(n.field), #field },)	\
   ,									\
     CAPI_##def(NO,)							\
     (									\
        CAPI_##def(CAPI_MAKE_DEBUG_3, def)()				\
     )									\
     NOT(CAPI_##def(NO,))						\
     (									\
       { IE_STRUCT, sizeof(void *),					\
	   CAPI_DEBUG_OFFSET_1(n.field.ptr), #field ".ptr" },		\
     )									\
   ,									\
      YES({ IE_BYTE_ARRAY, sizeof(u_int8_t [def]),			\
	      CAPI_DEBUG_OFFSET_1(n.field), #field "[" #def "]"},)	\
  )

#define CAPI_MAKE_DEBUG_1(n, ENUM, value)				\
  { IE_END, CAPI_P_REQ(ENUM), CAPI_REQ(ENUM), "CAPI_" #ENUM "_REQ" },	\
  CAPI_##ENUM##_REQ (CAPI_MAKE_DEBUG_2, data.ENUM##_REQ )()		\
  { IE_END, CAPI_P_CONF(ENUM), CAPI_CONF(ENUM), "CAPI_" #ENUM "_CONF" }, \
  CAPI_##ENUM##_CONF(CAPI_MAKE_DEBUG_2, data.ENUM##_CONF)()		\
  { IE_END, CAPI_P_IND(ENUM), CAPI_IND(ENUM), "CAPI_" #ENUM "_IND" },	\
  CAPI_##ENUM##_IND (CAPI_MAKE_DEBUG_2, data.ENUM##_IND )()		\
  { IE_END, CAPI_P_RESP(ENUM), CAPI_RESP(ENUM), "CAPI_" #ENUM "_RESP" }, \
  CAPI_##ENUM##_RESP(CAPI_MAKE_DEBUG_2, data.ENUM##_RESP)()

#define CAPI_MAKE_CASES(n, ENUM, value)				\
  case CAPI_P_REQ(ENUM):					\
    CAPI_INIT_2(CAPI_##ENUM##_REQ,&((n)->data.ENUM##_REQ))	\
    break;							\
  case CAPI_P_CONF(ENUM):					\
    CAPI_INIT_2(CAPI_##ENUM##_CONF,&((n)->data.ENUM##_CONF))	\
    break;							\
  case CAPI_P_IND(ENUM):					\
    CAPI_INIT_2(CAPI_##ENUM##_IND,&((n)->data.ENUM##_IND))	\
    break;							\
  case CAPI_P_RESP(ENUM):					\
    CAPI_INIT_2(CAPI_##ENUM##_RESP,&((n)->data.ENUM##_RESP))	\
    break;

#define CAPI_(m,n) NO /* invalid structure */

#define CAPI_MAKE_INIT_1(n, type, field, def)			\
  (n)->field##_##type = IE_##type;				\
  IE_##type##_COMPILE_M						\
  (,,,,,(n)->field##_##type##_LENGTH = sizeof(u_int8_t [def]);)

#if (IE_STRUCT_DECODED_EMPTY == 0)
#error "IE_STRUCT_DECODED_EMPTY cannot be zero"
#endif

#define CAPI_MAKE_INIT_2(n, type, field, def)			\
  IE_##type##_COMPILE_M						\
  (								\
     /* BYTE */							\
     (n)->field##_##type = IE_##type;				\
   ,								\
     /* WORD */							\
     (n)->field##_##type = IE_##type;				\
   ,								\
     /* DWORD */						\
     (n)->field##_##type = IE_##type;				\
   ,								\
     /* QWORD */						\
     (n)->field##_##type = IE_##type;				\
   ,								\
     /* STRUCT */						\
     CAPI_##def(NO,)						\
     (								\
      /* allow the application to set type */			\
      if((n)->field##_##type != IE_STRUCT_DECODED_EMPTY)	\
	{ (n)->field##_##type = IE_STRUCT_DECODED; }		\
								\
	/* use hints set by the environment to			\
	 * setup the *DECODED pointer and structure		\
	 */							\
        (n)->field.ptr = &def##_STRUCT;				\
	def##_INIT = 1;						\
     )								\
     NOT(CAPI_##def(NO,))					\
     (								\
      /* standard CAPI structure pointer */			\
      (n)->field##_##type = IE_##type##_CAPI;			\
     )								\
   ,								\
     /* BYTE ARRAY */						\
     (n)->field##_##type = IE_##type;				\
     (n)->field##_##type##_LENGTH = sizeof(u_int8_t [def]);	\
   )

/* this macro is used to initialize
 * the *DECODED structures before
 * passed to "capi_encode()" or
 * "capi_decode()"
 */
#define CAPI_INIT(what, ptr)			\
  { what(CAPI_MAKE_INIT_1,ptr)();		\
    (ptr)->what##_end = IE_END; }		\
/**/

/* internal use macro */
#define CAPI_INIT_2(what, ptr)			\
  { what(CAPI_MAKE_INIT_2,ptr)();		\
    (ptr)->what##_end = IE_END; }		\
/**/

/*---------------------------------------------------------------------------*
 *	definition of CAPI commands
 *---------------------------------------------------------------------------*/

/* CAPI sub commands */
enum {
  CAPI_REQ    = 0x80,
  CAPI_CONF   = 0x81,
  CAPI_IND    = 0x82,
  CAPI_RESP   = 0x83,
  CAPI_PACKED = 0x84  /* non-standard */
};

/* CAPI commands
 *
 * (the enums are generated later
 *  and will look like CAPI_XXX = value)
 */
#if 1
#define CAPI_COMMANDS(m,n) \
/*m(n, enum                         , value )*	\
 *m(n,------------------------------,-------)*/	\
  m(n, DATA_B3                      , 0x0086)	\
  m(n, CONNECT                      , 0x0002)	\
  m(n, CONNECT_ACTIVE               , 0x0003)	\
  m(n, CONNECT_B3                   , 0x0082)	\
  m(n, CONNECT_B3_ACTIVE            , 0x0083)	\
  m(n, CONNECT_B3_T90_ACTIVE        , 0x0088)	\
  m(n, DISCONNECT                   , 0x0004)	\
  m(n, DISCONNECT_B3                , 0x0084)	\
  m(n, ALERT                        , 0x0001)	\
  m(n, INFO                         , 0x0008)	\
  m(n, SELECT_B_PROTOCOL            , 0x0041)	\
  m(n, FACILITY                     , 0x0080)	\
  m(n, RESET_B3                     , 0x0087)	\
  m(n, MANUFACTURER                 , 0x00FF)	\
  m(n, LISTEN                       , 0x0005)	\
/**/
#else
#define CAPI_COMMANDS(m,n) \
/*m(n, enum                         , value )*	\
 *m(n,------------------------------,-------)*/	\
  m(n, ALERT                        , 0x0001)	\
  m(n, CONNECT                      , 0x0002)	\
  m(n, CONNECT_ACTIVE               , 0x0003)	\
  m(n, DISCONNECT                   , 0x0004)	\
  m(n, LISTEN                       , 0x0005)	\
  m(n, INFO                         , 0x0008)	\
  m(n, SELECT_B_PROTOCOL            , 0x0041)	\
  m(n, FACILITY                     , 0x0080)	\
  m(n, CONNECT_B3                   , 0x0082)	\
  m(n, CONNECT_B3_ACTIVE            , 0x0083)	\
  m(n, DISCONNECT_B3                , 0x0084)	\
  m(n, DATA_B3                      , 0x0086)	\
  m(n, RESET_B3                     , 0x0087)	\
  m(n, CONNECT_B3_T90_ACTIVE        , 0x0088)	\
  m(n, MANUFACTURER                 , 0x00FF)	\
/**/
#warning "CAPI commands sorted by value"
#endif

/*
 * Format of a standard CAPI command:
 *
 * MSB                                     LSB
 * +--------------------+--------------------+
 * | sub-command 8-bits | command 8-bits     |
 * +--------------------+--------------------+
 * | CAPI_REQ, CAPI_IND | e.g. CAPI_DATA_B3  |
 * | CAPI_CONF,         |                    |
 * | CAPI_RESP          |                    |
 * +--------------------+--------------------+
 *
 *
 * Format of a packed CAPI command:
 *
 * MSB                                                         LSB
 * +---------------+------------+---------+----------------------+
 * | sub-command   | originator | command | question or answer   |
 * | 8-bit         | 1-bit      | 6-bit   | 1-bit                |
 * +---------------+------------+---------+----------------------+
 * | CAPI_PACKED   | appl.:  0  |         | question: 0 (IND/REQ)|
 * |               | kernel: 1  |         | answer: 1 (CONF/RESP)|
 * +---------------+------------+---------+----------------------+
 *
 */

/* CAPI command generators 
 *
 * example:
 *
 * if(cmd == CAPI_REQ(DATA_B3))
 * ...
 */
/* standard version */
#if !defined(_KERNEL)
#define CAPI_REQ(cmd)     ((CAPI_##cmd)|(CAPI_REQ << 8))
#define CAPI_RESP(cmd)    ((CAPI_##cmd)|(CAPI_RESP << 8))
#endif
#define CAPI_CONF(cmd)    ((CAPI_##cmd)|(CAPI_CONF << 8))
#define CAPI_IND(cmd)     ((CAPI_##cmd)|(CAPI_IND << 8))

/* packed version */
#define CAPI_P_REQ(cmd)   ((CAPI_##cmd##_INDEX << 1)^(CAPI_PACKED << 8)^0x7e)
#define CAPI_P_CONF(cmd)  ((CAPI_##cmd##_INDEX << 1)^(CAPI_PACKED << 8)^0x81)
#define CAPI_P_IND(cmd)   ((CAPI_##cmd##_INDEX << 1)^(CAPI_PACKED << 8)^0x80)
#define CAPI_P_RESP(cmd)  ((CAPI_##cmd##_INDEX << 1)^(CAPI_PACKED << 8)^0x7f)

#define CAPI_P_MIN (((CAPI_COMMAND_INDEX_MAX-1) << 1)^(CAPI_PACKED << 8)^0x7e) /* inclusive */
#define CAPI_P_MAX (    (CAPI_COMMAND_INDEX_MAX << 1)^(CAPI_PACKED << 8)^0x80) /* exclusive */

#define __CAPI_COMMAND_PACK(n, ENUM, value)	\
  ((n) == (value)) ? ((CAPI_##ENUM##_INDEX) << 1) :

/* this macro takes any command and outputs a
 * packed CAPI command (which is better suited
 * for switching)
 */
#define CAPI_COMMAND_PACK(cmd)					\
  ((((cmd) >> 8) == CAPI_PACKED) ? (cmd) :			\
   ((((cmd) >> 8) == CAPI_REQ)  ? ((CAPI_PACKED << 8)^0x7e) :	\
    (((cmd) >> 8) == CAPI_CONF) ? ((CAPI_PACKED << 8)^0x81) :	\
    (((cmd) >> 8) == CAPI_IND)  ? ((CAPI_PACKED << 8)^0x80) :	\
    (((cmd) >> 8) == CAPI_RESP) ? ((CAPI_PACKED << 8)^0x7f) :	\
    0) ^							\
   (CAPI_COMMANDS(__CAPI_COMMAND_PACK, (cmd) & 0xFF) 0x7e))

/*---------------------------------------------------------------------------*
 *	definition of CAPI messages and substructures
 *
 * All CAPI messages and substructures are declared like macros.  One
 * reason for this, is to make future updating easier. When there is a
 * new element added to a message, one simply adds a line to a macro,
 * and then all translation tables and structures are updated at the
 * next compile. Another reason is that the CAPI messages contain
 * dynamic elements. For example the "STRUCT" element can have a size
 * from 0 to 65535 bytes inclusivly. The problem is that the offset to
 * the elements from the start of the message, then becomes a
 * variable, and then classical "struct { } name;" definitions cannot
 * be used.
 *
 * The structure definition macros should use the following format:
 *
 * #define CUSTOM(m,n) \
 *  m(n, type, field, def)\
 *  m(n, type, field, def)\
 *  END
 *
 * Description of the parameters passed to the "m" macro:
 *
 * ---------+--------------------------------------------------
 * n        | second argument passed to parent macro
 * ---------+--------------------------------------------------
 * type     | BYTE, WORD, DWORD, QWORD, STRUCT or BYTE_ARRAY
 * ---------+--------------------------------------------------
 * field    | choose an appropriate name for this field
 * ---------+--------------------------------------------------
 * def      | If type is STRUCT, this field is prefixed by
 *          | "CAPI_" to get the structure definition macro.
 *          | The default is to leave this field empty. This 
 *          | field does not affect structure generation, but 
 *          | is only used when generating
 *          | the "struct capi_message_decoded" translator.
 *          | When this field is non-empty the translator
 *          | will decode the STRUCT using the definition, 
 *          | before proceeding with the next information
 *          | element. The pointer to the *DECODED structure,
 *          | where the STRUCT should be decoded, is passed
 *          | indirectly using variables which name are
 *          | "CAPI_"field"_INIT" and "CAPI_"field"_STRUCT".
 *          |
 *          | If type is BYTE_ARRAY, this field specifices
 *          | the array size: 0, 1, 2 ... N
 *          |
 *          | Else this field has no effect.
 * ---------+--------------------------------------------------
 *
 * Here is an example on how to use the "CUSTOM" macro:
 *
 * CAPI_MAKE_STRUCT(CUSTOM);
 *
 * struct CUSTOM_DECODED custom = { 0 };
 *
 * u_int16_t len;
 *
 * u_int8_t buffer[256];
 *
 * CAPI_INIT(CUSTOM, &custom);
 *
 * custom.xxx = xxx;
 *
 * len = capi_encode(&buffer, sizeof(buffer), &custom);
 *
 * ...
 *
 * capi_decode(&buffer, len, &custom);
 *
 *---------------------------------------------------------------------------*/

#define CAPI_HEADER(m,n) \
  m(n, WORD  , wLen,)\
  m(n, WORD  , wApp,)\
  m(n, WORD  , wCmd,)\
  m(n, WORD  , wNum,)\
  m(n, DWORD , dwCid,)\
  END

/* unused CID value */
#define CAPI_CID_UNUSED         0

/* masks for "dwCid" */
#define CAPI_CIDMASK_CTLR       0x0000007FUL
#define CAPI_CIDMASK_CTLR_TYPE  0x00000080UL
#define CAPI_CIDMASK_PLCI       0x0000FF00UL
#define CAPI_CIDMASK_NCCI       0xFFFF0000UL
#define CAPI_CIDSHIFT_PLCI      8
#define CAPI_CIDSHIFT_NCCI      16

/* extract the controller from a CID */
#define CAPI_GET_CTLR_FROM_CID(dwCid) \
   ((dwCid) & CAPI_CIDMASK_CTLR)

/* extract the PLCI from a CID */
#define CAPI_GET_PLCI_FROM_CID(dwCid) \
   (((dwCid) & CAPI_CIDMASK_PLCI) >> CAPI_CIDSHIFT_PLCI)

/* extract the NCCI from a CID */
#define CAPI_GET_NCCI_FROM_CID(dwCid) \
   (((dwCid) & CAPI_CIDMASK_NCCI) >> CAPI_CIDSHIFT_NCCI)

CAPI_MAKE_STRUCT(CAPI_HEADER);

/* ================================================ */

#define CAPI_ALERT_REQ(m,n) \
  m(n, STRUCT, add_info, ADDITIONAL_INFO)\
  END

#define CAPI_ALERT_CONF(m,n) \
  m(n, WORD, wInfo,)\
  END

#define CAPI_ALERT_IND(m,n) \
  END /* not used */

#define CAPI_ALERT_RESP(m,n) \
  END /* not used */

/* ================================================ */

#define CAPI_CONNECT_REQ(m,n) \
  m(n, WORD  , wCIP,)\
  m(n, STRUCT, dst_telno,)\
  m(n, STRUCT, src_telno,)\
  m(n, STRUCT, dst_subaddr,)\
  m(n, STRUCT, src_subaddr,)\
  m(n, STRUCT, b_protocol, B_PROTOCOL)\
  m(n, STRUCT, BC,)\
  m(n, STRUCT, LLC,)\
  m(n, STRUCT, HLC,)\
  m(n, STRUCT, add_info, ADDITIONAL_INFO)\
  m(n, BYTE,   bFlag_0,) /* BSD specific */\
  m(n, STRUCT, src_telno_2,) /* BSD specific */\
  END

#define CAPI_CONNECT_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_CONNECT_IND(m,n) \
  m(n, WORD  , wCIP,)\
  m(n, STRUCT, dst_telno,)\
  m(n, STRUCT, src_telno,)\
  m(n, STRUCT, dst_subaddr,)\
  m(n, STRUCT, src_subaddr,)\
  m(n, STRUCT, BC,)\
  m(n, STRUCT, LLC,)\
  m(n, STRUCT, HLC,)\
  m(n, STRUCT, add_info, ADDITIONAL_INFO)\
  m(n, STRUCT, src_telno_2,)\
  END

#define CAPI_CONNECT_RESP(m,n) \
  m(n, WORD  , wReject,)\
  m(n, STRUCT, b_protocol, B_PROTOCOL)\
  m(n, STRUCT, dst_telno,)\
  m(n, STRUCT, dst_subaddr,)\
  m(n, STRUCT, LLC,)\
  m(n, STRUCT, add_info, ADDITIONAL_INFO)\
  END

/* definition of "wReject": */
#define CAPI_CALL_ACCEPT                        0
#define CAPI_CALL_IGNORE                        1
#define CAPI_CALL_REJECT_NORMAL_CLEARING        2
#define CAPI_CALL_REJECT_USER_BUSY              3
#define CAPI_CALL_REJECT_NO_CHANNEL             4
#define CAPI_CALL_REJECT_FACILITY               5
#define CAPI_CALL_REJECT_CHANNEL_UNACCEPTABLE   6
#define CAPI_CALL_REJECT_INCOMPAT_DESTINATION   7
#define CAPI_CALL_REJECT_OUT_OF_ORDER           8

/* definition of "wCIP": */
#define CAPI_CIP_NO                             0
#define CAPI_CIP_SPEECH                         1 
#define CAPI_CIP_UNRESTRICTED_DATA              2 
#define CAPI_CIP_RESTRICTED_DATA                3 
#define CAPI_CIP_3100Hz_AUDIO                   4 
#define CAPI_CIP_7kHz_AUDIO                     5 
#define CAPI_CIP_VIDEO                          6 
#define CAPI_CIP_PACKET_MODE                    7 
#define CAPI_CIP_DATA_56                        8 
#define CAPI_CIP_UNRESTRICTED_DATA_TONES        9 
#define CAPI_CIP_TELEPHONY                      16
#define CAPI_CIP_FAX_G2_G3                      17
#define CAPI_CIP_FAX_G4_CLASS1                  18
#define CAPI_CIP_TELETEX_BASIC_MIXED            19
#define CAPI_CIP_FAX_G4_CLASS2_CLASS3           19
#define CAPI_CIP_TELETEX_BASIC_PROCESSABLE      20
#define CAPI_CIP_TELETEX_BASIC                  21
#define CAPI_CIP_VIDEOTEX                       22
#define CAPI_CIP_TELEX                          23
#define CAPI_CIP_X400                           24
#define CAPI_CIP_X200                           25
#define CAPI_CIP_7kHz_TELEPHONY                 26
#define CAPI_CIP_VIDEO_TELEPHONY_1ST            27
#define CAPI_CIP_VIDEO_TELEPHONY_2ND            28

/* definition of "bFlag_0": */
#define CAPI_FLAG0_WANT_LATE_INBAND   0x01

/* ================================================ */

#define CAPI_CONNECT_ACTIVE_IND(m,n) \
  m(n, STRUCT, connected_telno,)\
  m(n, STRUCT, connected_subaddr,)\
  m(n, STRUCT, LLC,)\
  END

#define CAPI_CONNECT_ACTIVE_RESP(m,n) \
  END /* empty */

#define CAPI_CONNECT_ACTIVE_REQ(m,n) \
  END /* not used */

#define CAPI_CONNECT_ACTIVE_CONF(m,n) \
  END /* not used */

/* ================================================ */

#define CAPI_CONNECT_B3_ACTIVE_IND(m,n) \
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_CONNECT_B3_ACTIVE_RESP(m,n) \
  END /* empty */

#define CAPI_CONNECT_B3_ACTIVE_REQ(m,n) \
  END /* not used */

#define CAPI_CONNECT_B3_ACTIVE_CONF(m,n) \
  END /* not used */

/* ================================================ */

#define CAPI_CONNECT_B3_REQ(m,n) \
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_CONNECT_B3_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_CONNECT_B3_IND(m,n) \
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_CONNECT_B3_RESP(m,n) \
  m(n, WORD  , wReject,)\
  m(n, STRUCT, NCPI,)\
  END

/* ================================================ */

#define CAPI_CONNECT_B3_T90_ACTIVE_IND(m,n) \
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_CONNECT_B3_T90_ACTIVE_RESP(m,n) \
  END /* empty */

#define CAPI_CONNECT_B3_T90_ACTIVE_REQ(m,n) \
  END /* not used */

#define CAPI_CONNECT_B3_T90_ACTIVE_CONF(m,n) \
  END /* not used */

/* ================================================ */

#define CAPI_DATA_B3_REQ(m,n) \
  m(n, DWORD , dwPtr_1,)\
  m(n, WORD  , wLen,)\
  m(n, WORD  , wHandle,)\
  m(n, WORD  , wFlags,)\
  m(n, QWORD , qwPtr_2,)\
  END

#define CAPI_DATA_B3_CONF(m,n) \
  m(n, WORD  , wHandle,)\
  m(n, WORD  , wInfo,)\
  END

#define CAPI_DATA_B3_IND(m,n) \
  m(n, DWORD , dwPtr_1,)\
  m(n, WORD  , wLen,)\
  m(n, WORD  , wHandle,)\
  m(n, WORD  , wFlags,)\
  m(n, QWORD , qwPtr_2,)\
  END

#define CAPI_DATA_B3_RESP(m,n) \
  m(n, WORD  , wHandle,)\
  END

/* definition of "wFlags": */
#define CAPI_DATA_B3_FLAG_QUALIFIER     0x0001
#define CAPI_DATA_B3_FLAG_MORE_DATA     0x0002
#define CAPI_DATA_B3_FLAG_DELIVERY_CONF 0x0004
#define CAPI_DATA_B3_FLAG_EXPEDITED     0x0008
#define CAPI_DATA_B3_FLAG_UI_FRAME      0x0010
#define CAPI_DATA_B3_FLAG_BREAK         0x0010
#define CAPI_DATA_B3_FLAG_FRAME_ERROR   0x8000

/* NOTES:
 * dwPtr_1 is coded 0 if CPU is 64-bit
 * qwPtr_2 is coded 0 if CPU is 32-bit
 * wHandle is copied from corresponding
 * data-b3-request or data-b3-indication.
 */

/* ================================================ */

#define CAPI_DISCONNECT_B3_REQ(m,n) \
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_DISCONNECT_B3_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_DISCONNECT_B3_IND(m,n) \
  m(n, WORD  , wReason,)\
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_DISCONNECT_B3_RESP(m,n) \
  END /* empty */

/* ================================================ */

#define CAPI_DISCONNECT_REQ(m,n) \
  m(n, STRUCT, add_info, ADDITIONAL_INFO)\
  END

#define CAPI_DISCONNECT_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_DISCONNECT_IND(m,n) \
  m(n, WORD  , wReason,)\
  END

#define CAPI_DISCONNECT_RESP(m,n) \
  END /* empty */

/* ================================================ */

#define CAPI_FACILITY_REQ(m,n) \
  m(n, WORD  , wSelector,)\
  m(n, STRUCT, Param,)\
  END

#define CAPI_FACILITY_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  m(n, WORD  , wSelector,)\
  m(n, STRUCT, Param,)\
  END

#define CAPI_FACILITY_IND(m,n) \
  m(n, WORD  , wSelector,)\
  m(n, STRUCT, Param,)\
  END

#define CAPI_FACILITY_RESP(m,n) \
  m(n, WORD  , wSelector,)\
  m(n, STRUCT, Param,)\
  END

/* definition of "wSelector": */
#define CAPI_FACILITY_SELECTOR_HANDSET                  0
#define CAPI_FACILITY_SELECTOR_DTMF                     1
#define CAPI_FACILITY_SELECTOR_V42BIS                   2
#define CAPI_FACILITY_SELECTOR_SUPPL_SERVICES           3
#define CAPI_FACILITY_SELECTOR_POWER_MNGMNT_WAKEUP      4
#define CAPI_FACILITY_SELECTOR_LINE_INTERCONNECT        5
#define CAPI_FACILITY_SELECTOR_BROADBAND_EXTENSIONS     6
#define CAPI_FACILITY_SELECTOR_CONTROLLER_EVENTS        7
#define CAPI_FACILITY_SELECTOR_ECHO_CANCELLATION        8

/* ================================================ */

#define CAPI_INFO_REQ(m,n) \
  m(n, STRUCT, dst_telno,)\
  m(n, STRUCT, add_info, ADDITIONAL_INFO)\
  END

#define CAPI_INFO_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_INFO_IND(m,n) \
  m(n, WORD  , wInfoNum,)\
  m(n, STRUCT, InfoElement,)\
  END

#define CAPI_INFO_RESP(m,n) \
  END /* empty */

/* ================================================ */

#define CAPI_LISTEN_REQ(m,n) \
  m(n, DWORD , dwInfoMask,)\
  m(n, DWORD , dwCipMask1,)\
  m(n, DWORD , dwCipMask2,)\
  m(n, STRUCT, src_telno,)\
  m(n, STRUCT, src_subaddr,)\
  END

/* definition of "dwInfoMask": */
#define CAPI_INFO_MASK_NONE                     0x00000000UL
#define CAPI_INFO_MASK_CAUSE                    0x00000001UL
#define CAPI_INFO_MASK_DATE_TIME                0x00000002UL
#define CAPI_INFO_MASK_DISPLAY                  0x00000004UL
#define CAPI_INFO_MASK_USER_USER                0x00000008UL
#define CAPI_INFO_MASK_CALL_PROGRESSION         0x00000010UL
#define CAPI_INFO_MASK_FACILITY                 0x00000020UL
#define CAPI_INFO_MASK_CHARGING                 0x00000040UL
#define CAPI_INFO_MASK_CALLED_PARTY_NUMBER      0x00000080UL
#define CAPI_INFO_MASK_CHANNEL_ID               0x00000100UL
#define CAPI_INFO_MASK_EARLY_B3_CONNECT         0x00000200UL
#define CAPI_INFO_MASK_REDIRECTION_INFO         0x00000400UL
#define CAPI_INFO_MASK_SENDING_COMPLETE         0x00001000UL
#define CAPI_INFO_MASK_ALL                      0x000017FFUL

/* definition of "dwCipMask1": */
#define CAPI_CIP_MASK1_UNLISTEN                 0UL
#define CAPI_CIP_MASK1_ANY                      1UL
#define CAPI_CIP_MASK1(CIP)                     (1 << ((CAPI_CIP_##CIP) % 32))

/* definition of "dwCipMask2": */
#define CAPI_CIP_MASK2_UNLISTEN                 0UL

#define CAPI_LISTEN_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_LISTEN_IND(m,n) \
  END /* not used */

#define CAPI_LISTEN_RESP(m,n) \
  END /* not used */

/* ================================================ */

#define CAPI_MANUFACTURER_REQ(m,n) \
  m(n, DWORD , dwManuID,)\
  m(n, DWORD , dwClass,)\
  m(n, DWORD , dwFunction,)\
  m(n, STRUCT, ManuData,)\
  END

#define CAPI_MANUFACTURER_CONF(m,n) \
  m(n, DWORD , dwManuID,)\
  m(n, DWORD , dwClass,)\
  m(n, DWORD , dwFunction,)\
  m(n, STRUCT, ManuData,)\
  END

#define CAPI_MANUFACTURER_IND(m,n) \
  m(n, DWORD , dwManuID,)\
  m(n, DWORD , dwClass,)\
  m(n, DWORD , dwFunction,)\
  m(n, STRUCT, ManuData,)\
  END

#define CAPI_MANUFACTURER_RESP(m,n) \
  m(n, DWORD , dwManuID,)\
  m(n, DWORD , dwClass,)\
  m(n, DWORD , dwFunction,)\
  m(n, STRUCT, ManuData,)\
  END

/* ================================================ */

#define CAPI_RESET_B3_REQ(m,n) \
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_RESET_B3_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_RESET_B3_IND(m,n) \
  m(n, STRUCT, NCPI,)\
  END

#define CAPI_RESET_B3_RESP(m,n) \
  END /* empty */

/* ================================================ */

#define CAPI_SELECT_B_PROTOCOL_REQ(m,n) \
  m(n, STRUCT, b_protocol, B_PROTOCOL)\
  END

#define CAPI_SELECT_B_PROTOCOL_CONF(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_SELECT_B_PROTOCOL_IND(m,n) \
  END /* not used */

#define CAPI_SELECT_B_PROTOCOL_RESP(m,n) \
  END /* not used */

/* ================================================ */

/*---------------------------------------------------------------------------*
 *	definition of CAPI sub-structures
 *---------------------------------------------------------------------------*/

#define CAPI_B_PROTOCOL(m,n) \
  m(n, WORD  , wB1_protocol,)\
  m(n, WORD  , wB2_protocol,)\
  m(n, WORD  , wB3_protocol,)\
  m(n, STRUCT, B1_config,)\
  m(n, STRUCT, B2_config,)\
  m(n, STRUCT, B3_config,)\
  m(n, STRUCT, global_config,)\
  END

/* definition of "wB1_protocol": */
#define CAPI_B1_HDLC_64                 0
#define CAPI_B1_TRANSPARENT_64          1
#define CAPI_B1_V110_ASYNCH             2
#define CAPI_B1_V110_SYNCH_HDLC         3
#define CAPI_B1_T30_MODEM               4
#define CAPI_B1_HDLC_64_INVERTED        5
#define CAPI_B1_TRANSPARENT_56          6
#define CAPI_B1_MODEM_FULL              7
#define CAPI_B1_MODEM_ASYNCH            8
#define CAPI_B1_MODEM_SYNCH_HDLC        9

/* definition of "wB2_protocol": */
#define CAPI_B2_ISO7776_X75_SLP         0
#define CAPI_B2_TRANSPARENT             1
#define CAPI_B2_SDLC                    2
#define CAPI_B2_LAPD_X25                3
#define CAPI_B2_T30_FAX_G3              4
#define CAPI_B2_PPP                     5
#define CAPI_B2_TRANSPARENT_NO_B1_ERR   6
#define CAPI_B2_MODEM_FULL              7
#define CAPI_B2_ISO7776_X75_SLP_COMP    8
#define CAPI_B2_V120_ASYNCH             9
#define CAPI_B2_V120_ASYNCH_COMP        10
#define CAPI_B2_V120_TRANSPARENT        11
#define CAPI_B2_LAPD_FREE_SAPI          12

/* definition of "wB3_protocol": */
#define CAPI_B3_TRANSPARENT             0
#define CAPI_B3_T90NL                   1
#define CAPI_B3_ISO8208                 2
#define CAPI_B3_X25_DCE                 3
#define CAPI_B3_T30_FAX_G3              4
#define CAPI_B3_T30_FAX_G3_EXT          5
#define CAPI_B3_RESERVED                6
#define CAPI_B3_MODEM                   7

/* ================================================ */

#define CAPI_ADDITIONAL_INFO(m,n) \
  m(n, STRUCT, b_channel_info,)\
  m(n, STRUCT, keypad,)\
  m(n, STRUCT, useruser,)\
  m(n, STRUCT, facility,)\
  m(n, STRUCT, sending_complete,)\
  END

/* ================================================ */

#define CAPI_SENDING_COMPLETE(m,n) \
  m(n, WORD  , wMode,)\
  END

/* definition of "wMode": */
#define CAPI_SENDING_COMPLETE_NO        0
#define CAPI_SENDING_COMPLETE_YES       1

/* ================================================ */

#define CAPI_FACILITY_REQ_DTMF_PARAM(m,n) \
  m(n, WORD  , wFunction,)\
  m(n, WORD  , wToneDuration,)\
  m(n, WORD  , wGapDuration,)\
  m(n, STRUCT, Digits,)\
  END

/* definition of "wFunction": */
#define CAPI_DTMF_START_LISTEN  1
#define CAPI_DTMF_STOP_LISTEN   2
#define CAPI_DTMF_SEND_DIGITS   3

#define CAPI_FACILITY_CONF_DTMF_PARAM(m,n) \
  m(n, WORD  , wInfo,)\
  END

#define CAPI_FACILITY_IND_DTMF_PARAM(m,n) \
  m(n, BYTE_ARRAY, Digits, 0)\
  END

#define CAPI_FACILITY_RESP_DTMF_PARAM(m,n) \
  END /* empty */

/* ================================================
 *
 * B1 configuration for B1 protocol "2" (defaults: 0, 8, 0, 0)
 * B1 configuration for B1 protocol "3" (defaults: 57600, 0, 0, 0)
 */
#define CAPI_B1_CONFIG_V100(m,n) \
  m(n, WORD  , wRate,)\
  m(n, WORD  , wBitsPerChar,)\
  m(n, WORD  , wParity,)\
  m(n, WORD  , wStopbits,)\
  END

/* definition of "wRate": (some selected rates) */
#define CAPI_B1CFG_BITRATE_MAXIMUM      0
#define CAPI_B1CFG_BITRATE_ADAPTIVE     0
#define CAPI_B1CFG_BITRATE_2400         2400
#define CAPI_B1CFG_BITRATE_4800         4800
#define CAPI_B1CFG_BITRATE_7200         7200
#define CAPI_B1CFG_BITRATE_9600         9600
#define CAPI_B1CFG_BITRATE_12000        12000
#define CAPI_B1CFG_BITRATE_14400        14400
#define CAPI_B1CFG_BITRATE_28800        28800
#define CAPI_B1CFG_BITRATE_33600        33600
#define CAPI_B1CFG_BITRATE_57600        57600

/* definition of "wParity": */
#define CAPI_B1CFG_PARITY_NONE          0
#define CAPI_B1CFG_PARITY_ODD           1
#define CAPI_B1CFG_PARITY_EVEN          2

/* definition of "wStopbits": */
#define CAPI_B1CFG_STOPBIT_ONE          0
#define CAPI_B1CFG_STOPBIT_TWO          1

/* ================================================
 *
 * B1 configuration for B1 protocol "4" (defaults: 0, 0, 0, 0) 
 */
#define CAPI_B1_CONFIG_FAX_G3(m,n) \
  m(n, WORD  , wRate,)\
  m(n, WORD  , wTransmitLevel,)\
  m(n, WORD  , wReserved1,)\
  m(n, WORD  , wReserved2,)\
  END

/* definition of "wRate": (see previous) */

/* definition of "wTransmitLevel": */
#define CAPI_B1CFG_FG3_TX_LEVEL_LOUD    0
#define CAPI_B1CFG_FG3_TX_LEVEL_MED     7
#define CAPI_B1CFG_FG3_TX_LEVEL_SILENT  15

/* ================================================
 *
 * B1 configuration for B1 protocol "7" (defaults: 0, 8, 0, 0, 0x50, 3):
 * B1 configuration for B1 protocol "8" (defaults: 0, 8, 0, 0, 0x50, 3)
 * B1 configuration for B1 protocol "9" (defaults: 0, 0, 0, 0, 0x50, 3)
 */
#define CAPI_B1_CONFIG_MODEM(m,n) \
  m(n, WORD  , wRate,)\
  m(n, WORD  , wBitsPerChar,)\
  m(n, WORD  , wParity,)\
  m(n, WORD  , wStopbits,)\
  m(n, WORD  , wOptions,)\
  m(n, WORD  , wSpeedNegotiation,)\
  END

/* definition of "wRate": (see previous) */

/* definition of "wParity": (see previous) */

/* definition of "wStopbits": (see previous) */

/* definition of "wOptions": */
#define CAPI_B1CFG_OPT_DIS_RETRAIN      0x0001
#define CAPI_B1CFG_OPT_DIS_RINGTONE     0x0002
#define CAPI_B1CFG_OPT_GUARDTONE_MASK   0x000C
#define CAPI_B1CFG_OPT_GUARDTONE_NONE   0x0000
#define CAPI_B1CFG_OPT_GUARDTONE_1800   0x0004
#define CAPI_B1CFG_OPT_GUARDTONE_550    0x0008
#define CAPI_B1CFG_OPT_LOUDSPKR_MASK    0x0030
#define CAPI_B1CFG_OPT_LOUDSPKR_OFF     0x0000
#define CAPI_B1CFG_OPT_LOUDSPKR_TRAINON 0x0010
#define CAPI_B1CFG_OPT_LOUDSPKR_ALLON   0x0020
#define CAPI_B1CFG_OPT_VOLUME_MASK      0x00C0
#define CAPI_B1CFG_OPT_VOLUME_SILENT    0x0000
#define CAPI_B1CFG_OPT_VOLUME_LOW       0x0040
#define CAPI_B1CFG_OPT_VOLUME_HIGH      0x0080
#define CAPI_B1CFG_OPT_VOLUME_MAX       0x00C0

/* definition of "wSpeedNegotiation": */
#define CAPI_B1CFG_SPEED_NEG_NONE       0
#define CAPI_B1CFG_SPEED_NEG_MODCLASS   1
#define CAPI_B1CFG_SPEED_NEG_V100       2
#define CAPI_B1CFG_SPEED_NEG_V8         3

/* ================================================
 *
 * B2 configuration for B2 protocol "0" (defaults: 3, 1, 8, 7, empty) 
 */
#define CAPI_B2_CONFIG_X75_SLP(m,n) \
  m(n, BYTE  , bAddressA,)\
  m(n, BYTE  , bAddressB,)\
  m(n, BYTE  , bModuloMode,)\
  m(n, BYTE  , bWindowSize,)\
  m(n, STRUCT, XID,)\
  END

/* definition of "bModuloMode": */
#define CAPI_B2CFG_MODULO_NORMAL        8
#define CAPI_B2CFG_MODULO_EXTENDED      128

/* ================================================
 *
 * B2 configuration for B2 protocol "2" (defaults: 0xC1, 0, 8, 7, XXX) 
 */
#define CAPI_B2_CONFIG_SDLC(m,n) \
  m(n, BYTE  , bAddress,)\
  m(n, BYTE  , bReserved,)\
  m(n, BYTE  , bModuloMode,)\
  m(n, BYTE  , bWindowSize,)\
  m(n, STRUCT, XID,)\
  END

/* definition of "bModuloMode": (see previous) */

/* ================================================
 *
 * B2 configuration for B2 protocol "3" (defaults: 1, 0, 128, 3, empty) 
 */
#define CAPI_B2_CONFIG_LAPD_X25(m,n) \
  m(n, BYTE  , bAddressA,)\
  m(n, BYTE  , bReserved,)\
  m(n, BYTE  , bModuloMode,)\
  m(n, BYTE  , bWindowSize,)\
  m(n, STRUCT, XID,)\
  END

/* definition of "bModuloMode": (see previous) */

/* ================================================
 *
 * B2 configuration for B2 protocol "7" (defaults: 0) 
 */
#define CAPI_B2_CONFIG_MODEM_FULL(m,n) \
  m(n, WORD  , wOptions,)\
  END

/* definition of "wOptions": */
#define CAPI_B2CFG_MODEMOPT_DIS_V42         0x0001
#define CAPI_B2CFG_MODEMOPT_DIS_MNP         0x0002
#define CAPI_B2CFG_MODEMOPT_DIS_TRANSPARENT 0x0004
#define CAPI_B2CFG_MODEMOPT_DIS_V42NEG      0x0008
#define CAPI_B2CFG_MODEMOPT_DIS_COMP        0x0010

/* ================================================
 *
 * B2 configuration for B2 protocol "9": (0, 1, 128, 7, empty)
 * B2 configuration for B2 protocol "11": (0, 1, 128, 7, empty)
 */
#define CAPI_B2_CONFIG_V120(m,n) \
  m(n, BYTE  , bAddressA,)\
  m(n, BYTE  , bAddressB,)\
  m(n, BYTE  , bModuloMode,)\
  m(n, BYTE  , bWindowSize,)\
  m(n, STRUCT, XID,)\
  END

/* definition of "bModuloMode": (see previous) */

/* ================================================
 *
 * B2 configuration for B2 protocol "8": (defaults: 3, 1, 8, 7, 0, 2048, 250)
 * B2 configuratino for B2 protocol "10": (defaults: 0, 1, 128, 7, 0, 2048, 250)
 */
#define CAPI_B2_CONFIG_V42BIS(m,n) \
  m(n, BYTE  , bAddressA,)\
  m(n, BYTE  , bAddressB,)\
  m(n, BYTE  , bModuloMode,)\
  m(n, BYTE  , bWindowSize,)\
  m(n, WORD  , wDirection,)\
  m(n, WORD  , wNumCodeWords,)\
  m(n, WORD  , wMaxStringLength,)\
  END

/* definition of "wDirection": */
#define CAPI_B2CFG_V42BIS_DIR_ALL       0
#define CAPI_B2CFG_V42BIS_DIR_INCOMING  1
#define CAPI_B2CFG_V42BIS_DIR_OUTGOING  2

/* definition of "wMaxStringLength": 6..250 (inclusivly) */

/* ================================================
 *
 * B3 configuration for B3 protocol "1" (defaults: 0,0,1,1,0,0,8,2)
 * B3 configuration for B3 protocol "2" (defaults: 0,0,1,1,0,0,8,2)
 * B3 configuration for B3 protocol "3" (defaults: 0,0,1,1,0,0,8,128)
 */
#define CAPI_B3_CONFIG_T90NL(m,n) \
  m(n, WORD  , wLIC,)\
  m(n, WORD  , wHIC,)\
  m(n, WORD  , wLTC,)\
  m(n, WORD  , wHTC,)\
  m(n, WORD  , wLOC,)\
  m(n, WORD  , wHOC,)\
  m(n, WORD  , wModuloMode,)\
  m(n, WORD  , wWindowSize,)\
  END

/* ================================================
 *
 * B3 configuration for B3 protocol "4" (defaults: 0, 0, emtpy, empty) 
 */
#define CAPI_B3_CONFIG_FAX_G3(m,n) \
  m(n, WORD  , wResolution,)\
  m(n, WORD  , wFormat,)\
  m(n, STRUCT, CallingStationID,)\
  m(n, STRUCT, Headline,)\
  END

/* definition of "wResolution": */
#define CAPI_FAXG3_RESOLUTION_STANDARD  0
#define CAPI_FAXG3_RESOLUTION_FINE      1

/* definition of "wFormat": */
#define CAPI_FAXG3_FORMAT_SFF           0
#define CAPI_FAXG3_FORMAT_PLAIN_FAX     1
#define CAPI_FAXG3_FORMAT_PCX           2
#define CAPI_FAXG3_FORMAT_DCX           3
#define CAPI_FAXG3_FORMAT_TIFF          4
#define CAPI_FAXG3_FORMAT_ASCII         5
#define CAPI_FAXG3_FORMAT_EXT_ANSI      6
#define CAPI_FAXG3_FORMAT_BINARY_FILE   7

/* ================================================
 *
 * B3 configuration for B3 protocol "5" (defaults: 0,0,empty,empty) 
 */
#define CAPI_B3_CONFIG_FAX_G3_EXT(m,n) \
  m(n, WORD  , wOptions,)\
  m(n, WORD  , wFormat,)\
  m(n, STRUCT, CallingStationID,)\
  m(n, STRUCT, Headline,)\
  END

/* definition of "wOptions": */
#define CAPI_FAXG3OPT_ENABLE_HIGHRES    0x0001
#define CAPI_FAXG3OPT_FAX_POLL          0x0002
#define CAPI_FAXG3OPT_MORE_PAGES        0x0004
#define CAPI_FAXG3OPT_NO_ECM            0x8000

/* definition of "wFormat": (see previous) */

/* ================================================ */

#define CAPI_B_PROTOCOL_GLOBAL_OPTIONS(m,n) \
  m(n, WORD  , wBChannelOperation,)\
  END

/* definition of "wBChannelOperation": */
#define CAPI_BPROT_GLOBOPT_BCHNOP_DEFAULT       0
#define CAPI_BPROT_GLOBOPT_BCHNOP_DTE           1
#define CAPI_BPROT_GLOBOPT_BCHNOP_DCE           2

/* ================================================ 
 *
 * NCPI for B3 protocol "2":
 * NCPI for B3 protocol "3":
 */
#define CAPI_NCPI_STD(m,n) \
  m(n, BYTE  , b0,)\
  m(n, BYTE  , b1,)\
  m(n, BYTE  , b2,)\
  m(n, BYTE_ARRAY, bx, 0)\
  END

/* ================================================ 
 *
 * NCPI for B3 protocol "4":
 */
#define CAPI_NCPI_FAX_G3(m,n) \
  m(n, WORD  , wSpeed,)\
  m(n, WORD  , wResolution,)\
  m(n, WORD  , wFormat,)\
  m(n, WORD  , wPages,)\
  m(n, STRUCT, receive_id,)\
  END

/* ================================================ 
 *
 * NCPI for B3 protocol "5":
 */
#define CAPI_NCPI_FAX_G3_EXT(m,n) \
  m(n, WORD  , wSpeed,)\
  m(n, WORD  , wOptions,)\
  m(n, WORD  , wFormat,)\
  m(n, WORD  , wPages,)\
  m(n, STRUCT, receive_id,)\
  END

/* ================================================ 
 *
 * NCPI for B3 protocol "7":
 */
#define CAPI_NCPI_MODEM(m,n) \
  m(n, WORD  , wSpeed,)\
  m(n, WORD  , wProtocol,)\
  END

/* ================================================ */

#define CAPI_PROFILE_DATA(m,n) \
  m(n, WORD  , wNumCtlr,)\
  m(n, WORD  , wNumBChannels,)\
  m(n, DWORD , dwGlobalOptions,)\
  m(n, DWORD , dwB1ProtocolSupport,)\
  m(n, DWORD , dwB2ProtocolSupport,)\
  m(n, DWORD , dwB3ProtocolSupport,)\
  m(n, BYTE_ARRAY, bReserved, 24)\
  m(n, BYTE_ARRAY, bManufacturerSpecific, 20)\
  END

/* definition of "dwGlobalOptions": */
#define CAPI_PROFILE_INTERNAL_CTLR_SUPPORT      0x0001
#define CAPI_PROFILE_EXTERNAL_EQUIPMENT_SUPPORT 0x0002
#define CAPI_PROFILE_HANDSET_SUPPORT            0x0004
#define CAPI_PROFILE_DTMF_SUPPORT               0x0008
#define CAPI_PROFILE_SUPPLEMENTARY_SERVICES     0x0010
#define CAPI_PROFILE_CHANNEL_ALLOCATION         0x0020
#define CAPI_PROFILE_BCHANNEL_OPERATION         0x0040
#define CAPI_PROFILE_LINE_INTERCONNECT          0x0080
#define CAPI_PROFILE_BROADBAND_EXTENSIONS       0x0100
#define CAPI_PROFILE_ECHO_CANCELLATION          0x0200

/* definition of "dwB1ProtocolSupport": */
#define CAPI_B1_MASK(PROT) (1 << ((CAPI_B1_##PROT) % 32))

/* definition of "dwB2ProtocolSupport": */
#define CAPI_B2_MASK(PROT) (1 << ((CAPI_B2_##PROT) % 32))

/* definition of "dwB3ProtocolSupport": */
#define CAPI_B3_MASK(PROT) (1 << ((CAPI_B3_##PROT) % 32))

/* ================================================ */

#define CAPI_B_CHANNEL_INFO_ANY(m,n) \
  m(n, WORD  , wChannel,)\
  END

/* definition of "wChannel": */
#define CAPI_BCHAN_MODE_B               0
#define CAPI_BCHAN_MODE_D               1
#define CAPI_BCHAN_MODE_NONE            2
#define CAPI_BCHAN_MODE_LEASED_ALLOC    3
#define CAPI_BCHAN_MODE_CHANNEL_ID      4

/* ================================================ */

#define CAPI_B_CHANNEL_INFO_LEASED_ALLOC(m,n) \
  m(n, WORD  , wChannel,)\
  m(n, WORD  , wOperation,)\
  m(n, BYTE_ARRAY, abChannelMaskArray, 31)\
  END

/* definition of "wChannel": (see previous) */

/* ================================================ */

#define CAPI_B_CHANNEL_INFO_CHANNEL_ID(m,n) \
  m(n, WORD  , wChannel,)\
  m(n, STRUCT, chanID,)\
  END

/* definition of "wChannel": (see previous) */

/* ================================================ 
 *
 * FACILITY: Line interconnect structures
 */
#define CAPI_LI_CONN_REQ_PART(m,n) \
  m(n, DWORD, dwCid,)\
  m(n, DWORD, dwDataPath,)\
  END

#define CAPI_LI_CONN_CONF_PART(m,n) \
  m(n, DWORD, dwCid,)\
  m(n, WORD , wInfo,)\
  END

#define CAPI_LI_DISC_REQ_PART(m,n) \
  m(n, DWORD, dwCid,)\
  END

#define CAPI_LI_DISC_CONF_PART(m,n) \
  m(n, DWORD, dwCid,)\
  m(n, WORD , wInfo,)\
  END

#define CAPI_LI_CONN_REQ_PARAM(m,n) \
  m(n, DWORD , dwDataPath,)\
  m(n, STRUCT, conn_req_part,)\
  END

#define CAPI_LI_DISC_REQ_PARAM(m,n) \
  m(n, STRUCT, disc_req_part,)\
  END

#define CAPI_LI_SUPP_CONF_PARAM(m,n) \
  m(n, WORD , wInfo,)\
  m(n, DWORD, dwSupportedServices,)\
  m(n, DWORD, dwInterconnectsCtrl,)\
  m(n, DWORD, dwParticipantsCtrl,)\
  m(n, DWORD, dwInterconnectsGlobal,)\
  m(n, DWORD, dwParticipantsGlobal,)\
  END

#define CAPI_LI_CONN_CONF_PARAM(m,n) \
  m(n, WORD  , wInfo,)\
  m(n, STRUCT, conn_conf_part,)\
  END

#define CAPI_LI_DISC_CONF_PARAM(m,n) \
  m(n, WORD  , wInfo,)\
  m(n, STRUCT, disc_conf_part,)\
  END

#define CAPI_LI_CONN_IND_PARAM(m,n) \
  m(n, DWORD , dwCid,)\
  END

#define CAPI_LI_DISC_IND_PARAM(m,n) \
  m(n, DWORD, dwCid,)\
  m(n, WORD , wServiceReason,)\
  END

#define CAPI_LINE_INTERCONNECT_PARAM(m,n)\
  m(n, WORD  , wFunction,)\
  m(n, STRUCT, Param,)\
  END

/*---------------------------------------------------------------------------*
 *	declare all CAPI structures and some enums
 *---------------------------------------------------------------------------*/
#define CAPI_SUB_STRUCTURES(m,n) \
  m(n, B_PROTOCOL)\
  m(n, ADDITIONAL_INFO)\
  m(n, SENDING_COMPLETE)\
  m(n, FACILITY_REQ_DTMF_PARAM)\
  m(n, FACILITY_CONF_DTMF_PARAM)\
  m(n, FACILITY_IND_DTMF_PARAM)\
  m(n, FACILITY_RESP_DTMF_PARAM)\
  m(n, B1_CONFIG_V100)\
  m(n, B1_CONFIG_FAX_G3)\
  m(n, B1_CONFIG_MODEM)\
  m(n, B2_CONFIG_X75_SLP)\
  m(n, B2_CONFIG_SDLC)\
  m(n, B2_CONFIG_LAPD_X25)\
  m(n, B2_CONFIG_MODEM_FULL)\
  m(n, B2_CONFIG_V120)\
  m(n, B2_CONFIG_V42BIS)\
  m(n, B3_CONFIG_T90NL)\
  m(n, B3_CONFIG_FAX_G3)\
  m(n, B3_CONFIG_FAX_G3_EXT)\
  m(n, B_PROTOCOL_GLOBAL_OPTIONS)\
  m(n, NCPI_STD)\
  m(n, NCPI_FAX_G3)\
  m(n, NCPI_FAX_G3_EXT)\
  m(n, NCPI_MODEM)\
  m(n, PROFILE_DATA)\
  m(n, B_CHANNEL_INFO_ANY)\
  m(n, B_CHANNEL_INFO_LEASED_ALLOC)\
  m(n, B_CHANNEL_INFO_CHANNEL_ID)\
  m(n, LI_CONN_REQ_PART)\
  m(n, LI_CONN_CONF_PART)\
  m(n, LI_DISC_REQ_PART)\
  m(n, LI_DISC_CONF_PART)\
  m(n, LI_CONN_REQ_PARAM)\
  m(n, LI_DISC_REQ_PARAM)\
  m(n, LI_SUPP_CONF_PARAM)\
  m(n, LI_CONN_CONF_PARAM)\
  m(n, LI_DISC_CONF_PARAM)\
  m(n, LI_CONN_IND_PARAM)\
  m(n, LI_DISC_IND_PARAM)\
  m(n, LINE_INTERCONNECT_PARAM)\

/* for each command generate eight structures */
CAPI_COMMANDS(CAPI_MAKE_DEF_1,);

enum
{
    CAPI_COMMANDS(CAPI_MAKE_DEF_2,)
    CAPI_COMMAND_INDEX_MAX /* exclusive */
};

/* for each substructure generate two structures */
CAPI_SUB_STRUCTURES(CAPI_MAKE_DEF_3,);

/* structure for decoding of CAPI messages */
struct capi_message_decoded 
{
    struct CAPI_HEADER_DECODED head;

    union
    {
        CAPI_COMMANDS(CAPI_MAKE_UNION_1,DECODED);

    } data;

    struct CAPI_ADDITIONAL_INFO_DECODED ADDITIONAL_INFO;
    struct CAPI_B_PROTOCOL_DECODED B_PROTOCOL;

#if !defined(CAPI_NO_COMPAT_CODE)

    /* these fields are read-only 
     * and very depreceated ! 
     * Use the HEADER* macros 
     * instead.
     */
    u_int8_t Command;
    u_int8_t Subcommand;
    u_int16_t Messagenumber;

#endif

} __packed;

/* structure for encoding of CAPI messages */
struct capi_message_encoded
{
    struct CAPI_HEADER_ENCODED head;
    union 
    {
        union
	{
	  /* padding fields */
	  u_int8_t  b [1024 - sizeof(struct CAPI_HEADER_ENCODED)];
	  u_int16_t w [0];
	  u_int32_t d [0];
	  u_int64_t q [0];
	} any;

        CAPI_COMMANDS(CAPI_MAKE_UNION_1,ENCODED);

    } data;
} __packed;

/* structure for decoding of CAPI STRUCT element */
union capi_struct_decoded {

    CAPI_SUB_STRUCTURES(CAPI_MAKE_UNION_2,DECODED);

} __packed;

/* structure for encoding of CAPI STRUCT element */
union capi_struct_encoded {

    CAPI_SUB_STRUCTURES(CAPI_MAKE_UNION_2,ENCODED);

    /* padding fields */
    u_int8_t  b [1024 - sizeof(struct CAPI_HEADER_ENCODED)];
    u_int16_t w [0];
    u_int32_t d [0];
    u_int64_t q [0];

} __packed;


/*---------------------------------------------------------------------------*
 *	CAPI ioctl definitions
 *---------------------------------------------------------------------------*/
#if (defined(CAPI_MAKE_IOCTL) || defined(_KERNEL))

/* the following structure uses
 * host endian format:
 */
struct capi_register_req {

  /* input parameters */
  u_int32_t max_logical_connections;
  u_int32_t max_b_data_blocks;
  u_int32_t max_b_data_len;

  /* result parameters */
  u_int32_t app_id;
};

#define CAPI_REGISTER_REQ _IOWR('C', 1, struct capi_register_req)

#define CAPI_START_D_CHANNEL_REQ _IO('C', 2)

#define CAPI_MANUFACTURER_LEN 64

struct capi_get_manufacturer_req {
  u_int32_t controller;
  u_int8_t name[CAPI_MANUFACTURER_LEN]; /* zero terminated! */
};

#define CAPI_GET_MANUFACTURER_REQ _IOWR('C', 3, struct capi_get_manufacturer_req)

struct capi_get_version_req_vdata {
  u_int8_t raw[0];
  u_int8_t CAPI_major;
  u_int8_t CAPI_minor;
  u_int8_t manufacturer_major;
  u_int8_t manufacturer_minor;
  u_int8_t BSD_major;
  u_int8_t BSD_minor;
  u_int8_t stack_major;
  u_int8_t stack_minor;
} __packed;

struct capi_get_version_req {
  u_int32_t controller;
  struct capi_get_version_req_vdata version;
};

#define CAPI_GET_VERSION_REQ _IOWR('C', 4, struct capi_get_version_req)

#define CAPI_SERIAL_LEN	8

struct capi_get_serial_req {
  u_int32_t controller;
  u_int8_t serial_number[CAPI_SERIAL_LEN]; /* zero terminated! */
};

#define CAPI_GET_SERIAL_REQ _IOWR('C', 5, struct capi_get_serial_req)

#define CAPI_IOCTL_TEST_REQ _IO('C', 6)

struct capi_get_profile_req {
  u_int32_t controller;

  /* the following structure uses little endian format ! */
  struct CAPI_PROFILE_DATA_ENCODED profile;
};

#define CAPI_GET_PROFILE_REQ _IOWR('C', 7, struct capi_get_profile_req)

#define CAPI_SET_STACK_VERSION_REQ _IOW('C', 8, u_int32_t)

#endif

/*---------------------------------------------------------------------------*
 *	CAPI library prototypes (from libcapi.c)
 *---------------------------------------------------------------------------*/
#if !(defined(_KERNEL) || defined(CAPI_NO_SHARED_LIBRARY))

struct isdn_dr_prot;
struct timeval;

extern u_int16_t
capi20_register(u_int32_t max_logical_connections,
		u_int32_t max_b_data_blocks,
		u_int32_t max_b_data_len,
		u_int32_t *app_id_ptr,
		u_int32_t stack_version);

extern u_int16_t
capi20_release(u_int32_t app_id);

extern u_int16_t
capi20_put_message(u_int32_t app_id, void *buf_ptr);

extern u_int16_t 
capi20_get_message(u_int32_t app_id, u_int8_t **buf_pp);

extern u_int16_t
capi20_wait_for_message(u_int32_t app_id, struct timeval *timeval_ptr);

extern u_int16_t
capi20_get_manufacturer(u_int32_t controller, u_int8_t *buf_ptr, 
			u_int16_t buf_len);

extern u_int16_t
capi20_get_version(u_int32_t controller, u_int8_t *buf_ptr, u_int16_t buf_len);

extern u_int16_t
capi20_get_serial_number(u_int32_t controller, u_int8_t *buf_ptr,
			 u_int16_t buf_len);

extern u_int16_t 
capi20_get_profile(u_int32_t controller, void *buf_ptr, u_int16_t buf_len);

extern u_int16_t
capi20_is_installed(void);

extern int
capi20_fileno(u_int32_t app_id);


/* extensions */

extern u_int16_t
capi_firmware_download(u_int32_t controller, struct isdn_dr_prot *protocols_ptr,
		       u_int16_t protocols_len);

extern void
capi_translate_to_message_decoded(struct capi_message_decoded *mp, 
				  void *buf_ptr);

extern u_int16_t
capi_get_message_decoded(struct capi_message_decoded *mp, u_int32_t app_id);

extern void
capi_translate_from_message_decoded(struct capi_message_decoded *mp, 
				    void *buf_ptr, u_int16_t buf_len);

extern u_int16_t
capi_put_message_decoded(struct capi_message_decoded *mp);

extern const u_int8_t *
capi_get_error_string(u_int16_t wError);

extern u_int16_t
capi_message_decoded_to_string(u_int8_t *dst, u_int16_t len, 
			       const struct capi_message_decoded *mp);

extern const u_int8_t *
capi_get_command_string(u_int16_t wCmd);

/* aliases */

#define capi20_waitformessage   capi20_wait_for_message
#define capi20_isinstalled      capi20_is_installed

#define capi20_cmsg2message     capi_translate_from_message_decoded
#define capi_cmsg2message       capi_translate_from_message_decoded

#define capi20_message2cmsg     capi_translate_to_message_decoded
#define capi_message2cmsg       capi_translate_to_message_decoded

#define CAPI_GET_CMSG           capi_get_message_decoded
#define capi20_get_cmsg         capi_get_message_decoded
#define capi_get_cmsg           capi_get_message_decoded

#define CAPI_PUT_CMSG           capi_put_message_decoded
#define capi20_put_cmsg         capi_put_message_decoded
#define capi_put_cmsg           capi_put_message_decoded

#define capi20_info2str         capi_get_error_string
#define capi_info2str           capi_get_error_string

#define capi20_cmd2str(a,b)     capi_get_command_string(((b) << 8)|(a))
#define capi_cmd2str(a,b)       capi_get_command_string(((b) << 8)|(a))

#define CAPI_CMSG_HEADER        capi_cmsg_header
#define capi20_cmsg_header      capi_cmsg_header

/* standard CAPI2.0 function aliases */

#define CAPI20_REGISTER          capi20_register
#define CAPI20_RELEASE           capi20_release
#define CAPI20_PUT_MESSAGE       capi20_put_message
#define CAPI20_GET_MESSAGE       capi20_get_message
#define CAPI20_WaitforMessage    capi20_wait_for_message
#define CAPI20_GET_MANUFACTURER  capi20_get_manufacturer
#define CAPI20_GET_VERSION       capi20_get_version
#define CAPI20_GET_SERIAL_NUMBER capi20_get_serial_number
#define CAPI20_GET_PROFILE       capi20_get_profile
#define CAPI20_ISINSTALLED       capi20_is_installed

static __inline u_int16_t 
capi_message_decoded_header(struct capi_message_decoded *mp, u_int16_t wApp, 
			    u_int16_t wCmd, u_int16_t wNum, u_int32_t dwCid)
{
	/* set default value */
	bzero(mp, sizeof(*mp));
  
	/* fill out header */
	mp->head.wApp = wApp;
	mp->head.wCmd = wCmd;
	mp->head.wNum = wNum;
	mp->head.dwCid = dwCid;

	return 0;
}

#define capi_cmsg_header(mp,app,cmd,subcmd,num,cid) \
        capi_message_decoded_header(mp,app,(cmd)|((subcmd) << 8),num,cid)

#if 0
/* these are not supported */
#define capi20_cmsg_answer      capi_cmsg_answer
#define capi_msg_answer
#endif

#define capi20_cmsg2str capi_cmsg2str

static __inline u_int8_t *
capi_cmsg2str(struct capi_message_decoded *mp)
{
	static u_int8_t buffer[1024];

	capi_message_decoded_to_string(&buffer[0], sizeof(buffer), mp);

	return &buffer[0];
}

#define capi20_message2str capi_message2str

static __inline u_int8_t *
capi_message2str(u_int8_t *buf)
{
	struct capi_message_decoded msg;

	capi_translate_to_message_decoded(&msg, buf);

	return capi_cmsg2str(&msg);
}

#endif

#ifdef CAPI_MAKE_TRANSLATOR

/*---------------------------------------------------------------------------*
 *	decode CAPI Information Elements
 *
 * missing Information Elements gets a value of zero
 *---------------------------------------------------------------------------*/
static void
capi_decode(void *ptr, u_int16_t len, void *ie)
{
#if 0
	u_int16_t len_old = len;
#endif
	u_int8_t what;

	while(1)
	{
	  what = ((u_int8_t *)(ie))[0];
	  ie = ADD_BYTES(ie, 1);

	  switch(what) {
	  case IE_END:
	    goto done;

	  case IE_BYTE:
	    if(len >= 1)
	    {
	      ((u_int8_t *)(ie))[0] =
		((u_int8_t *)(ptr))[0];
	      ie = ADD_BYTES(ie, 1);
	      ptr = ADD_BYTES(ptr, 1);
	      len -= 1;
	      break;
	    }
	    ((u_int8_t *)(ie))[0] = 0;
	    ie = ADD_BYTES(ie, 1);
	    goto ie_error;

	  case IE_WORD:
	    if(len >= 2)
	    {
	      ((u_int16_p_t *)(ie))->data =
		le16toh(((u_int16_p_t *)(ptr))->data);

	      ie = ADD_BYTES(ie, 2);
	      ptr = ADD_BYTES(ptr, 2);
	      len -= 2;
	      break;
	    }
	    ((u_int16_p_t *)(ie))->data = 0;
	    ie = ADD_BYTES(ie, 2);
	    goto ie_error;

	  case IE_DWORD:
	    if(len >= 4)
	    {
	      ((u_int32_p_t *)(ie))->data =
		le32toh(((u_int32_p_t *)(ptr))->data);

	      ie = ADD_BYTES(ie, 4);
	      ptr = ADD_BYTES(ptr, 4);
	      len -= 4;
	      break;
	    }
	    ((u_int32_p_t *)(ie))->data = 0;
	    ie = ADD_BYTES(ie, 4);
	    goto ie_error;

	  case IE_QWORD:
	    if(len >= 8)
	    {
	      ((u_int64_p_t *)(ie))->data = 
		le64toh(((u_int64_p_t *)(ptr))->data);

	      ie = ADD_BYTES(ie, 8);
	      ptr = ADD_BYTES(ptr, 8);
	      len -= 8;
	      break;
	    }
	    ((u_int64_p_t *)(ie))->data = 0;
	    ie = ADD_BYTES(ie, 8);
	    goto ie_error;

	  ie_error:
	    /* skip remaining data */
	    ptr = ADD_BYTES(ptr, len);
	    len = 0;
	    break;

	  case IE_STRUCT_CAPI:
	    /* pre-store pointer to CAPI structure */
	    ((void_p_t *)(ADD_BYTES(ie,2)))->data = ptr;

	  case IE_STRUCT_DECODED:
	  case IE_STRUCT_DECODED_EMPTY:
	  case IE_STRUCT: /* default */
	    {
	      register u_int16_t temp;

	      if(len >= 1)
	      {
		  if(((u_int8_t *)(ptr))[0] == 0xFF)
		  {
		      /* length is escaped */

		      if(len >= 3)
		      {
			  temp = ((u_int8_t *)(ptr))[2];
			  temp <<= 8;
			  temp |= ((u_int8_t *)(ptr))[1];

			  ptr = ADD_BYTES(ptr, 3);
			  len -= 3;
		      }
		      else
		      {
			  temp = 255; /* dummy */
		      }
		  }
		  else
		  {
		      temp = ((u_int8_t *)(ptr))[0];

		      ptr = ADD_BYTES(ptr, 1);
		      len -= 1;
		  }
	      }
	      else
	      {
		  temp = 255; /* dummy */
	      }

	      /* check structure length */

	      if(temp > len)
	      {
		  /* skip remaining data and
		   * set lengths to zero
		   */
		  ptr = ADD_BYTES(ptr, len);
		  len = 0;
		  temp = 0;
	      }

	      /* store length */
	      if(what == IE_STRUCT)
	      {
		  ((u_int16_p_t *)(ie))->data = temp;
		  ie = ADD_BYTES(ie, 2);

		  /* store pointer to data structure */
		  ((void_p_t *)(ie))->data = ptr;
	      }
	      else
	      {
		  /* this field should not be used */
		  ((u_int16_p_t *)(ie))->data = 0;
		  ie = ADD_BYTES(ie, 2);

		  /* ((void_p_t *)(ie))->data should already have
		   * been set !
		   */
	      }

	      if((what == IE_STRUCT_DECODED) || 
		 (what == IE_STRUCT_DECODED_EMPTY))
	      {
		  /* need to update the structure type, 
		   * hence the application might want to 
		   * check this field !
		   */
		  (((u_int8_t *)(ie)) - 3)[0] = (temp) ?
		    IE_STRUCT_DECODED : IE_STRUCT_DECODED_EMPTY;

		  capi_decode(ptr, temp, ((void_p_t *)(ie))->data);
	      }
	      else
	      {
		  if(temp == 0)
		  {
		      /* In case this is a CAPI structure, one
		       * needs to set the pointer to a valid 
		       * structure, hence the real structure
		       * might have an invalid length field !
		       * Else try to catch the cases where the
		       * software is looking up fields before
		       * checking the length!
		       */
		      static const u_int8_t empty_struct[8] = { /* zero */ };

		      /* structure is empty or non-existing */
		      ((void_p_t *)(ie))->data = (void *)&empty_struct;
		  }
	      }
	      ie = ADD_BYTES(ie, sizeof(void *));
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  case IE_BYTE_ARRAY:
	    {
	      register u_int16_t temp;

	      temp = ((u_int16_p_t *)(ie))->data;

	      ie = ADD_BYTES(ie, 2);

	      if(temp > len)
	      {
		  /* part of array is zero ! */
		  bzero(ADD_BYTES(ie,len), temp-len);
		  temp = len;
	      }

	      bcopy(ptr, ie, temp);

	      ie = ADD_BYTES(ie, temp);
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  default:
	    panic("%s: %s: invalid info=%d\n",
		  __FILE__, __FUNCTION__, what);
	    break;
	  }
	}
 done:
#if 0
	return (len_old - len);
#else
	return;
#endif
}

/*---------------------------------------------------------------------------*
 *	encode CAPI Information Elements
 *
 * returns encoded data length
 *---------------------------------------------------------------------------*/
static u_int16_t
capi_encode(void *ptr, u_int16_t len, void *ie)
{
	u_int16_t len_old = len;
	u_int8_t what;

	while(1)
	{
	  what = ((u_int8_t *)(ie))[0];
	  ie = ADD_BYTES(ie, 1);

	  switch(what) {
	  case IE_END:
	    goto done;

	  case IE_BYTE:
	    if(len < 1) goto error; /* overflow */

	    ((u_int8_t *)(ptr))[0] = 
	      ((u_int8_t *)(ie))[0];
	    ie = ADD_BYTES(ie, 1);
	    ptr = ADD_BYTES(ptr, 1);
	    len -= 1;
	    break;

	  case IE_WORD:
	    if(len < 2) goto error; /* overflow */

	    ((u_int16_p_t *)(ptr))->data =
	      htole16(((u_int16_p_t *)(ie))->data);
	    ie = ADD_BYTES(ie, 2);
	    ptr = ADD_BYTES(ptr, 2);
	    len -= 2;
	    break;

	  case IE_DWORD:
	    if(len < 4) goto error; /* overflow */

	    ((u_int32_p_t *)(ptr))->data =
	      htole32(((u_int32_p_t *)(ie))->data);
	    ie = ADD_BYTES(ie, 4);
	    ptr = ADD_BYTES(ptr, 4);
	    len -= 4;
	    break;

	  case IE_QWORD:
	    if(len < 8) goto error; /* overflow */

	    ((u_int64_p_t *)(ptr))->data =
	      htole64(((u_int64_p_t *)(ie))->data);
	    ie = ADD_BYTES(ie, 8);
	    ptr = ADD_BYTES(ptr, 8);
	    len -= 8;
	    break;

	  case IE_STRUCT_DECODED_EMPTY:
	    if(len < 1) goto error; /* overflow */

	    ((u_int8_t *)(ptr))[0] = 0;
	    ie = ADD_BYTES(ie, sizeof(void *)+2);
	    ptr = ADD_BYTES(ptr, 1);
	    len -= 1;
	    break;

	  case IE_STRUCT_DECODED:
	  case IE_STRUCT_CAPI:
	  case IE_STRUCT:
	    {
	      register u_int16_t temp;
	      register void *var;

	      temp = ((u_int16_p_t *)(ie))->data;
	      ie = ADD_BYTES(ie, 2);
	      var = ((void_p_t *)(ie))->data;
	      ie = ADD_BYTES(ie, sizeof(void *));

	      if(what == IE_STRUCT_CAPI)
	      {
		  if(var == NULL)
		  {
		      temp = 0;
		  }
		  else
		  {
		    /* get length field from CAPI structure */

		    if(((u_int8_t *)(var))[0] == 0xFF)
		    {
		      temp = ((((u_int8_t *)(var))[1]) |
			      (((u_int8_t *)(var))[2] << 8));
		      var = ADD_BYTES(var, 3);

		      if(temp < 3) goto error; /* overflow */
		    }
		    else
		    {
		      temp = ((u_int8_t *)(var))[0];
		      var = ADD_BYTES(var, 1);

		      if(temp < 1) goto error; /* overflow */
		    }
		  }
	      }

	      if(what == IE_STRUCT_DECODED)
	      {
		  if(len < 1) goto error; /* overflow */

		  temp = capi_encode(ADD_BYTES(ptr,1),len-1,var);

		  if(temp >= 0x00FF)
		  {
		      register u_int16_t temp2 = temp;

		      if((len-temp2) < 3) goto error; /* overflow */

		      /* move all data up two bytes */
		      while(temp2--)
		      {
			  *(((u_int8_t *)(ptr)) + 3 + temp2) =
			    *(((u_int8_t *)(ptr)) + 1 + temp2);
		      }
		  }
	      }

	      if(temp >= 0x00FF)
	      {
		if((len < 3) || (temp > (len - 3)))
		{
		    goto error; /* overflow */
		}

		((u_int8_t *)(ptr))[0] = 0xFF;
		((u_int8_t *)(ptr))[1] = temp;
		((u_int8_t *)(ptr))[2] = temp >> 8;
		ptr = ADD_BYTES(ptr, 3);
		len -= 3;
	      }
	      else
	      {
		if((len < 1) || (temp > (len - 1)))
		{
		    goto error; /* overflow */
		}

		((u_int8_t *)(ptr))[0] = temp;
		ptr = ADD_BYTES(ptr, 1);
		len -= 1;
	      }

	      if(what != IE_STRUCT_DECODED)
	      {
		  bcopy(var, ptr, temp);
	      }
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  case IE_BYTE_ARRAY:
	    {
	      register u_int16_t temp;

	      temp = ((u_int16_p_t *)(ie))->data;

	      if(len < temp) goto error; /* overflow */

	      ie = ADD_BYTES(ie, 2);

	      bcopy(ie, ptr, temp);
	      ie = ADD_BYTES(ie, temp);
	      ptr = ADD_BYTES(ptr, temp);
	      len -= temp;
	    }
	    break;

	  default:
	    panic("%s: %s: invalid info=%d\n",
		  __FILE__, __FUNCTION__, what);
	    break;

	  error:
	    /* just skip the remaining
	     * fields !
	     */
	    printf("%s: %s: warning: out of memory!\n",
		   __FILE__, __FUNCTION__);
	    goto done;
	  }
	}
 done:
	return (len_old - len);
}

#endif

/*---------------------------------------------------------------------------*
 *	"struct capi_message_decoded" access macros
 *
 * NOTE: if you want your software to be portable,
 *       use these macros when accessing this 
 *       structure!
 *---------------------------------------------------------------------------*/

#define HEADER_LEN(x)    ((x)->head.wLen)
#define HEADER_APP(x)    ((x)->head.wApp)
#define HEADER_CMD(x)    ((x)->head.wCmd) /* command and sub-command in one ! */
#define HEADER_MSGNUM(x) ((x)->head.wNum)
#define HEADER_CID(x)    ((x)->head.dwCid)

#define ALERT_REQ_ADDITIONALINFO(x) ((x)->data.ALERT_REQ.add_info_STRUCT)
#define ALERT_REQ_BCHANNELINFORMATION(x) ((x)->ADDITIONAL_INFO.b_channel_info.ppbyte[0])
#define ALERT_REQ_KEYPADFACILITY(x) ((x)->ADDITIONAL_INFO.keypad.ppbyte[0])
#define ALERT_REQ_USERUSERDATA(x) ((x)->ADDITIONAL_INFO.useruser.ppbyte[0])
#define ALERT_REQ_FACILITYDATAARRAY(x) ((x)->ADDITIONAL_INFO.facility.ppbyte[0])
#define ALERT_REQ_SENDINGCOMPLETE(x) ((x)->ADDITIONAL_INFO.sending_complete.ppbyte[0])
#define ALERT_CONF_INFO(x) ((x)->data.ALERT_CONF.wInfo)
#define CONNECT_REQ_CIPVALUE(x) ((x)->data.CONNECT_REQ.wCIP)
#define CONNECT_REQ_CALLEDPARTYNUMBER(x) ((x)->data.CONNECT_REQ.dst_telno.ppbyte[0])
#define CONNECT_REQ_CALLINGPARTYNUMBER(x) ((x)->data.CONNECT_REQ.src_telno.ppbyte[0])
#define CONNECT_REQ_CALLEDPARTYSUBADDRESS(x) ((x)->data.CONNECT_REQ.dst_subaddr.ppbyte[0])
#define CONNECT_REQ_CALLINGPARTYSUBADDRESS(x) ((x)->data.CONNECT_REQ.src_subaddr.ppbyte[0])
#define CONNECT_REQ_BPROTOCOL(x) ((x)->data.CONNECT_REQ.b_protocol_STRUCT)
#define CONNECT_REQ_B1PROTOCOL(x) ((x)->B_PROTOCOL.wB1_protocol)
#define CONNECT_REQ_B2PROTOCOL(x) ((x)->B_PROTOCOL.wB2_protocol)
#define CONNECT_REQ_B3PROTOCOL(x) ((x)->B_PROTOCOL.wB3_protocol)
#define CONNECT_REQ_B1CONFIGURATION(x) ((x)->B_PROTOCOL.B1_config.ppbyte[0])
#define CONNECT_REQ_B2CONFIGURATION(x) ((x)->B_PROTOCOL.B2_config.ppbyte[0])
#define CONNECT_REQ_B3CONFIGURATION(x) ((x)->B_PROTOCOL.B3_config.ppbyte[0])
#define CONNECT_REQ_GLOBALCONFIGURATION(x) ((x)->B_PROTOCOL.global_config.ppbyte[0])
#define CONNECT_REQ_BC(x) ((x)->data.CONNECT_REQ.BC.ppbyte[0])
#define CONNECT_REQ_LLC(x) ((x)->data.CONNECT_REQ.LLC.ppbyte[0])
#define CONNECT_REQ_HLC(x) ((x)->data.CONNECT_REQ.HLC.ppbyte[0])
#define CONNECT_REQ_ADDITIONALINFO(x) ((x)->data.CONNECT_REQ.add_info_STRUCT)
#define CONNECT_REQ_BCHANNELINFORMATION(x) ((x)->ADDITIONAL_INFO.b_channel_info.ppbyte[0])
#define CONNECT_REQ_KEYPADFACILITY(x) ((x)->ADDITIONAL_INFO.keypad.ppbyte[0])
#define CONNECT_REQ_USERUSERDATA(x) ((x)->ADDITIONAL_INFO.useruser.ppbyte[0])
#define CONNECT_REQ_FACILITYDATAARRAY(x) ((x)->ADDITIONAL_INFO.facility.ppbyte[0])
#define CONNECT_REQ_SENDINGCOMPLETE(x) ((x)->ADDITIONAL_INFO.sending_complete.ppbyte[0])
#define CONNECT_REQ_FLAG0(x) ((x)->data.CONNECT_REQ.bFlag_0)
#define CONNECT_REQ_SECONDCALLINGPARTYNUMBER(x) ((x)->data.CONNECT_REQ.src_telno_2.ppbyte[0])
#define CONNECT_CONF_INFO(x) ((x)->data.CONNECT_CONF.wInfo)
#define CONNECT_IND_CIPVALUE(x) ((x)->data.CONNECT_IND.wCIP)
#define CONNECT_IND_CALLEDPARTYNUMBER(x) ((x)->data.CONNECT_IND.dst_telno.ppbyte[0])
#define CONNECT_IND_CALLINGPARTYNUMBER(x) ((x)->data.CONNECT_IND.src_telno.ppbyte[0])
#define CONNECT_IND_CALLEDPARTYSUBADDRESS(x) ((x)->data.CONNECT_IND.dst_subaddr.ppbyte[0])
#define CONNECT_IND_CALLINGPARTYSUBADDRESS(x) ((x)->data.CONNECT_IND.src_subaddr.ppbyte[0])
#define CONNECT_IND_BC(x) ((x)->data.CONNECT_IND.BC.ppbyte[0])
#define CONNECT_IND_LLC(x) ((x)->data.CONNECT_IND.LLC.ppbyte[0])
#define CONNECT_IND_HLC(x) ((x)->data.CONNECT_IND.HLC.ppbyte[0])
#define CONNECT_IND_ADDITIONALINFO(x) ((x)->data.CONNECT_IND.add_info_STRUCT)
#define CONNECT_IND_BCHANNELINFORMATION(x) ((x)->ADDITIONAL_INFO.b_channel_info.ppbyte[0])
#define CONNECT_IND_KEYPADFACILITY(x) ((x)->ADDITIONAL_INFO.keypad.ppbyte[0])
#define CONNECT_IND_USERUSERDATA(x) ((x)->ADDITIONAL_INFO.useruser.ppbyte[0])
#define CONNECT_IND_FACILITYDATAARRAY(x) ((x)->ADDITIONAL_INFO.facility.ppbyte[0])
#define CONNECT_IND_SENDINGCOMPLETE(x) ((x)->ADDITIONAL_INFO.sending_complete.ppbyte[0])
#define CONNECT_IND_SECONDCALLINGPARTYNUMBER(x) ((x)->data.CONNECT_IND.src_telno_2.ppbyte[0])
#define CONNECT_RESP_REJECT(x) ((x)->data.CONNECT_RESP.wReject)
#define CONNECT_RESP_BPROTOCOL(x) ((x)->data.CONNECT_RESP.b_protocol_STRUCT)
#define CONNECT_RESP_B1PROTOCOL(x) ((x)->B_PROTOCOL.wB1_protocol)
#define CONNECT_RESP_B2PROTOCOL(x) ((x)->B_PROTOCOL.wB2_protocol)
#define CONNECT_RESP_B3PROTOCOL(x) ((x)->B_PROTOCOL.wB3_protocol)
#define CONNECT_RESP_B1CONFIGURATION(x) ((x)->B_PROTOCOL.B1_config.ppbyte[0])
#define CONNECT_RESP_B2CONFIGURATION(x) ((x)->B_PROTOCOL.B2_config.ppbyte[0])
#define CONNECT_RESP_B3CONFIGURATION(x) ((x)->B_PROTOCOL.B3_config.ppbyte[0])
#define CONNECT_RESP_GLOBALCONFIGURATION(x) ((x)->B_PROTOCOL.global_config.ppbyte[0])
#define CONNECT_RESP_CONNECTEDNUMBER(x) ((x)->data.CONNECT_RESP.dst_telno.ppbyte[0])
#define CONNECT_RESP_CONNECTEDSUBADDRESS(x) ((x)->data.CONNECT_RESP.dst_subaddr.ppbyte[0])
#define CONNECT_RESP_LLC(x) ((x)->data.CONNECT_RESP.LLC.ppbyte[0])
#define CONNECT_RESP_ADDITIONALINFO(x) ((x)->data.CONNECT_RESP.add_info_STRUCT)
#define CONNECT_RESP_BCHANNELINFORMATION(x) ((x)->ADDITIONAL_INFO.b_channel_info.ppbyte[0])
#define CONNECT_RESP_KEYPADFACILITY(x) ((x)->ADDITIONAL_INFO.keypad.ppbyte[0])
#define CONNECT_RESP_USERUSERDATA(x) ((x)->ADDITIONAL_INFO.useruser.ppbyte[0])
#define CONNECT_RESP_FACILITYDATAARRAY(x) ((x)->ADDITIONAL_INFO.facility.ppbyte[0])
#define CONNECT_RESP_SENDINGCOMPLETE(x) ((x)->ADDITIONAL_INFO.sending_complete.ppbyte[0])
#define CONNECT_ACTIVE_IND_CONNECTEDNUMBER(x) ((x)->data.CONNECT_ACTIVE_IND.connected_telno.ppbyte[0])
#define CONNECT_ACTIVE_IND_CONNECTEDSUBADDRESS(x) ((x)->data.CONNECT_ACTIVE_IND.connected_subaddr.ppbyte[0])
#define CONNECT_ACTIVE_IND_LLC(x) ((x)->data.CONNECT_ACTIVE_IND.LLC.ppbyte[0])
#define DISCONNECT_REQ_ADDITIONALINFO(x) ((x)->data.DISCONNECT_REQ.add_info_STRUCT)
#define DISCONNECT_REQ_BCHANNELINFORMATION(x) ((x)->ADDITIONAL_INFO.b_channel_info.ppbyte[0])
#define DISCONNECT_REQ_KEYPADFACILITY(x) ((x)->ADDITIONAL_INFO.keypad.ppbyte[0])
#define DISCONNECT_REQ_USERUSERDATA(x) ((x)->ADDITIONAL_INFO.useruser.ppbyte[0])
#define DISCONNECT_REQ_FACILITYDATAARRAY(x) ((x)->ADDITIONAL_INFO.facility.ppbyte[0])
#define DISCONNECT_REQ_SENDINGCOMPLETE(x) ((x)->ADDITIONAL_INFO.sending_complete.ppbyte[0])
#define DISCONNECT_CONF_INFO(x) ((x)->data.DISCONNECT_CONF.wInfo)
#define DISCONNECT_IND_REASON(x) ((x)->data.DISCONNECT_IND.wReason)
#define LISTEN_REQ_INFOMASK(x) ((x)->data.LISTEN_REQ.dwInfoMask)
#define LISTEN_REQ_CIPMASK(x) ((x)->data.LISTEN_REQ.dwCipMask1)
#define LISTEN_REQ_CIPMASK2(x) ((x)->data.LISTEN_REQ.dwCipMask2)
#define LISTEN_REQ_CALLINGPARTYNUMBER(x) ((x)->data.LISTEN_REQ.src_telno.ppbyte[0])
#define LISTEN_REQ_CALLINGPARTYSUBADDRESS(x) ((x)->data.LISTEN_REQ.src_subaddr.ppbyte[0])
#define LISTEN_CONF_INFO(x) ((x)->data.LISTEN_CONF.wInfo)
#define INFO_REQ_CALLEDPARTYNUMBER(x) ((x)->data.INFO_REQ.dst_telno.ppbyte[0])
#define INFO_REQ_ADDITIONALINFO(x) ((x)->data.INFO_REQ.add_info_STRUCT)
#define INFO_REQ_BCHANNELINFORMATION(x) ((x)->ADDITIONAL_INFO.b_channel_info.ppbyte[0])
#define INFO_REQ_KEYPADFACILITY(x) ((x)->ADDITIONAL_INFO.keypad.ppbyte[0])
#define INFO_REQ_USERUSERDATA(x) ((x)->ADDITIONAL_INFO.useruser.ppbyte[0])
#define INFO_REQ_FACILITYDATAARRAY(x) ((x)->ADDITIONAL_INFO.facility.ppbyte[0])
#define INFO_REQ_SENDINGCOMPLETE(x) ((x)->ADDITIONAL_INFO.sending_complete.ppbyte[0])
#define INFO_CONF_INFO(x) ((x)->data.INFO_CONF.wInfo)
#define INFO_IND_INFONUMBER(x) ((x)->data.INFO_IND.wInfoNum)
#define INFO_IND_INFOELEMENT(x) ((x)->data.INFO_IND.InfoElement.ppbyte[0])
#define SELECT_B_PROTOCOL_REQ_BPROTOCOL(x) ((x)->data.SELECT_B_PROTOCOL_REQ.b_protocol_STRUCT)
#define SELECT_B_PROTOCOL_REQ_B1PROTOCOL(x) ((x)->B_PROTOCOL.wB1_protocol)
#define SELECT_B_PROTOCOL_REQ_B2PROTOCOL(x) ((x)->B_PROTOCOL.wB2_protocol)
#define SELECT_B_PROTOCOL_REQ_B3PROTOCOL(x) ((x)->B_PROTOCOL.wB3_protocol)
#define SELECT_B_PROTOCOL_REQ_B1CONFIGURATION(x) ((x)->B_PROTOCOL.B1_config.ppbyte[0])
#define SELECT_B_PROTOCOL_REQ_B2CONFIGURATION(x) ((x)->B_PROTOCOL.B2_config.ppbyte[0])
#define SELECT_B_PROTOCOL_REQ_B3CONFIGURATION(x) ((x)->B_PROTOCOL.B3_config.ppbyte[0])
#define SELECT_B_PROTOCOL_REQ_GLOBALCONFIGURATION(x) ((x)->B_PROTOCOL.global_config.ppbyte[0])
#define SELECT_B_PROTOCOL_CONF_INFO(x) ((x)->data.SELECT_B_PROTOCOL_CONF.wInfo)
#define FACILITY_REQ_FACILITYSELECTOR(x) ((x)->data.FACILITY_REQ.wSelector)
#define FACILITY_REQ_FACILITYREQUESTPARAMETER(x) ((x)->data.FACILITY_REQ.Param.ppbyte[0])
#define FACILITY_CONF_INFO(x) ((x)->data.FACILITY_CONF.wInfo)
#define FACILITY_CONF_FACILITYSELECTOR(x) ((x)->data.FACILITY_CONF.wSelector)
#define FACILITY_CONF_FACILITYCONFIRMATIONPARAMETER(x) ((x)->data.FACILITY_CONF.Param.ppbyte[0])
#define FACILITY_IND_FACILITYSELECTOR(x) ((x)->data.FACILITY_IND.wSelector)
#define FACILITY_IND_FACILITYINDICATIONPARAMETER(x) ((x)->data.FACILITY_IND.Param.ppbyte[0])
#define FACILITY_RESP_FACILITYSELECTOR(x) ((x)->data.FACILITY_RESP.wSelector)
#define FACILITY_RESP_FACILITYRESPONSEPARAMETERS(x) ((x)->data.FACILITY_RESP.Param.ppbyte[0])
#define CONNECT_B3_REQ_NCPI(x) ((x)->data.CONNECT_B3_REQ.NCPI.ppbyte[0])
#define CONNECT_B3_CONF_INFO(x) ((x)->data.CONNECT_B3_CONF.wInfo)
#define CONNECT_B3_IND_NCPI(x) ((x)->data.CONNECT_B3_IND.NCPI.ppbyte[0])
#define CONNECT_B3_RESP_REJECT(x) ((x)->data.CONNECT_B3_RESP.wReject)
#define CONNECT_B3_RESP_NCPI(x) ((x)->data.CONNECT_B3_RESP.NCPI.ppbyte[0])
#define CONNECT_B3_ACTIVE_IND_NCPI(x) ((x)->data.CONNECT_B3_ACTIVE_IND.NCPI.ppbyte[0])
#define DISCONNECT_B3_REQ_NCPI(x) ((x)->data.DISCONNECT_B3_REQ.NCPI.ppbyte[0])
#define DISCONNECT_B3_CONF_INFO(x) ((x)->data.DISCONNECT_B3_CONF.wInfo)
#define DISCONNECT_B3_IND_REASON_B3(x) ((x)->data.DISCONNECT_B3_IND.wReason)
#define DISCONNECT_B3_IND_NCPI(x) ((x)->data.DISCONNECT_B3_IND.NCPI.ppbyte[0])
#define DATA_B3_REQ_DATA(x)   \
  (((void_p_t *)	      \
    ((sizeof(void *) <= 4) ? (void *)&((x)->data.DATA_B3_REQ.dwPtr_1) :	\
     (sizeof(void *) <= 8) ? (void *)&((x)->data.DATA_B3_REQ.qwPtr_2) : \
     (void *)0 ))->data)
#define DATA_B3_REQ_DATALENGTH(x) ((x)->data.DATA_B3_REQ.wLen)
#define DATA_B3_REQ_DATAHANDLE(x) ((x)->data.DATA_B3_REQ.wHandle)
#define DATA_B3_REQ_FLAGS(x) ((x)->data.DATA_B3_REQ.wFlags)
#define DATA_B3_CONF_DATAHANDLE(x) ((x)->data.DATA_B3_CONF.wHandle)
#define DATA_B3_CONF_INFO(x) ((x)->data.DATA_B3_CONF.wInfo)
#define DATA_B3_IND_DATA(x) \
  (((void_p_t *)	    \
    ((sizeof(void *) <= 4) ? (void *)&((x)->data.DATA_B3_IND.dwPtr_1) : \
     (sizeof(void *) <= 8) ? (void *)&((x)->data.DATA_B3_IND.qwPtr_2) : \
     (void *)0))->data)
#define DATA_B3_IND_DATALENGTH(x) ((x)->data.DATA_B3_IND.wLen)
#define DATA_B3_IND_DATAHANDLE(x) ((x)->data.DATA_B3_IND.wHandle)
#define DATA_B3_IND_FLAGS(x) ((x)->data.DATA_B3_IND.wFlags)
#define DATA_B3_RESP_DATAHANDLE(x) ((x)->data.DATA_B3_RESP.wHandle)
#define RESET_B3_REQ_NCPI(x) ((x)->data.RESET_B3_REQ.NCPI.ppbyte[0])
#define RESET_B3_CONF_INFO(x) ((x)->data.RESET_B3_CONF.wInfo)
#define RESET_B3_IND_NCPI(x) ((x)->data.RESET_B3_IND.NCPI.ppbyte[0])
#define CONNECT_B3_T90_ACTIVE_IND_NCPI(x) ((x)->data.CONNECT_B3_T90_ACTIVE_IND.NCPI.ppbyte[0])
#define MANUFACTURER_REQ_MANUID(x) ((x)->data.MANUFACTURER_REQ.dwManuID)
#define MANUFACTURER_REQ_CLASS(x) ((x)->data.MANUFACTURER_REQ.dwClass)
#define MANUFACTURER_REQ_FUNCTION(x) ((x)->data.MANUFACTURER_REQ.dwFunction)
#define MANUFACTURER_REQ_MANUDATA(x) ((x)->data.MANUFACTURER_REQ.ManuData.ppbyte[0])
#define MANUFACTURER_CONF_MANUID(x) ((x)->data.MANUFACTURER_CONF.dwManuID)
#define MANUFACTURER_CONF_CLASS(x) ((x)->data.MANUFACTURER_CONF.dwClass)
#define MANUFACTURER_CONF_FUNCTION(x) ((x)->data.MANUFACTURER_CONF.dwFunction)
#define MANUFACTURER_CONF_MANUDATA(x) ((x)->data.MANUFACTURER_CONF.ManuData.ppbyte[0])
#define MANUFACTURER_IND_MANUID(x) ((x)->data.MANUFACTURER_IND.dwManuID)
#define MANUFACTURER_IND_CLASS(x) ((x)->data.MANUFACTURER_IND.dwClass)
#define MANUFACTURER_IND_FUNCTION(x) ((x)->data.MANUFACTURER_IND.dwFunction)
#define MANUFACTURER_IND_MANUDATA(x) ((x)->data.MANUFACTURER_IND.ManuData.ppbyte[0])
#define MANUFACTURER_RESP_MANUID(x) ((x)->data.MANUFACTURER_RESP.dwManuID)
#define MANUFACTURER_RESP_CLASS(x) ((x)->data.MANUFACTURER_RESP.dwClass)
#define MANUFACTURER_RESP_FUNCTION(x) ((x)->data.MANUFACTURER_RESP.dwFunction)
#define MANUFACTURER_RESP_MANUDATA(x) ((x)->data.MANUFACTURER_RESP.ManuData.ppbyte[0])

/*---------------------------------------------------------------------------*
 * This section is just for compatibility. Don't use anything from
 * here in new software !
 *---------------------------------------------------------------------------*/
#if !defined(CAPI_NO_COMPAT_CODE)

#define __CAPIUTILS_H__ /* no need to include this file */

#define ALERT_CONF_PLCI HEADER_CID
#define ALERT_REQ_PLCI HEADER_CID
#define CONNECT_ACTIVE_IND_PLCI HEADER_CID
#define CONNECT_ACTIVE_RESP_PLCI HEADER_CID
#define CONNECT_B3_ACTIVE_IND_NCCI HEADER_CID
#define CONNECT_B3_ACTIVE_RESP_NCCI HEADER_CID
#define CONNECT_B3_CONF_NCCI HEADER_CID
#define CONNECT_B3_IND_NCCI HEADER_CID
#define CONNECT_B3_REQ_PLCI HEADER_CID
#define CONNECT_B3_RESP_NCCI HEADER_CID
#define CONNECT_B3_T90_ACTIVE_IND_NCCI HEADER_CID
#define CONNECT_B3_T90_ACTIVE_RESP_NCCI HEADER_CID
#define CONNECT_CONF_PLCI HEADER_CID
#define CONNECT_IND_PLCI HEADER_CID
#define CONNECT_REQ_CONTROLLER HEADER_CID
#define CONNECT_RESP_PLCI HEADER_CID
#define DATA_B3_CONF_NCCI HEADER_CID
#define DATA_B3_IND_NCCI HEADER_CID
#define DATA_B3_REQ_NCCI HEADER_CID
#define DATA_B3_RESP_NCCI HEADER_CID
#define DISCONNECT_B3_CONF_NCCI HEADER_CID
#define DISCONNECT_B3_IND_NCCI HEADER_CID
#define DISCONNECT_B3_REQ_NCCI HEADER_CID
#define DISCONNECT_B3_RESP_NCCI HEADER_CID
#define DISCONNECT_CONF_PLCI HEADER_CID
#define DISCONNECT_IND_PLCI HEADER_CID
#define DISCONNECT_REQ_PLCI HEADER_CID
#define DISCONNECT_RESP_PLCI HEADER_CID
#define FACILITY_CONF_CONTROLLER HEADER_CID
#define FACILITY_CONF_NCCI HEADER_CID
#define FACILITY_CONF_PLCI HEADER_CID
#define FACILITY_IND_CONTROLLER HEADER_CID
#define FACILITY_IND_NCCI HEADER_CID
#define FACILITY_IND_PLCI HEADER_CID
#define FACILITY_REQ_CONTROLLER HEADER_CID
#define FACILITY_REQ_NCCI HEADER_CID
#define FACILITY_REQ_PLCI HEADER_CID
#define FACILITY_RESP_CONTROLLER HEADER_CID
#define FACILITY_RESP_NCCI HEADER_CID
#define FACILITY_RESP_PLCI HEADER_CID
#define INFO_CONF_PLCI HEADER_CID
#define INFO_IND_CONTROLLER HEADER_CID
#define INFO_IND_PLCI HEADER_CID
#define INFO_REQ_CONTROLLER HEADER_CID
#define INFO_REQ_PLCI HEADER_CID
#define INFO_RESP_CONTROLLER HEADER_CID
#define INFO_RESP_PLCI HEADER_CID
#define LISTEN_CONF_CONTROLLER HEADER_CID
#define LISTEN_REQ_CONTROLLER HEADER_CID
#define MANUFACTURER_CONF_CONTROLLER HEADER_CID
#define MANUFACTURER_IND_CONTROLLER HEADER_CID
#define MANUFACTURER_REQ_CONTROLLER HEADER_CID
#define MANUFACTURER_RESP_CONTROLLER HEADER_CID
#define RESET_B3_CONF_NCCI HEADER_CID
#define RESET_B3_IND_NCCI HEADER_CID
#define RESET_B3_REQ_NCCI HEADER_CID
#define RESET_B3_RESP_NCCI HEADER_CID
#define SELECT_B_PROTOCOL_CONF_PLCI HEADER_CID
#define SELECT_B_PROTOCOL_REQ_PLCI HEADER_CID

typedef u_int8_t *_cstruct;
typedef u_int8_t *CAPI_MESSAGE;

typedef u_int8_t _cmstruct;
typedef u_int8_t _cbyte;
typedef u_int16_t _cword;
typedef u_int32_t _cdword;
typedef u_int64_t _cqword;

typedef struct capi_message_decoded _cmsg;

#define CapiToManyAppls                         0x1001
#define CapiLogBlkSizeToSmall                   0x1002
#define CapiBuffExeceeds64k                     0x1003
#define CapiMsgBufSizeToSmall                   0x1004
#define CapiAnzLogConnNotSupported              0x1005
#define CapiRegReserved                         0x1006
#define CapiRegBusy                             0x1007
#define CapiRegOSResourceErr                    0x1008
#define CapiRegNotInstalled                     0x1009
#define CapiRegCtrlerNotSupportExtEquip         0x100a
#define CapiRegCtrlerOnlySupportExtEquip        0x100b

#define CapiNoError                             0x0000
#define CapiIllAppNr                            0x1101
#define CapiIllCmdOrSubcmdOrMsgToSmall          0x1102
#define CapiSendQueueFull                       0x1103
#define CapiReceiveQueueEmpty                   0x1104
#define CapiReceiveOverflow                     0x1105
#define CapiUnknownNotPar                       0x1106
#define CapiMsgBusy                             0x1107
#define CapiMsgOSResourceErr                    0x1108
#define CapiMsgNotInstalled                     0x1109
#define CapiMsgCtrlerNotSupportExtEquip         0x110a
#define CapiMsgCtrlerOnlySupportExtEquip        0x110b

#define CAPI_REGISTER_ERROR     unsigned
#define MESSAGE_EXCHANGE_ERROR  unsigned

#define CAPIMSG_BASELEN         8
#define CAPIMSG_U8(x, off)      (x[off])
#define CAPIMSG_U16(x, off)     le16toh(((u_int16_p_t *)&(x[off]))->data)
#define CAPIMSG_U32(x, off)     le32toh(((u_int32_p_t *)&(x[off]))->data)
#define CAPIMSG_LEN(m)          CAPIMSG_U16(m,0)
#define CAPIMSG_APPID(m)        CAPIMSG_U16(m,2)
#define CAPIMSG_COMMAND(m)      CAPIMSG_U8(m,4)
#define CAPIMSG_SUBCOMMAND(m)   CAPIMSG_U8(m,5)
#define CAPIMSG_CMD(m)          (((m[4])<<8)|(m[5]))
#define CAPIMSG_MSGID(m)        CAPIMSG_U16(m,6)
#define CAPIMSG_CONTROLLER(m)   (m[8] & 0x7f)
#define CAPIMSG_CONTROL(m)      CAPIMSG_U32(m, 8)
#define CAPIMSG_NCCI(m)         CAPIMSG_CONTROL(m)
#define CAPIMSG_DATALEN(m)      CAPIMSG_U16(m,16)

static __inline void capimsg_setu8(void *m, u_int32_t off, u_int8_t val)
{
	((u_int8_t *)ADD_BYTES(m, off))[0] = val;
}

static __inline void capimsg_setu16(void *m, u_int32_t off, u_int16_t val)
{
	((u_int16_p_t *)ADD_BYTES(m, off))->data = htole16(val);
}

static __inline void capimsg_setu32(void *m, u_int32_t off, u_int32_t val)
{
	((u_int32_p_t *)ADD_BYTES(m, off))->data = htole32(val);
}

#define CAPIMSG_SETLEN(m, len)          capimsg_setu16(m, 0, len)
#define CAPIMSG_SETAPPID(m, applid)     capimsg_setu16(m, 2, applid)
#define CAPIMSG_SETCOMMAND(m,cmd)       capimsg_setu8(m, 4, cmd)
#define CAPIMSG_SETSUBCOMMAND(m, cmd)   capimsg_setu8(m, 5, cmd)
#define CAPIMSG_SETMSGID(m, msgid)      capimsg_setu16(m, 6, msgid)
#define CAPIMSG_SETCONTROL(m, contr)    capimsg_setu32(m, 8, contr)
#define CAPIMSG_SETDATALEN(m, len)      capimsg_setu16(m, 16, len)

#define IS_ALERT_IND(x)                 (((x)->head.wCmd == CAPI_IND(ALERT)) || \
					 ((x)->head.wCmd == CAPI_P_IND(ALERT)))
#define IS_ALERT_CONF(x)                (((x)->head.wCmd == CAPI_CONF(ALERT)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(ALERT)))
#define IS_CONNECT_IND(x)               (((x)->head.wCmd == CAPI_IND(CONNECT)) || \
					 ((x)->head.wCmd == CAPI_P_IND(CONNECT)))
#define IS_CONNECT_CONF(x)              (((x)->head.wCmd == CAPI_CONF(CONNECT)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(CONNECT)))
#define IS_CONNECT_ACTIVE_IND(x)        (((x)->head.wCmd == CAPI_IND(CONNECT_ACTIVE)) || \
					 ((x)->head.wCmd == CAPI_P_IND(CONNECT_ACTIVE)))
#define IS_CONNECT_ACTIVE_CONF(x)       (((x)->head.wCmd == CAPI_CONF(CONNECT_ACTIVE)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(CONNECT_ACTIVE)))
#define IS_DISCONNECT_IND(x)            (((x)->head.wCmd == CAPI_IND(DISCONNECT)) || \
					 ((x)->head.wCmd == CAPI_P_IND(DISCONNECT)))
#define IS_DISCONNECT_CONF(x)           (((x)->head.wCmd == CAPI_CONF(DISCONNECT)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(DISCONNECT)))
#define IS_LISTEN_IND(x)                (((x)->head.wCmd == CAPI_IND(LISTEN)) || \
					 ((x)->head.wCmd == CAPI_P_IND(LISTEN)))
#define IS_LISTEN_CONF(x)               (((x)->head.wCmd == CAPI_CONF(LISTEN)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(LISTEN)))
#define IS_INFO_IND(x)                  (((x)->head.wCmd == CAPI_IND(INFO)) || \
					 ((x)->head.wCmd == CAPI_P_IND(INFO)))
#define IS_INFO_CONF(x)                 (((x)->head.wCmd == CAPI_CONF(INFO)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(INFO)))
#define IS_SELECT_B_PROTOCOL_IND(x)     (((x)->head.wCmd == CAPI_IND(SELECT_B_PROTOCOL)) || \
					 ((x)->head.wCmd == CAPI_P_IND(SELECT_B_PROTOCOL)))
#define IS_SELECT_B_PROTOCOL_CONF(x)    (((x)->head.wCmd == CAPI_CONF(SELECT_B_PROTOCOL)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(SELECT_B_PROTOCOL)))
#define IS_FACILITY_IND(x)              (((x)->head.wCmd == CAPI_IND(FACILITY)) || \
					 ((x)->head.wCmd == CAPI_P_IND(FACILITY)))
#define IS_FACILITY_CONF(x)             (((x)->head.wCmd == CAPI_CONF(FACILITY)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(FACILITY)))
#define IS_CONNECT_B3_IND(x)            (((x)->head.wCmd == CAPI_IND(CONNECT_B3)) || \
					 ((x)->head.wCmd == CAPI_P_IND(CONNECT_B3)))
#define IS_CONNECT_B3_CONF(x)           (((x)->head.wCmd == CAPI_CONF(CONNECT_B3)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(CONNECT_B3)))
#define IS_CONNECT_B3_ACTIVE_IND(x)     (((x)->head.wCmd == CAPI_IND(CONNECT_B3_ACTIVE)) || \
					 ((x)->head.wCmd == CAPI_P_IND(CONNECT_B3_ACTIVE)))
#define IS_CONNECT_B3_ACTIVE_CONF(x)    (((x)->head.wCmd == CAPI_CONF(CONNECT_B3_ACTIVE)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(CONNECT_B3_ACTIVE)))
#define IS_DISCONNECT_B3_IND(x)         (((x)->head.wCmd == CAPI_IND(DISCONNECT_B3)) || \
					 ((x)->head.wCmd == CAPI_P_IND(DISCONNECT_B3)))
#define IS_DISCONNECT_B3_CONF(x)        (((x)->head.wCmd == CAPI_CONF(DISCONNECT_B3)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(DISCONNECT_B3)))
#define IS_DATA_B3_IND(x)               (((x)->head.wCmd == CAPI_IND(DATA_B3)) || \
					 ((x)->head.wCmd == CAPI_P_IND(DATA_B3)))
#define IS_DATA_B3_CONF(x)              (((x)->head.wCmd == CAPI_CONF(DATA_B3)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(DATA_B3)))
#define IS_RESET_B3_IND(x)              (((x)->head.wCmd == CAPI_IND(RESET_B3)) || \
					 ((x)->head.wCmd == CAPI_P_IND(RESET_B3)))
#define IS_RESET_B3_CONF(x)             (((x)->head.wCmd == CAPI_CONF(RESET_B3)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(RESET_B3)))
#define IS_CONNECT_B3_T90_ACTIVE_IND(x) (((x)->head.wCmd == CAPI_IND(CONNECT_B3_T90_ACTIVE)) || \
					 ((x)->head.wCmd == CAPI_P_IND(CONNECT_B3_T90_ACTIVE)))
#define IS_CONNECT_B3_T90_ACTIVE_CONF(x)(((x)->head.wCmd == CAPI_CONF(CONNECT_B3_T90_ACTIVE)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(CONNECT_B3_T90_ACTIVE)))
#define IS_MANUFACTURER_IND(x)          (((x)->head.wCmd == CAPI_IND(MANUFACTURER)) || \
					 ((x)->head.wCmd == CAPI_P_IND(MANUFACTURER)))
#define IS_MANUFACTURER_CONF(x)         (((x)->head.wCmd == CAPI_CONF(MANUFACTURER)) || \
					 ((x)->head.wCmd == CAPI_P_CONF(MANUFACTURER)))

#define ALERT_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(ALERT),wNum,dwCid)
#define ALERT_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(ALERT),wNum,dwCid)
#define CONNECT_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(CONNECT),wNum,dwCid)
#define CONNECT_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(CONNECT),wNum,dwCid)
#define CONNECT_ACTIVE_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(CONNECT_ACTIVE),wNum,dwCid)
#define CONNECT_ACTIVE_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(CONNECT_ACTIVE),wNum,dwCid)
#define DISCONNECT_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(DISCONNECT),wNum,dwCid)
#define DISCONNECT_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(DISCONNECT),wNum,dwCid)
#define LISTEN_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(LISTEN),wNum,dwCid)
#define LISTEN_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(LISTEN),wNum,dwCid)
#define INFO_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(INFO),wNum,dwCid)
#define INFO_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(INFO),wNum,dwCid)
#define SELECT_B_PROTOCOL_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(SELECT_B_PROTOCOL),wNum,dwCid)
#define SELECT_B_PROTOCOL_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(SELECT_B_PROTOCOL),wNum,dwCid)
#define FACILITY_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(FACILITY),wNum,dwCid)
#define FACILITY_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(FACILITY),wNum,dwCid)
#define CONNECT_B3_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(CONNECT_B3),wNum,dwCid)
#define CONNECT_B3_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(CONNECT_B3),wNum,dwCid)
#define CONNECT_B3_ACTIVE_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(CONNECT_B3_ACTIVE),wNum,dwCid)
#define CONNECT_B3_ACTIVE_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(CONNECT_B3_ACTIVE),wNum,dwCid)
#define DISCONNECT_B3_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(DISCONNECT_B3),wNum,dwCid)
#define DISCONNECT_B3_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(DISCONNECT_B3),wNum,dwCid)
#define DATA_B3_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(DATA_B3),wNum,dwCid)
#define DATA_B3_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(DATA_B3),wNum,dwCid)
#define RESET_B3_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(RESET_B3),wNum,dwCid)
#define RESET_B3_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(RESET_B3),wNum,dwCid)
#define CONNECT_B3_T90_ACTIVE_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(CONNECT_B3_T90_ACTIVE),wNum,dwCid)
#define CONNECT_B3_T90_ACTIVE_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(CONNECT_B3_T90_ACTIVE),wNum,dwCid)
#define MANUFACTURER_REQ_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_REQ(MANUFACTURER),wNum,dwCid)
#define MANUFACTURER_RESP_HEADER(mp,wApp,wNum,dwCid) capi_message_decoded_header(mp,wApp,CAPI_P_RESP(MANUFACTURER),wNum,dwCid)

#endif

#if !(defined(_KERNEL) || defined(CAPI_NO_FILL_FUNCTIONS) || \
      defined(CAPI_NO_COMPAT_CODE))

/*
 * NOTE: REQ and RESP version are listed for all commands, 
 * even if some are not used !
 */
static __inline u_int16_t ALERT_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(ALERT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#ifndef CAPI_LIBRARY_V2
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_ALERT_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(ALERT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#ifndef CAPI_LIBRARY_V2
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return;
}

static __inline u_int16_t ALERT_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(ALERT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_ALERT_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(ALERT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t CONNECT_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wCIP
  , u_int8_t *dst_telno
  , u_int8_t *src_telno
  , u_int8_t *dst_subaddr
  , u_int8_t *src_subaddr
  , u_int16_t wB1_protocol
  , u_int16_t wB2_protocol
  , u_int16_t wB3_protocol
  , u_int8_t *B1_config
  , u_int8_t *B2_config
  , u_int8_t *B3_config
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *global_config
#endif
  , u_int8_t *BC
  , u_int8_t *LLC
  , u_int8_t *HLC
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_REQ.wCIP = wCIP;
  mp->data.CONNECT_REQ.dst_telno.ptr = dst_telno;
  mp->data.CONNECT_REQ.src_telno.ptr = src_telno;
  mp->data.CONNECT_REQ.dst_subaddr.ptr = dst_subaddr;
  mp->data.CONNECT_REQ.src_subaddr.ptr = src_subaddr;
  mp->B_PROTOCOL.wB1_protocol = wB1_protocol;
  mp->B_PROTOCOL.wB2_protocol = wB2_protocol;
  mp->B_PROTOCOL.wB3_protocol = wB3_protocol;
  mp->B_PROTOCOL.B1_config.ptr = B1_config;
  mp->B_PROTOCOL.B2_config.ptr = B2_config;
  mp->B_PROTOCOL.B3_config.ptr = B3_config;
#ifndef CAPI_LIBRARY_V2
  mp->B_PROTOCOL.global_config.ptr = global_config;
#endif
  mp->data.CONNECT_REQ.BC.ptr = BC;
  mp->data.CONNECT_REQ.LLC.ptr = LLC;
  mp->data.CONNECT_REQ.HLC.ptr = HLC;
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#ifndef CAPI_LIBRARY_V2
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wCIP
  , u_int8_t *dst_telno
  , u_int8_t *src_telno
  , u_int8_t *dst_subaddr
  , u_int8_t *src_subaddr
  , u_int16_t wB1_protocol
  , u_int16_t wB2_protocol
  , u_int16_t wB3_protocol
  , u_int8_t *B1_config
  , u_int8_t *B2_config
  , u_int8_t *B3_config
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *global_config
#endif
  , u_int8_t *BC
  , u_int8_t *LLC
  , u_int8_t *HLC
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_REQ.wCIP = wCIP;
  mp->data.CONNECT_REQ.dst_telno.ptr = dst_telno;
  mp->data.CONNECT_REQ.src_telno.ptr = src_telno;
  mp->data.CONNECT_REQ.dst_subaddr.ptr = dst_subaddr;
  mp->data.CONNECT_REQ.src_subaddr.ptr = src_subaddr;
  mp->B_PROTOCOL.wB1_protocol = wB1_protocol;
  mp->B_PROTOCOL.wB2_protocol = wB2_protocol;
  mp->B_PROTOCOL.wB3_protocol = wB3_protocol;
  mp->B_PROTOCOL.B1_config.ptr = B1_config;
  mp->B_PROTOCOL.B2_config.ptr = B2_config;
  mp->B_PROTOCOL.B3_config.ptr = B3_config;
#ifndef CAPI_LIBRARY_V2
  mp->B_PROTOCOL.global_config.ptr = global_config;
#endif
  mp->data.CONNECT_REQ.BC.ptr = BC;
  mp->data.CONNECT_REQ.LLC.ptr = LLC;
  mp->data.CONNECT_REQ.HLC.ptr = HLC;
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#ifndef CAPI_LIBRARY_V2
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return;
}

static __inline u_int16_t CONNECT_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wReject
  , u_int16_t wB1_protocol
  , u_int16_t wB2_protocol
  , u_int16_t wB3_protocol
  , u_int8_t *B1_config
  , u_int8_t *B2_config
  , u_int8_t *B3_config
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *global_config
#endif
  , u_int8_t *dst_telno
  , u_int8_t *dst_subaddr
  , u_int8_t *LLC
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#if 0 
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_RESP.wReject = wReject;
  mp->B_PROTOCOL.wB1_protocol = wB1_protocol;
  mp->B_PROTOCOL.wB2_protocol = wB2_protocol;
  mp->B_PROTOCOL.wB3_protocol = wB3_protocol;
  mp->B_PROTOCOL.B1_config.ptr = B1_config;
  mp->B_PROTOCOL.B2_config.ptr = B2_config;
  mp->B_PROTOCOL.B3_config.ptr = B3_config;
#ifndef CAPI_LIBRARY_V2
  mp->B_PROTOCOL.global_config.ptr = global_config;
#endif
  mp->data.CONNECT_RESP.dst_telno.ptr = dst_telno;
  mp->data.CONNECT_RESP.dst_subaddr.ptr = dst_subaddr;
  mp->data.CONNECT_RESP.LLC.ptr = LLC;
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#if 0
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wReject
  , u_int16_t wB1_protocol
  , u_int16_t wB2_protocol
  , u_int16_t wB3_protocol
  , u_int8_t *B1_config
  , u_int8_t *B2_config
  , u_int8_t *B3_config
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *global_config
#endif
  , u_int8_t *dst_telno
  , u_int8_t *dst_subaddr
  , u_int8_t *LLC
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#if 0
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_RESP.wReject = wReject;
  mp->B_PROTOCOL.wB1_protocol = wB1_protocol;
  mp->B_PROTOCOL.wB2_protocol = wB2_protocol;
  mp->B_PROTOCOL.wB3_protocol = wB3_protocol;
  mp->B_PROTOCOL.B1_config.ptr = B1_config;
  mp->B_PROTOCOL.B2_config.ptr = B2_config;
  mp->B_PROTOCOL.B3_config.ptr = B3_config;
#ifndef CAPI_LIBRARY_V2
  mp->B_PROTOCOL.global_config.ptr = global_config;
#endif
  mp->data.CONNECT_RESP.dst_telno.ptr = dst_telno;
  mp->data.CONNECT_RESP.dst_subaddr.ptr = dst_subaddr;
  mp->data.CONNECT_RESP.LLC.ptr = LLC;
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#if 0
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return;
}

static __inline u_int16_t CONNECT_ACTIVE_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_ACTIVE_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t CONNECT_ACTIVE_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_ACTIVE_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t DISCONNECT_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#if 0
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(DISCONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#if 0
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_DISCONNECT_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#if 0
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(DISCONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#if 0
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return;
}

static __inline u_int16_t DISCONNECT_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(DISCONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_DISCONNECT_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(DISCONNECT);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t LISTEN_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int32_t dwInfoMask
  , u_int32_t dwCipMask1
  , u_int32_t dwCipMask2
  , u_int8_t *src_telno
  , u_int8_t *src_subaddr
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(LISTEN);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.LISTEN_REQ.dwInfoMask = dwInfoMask;
  mp->data.LISTEN_REQ.dwCipMask1 = dwCipMask1;
  mp->data.LISTEN_REQ.dwCipMask2 = dwCipMask2;
  mp->data.LISTEN_REQ.src_telno.ptr = src_telno;
  mp->data.LISTEN_REQ.src_subaddr.ptr = src_subaddr;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_LISTEN_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int32_t dwInfoMask
  , u_int32_t dwCipMask1
  , u_int32_t dwCipMask2
  , u_int8_t *src_telno
  , u_int8_t *src_subaddr
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(LISTEN);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.LISTEN_REQ.dwInfoMask = dwInfoMask;
  mp->data.LISTEN_REQ.dwCipMask1 = dwCipMask1;
  mp->data.LISTEN_REQ.dwCipMask2 = dwCipMask2;
  mp->data.LISTEN_REQ.src_telno.ptr = src_telno;
  mp->data.LISTEN_REQ.src_subaddr.ptr = src_subaddr;
  
  return;
}

static __inline u_int16_t LISTEN_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(LISTEN);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_LISTEN_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(LISTEN);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t INFO_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *dst_telno
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(INFO);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.INFO_REQ.dst_telno.ptr = dst_telno;
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#ifndef CAPI_LIBRARY_V2
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_INFO_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *dst_telno
  , u_int8_t *b_channel_info
  , u_int8_t *keypad
  , u_int8_t *useruser
  , u_int8_t *facility
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *sending_complete
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(INFO);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.INFO_REQ.dst_telno.ptr = dst_telno;
  mp->ADDITIONAL_INFO.b_channel_info.ptr = b_channel_info;
  mp->ADDITIONAL_INFO.keypad.ptr = keypad;
  mp->ADDITIONAL_INFO.useruser.ptr = useruser;
  mp->ADDITIONAL_INFO.facility.ptr = facility;
#ifndef CAPI_LIBRARY_V2
  mp->ADDITIONAL_INFO.sending_complete.ptr = sending_complete;
#endif
  
  return;
}

static __inline u_int16_t INFO_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(INFO);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_INFO_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(INFO);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t SELECT_B_PROTOCOL_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wB1_protocol
  , u_int16_t wB2_protocol
  , u_int16_t wB3_protocol
  , u_int8_t *B1_config
  , u_int8_t *B2_config
  , u_int8_t *B3_config
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *global_config
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(SELECT_B_PROTOCOL);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->B_PROTOCOL.wB1_protocol = wB1_protocol;
  mp->B_PROTOCOL.wB2_protocol = wB2_protocol;
  mp->B_PROTOCOL.wB3_protocol = wB3_protocol;
  mp->B_PROTOCOL.B1_config.ptr = B1_config;
  mp->B_PROTOCOL.B2_config.ptr = B2_config;
  mp->B_PROTOCOL.B3_config.ptr = B3_config;
#ifndef CAPI_LIBRARY_V2
  mp->B_PROTOCOL.global_config.ptr = global_config;
#endif
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_SELECT_B_PROTOCOL_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wB1_protocol
  , u_int16_t wB2_protocol
  , u_int16_t wB3_protocol
  , u_int8_t *B1_config
  , u_int8_t *B2_config
  , u_int8_t *B3_config
#ifndef CAPI_LIBRARY_V2
  , u_int8_t *global_config
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(SELECT_B_PROTOCOL);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->B_PROTOCOL.wB1_protocol = wB1_protocol;
  mp->B_PROTOCOL.wB2_protocol = wB2_protocol;
  mp->B_PROTOCOL.wB3_protocol = wB3_protocol;
  mp->B_PROTOCOL.B1_config.ptr = B1_config;
  mp->B_PROTOCOL.B2_config.ptr = B2_config;
  mp->B_PROTOCOL.B3_config.ptr = B3_config;
#ifndef CAPI_LIBRARY_V2
  mp->B_PROTOCOL.global_config.ptr = global_config;
#endif
  
  return;
}

static __inline u_int16_t SELECT_B_PROTOCOL_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(SELECT_B_PROTOCOL);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_SELECT_B_PROTOCOL_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(SELECT_B_PROTOCOL);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t FACILITY_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wSelector
  , u_int8_t *Param
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(FACILITY);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.FACILITY_REQ.wSelector = wSelector;
  mp->data.FACILITY_REQ.Param.ptr = Param;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_FACILITY_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wSelector
  , u_int8_t *Param
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(FACILITY);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.FACILITY_REQ.wSelector = wSelector;
  mp->data.FACILITY_REQ.Param.ptr = Param;
  
  return;
}

static __inline u_int16_t FACILITY_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wSelector
  , u_int8_t *Param
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(FACILITY);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.FACILITY_RESP.wSelector = wSelector;
  mp->data.FACILITY_RESP.Param.ptr = Param;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_FACILITY_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wSelector
#if 0
  , u_int8_t *Param
#endif
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(FACILITY);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.FACILITY_RESP.wSelector = wSelector;
#if 0
  mp->data.FACILITY_RESP.Param.ptr = Param;
#endif
  
  return;
}

static __inline u_int16_t CONNECT_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_B3_REQ.NCPI.ptr = NCPI;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_B3_REQ.NCPI.ptr = NCPI;
  
  return;
}

static __inline u_int16_t CONNECT_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wReject
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_B3_RESP.wReject = wReject;
  mp->data.CONNECT_B3_RESP.NCPI.ptr = NCPI;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wReject
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.CONNECT_B3_RESP.wReject = wReject;
  mp->data.CONNECT_B3_RESP.NCPI.ptr = NCPI;
  
  return;
}

static __inline u_int16_t CONNECT_B3_ACTIVE_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_B3_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_B3_ACTIVE_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_B3_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t CONNECT_B3_ACTIVE_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_B3_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_B3_ACTIVE_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_B3_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t DISCONNECT_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(DISCONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.DISCONNECT_B3_REQ.NCPI.ptr = NCPI;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_DISCONNECT_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(DISCONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.DISCONNECT_B3_REQ.NCPI.ptr = NCPI;
  
  return;
}

static __inline u_int16_t DISCONNECT_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(DISCONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_DISCONNECT_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(DISCONNECT_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t DATA_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , void *Ptr
  , u_int16_t wLen
  , u_int16_t wHandle
  , u_int16_t wFlags
)
{
#if 0
  /* set default value */
  bzero(mp, sizeof(*mp));
#else
#if ((CAPI_HEADER(1+ NO,)(0)) != 5)
#error "This code needs to be updated!"
#endif
#if ((CAPI_DATA_B3_REQ(1+ NO,)(0)) != 5)
#error "This code needs to be updated!"
#endif
#endif
 
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(DATA_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  DATA_B3_REQ_DATA(mp) = (u_int8_t *)Ptr;
  mp->data.DATA_B3_REQ.wLen = wLen;
  mp->data.DATA_B3_REQ.wHandle = wHandle;
  mp->data.DATA_B3_REQ.wFlags = wFlags;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_DATA_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , void *Ptr
  , u_int16_t wLen
  , u_int16_t wHandle
  , u_int16_t wFlags
)
{
#if 0
  /* set default value */
  bzero(mp, sizeof(*mp));
#else
#if ((CAPI_HEADER(1+ NO,)(0)) != 5)
#error "This code needs to be updated!"
#endif
#if ((CAPI_DATA_B3_REQ(1+ NO,)(0)) != 5)
#error "This code needs to be updated!"
#endif
#endif
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(DATA_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  DATA_B3_REQ_DATA(mp) = (u_int8_t *)Ptr;
  mp->data.DATA_B3_REQ.wLen = wLen;
  mp->data.DATA_B3_REQ.wHandle = wHandle;
  mp->data.DATA_B3_REQ.wFlags = wFlags;
  
  return;
}

static __inline u_int16_t DATA_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wHandle
)
{
#if 0
  /* set default value */
  bzero(mp, sizeof(*mp));
#else
#if ((CAPI_HEADER(1+ NO,)(0)) != 5)
#error "This code needs to be updated!"
#endif
#if ((CAPI_DATA_B3_RESP(1+ NO,)(0)) != 1)
#error "This code needs to be updated!"
#endif
#endif
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(DATA_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.DATA_B3_RESP.wHandle = wHandle;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_DATA_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int16_t wHandle
)
{
#if 0
  /* set default value */
  bzero(mp, sizeof(*mp));
#else
#if ((CAPI_HEADER(1+ NO,)(0)) != 5)
#error "This code needs to be updated!"
#endif
#if ((CAPI_DATA_B3_RESP(1+ NO,)(0)) != 1)
#error "This code needs to be updated!"
#endif
#endif
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(DATA_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.DATA_B3_RESP.wHandle = wHandle;
  
  return;
}

static __inline u_int16_t RESET_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(RESET_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.RESET_B3_REQ.NCPI.ptr = NCPI;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_RESET_B3_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int8_t *NCPI
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(RESET_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.RESET_B3_REQ.NCPI.ptr = NCPI;
  
  return;
}

static __inline u_int16_t RESET_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(RESET_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_RESET_B3_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(RESET_B3);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t CONNECT_B3_T90_ACTIVE_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_B3_T90_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_B3_T90_ACTIVE_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(CONNECT_B3_T90_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t CONNECT_B3_T90_ACTIVE_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_B3_T90_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_CONNECT_B3_T90_ACTIVE_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(CONNECT_B3_T90_ACTIVE);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  
  return;
}

static __inline u_int16_t MANUFACTURER_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int32_t dwManuID
  , u_int32_t dwClass
  , u_int32_t dwFunction
  , u_int8_t *ManuData
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(MANUFACTURER);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.MANUFACTURER_REQ.dwManuID = dwManuID;
  mp->data.MANUFACTURER_REQ.dwClass = dwClass;
  mp->data.MANUFACTURER_REQ.dwFunction = dwFunction;
  mp->data.MANUFACTURER_REQ.ManuData.ptr = ManuData;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_MANUFACTURER_REQ
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int32_t dwManuID
  , u_int32_t dwClass
  , u_int32_t dwFunction
  , u_int8_t *ManuData
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_REQ(MANUFACTURER);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.MANUFACTURER_REQ.dwManuID = dwManuID;
  mp->data.MANUFACTURER_REQ.dwClass = dwClass;
  mp->data.MANUFACTURER_REQ.dwFunction = dwFunction;
  mp->data.MANUFACTURER_REQ.ManuData.ptr = ManuData;
  
  return;
}

static __inline u_int16_t MANUFACTURER_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int32_t dwManuID
  , u_int32_t dwClass
  , u_int32_t dwFunction
  , u_int8_t *ManuData
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(MANUFACTURER);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.MANUFACTURER_RESP.dwManuID = dwManuID;
  mp->data.MANUFACTURER_RESP.dwClass = dwClass;
  mp->data.MANUFACTURER_RESP.dwFunction = dwFunction;
  mp->data.MANUFACTURER_RESP.ManuData.ptr = ManuData;
  
  return capi_put_message_decoded(mp);
}

static __inline void capi_fill_MANUFACTURER_RESP
(
  struct capi_message_decoded *mp
  , u_int16_t wApp
  , u_int16_t wNum
  , u_int32_t dwCid
  , u_int32_t dwManuID
  , u_int32_t dwClass
  , u_int32_t dwFunction
  , u_int8_t *ManuData
)
{
  /* set default value */
  bzero(mp, sizeof(*mp));
  
  /* fill out header */
  mp->head.wApp = wApp;
  mp->head.wCmd = CAPI_P_RESP(MANUFACTURER);
  mp->head.wNum = wNum;
  mp->head.dwCid = dwCid;
  
  /* fill out data */
  mp->data.MANUFACTURER_RESP.dwManuID = dwManuID;
  mp->data.MANUFACTURER_RESP.dwClass = dwClass;
  mp->data.MANUFACTURER_RESP.dwFunction = dwFunction;
  mp->data.MANUFACTURER_RESP.ManuData.ptr = ManuData;
  
  return;
}

#endif

#if !defined(CAPI_NO_CAUSES)

/* NOTE: error 0x0000 is undefined and 
 * means no error or no cause.
 */

#define CAPI_MAKE_ERRORS(n, ENUM, value) \
enum { ENUM = (value) };

#define CAPI_MAKE_ERROR_DESC(n, ENUM, value) \
  { (value), #value ": " #ENUM },

#define CAPI_ERRORS(m,n) \
/*m(n, enum                                   , value )*  \
 *m(n,----------------------------------------,-------)*/ \
\
  /* class 0x00xx - CAPI information values */\
  m(n, CAPI_ERROR_NCPI_IGNORED                , 0x0001)\
  m(n, CAPI_ERROR_FLAGS_IGNORED               , 0x0002)\
  m(n, CAPI_ERROR_ALERT_ALREADY_SENT          , 0x0003)\
\
  /* class 0x10xx - CAPI register error */\
  m(n, CAPI_ERROR_TOO_MANY_APPLICATIONS       , 0x1001)\
  m(n, CAPI_ERROR_INVALID_BLOCK_SIZE          , 0x1002)\
  m(n, CAPI_ERROR_INVALID_BUFFER_SIZE         , 0x1003)\
  m(n, CAPI_ERROR_INVALID_MSG_BUFFER          , 0x1004)\
  m(n, CAPI_ERROR_INVALID_CONNECT_NUM         , 0x1005)\
  m(n, CAPI_ERROR_RESERVED_1006               , 0x1006)\
  m(n, CAPI_ERROR_BUSY                        , 0x1007)\
  m(n, CAPI_ERROR_OS_RESOURCE_ERROR           , 0x1008)\
  m(n, CAPI_ERROR_CAPI_NOT_INSTALLED          , 0x1009)\
  m(n, CAPI_ERROR_NO_EXTERNAL_EQUIPMENT       , 0x100A)\
  m(n, CAPI_ERROR_NO_INTERNAL_EQUIPMENT       , 0x100B)\
\
  /* class 0x10xx - additional CAPI register errors (specific to BSD) */\
  m(n, CAPI_ERROR_INVALID_PARAM               , 0x10F0)\
  m(n, CAPI_ERROR_UNSUPPORTED_VERSION         , 0x10F1)\
  m(n, CAPI_ERROR_TOO_MANY_CONTROLLERS        , 0x10F2)\
  m(n, CAPI_ERROR_INVALID_CONTROLLER          , 0x10F3)\
  m(n, CAPI_ERROR_RESET_NOT_SUPPORTED         , 0x10F4)\
  m(n, CAPI_ERROR_NO_RIGHTS_TO_RESET          , 0x10F5)\
\
  /* class 0x11xx - CAPI message exchange error */\
  m(n, CAPI_ERROR_INVALID_APPLICATION_ID      , 0x1101)\
  m(n, CAPI_ERROR_ILLEGAL_COMMAND             , 0x1102)\
  m(n, CAPI_ERROR_PUT_QUEUE_FULL              , 0x1103)\
  m(n, CAPI_ERROR_GET_QUEUE_EMPTY             , 0x1104)\
  m(n, CAPI_ERROR_GET_QUEUE_OVERFLOW          , 0x1105)\
  m(n, CAPI_ERROR_UNKNOWN_NOTIFICATION_PARAM  , 0x1106)\
  m(n, CAPI_ERROR_BUSY_2                      , 0x1107)\
  m(n, CAPI_ERROR_OS_RESOURCE_ERROR_2         , 0x1108)\
  m(n, CAPI_ERROR_CAPI_NOT_INSTALLED_2        , 0x1109)\
  m(n, CAPI_ERROR_NO_EXTERNAL_EQUIPMENT_2     , 0x110A)\
  m(n, CAPI_ERROR_NO_INTERNAL_EQUIPMENT_2     , 0x110B)\
\
  /* class 0x11xx - additional CAPI message exchange errors */\
  m(n, CAPI_ERROR_INVALID_PARAM_2             , 0x11F0)\
\
  /* class 0x20xx - CAPI coding error */\
  m(n, CAPI_ERROR_MSG_NOT_ALLOWED_YET         , 0x2001) /* wrong expression in identifier */\
  m(n, CAPI_ERROR_MSG_NOT_ALLOWED_NOW         , 0x2001)\
  m(n, CAPI_ERROR_ILLEGAL_IDENTIFIER          , 0x2002)\
  m(n, CAPI_ERROR_NO_PLCI_AVAILABLE           , 0x2003)\
  m(n, CAPI_ERROR_NO_NCCI_AVAILABLE           , 0x2004)\
  m(n, CAPI_ERROR_NO_LISTEN_AVAILABLE         , 0x2005)\
  m(n, CAPI_ERROR_NO_FAX_RESOURCE_AVAILABLE   , 0x2006)\
  m(n, CAPI_ERROR_ILLEGAL_MSG_PARAMETER       , 0x2007)\
  m(n, CAPI_ERROR_NO_INTERCONNECT_RESOURCES_AVAILABLE , 0x2008)\
\
  /* class 0x30xx - CAPI service request error */\
  m(n, CAPI_ERROR_B1_PROTOCOL_NOT_SUPPORTED   , 0x3001)\
  m(n, CAPI_ERROR_B2_PROTOCOL_NOT_SUPPORTED   , 0x3002)\
  m(n, CAPI_ERROR_B3_PROTOCOL_NOT_SUPPORTED   , 0x3003)\
  m(n, CAPI_ERROR_INVALID_B1_PARAMETER        , 0x3004)\
  m(n, CAPI_ERROR_INVALID_B2_PARAMETER        , 0x3005)\
  m(n, CAPI_ERROR_INVALID_B3_PARAMETER        , 0x3006)\
  m(n, CAPI_ERROR_INVALID_B_PROT_COMBINATION  , 0x3007)\
  m(n, CAPI_ERROR_NCPI_NOT_SUPPORTED          , 0x3008)\
  m(n, CAPI_ERROR_UNKNOWN_CIP_VALUE           , 0x3009)\
  m(n, CAPI_ERROR_FLAGS_NOT_SUPPORTED         , 0x300A)\
  m(n, CAPI_ERROR_FACILITY_NOT_SUPPORTED      , 0x300B)\
  m(n, CAPI_ERROR_DATA_LENGTH_NOT_SUPPORTED   , 0x300C)\
  m(n, CAPI_ERROR_RESET_B3_NOT_SUPPORTED      , 0x300D)\
  m(n, CAPI_ERROR_SUPPL_SERVICE_NOT_SUPPORTED , 0x300E)\
  m(n, CAPI_ERROR_UNSUPPORTED_INTEROPERABILITY, 0x300F)\
  m(n, CAPI_ERROR_SUPPL_NOT_ALLOWED_YET       , 0x3010)\
  m(n, CAPI_ERROR_FACILITY_FUNC_NOT_SUPPORTED , 0x3011)\
\
  /* disconnect and disconnect B3 reason */\
  m(n, CAPI_ERROR_PROTOCOL_LAYER_1            , 0x3301)\
  m(n, CAPI_ERROR_PROTOCOL_LAYER_2            , 0x3302)\
  m(n, CAPI_ERROR_PROTOCOL_LAYER_3            , 0x3303)\
  m(n, CAPI_ERROR_CALL_TO_OTHER_APPLICATION   , 0x3304)\
  m(n, CAPI_ERROR_SUPPL_REJECT_BY_SUPERVISION , 0x3305)\
\
  /* T.30 specific reasons */\
  m(n, CAPI_ERROR_B3_NO_FAX_G3_REMOTE         , 0x3311)\
  m(n, CAPI_ERROR_B3_TRAINING_ERROR           , 0x3312)\
  m(n, CAPI_ERROR_B3_MODE_SELECT_ERROR        , 0x3313)\
  m(n, CAPI_ERROR_B3_REMOTE_ABORT             , 0x3314)\
  m(n, CAPI_ERROR_B3_REMOTE_PROCEDURE_ERROR   , 0x3315)\
  m(n, CAPI_ERROR_B3_TX_DATA_UNDERRUN         , 0x3316)\
  m(n, CAPI_ERROR_B3_RX_DATA_OVERFLOW         , 0x3317)\
  m(n, CAPI_ERROR_B3_LOCAL_ABORT              , 0x3318)\
  m(n, CAPI_ERROR_B3_ILLEGAL_PARAM_CODING     , 0x3319)\
\
  /* modem specific causes */\
  m(n, CAPI_ERROR_B3_NORMAL                   , 0x3500)\
  m(n, CAPI_ERROR_B3_CARRIER_LOST             , 0x3501)\
  m(n, CAPI_ERROR_B3_NEGOTIATION_ERROR        , 0x3502)\
  m(n, CAPI_ERROR_B3_NO_ANSWER                , 0x3503)\
  m(n, CAPI_ERROR_B3_ONLY_SYNCH_MODE          , 0x3504)\
  m(n, CAPI_ERROR_B3_FRAMING_ERROR            , 0x3505)\
  m(n, CAPI_ERROR_B3_PROTOCOL_NEGOTIATION     , 0x3506)\
  m(n, CAPI_ERROR_B3_REMOTE_PROTOCOL_ERROR    , 0x3507)\
  m(n, CAPI_ERROR_B3_SYNCH_MISSING            , 0x3508)\
  m(n, CAPI_ERROR_B3_REMOTE_DISCONNECT        , 0x3509)\
  m(n, CAPI_ERROR_B3_MODEM_NO_ANSWER          , 0x350A)\
  m(n, CAPI_ERROR_B3_PROTOCOL_ERROR           , 0x350B)\
  m(n, CAPI_ERROR_B3_COMPRESSION_ERROR        , 0x350C)\
  m(n, CAPI_ERROR_B3_NO_CONNECT               , 0x350D)\
  m(n, CAPI_ERROR_B3_NO_FALLBACK_ALLOWED      , 0x350E)\
  m(n, CAPI_ERROR_B3_NO_MODEM_REMOTE          , 0x350F)\
  m(n, CAPI_ERROR_B3_HANDSHAKE_ERROR          , 0x3510)\
\
  /* reason values for Line Interconnect */\
  m(n, CAPI_ERROR_PLCI_HAS_NO_B_CHANNEL       , 0x3800)\
  m(n, CAPI_ERROR_LINES_NOT_COMPATIBLE        , 0x3801)\
  m(n, CAPI_ERROR_PLCI_NOT_IN_INTERCONNECTION , 0x3802)\
\
  m(n, CAPI_ERROR_Q850_CAUSE_MASK             , 0x3400)\
  m(n, CAPI_ERROR_SUPPL_SVC_MASK              , 0x3600) /* see ETS 300 196-1 [D.2] */\
  m(n, CAPI_ERROR_SUPPL_SVC_CXT_MASK          , 0x3700) /* see ETS 300 196-1 [D.1] "Invoke Problem" */\
/**/

CAPI_ERRORS(CAPI_MAKE_ERRORS,);

#endif

#ifdef __cplusplus
}
#endif

#endif /* __CAPI20_H */
