/*-
 * Copyright (c) 2008 Hans Petter Selasky. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
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
 */

/* libkern */

int	bcmp(const void *b1, const void *b2, size_t length);
int	strncmp(const char *s1, const char *s2, size_t n);
char   *strstr(const char *s, const char *find);
size_t	strlen(const char *str);
size_t	strlcpy(char *dst, const char *src, size_t siz);
size_t	strlcat(char *dst, const char *src, size_t siz);
int	strcmp(const char *s1, const char *s2);
int	strcasecmp(const char *s1, const char *s2);
int	strncasecmp(const char *s1, const char *s2, size_t n);
void   *memset(void *, int, size_t);
void   *memcpy(void *dst, const void *src, size_t len);
void	bcopy(const void *src, void *dst, size_t len);
void	bzero(void *dst, size_t len);

/* C-type */

#define	isspace(c)      ((c) == ' ' || ((c) >= '\t' && (c) <= '\r'))
#define	isascii(c)      (((c) & ~0x7f) == 0)
#define	isupper(c)      ((c) >= 'A' && (c) <= 'Z')
#define	islower(c)      ((c) >= 'a' && (c) <= 'z')
#define	isalpha(c)      (isupper(c) || islower(c))
#define	isdigit(c)      ((c) >= '0' && (c) <= '9')
#define	isxdigit(c)     (isdigit(c) \
                          || ((c) >= 'A' && (c) <= 'F') \
                          || ((c) >= 'a' && (c) <= 'f'))
#define	isprint(c)      ((c) >= ' ' && (c) <= '~')

#define	toupper(c)      ((c) - 0x20 * (((c) >= 'a') && ((c) <= 'z')))
#define	tolower(c)      ((c) + 0x20 * (((c) >= 'A') && ((c) <= 'Z')))

/* printf */

int printf(const char *format, ...) __printflike(1,2);
int vprintf(const char *format, va_list ap);
int vsnprintf(char *dst, size_t size, const char * fmt, va_list ap);
int snprintf(char *dst, size_t size, const char * fmt, ...) __printflike(2,4);

