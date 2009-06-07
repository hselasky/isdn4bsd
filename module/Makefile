_OSV8!=(uname -s -r | grep "FreeBSD 8") || true

.if empty(_OSV8)
.include "Makefile.v1"
.else
.include "Makefile.v2"
.endif
