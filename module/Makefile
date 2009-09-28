_OSV89!=(uname -s -r | grep -E "FreeBSD [89]") || (echo -n "")

.if empty(_OSV89)
.include "Makefile.v1"
.else
.include "Makefile.v2"
.endif
