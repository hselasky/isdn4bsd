SUBDIR= i4b

OSNAME!= uname

.if (${OSNAME} == NetBSD)
SUBDIR+= mount_devfs
.endif

.include <bsd.subdir.mk>
