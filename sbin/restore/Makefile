.PATH: ${.CURDIR:H}/dump

PACKAGE=ufs
PROG=	restore
LINKS=	${BINDIR}/restore ${BINDIR}/rrestore
MAN=	restore.8
MLINKS=	restore.8 rrestore.8
SRCS=	main.c interactive.c restore.c dirs.c symtab.c tape.c utilities.c \
	dumprmt.c
WARNS?=	2
CFLAGS+= -DRRESTORE -D_ACL_PRIVATE

.include <bsd.prog.mk>
