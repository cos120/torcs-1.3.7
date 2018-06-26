##############################################################################
#
#    file                 : Makefile
#    created              : 2018年 06月 19日 星期二 18:06:41 CST
#    copyright            : (C) 2002 zj
#
##############################################################################
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
##############################################################################

ROBOT       = SRC_B
MODULE      = ${ROBOT}.so
MODULEDIR   = drivers/${ROBOT}
SOURCES     = ${ROBOT}.cpp

SHIPDIR     = drivers/${ROBOT}
SHIP        = ${ROBOT}.xml p406.rgb logo.rgb
SHIPSUBDIRS = 

PKGSUBDIRS  = ${SHIPSUBDIRS}
src-robots-SRC_B_PKGFILES = $(shell find * -maxdepth 0 -type f -print)
src-robots-SRC_B_PKGDIR   = ${PACKAGE}-${VERSION}/$(subst ${TORCS_BASE},,$(shell pwd))

include ${MAKE_DEFAULT}
