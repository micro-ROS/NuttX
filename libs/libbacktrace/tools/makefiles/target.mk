#
# Copyright 2015 Stephen Street <stephen@redrocketcomputing.com>
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/. 
#

.SUFFIXES:

include ${PROJECT_ROOT}/Makefile.common

ifndef BUILD_PATH
BUILD_PATH := $(subst ${PROJECT_ROOT},${BUILD_ROOT},${CURDIR})
endif

${BUILD_PATH}:
	+$(Q)[ -d $@ ] || mkdir -p $@
	+$(Q)${MAKE} --no-print-directory -C $@ -f ${CURDIR}/$(lastword $(subst /, ,${CURDIR})).mk SOURCE_DIR=${CURDIR} ${MAKECMDGOALS}
.PHONY: ${BUILD_PATH}

Makefile : ;
%.mk :: ;

% :: ${BUILD_PATH} ; @:
