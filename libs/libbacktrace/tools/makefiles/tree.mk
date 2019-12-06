#
# Copyright 2015 Stephen Street <stephen@redrocketcomputing.com>
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/. 
#

where-am-i := ${CURDIR}/$(lastword $(subst $(lastword ${MAKEFILE_LIST}),,${MAKEFILE_LIST}))

include ${PROJECT_ROOT}/Makefile.common

BUILD_PATH := $(subst ${PROJECT_ROOT},${BUILD_ROOT},${CURDIR})
SUBDIRS ?= $(subst ${CURDIR}/,,$(shell $(FIND) ${CURDIR} -mindepth 2 -maxdepth 2 -name "*.mk" -and -not -name "subdir.mk" -printf "%h "))

all: targets

clean: targets

#$(info SUBDIR=${SUBDIRS})

${SUBDIRS}:
	@echo "ENTERING $@"
	$(Q)${MAKE} --no-print-directory -C $@ -f $@.mk ${MAKECMDGOALS}
.PHONY: ${SUBDIRS}
