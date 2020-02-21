#
# Copyright 2015 Stephen Street <stephen@redrocketcomputing.com>
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/. 
#

ifeq ($(findstring ${BUILD_ROOT},${CURDIR}),)
include ${PROJECT_ROOT}/tools/makefiles/target.mk
else

EXTRA_DEPS += ${BUILD_ROOT}/backtrace/libbacktrace.a

TARGET := ping-pong.afx

include ${PROJECT_ROOT}/tools/makefiles/project.mk

CPPFLAGS += -I ${SOURCE_DIR}/../include
LDFLAGS += -L ${BUILD_ROOT}/backtrace --specs=rdimon.specs --specs=nano.specs
LDLIBS += -lbacktrace

endif



