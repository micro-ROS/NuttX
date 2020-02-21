#
# Copyright 2015 Stephen Street <stephen@redrocketcomputing.com>
# 
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/. 
#

SUBDIRS ?= $(subst ${SOURCE_DIR}/,,$(shell $(FIND) ${SOURCE_DIR} -mindepth 2 -name "subdir.mk" -printf "%h "))
SRC :=

include ${PROJECT_ROOT}/Makefile.common
include $(patsubst %, ${SOURCE_DIR}/%/subdir.mk, ${SUBDIRS})
-include ${SOURCE_DIR}/subdir.mk

OBJ := $(patsubst %.c,%.o,$(filter %.c,${SRC})) $(patsubst %.cpp,%.o,$(filter %.cpp,${SRC})) $(patsubst %.s,%.o,$(filter %.s,${SRC})) $(patsubst %.S,%.o,$(filter %.S,${SRC}))
OBJ := $(subst ${PROJECT_ROOT},${BUILD_ROOT},${OBJ})

#$(info SUBDIRS=${SUBDIRS})
#$(info SRC=${SRC})
#$(info OBJ=${OBJ})
#$(info SOURCE_DIR=${SOURCE_DIR})
#$(info TARGET=${TARGET})
#$(info TARGET=$(addprefix ${CURDIR}/,${TARGET}))
#$(info CURDIR=${CURDIR})

.SECONDARY:

all: ${SUBDIRS} $(addprefix ${CURDIR}/,${TARGET})

clean: ${SUBDIRS}
	@echo "CLEANING ${TARGET}"
	$(Q)${RM} ${TARGET} ${TARGET:%.afx=%.map} ${TARGET:%.afx=%.img} ${TARGET:%.afx=%.bin} ${TARGET:%.afx=%.map} ${TARGET:%.afx=%.smap} ${OBJ} ${OBJ:%.o=%.d} ${OBJ:%.o=%.dis}

${SUBDIRS}:
	$(Q)mkdir -p ${CURDIR}/$@

${CURDIR}/%.a: ${OBJ} ${EXTRA_DEPS}
	@echo "ARCHIVING $@"
	$(Q)$(AR) ${ARFLAGS} $@ ${OBJ}

${CURDIR}/%.afx: ${OBJ} ${EXTRA_DEPS}
	@echo "LINKING $@"
	$(Q)$(CC) -Wl,--cref -Wl,-Map,"$(basename ${@}).map" ${LDFLAGS} ${LOADLIBES} -o $@ ${OBJ} ${LDLIBS}

${CURDIR}/%.elf: ${OBJ} ${EXTRA_DEPS}
	@echo "LINKING $@"
	$(Q)$(CC) -Wl,--cref -Wl,-Map,"$(basename ${@}).map" ${LDFLAGS} ${LOADLIBES} -o $@ ${OBJ} ${LDLIBS}

${CURDIR}/%.o: ${SOURCE_DIR}/%.c
	@echo "COMPILING $<"
	$(Q)$(CC) ${CPPFLAGS} ${CFLAGS} -MMD -MP -c -o $@ $<
    
${CURDIR}/%.o: ${SOURCE_DIR}/%.cpp
	@echo "COMPILING $<"
	$(Q)$(CXX) ${CPPFLAGS} ${CXXFLAGS} -MMD -MP -c -o $@ $<

${CURDIR}/%.o: ${SOURCE_DIR}/%.s
	@echo "ASSEMBLING $<"
	$(Q)$(AS) ${ASFLAGS} -o $@ $<

${CURDIR}/%.o: ${SOURCE_DIR}/%.S
	@echo "ASSEMBLING $<"
	$(Q)$(CC) ${CPPFLAGS} ${ASFLAGS} -c -o $@ $<

ifneq (${MAKECMDGOALS},clean)
-include ${OBJ:.o=.d}
endif
