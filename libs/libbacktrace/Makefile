# libbacktrace/Makefile

-include $(TOPDIR)/Make.defs

DELIM ?= $(strip /)

ASRCS =
CSRCS =

DEPPATH := --dep-path .
VPATH := .

#CXXFLAGS := ${CROSS_FLAGS} -fno-omit-frame-pointer -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wunused -Wuninitialized -Wmissing-declarations -std=c11 -mpoke-function-name -fno-omit-frame-pointer
#CFLAGS += -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -Wall -Wunused -Wuninitialized -Wmissing-declarations -mpoke-function-name -funwind-tables -fno-omit-frame-pointer

ifeq ($(CONFIG_LIBBACKTRACE),y)
CSRCS += backtrace/backtrace.c
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

BIN ?= libbacktrace$(LIBEXT)

all: $(BIN)
.PHONY:  depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) > Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
