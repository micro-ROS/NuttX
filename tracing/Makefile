
-include $(TOPDIR)/Make.defs

DELIM ?= $(strip /)

ASRCS =
CSRCS =

DEPPATH := --dep-path .
VPATH := .

CFLAGS += -Os -g -Wall -I./include -std=gnu99 -I$(TOPDIR)/arch/arm/src/stm32/ -l$(TOPDIR)

ifeq ($(CONFIG_ENABLE_TRACING), y)
CSRCS += utils.c
endif

ifeq ($(CONFIG_TRACE_USE_CTF),y)
CSRCS += tracing_core.c tracing_buffer.c \
	 tracing_format_common.c \
         ctf/ctf_top.c
endif

ifeq ($(CONFIG_TRACE_TIMESTAMP_CUSTOM_BOARD_TIMER_FREERUNER),y)
CSRCS +=  backends/timers/backend_timer_freerunner.c
else
CSRCS +=  backends/timers/backend_timer_systick.c
endif

ifeq ($(CONFIG_CTF_BACKEND_TRACING_SERIAL),y)
CSRCS +=  backends/tracing/tracing_backend_uart.c
endif

ifeq ($(CONFIG_TRACE_USE_NOCTF),y)
CSRCS += cpu_stats.c
endif

ifeq ($(CONFIG_TRACE_CTF_PWR_MEASUREMENT),y)
CSRCS += backends/gpios/backend_gpios.c
endif

ifeq ($(CONFIG_TRACE_CTF_MEMORY_STATIC_INFO),y)
CSRCS += tracing_elements/tracing_stack.c
else 
ifeq ($(CONFIG_TRACE_CTF_FUNCTIONS_USAGE), y)
CSRCS += tracing_elements/tracing_stack.c
endif
endif

ifeq ($(CONFIG_ASYNC_CTF_TRACING),y)
CSRCS += tracing_format_async.c
else
ifeq ($(CONFIG_TRACE_USE_CTF),y)
CSRCS += tracing_format_sync.c
endif
endif

AOBJS = $(ASRCS:.S=$(OBJEXT))
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

CFLAGS += -finstrument-functions-exclude-file-list="$(CSRCS)"

BIN ?= libtracing$(LIBEXT)

all: $(BIN)
.PHONY:  depend clean distclean

$(AOBJS): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

$(BIN): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))
	#$(CROSSDEV)ar rcv $@ $(OBJS)

.depend: Makefile $(SRCS)
	$(Q) $(MKDEP) $(DEPPATH) "$(CC)" -- $(CFLAGS) -- $(SRCS) > Make.dep
	$(Q) touch $@

depend: .depend

clean:
	$(Q) find . -iname "*.o" -exec rm {} \;
	$(call DELFILE, $(BIN))
	$(call CLEAN)

distclean: clean
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
