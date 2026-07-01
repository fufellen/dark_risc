# Copyright (c) 2026

include ../config.mk

PROJ = lwipdemo

LWIP_ROOT ?= ../../../../third_party/lwip
LWIPDIR = $(LWIP_ROOT)/src

override CFLAGS := -I. -Iarch -I$(LWIPDIR)/include $(CFLAGS)
override CFLAGS += -Wno-incompatible-pointer-types -Wno-discarded-qualifiers
override CFLAGS += -Wno-builtin-declaration-mismatch -Wno-implicit-function-declaration

ifdef LIDARSIM_DDR3_DIAG
override CFLAGS += -DLIDARSIM_DDR3_DIAG=1 -DDARKDDR3_BASE=0xc0000000u
endif

ifdef LIDARSIM_PSRAM_MMIO
override CFLAGS += -DLIDARSIM_PSRAM_MMIO=1 -DDARKPSRAM_BASE=0xc0000000u
endif

ifdef LIDARSIM_PSRAM_SIM_SELFTEST
override CFLAGS += -DLIDARSIM_PSRAM_SIM_SELFTEST=1
endif

ifdef LIDARSIM_PSRAM_TCP_SIM_SELFTEST
override CFLAGS += -DLIDARSIM_PSRAM_TCP_SIM_SELFTEST=1
endif

ifdef LIDARSIM_PSRAM_TCP_SIM_FRAMES
override CFLAGS += -DLIDARSIM_PSRAM_TCP_SIM_FRAMES=$(LIDARSIM_PSRAM_TCP_SIM_FRAMES)
endif

ifdef LIDARSIM_DDR3_TIMEOUT
override CFLAGS += -DLIDARSIM_DDR3_TIMEOUT=$(LIDARSIM_DDR3_TIMEOUT)
endif

ifdef LIDARSIM_PSRAM_TIMEOUT
override CFLAGS += -DLIDARSIM_PSRAM_TIMEOUT=$(LIDARSIM_PSRAM_TIMEOUT)
endif

ifdef LIDARSIM_DIAG_BEACON
override CFLAGS += -DLIDARSIM_DIAG_BEACON=1
endif

LWIP_OBJS = \
    init.o \
    def.o \
    inet_chksum.o \
    ip.o \
    mem.o \
    memp.o \
    netif.o \
    pbuf.o \
    stats.o \
    sys.o \
    timeouts.o \
    tcp.o \
    tcp_in.o \
    tcp_out.o \
    udp.o \
    etharp.o \
    ip4.o \
    ip4_addr.o \
    ethernet.o

OBJS = main.o compat.o $(LWIP_OBJS)
SRCS = main.c compat.c

DEPS = $(SRCS) $(OBJS) $(LIBS)
TARGETS = $(PROJ).a

vpath %.c $(LWIPDIR)/core $(LWIPDIR)/core/ipv4 $(LWIPDIR)/netif

.PHONY: all

all: $(TARGETS) $(DEPS)
	@echo sources ok for $(TARGETS).

clean:
	-rm -f $(OBJS) $(PROJ).{S,bin,lst,map,ram,rom,x86,text,data,bin,ld,o,mem,rom.mem,ram.mem,a} $(PROJ)_uart.bin

%.o: %.c Makefile lwipopts.h arch/cc.h arch/sys_arch.h arch/perf.h arch/stdlib.h arch/stdint.h arch/stddef.h arch/string.h
	$(CC) $(CFLAGS) -c $< -o $@

$(PROJ).a: $(OBJS) Makefile
	$(AR) $(ARFLAGS) $@ $(OBJS)
	$(RL) $@
