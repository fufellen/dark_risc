# Copyright (c) 2026

include ../config.mk

PROJ = ddr3memtest
OBJS = main.o
ASMS = main.s
SRCS = main.c

ifdef DDR3MEMTEST_SIM
override CFLAGS += -DDDR3MEMTEST_WORDS=64
override CFLAGS += -DDDR3MEMTEST_RANDOM_OPS=64
override CFLAGS += -DDDR3MEMTEST_MAX_ADDR_BIT=7
override CFLAGS += -DDDR3MEMTEST_RETENTION_REFRESHES=2
endif

ifdef DDR3MEMTEST_WORDS
override CFLAGS += -DDDR3MEMTEST_WORDS=$(DDR3MEMTEST_WORDS)
endif

ifdef DDR3MEMTEST_RANDOM_OPS
override CFLAGS += -DDDR3MEMTEST_RANDOM_OPS=$(DDR3MEMTEST_RANDOM_OPS)
endif

ifdef DDR3MEMTEST_MAX_ADDR_BIT
override CFLAGS += -DDDR3MEMTEST_MAX_ADDR_BIT=$(DDR3MEMTEST_MAX_ADDR_BIT)
endif

ifdef DDR3MEMTEST_RETENTION_REFRESHES
override CFLAGS += -DDDR3MEMTEST_RETENTION_REFRESHES=$(DDR3MEMTEST_RETENTION_REFRESHES)
endif

ifdef DDR3MEMTEST_BASE_ADDR
override CFLAGS += -DDDR3MEMTEST_BASE_ADDR=$(DDR3MEMTEST_BASE_ADDR)
endif

ifdef DDR3MEMTEST_LOW_PROBE
override CFLAGS += -DDDR3MEMTEST_LOW_PROBE=$(DDR3MEMTEST_LOW_PROBE)
endif

DEPS = $(SRCS) $(ASMS) $(OBJS) $(LIBS)
TARGETS = $(PROJ).a

.PHONY: all

all: $(TARGETS) $(DEPS)
	@echo sources ok for $(TARGETS).

clean:
	-rm -f $(ASMS) $(OBJS) $(PROJ).{S,bin,lst,map,ram,rom,x86,text,data,bin,ld,o,mem,rom.mem,ram.mem,a} $(PROJ)_uart.bin

%.o: %.s Makefile
	$(AS) $(ASFLAGS) -c $< -o $@

%.s: %.c Makefile
	$(CC) $(CFLAGS) -S $< -o $@

$(PROJ).a: $(OBJS) Makefile
	$(AR) $(ARFLAGS) $@ $(OBJS)
	$(RL) $@
