include ../config.mk

PROJ = w5500demo
OBJS = main.o
ASMS = main.s
SRCS = main.c

DEPS = $(SRCS) $(ASMS) $(OBJS) $(LIBS)

TARGETS = $(PROJ).a

# для быстрой симуляции можно уменьшить кадр:
# make ... W5500DEMO_MSOP_POINTS=24
ifdef W5500DEMO_MSOP_POINTS
CFLAGS += -DW5500DEMO_MSOP_POINTS=$(W5500DEMO_MSOP_POINTS)u
endif
ifdef W5500DEMO_FRAMES
CFLAGS += -DW5500DEMO_FRAMES=$(W5500DEMO_FRAMES)u
endif

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
