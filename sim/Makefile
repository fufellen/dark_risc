# Copyright (c) 2018, Marcelo Samsoniuk
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
## Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
## Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
## Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 

ifndef VSIM
    VSIM = vsim
endif

MODELSIM_DO = darksimv_modelsim.do

RTLS =  darksimv.v                      \
        ../rtl/darksocv.v               \
        ../rtl/darkbridge.v             \
        ../rtl/darkuart.v               \
        ../rtl/darkriscv.v              \
        ../rtl/darkpll.v                \
        ../rtl/darkram.v                \
        ../rtl/darkio.v                 \
        ../rtl/darketh_mmio.sv          \
        ../rtl/darkcache.v              \
        ../rtl/darkmac.v                \
        ../rtl/lib/sdram/mt48lc16m16a2_ctrl.v

INCS = ../rtl/config.vh

ifeq ($(APPLICATION),spidemo)
RTLS += ../rtl/darkspi.v                \
        ../rtl/lib/spi/lis3dh_stub.v
RTLS += ../rtl/lib/spi/spi_master.v
SPIBB := 1
ifdef SPIBB
RTLS += ../rtl/lib/spi/spi_master_bb.v
endif
SPI3WIRE:=1
DUT:=darksocv
include cosim.mk
COSIM_SRC := $(RTLS)
ifdef VLATOR
VOPT+=-DSIMULATION=1 -DSPI=1 -GSPI_DIV_COEF=1
ifdef SPIBB
VOPT+=-DSPIBB=1
endif
ifdef COSIM
COSIM_TARGET := $(DUT).cosim
endif
endif
endif

.PHONY: all modelsim clean

ifdef HARVARD
	SRCS = ../src/darksocv.rom.mem ../src/darksocv.ram.mem
else
	SRCS = ../src/darksocv.mem
endif

DEPS = $(SRCS) $(RTLS) $(INCS)

all: modelsim $(COSIM_TARGET)
	@echo simulation ok.

modelsim: $(DEPS) $(MODELSIM_DO)
	$(VSIM) -c -do "do $(MODELSIM_DO); quit -f"

clean:
	-rm -f darksocv darksocv.vcd darksocv.txt $(GUI_VCD_FILE) imgui.ini
	-rm -rf tmp_gui ../build/modelsim
