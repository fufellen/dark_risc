# do C:/workspace/verilog/soft_mcu/dark_risc/sim/spibb_flash_tb.do
set script_dir "C:/workspace/verilog/soft_mcu/dark_risc/sim"
set proj_dir [file dirname $script_dir]

if {[file exists work]} { vdel -all -lib work }
vlib work

vlog -sv [file join $proj_dir rtl lib spi spi_master_bb.v]
vlog -sv [file join $script_dir spi_flash_model.sv]
vlog -sv [file join $script_dir spibb_flash_tb.sv]

vsim -c -t 1ps -voptargs="+acc" work.spibb_flash_tb
run -all
