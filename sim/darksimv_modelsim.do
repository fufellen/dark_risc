# Run from the ModelSim console:
# do C:/workspace/dark_risc/sim/darksimv_modelsim.do

catch {quit -sim}
transcript off

set script_file [info script]
if {[file extension $script_file] ne ".do"} {
    set script_file ""
    for {set frame_idx [info frame]} {$frame_idx >= 0} {incr frame_idx -1} {
        if {[catch {set frame_cmd [dict get [info frame $frame_idx] cmd]}]} {
            continue
        }
        if {[llength $frame_cmd] >= 2 && [lindex $frame_cmd 0] eq "do"} {
            set script_file [lindex $frame_cmd 1]
            break
        }
    }
}
if {$script_file eq ""} {
    error "Cannot determine current .do file path"
}

set script_dir [file normalize [file dirname $script_file]]
set project_dir [file normalize [file join $script_dir ".."]]
set sim_dir [file normalize [file join $project_dir "build" "modelsim" "darksimv" "run"]]
set modelsim_ini [file join $sim_dir "modelsim.ini"]
set mem_file [file join $project_dir "src" "darksocv.mem"]

if {![file exists $mem_file]} {
    error "Firmware image not found: $mem_file"
}

file delete -force $sim_dir
file mkdir $sim_dir

cd $sim_dir
catch {unset ::env(MODELSIM)}
vmap -c
set ::env(MODELSIM) $modelsim_ini
vlib work
vmap work [file join $sim_dir "work"]
transcript on

# Compile from sim/ so legacy relative includes like "../rtl/config.vh" work.
cd $script_dir

set incdir "+incdir+[file join $project_dir "rtl"]"
set rtl_files [list \
    [file join $script_dir "darksimv.v"] \
    [file join $project_dir "rtl" "darksocv.v"] \
    [file join $project_dir "rtl" "darkbridge.v"] \
    [file join $project_dir "rtl" "darkuart.v"] \
    [file join $project_dir "rtl" "darkriscv.v"] \
    [file join $project_dir "rtl" "darkpll.v"] \
    [file join $project_dir "rtl" "darkram.v"] \
    [file join $project_dir "rtl" "darkio.v"] \
    [file join $project_dir "rtl" "darkcache.v"] \
    [file join $project_dir "rtl" "darkmac.v"] \
    [file join $project_dir "rtl" "lib" "sdram" "mt48lc16m16a2_ctrl.v"] \
]

set vlog_cmd [list vlog -sv $incdir]
foreach rtl_file $rtl_files {
    lappend vlog_cmd $rtl_file
}
eval $vlog_cmd

# Run from this 4-level-deep directory so rtl/darkram.v can resolve:
# "../../../../src/darksocv.mem"
cd $sim_dir
vsim -wlf [file join $sim_dir "vsim.wlf"] -t 1ns -voptargs="+acc" work.darksimv

add wave -position insertpoint sim:/darksimv/CLK
add wave -position insertpoint sim:/darksimv/RES
add wave -position insertpoint sim:/darksimv/TX
add wave -position insertpoint sim:/darksimv/RX
add wave -position insertpoint sim:/darksimv/soc0/*
catch {add wave -position insertpoint sim:/darksimv/soc0/bridge0/*}
catch {add wave -position insertpoint sim:/darksimv/soc0/bridge0/core0/*}
catch {add wave -position insertpoint sim:/darksimv/soc0/bram0/*}
catch {add wave -position insertpoint sim:/darksimv/soc0/io0/*}

catch {config wave -signalnamewidth 1}
radix hexadecimal

run -all
catch {wave zoom full}
