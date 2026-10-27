# -----------------------------------------------------------------------------
# vivado_build.tcl: create and build the Alchitry Au project in one step.
#
# In Vivado, open the Tcl Console (Window > Tcl Console) and run ONE of:
#
#   source C:/path/to/imu-attitude-fpga/boards/alchitry_au/build_hil.tcl
#   source C:/path/to/imu-attitude-fpga/boards/alchitry_au/build_imu.tcl
#
# (Use forward slashes in the path.) Those two files set BUILD and source this one.
#
# Result: vivado/<build>/  (project you can open in the GUI later)
#         vivado/<build>.bin  (program this with the Alchitry Loader)
#         vivado/<build>_timing.rpt, vivado/<build>_utilization.rpt
# -----------------------------------------------------------------------------
if {![info exists BUILD]} { set BUILD hil }

set here [file dirname [file normalize [info script]]]
set root [file normalize [file join $here .. ..]]
set out  [file join $root vivado]
file mkdir $out

set common [list \
    rtl/cordic_vec.sv rtl/attitude_core.sv rtl/uart_tx.sv]

switch -- $BUILD {
    hil {
        set name  au_hil
        set top   top_alchitry_au_hil
        set srcs  [concat $common [list rtl/uart_rx.sv rtl/hil_rx.sv rtl/hil_tx.sv \
                                        boards/alchitry_au/top_alchitry_au_hil.sv]]
        set xdc   boards/alchitry_au/alchitry_au_hil.xdc
    }
    imu {
        set name  au_imu
        set top   top_alchitry_au
        set srcs  [concat $common [list rtl/spi_master.sv rtl/imu_spi_reader.sv \
                                        rtl/telemetry_tx.sv rtl/imu_attitude_system.sv \
                                        boards/alchitry_au/top_alchitry_au.sv]]
        set xdc   boards/alchitry_au/alchitry_au.xdc
    }
    default { error "BUILD must be hil or imu (got '$BUILD')" }
}

puts "== Building '$name' (top: $top) in $out/$name"
create_project $name [file join $out $name] -part xc7a35tftg256-1 -force

foreach f $srcs { add_files -norecurse [file join $root $f] }
set_property file_type SystemVerilog [get_files -of_objects [get_filesets sources_1] *.sv]
add_files -fileset constrs_1 -norecurse [file join $root $xdc]
set_property top $top [get_filesets sources_1]
update_compile_order -fileset sources_1

# Also write a .bin for the Alchitry Loader
set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]

launch_runs synth_1 -jobs 4
wait_on_run synth_1
if {[get_property PROGRESS [get_runs synth_1]] ne "100%"} {
    error "Synthesis failed: see the Messages window or $out/$name/$name.runs/synth_1/runme.log"
}

launch_runs impl_1 -to_step write_bitstream -jobs 4
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] ne "100%"} {
    error "Implementation failed: see $out/$name/$name.runs/impl_1/runme.log"
}

open_run impl_1
report_timing_summary -file [file join $out ${name}_timing.rpt]
report_utilization    -file [file join $out ${name}_utilization.rpt]

set bin [glob -nocomplain [file join $out $name $name.runs impl_1 *.bin]]
if {[llength $bin]} { file copy -force [lindex $bin 0] [file join $out $name.bin] }

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts ""
puts "== Done: $name"
puts "   Worst setup slack (WNS) at 100 MHz: $wns ns  (positive = timing met)"
puts "   Bitstream for the Alchitry Loader:   [file join $out $name.bin]"
puts "   Reports: [file join $out ${name}_timing.rpt]"
puts "            [file join $out ${name}_utilization.rpt]"
