# Build the hardware-in-the-loop design (no IMU needed). See vivado_build.tcl.
set BUILD hil
source [file join [file dirname [file normalize [info script]]] vivado_build.tcl]
