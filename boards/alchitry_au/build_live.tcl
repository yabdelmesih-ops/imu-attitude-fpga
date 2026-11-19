# Build the live sensor design (MPU-6050 over the Qwiic port). See vivado_build.tcl.
set BUILD live
source [file join [file dirname [file normalize [info script]]] vivado_build.tcl]
