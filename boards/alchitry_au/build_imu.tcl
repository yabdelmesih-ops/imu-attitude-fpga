# Build the live-IMU design (IMU on the Br breakout). See vivado_build.tcl.
set BUILD imu
source [file join [file dirname [file normalize [info script]]] vivado_build.tcl]
