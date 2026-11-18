# Build the I2C bring-up test (reads the MPU-6050 WHO_AM_I register). See vivado_build.tcl.
set BUILD whoami
source [file join [file dirname [file normalize [info script]]] vivado_build.tcl]
