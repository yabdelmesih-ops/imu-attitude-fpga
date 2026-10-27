## Basys 3 (Artix-7 XC7A35T) constraints for top_basys3
## IMU breakout on Pmod header JA (top row):
##   JA1 = CS   JA2 = MOSI (SDA/SDI)   JA3 = MISO (AD0/SDO)   JA4 = SCLK (SCL)
##   JA5 = GND  JA6 = 3.3 V
## Pin numbers are from Digilent's Basys-3-Master.xdc - double-check against
## your copy before programming.

## 100 MHz clock
set_property -dict { PACKAGE_PIN W5   IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk -period 10.00 -waveform {0 5} [get_ports clk]

## Centre button = reset
set_property -dict { PACKAGE_PIN U18  IOSTANDARD LVCMOS33 } [get_ports btn_rst]

## IMU SPI on Pmod JA
set_property -dict { PACKAGE_PIN J1   IOSTANDARD LVCMOS33 } [get_ports imu_cs_n]
set_property -dict { PACKAGE_PIN L2   IOSTANDARD LVCMOS33 } [get_ports imu_mosi]
set_property -dict { PACKAGE_PIN J2   IOSTANDARD LVCMOS33 } [get_ports imu_miso]
set_property -dict { PACKAGE_PIN G2   IOSTANDARD LVCMOS33 } [get_ports imu_sclk]

## USB-UART (FPGA -> PC)
set_property -dict { PACKAGE_PIN A18  IOSTANDARD LVCMOS33 } [get_ports uart_txd]

## LEDs
set_property -dict { PACKAGE_PIN U16  IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN E19  IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN U19  IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN V19  IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN W18  IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN U15  IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN U14  IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN V14  IOSTANDARD LVCMOS33 } [get_ports {led[7]}]
set_property -dict { PACKAGE_PIN V13  IOSTANDARD LVCMOS33 } [get_ports {led[8]}]
set_property -dict { PACKAGE_PIN V3   IOSTANDARD LVCMOS33 } [get_ports {led[9]}]
set_property -dict { PACKAGE_PIN W3   IOSTANDARD LVCMOS33 } [get_ports {led[10]}]
set_property -dict { PACKAGE_PIN U3   IOSTANDARD LVCMOS33 } [get_ports {led[11]}]
set_property -dict { PACKAGE_PIN P3   IOSTANDARD LVCMOS33 } [get_ports {led[12]}]
set_property -dict { PACKAGE_PIN N3   IOSTANDARD LVCMOS33 } [get_ports {led[13]}]
set_property -dict { PACKAGE_PIN P1   IOSTANDARD LVCMOS33 } [get_ports {led[14]}]
set_property -dict { PACKAGE_PIN L1   IOSTANDARD LVCMOS33 } [get_ports {led[15]}]

## Slow, asynchronous I/O: no timing relationship to the system clock
set_false_path -from [get_ports btn_rst]
set_false_path -from [get_ports imu_miso]
set_false_path -to   [get_ports {imu_cs_n imu_mosi imu_sclk uart_txd led[*]}]

set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property CFGBVS VCCO [current_design]
