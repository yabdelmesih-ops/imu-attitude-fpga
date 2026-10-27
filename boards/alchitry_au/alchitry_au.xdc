## Alchitry Au (Xilinx Artix-7 XC7A35T-1FTG256) constraints for top_alchitry_au
## Vivado part: xc7a35tftg256-1
##
## On-board pins (clock, reset, LEDs, USB-UART) match Alchitry's board files and
## the LiteX alchitry_au platform. Header pins are from the Au header-to-package
## mapping (github.com/labeth/alchitry-au-docs). Check them against the Alchitry Br
## reference card before wiring.
##
## IMU on header bank A, all on FPGA bank 14 (fixed 3.3 V):
##   A2 = CS    A3 = MOSI (SDA/SDI)    A5 = MISO (AD0/SDO)    A6 = SCLK (SCL)
##   A4 = GND   A7 = +3.3 V

## 100 MHz clock
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk -period 10.00 -waveform {0 5} [get_ports clk]

## Reset button (active low)
set_property -dict { PACKAGE_PIN P6  IOSTANDARD LVCMOS33 } [get_ports rst_n]

## USB-UART, FPGA transmit -> PC
set_property -dict { PACKAGE_PIN P16 IOSTANDARD LVCMOS33 } [get_ports usb_tx]

## IMU SPI on header bank A
set_property -dict { PACKAGE_PIN T8  IOSTANDARD LVCMOS33 } [get_ports imu_cs_n]
set_property -dict { PACKAGE_PIN T7  IOSTANDARD LVCMOS33 } [get_ports imu_mosi]
set_property -dict { PACKAGE_PIN T5  IOSTANDARD LVCMOS33 PULLUP TRUE } [get_ports imu_miso]
set_property -dict { PACKAGE_PIN R5  IOSTANDARD LVCMOS33 } [get_ports imu_sclk]

## LEDs (active high)
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN K12 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN L14 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN L13 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN M12 IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports {led[7]}]

## Slow, asynchronous I/O: no timing relationship to the system clock
set_false_path -from [get_ports {rst_n imu_miso}]
set_false_path -to   [get_ports {imu_cs_n imu_mosi imu_sclk usb_tx led[*]}]

## Configuration
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
