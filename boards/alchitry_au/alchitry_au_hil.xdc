## Alchitry Au (XC7A35T-1FTG256): hardware-in-the-loop build (top_alchitry_au_hil)
## Vivado part: xc7a35tftg256-1. Needs only the board and its USB cable.

## 100 MHz clock
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk -period 10.00 -waveform {0 5} [get_ports clk]

## Reset button (active low)
set_property -dict { PACKAGE_PIN P6  IOSTANDARD LVCMOS33 } [get_ports rst_n]

## USB-UART
set_property -dict { PACKAGE_PIN P15 IOSTANDARD LVCMOS33 } [get_ports usb_rx]
set_property -dict { PACKAGE_PIN P16 IOSTANDARD LVCMOS33 } [get_ports usb_tx]

## LEDs (active high)
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN K12 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN L14 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN L13 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN M12 IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports {led[7]}]

## Asynchronous I/O
set_false_path -from [get_ports {rst_n usb_rx}]
set_false_path -to   [get_ports {usb_tx led[*]}]

## Configuration
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
