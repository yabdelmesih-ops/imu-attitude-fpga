## Alchitry Au (XC7A35T-1FTG256): live sensor build (top_alchitry_au_live)
## MPU-6050 (GY-521) on the Qwiic port. Pin mapping from Alchitry Labs (AuPin.kt):
## Qwiic SDA = A23 = K5, Qwiic SCL = A24 = E6, both in bank 35 (3.3 V).

## 100 MHz clock
set_property -dict { PACKAGE_PIN N14 IOSTANDARD LVCMOS33 } [get_ports clk]
create_clock -add -name sys_clk -period 10.00 -waveform {0 5} [get_ports clk]

## Reset button (active low)
set_property -dict { PACKAGE_PIN P6  IOSTANDARD LVCMOS33 } [get_ports rst_n]

## USB-UART to the PC (FPGA transmits on P16)
set_property -dict { PACKAGE_PIN P16 IOSTANDARD LVCMOS33 } [get_ports usb_tx]

## Qwiic I2C (open-drain; the sensor board has its own pull-ups, these are a backup)
set_property -dict { PACKAGE_PIN E6 IOSTANDARD LVCMOS33 PULLUP TRUE } [get_ports qwiic_scl]
set_property -dict { PACKAGE_PIN K5 IOSTANDARD LVCMOS33 PULLUP TRUE } [get_ports qwiic_sda]

## LEDs (active high, LED0 = top)
set_property -dict { PACKAGE_PIN K13 IOSTANDARD LVCMOS33 } [get_ports {led[0]}]
set_property -dict { PACKAGE_PIN K12 IOSTANDARD LVCMOS33 } [get_ports {led[1]}]
set_property -dict { PACKAGE_PIN L14 IOSTANDARD LVCMOS33 } [get_ports {led[2]}]
set_property -dict { PACKAGE_PIN L13 IOSTANDARD LVCMOS33 } [get_ports {led[3]}]
set_property -dict { PACKAGE_PIN M16 IOSTANDARD LVCMOS33 } [get_ports {led[4]}]
set_property -dict { PACKAGE_PIN M14 IOSTANDARD LVCMOS33 } [get_ports {led[5]}]
set_property -dict { PACKAGE_PIN M12 IOSTANDARD LVCMOS33 } [get_ports {led[6]}]
set_property -dict { PACKAGE_PIN N16 IOSTANDARD LVCMOS33 } [get_ports {led[7]}]

## Asynchronous I/O
set_false_path -from [get_ports {rst_n qwiic_sda}]
set_false_path -to   [get_ports {usb_tx led[*] qwiic_scl qwiic_sda}]

## Configuration
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
