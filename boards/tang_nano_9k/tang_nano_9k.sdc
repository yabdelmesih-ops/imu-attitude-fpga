// Timing constraints: 27 MHz on-board oscillator (37.037 ns period)
create_clock -name clk_27m -period 37.037 -waveform {0 18.518} [get_ports {clk_27m}]
