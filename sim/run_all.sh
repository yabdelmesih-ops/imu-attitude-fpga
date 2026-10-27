#!/usr/bin/env bash
# Regenerate golden vectors, run every testbench, and analyze the results.
# Requires: Icarus Verilog >= 12 (iverilog, vvp), Python 3 with numpy + matplotlib.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p out vectors

echo "== generating vectors from the Python model"
python3 ../model/gen_stimulus.py

run() {   # name, iverilog args...
    local name=$1; shift
    echo "== $name"
    iverilog -g2012 -Wall -o "out/$name.vvp" "$@"
    vvp -n "out/$name.vvp" | tee "out/$name.log" | grep -E "PASS|FAIL|latency|init writes"
    grep -q "TEST PASS" "out/$name.log"
}

RTL=(../rtl/*.sv)
run tb_cordic      ../rtl/cordic_vec.sv ../tb/tb_cordic.sv
run tb_attitude    ../rtl/cordic_vec.sv ../rtl/attitude_core.sv ../tb/tb_attitude_core.sv
run tb_top_tang9k  "${RTL[@]}" ../boards/tang_nano_9k/top_tang_nano_9k.sv ../tb/mpu_spi_model.sv ../tb/tb_top.sv   # ~1 min
if [[ "${1:-}" == "--all-boards" ]]; then
    run tb_top_basys3 -DBASYS3 "${RTL[@]}" ../boards/basys3/top_basys3.sv ../tb/mpu_spi_model.sv ../tb/tb_top.sv
fi

echo "== analysis"
python3 ../model/compare.py
echo "== all tests passed"
