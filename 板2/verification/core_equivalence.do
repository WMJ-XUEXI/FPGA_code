onerror {quit -code 1}
vlib work_core_equivalence
vlog -work work_core_equivalence cnn_serial_core_baseline.v ../rtl/cnn_serial_core.v tb_core_equivalence.v
vsim -c work_core_equivalence.tb
run -all
quit -code 0
