onerror {quit -code 1}
vlib work_banks
vlog -work work_banks ../rtl/plate_capture.v tb_banks.v
vsim -c work_banks.tb
run -all
quit -code 0
