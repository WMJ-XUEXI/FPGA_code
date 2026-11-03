onerror {quit -code 1}
vlib work
vlog -work work ../rtl/plate_confirmation.v tb_confirmation.v
vsim -c work.tb
run -all
quit -code 0
