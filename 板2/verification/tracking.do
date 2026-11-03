onerror {quit -code 1}
vlib work_tracking
vlog -work work_tracking ../rtl/blue_plate_locator.v tb_tracking.v
vsim -c work_tracking.tb
run -all
quit -code 0
