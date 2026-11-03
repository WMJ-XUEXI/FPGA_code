onerror {quit -code 1}
vlib work_locator_background
vlog -work work_locator_background ../rtl/blue_plate_locator.v tb_locator_background.v
vsim -c work_locator_background.tb
run -all
quit -code 0
