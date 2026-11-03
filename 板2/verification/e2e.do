onerror {quit -code 1}
vlib work_e2e
vlog -work work_e2e ../rtl/blue_plate_locator.v ../rtl/cnn_serial_core.v ../rtl/plate_capture.v ../rtl/plate_confirmation.v tb_e2e.v
vsim -c work_e2e.tb
run -all
quit -code 0
