onerror {quit -code 1}
vlib work_reject
vlog -work work_reject ../rtl/blue_plate_locator.v ../rtl/cnn_serial_core.v ../rtl/plate_capture.v ../rtl/plate_confirmation.v tb_e2e.v
vsim -c work_reject.tb +PHOTO=board2_full.hex +REJECT
run -all
quit -code 0
