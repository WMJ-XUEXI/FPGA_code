onerror {quit -code 1}
vlib work_capture
vlog -work work_capture ../rtl/cnn_serial_core.v ../rtl/plate_capture.v tb_capture.v
vsim -c work_capture.tb
run -all
quit -code 0
