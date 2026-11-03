onerror {quit -code 1}
vlib work_protocol
vlog -work work_protocol ../rtl/plate_capture.v tb_capture_protocol.v
vsim -c work_protocol.tb
run -all
quit -code 0
