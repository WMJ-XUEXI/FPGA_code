onerror {quit -code 1}
vlib work_character_confidence
vlog -work work_character_confidence ../rtl/plate_capture.v tb_character_confidence.v
vsim -c work_character_confidence.tb
run -all
quit -code 0
