onerror {quit -code 1}
vlib work_osd
vlog -work work_osd ../rtl/recognition_osd.v tb_osd.v
vsim -c work_osd.tb
run -all
quit -code 0
