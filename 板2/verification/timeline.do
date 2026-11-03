onerror {quit -code 1}
vlib work_timeline
vlog -work work_timeline ../rtl/parking_records.v ../rtl/recognition_osd.v tb_timeline.v
vsim -c work_timeline.tb
run -all
quit -code 0
