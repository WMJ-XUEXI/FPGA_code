onerror {quit -code 1}
vlib work_panels
vlog -work work_panels ../rtl/parking_records.v ../rtl/plate_totals.v ../rtl/recognition_osd.v tb_panels.v
vsim -c work_panels.tb
run -all
quit -code 0
