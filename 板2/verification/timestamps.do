onerror {quit -code 1}
vlib work_timestamps
vlog -work work_timestamps ../rtl/parking_records.v ../rtl/plate_totals.v tb_timestamps.v
vsim -c work_timestamps.tb
run -all
quit -code 0
