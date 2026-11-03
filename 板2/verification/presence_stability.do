onerror {quit -code 1}
vlib work_presence_stability
vlog -work work_presence_stability ../rtl/parking_records.v ../rtl/plate_confirmation.v tb_presence_stability.v
vsim -c work_presence_stability.tb
run -all
quit -code 0
