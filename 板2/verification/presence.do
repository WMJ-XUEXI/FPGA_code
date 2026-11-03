onerror {quit -code 1}
vlib work_presence
vlog -work work_presence ../rtl/parking_records.v tb_presence.v
vsim -c work_presence.tb
run -all
quit -code 0
