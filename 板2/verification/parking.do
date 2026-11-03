onerror {quit -code 1}
vlib work_parking
vlog -work work_parking ../rtl/parking_records.v tb_parking.v
vsim -c work_parking.tb
run -all
quit -code 0
