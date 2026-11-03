onerror {quit -code 1}
vlib work_table
vlog -work work_table ../rtl/recognition_osd.v tb_table.v
vsim -c work_table.tb
run -all
quit -code 0
