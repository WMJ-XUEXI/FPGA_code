onerror {quit -code 1}
vlib work_panel_keys
vlog -work work_panel_keys ../rtl/parking_records.v tb_panel_keys.v
vsim -c work_panel_keys.tb
run -all
quit -code 0
