onerror {quit -code 1}
vlib work_keys
vlog -work work_keys ../rtl/parking_records.v ../rtl/calendar_clock.v ../rtl/plate_totals.v ../rtl/blueplate_video_mixer.v tb_keys.v
vsim -c work_keys.tb
run -all
quit -code 0
