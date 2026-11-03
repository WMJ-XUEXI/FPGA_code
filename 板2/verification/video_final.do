onerror {quit -code 1}
vlib work_video_final
vlog -work work_video_final ../rtl/blue_plate_locator.v ../rtl/cnn_serial_core.v ../rtl/plate_capture.v ../rtl/plate_confirmation.v ../rtl/parking_records.v ../rtl/recognition_osd.v ../rtl/calendar_clock.v ../rtl/plate_totals.v ../rtl/blueplate_video_mixer.v ../user_source/hdl_source/anlogic_logo_rom.v tb_video_final.v
vsim -c work_video_final.tb
run -all
quit -code 0
