onerror {quit -code 1}
vlib work_calendar
vlog -work work_calendar ../rtl/calendar_clock.v tb_calendar.v
vsim -c work_calendar.tb
run -all
quit -code 0
