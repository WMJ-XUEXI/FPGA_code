import_device ph1_35p.db -package PH1P35MDG324 -speed 3
open_project {PARKVISION_FINAL_DEMO.al} -single_run
elaborate -top design_top_wrapper
read_adc {D:/Desktop/FPGA/code/111/baseline/touch_merge_20261027/td_project/pin_touch_ethernet.adc}
read_ip_adc -ip ddr2 -file {D:/Desktop/FPGA/code/111/baseline/user_source/hdl_source/ph1p35_ddr/ddr2/src/adc/ddr2.adc}
read_sdc {D:/Desktop/FPGA/code/111/baseline/td_project/camera_to_dsi_display_vehicle_ethernet.sdc}
read_sdc -ip ddr2 {D:/Desktop/FPGA/code/111/baseline/user_source/hdl_source/ph1p35_ddr/ddr2/src/sdc/ddr2.sdc}
read_sdc -ip w128_d512_fifo {D:/Desktop/FPGA/code/111/baseline/user_source/ip_source/w128_d512_fifo/w128_d512_fifo.tcl}
optimize_rtl
optimize_gate -packarea unit_gate.area
report_area -file unit_gate.area
export_db unit_gate.db
exit
