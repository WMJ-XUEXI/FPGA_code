import_device ph1_35p.db -package PH1P35MDG324 -speed 3
open_project {PARKVISION_FINAL_DEMO.al} -single_run
import_db unit_gate.db
place
update_timing -mode manhattan
report_area -io_info -file unit_place.area
report_timing_status -file unit_place.ts
report_timing_summary -file unit_place.timing
export_db unit_place.db
route
update_timing -mode final
report_area -io_info -file unit_phy.area
report_timing_status -file unit_phy.ts
report_timing_summary -file unit_pr.timing
report_timing_exception -file unit_exception.timing
export_db unit_pr.db
exit
