import_device ph1_35p.db -package PH1P35MDG324 -speed 3
open_project {E:/Qian_FPGA/example/cnn/sc520_confusion/td_project/sc520_confusion.al} -single_run
load_run_param -run syn_1
commit_param -step design
elaborate -top design_top_wrapper
read_adc {E:/Qian_FPGA/example/cnn/sc520_confusion/user_source/constraints_source/pin.adc}
export_db {E:/Qian_FPGA/example/cnn/sc520_confusion/verification/gui_elaborate.db}
puts "AUTO_RECOG_GUI_READ_VALIDATED"
exit
