`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0;wire rd,vs,hs,de;wire [23:0] rgb;
 blueplate_video_mixer dut(clk,rst,1'b0,1'b0,1'b0,1'b0,1'b0,4'd0,4'b1111,rd,24'd0,vs,hs,de,rgb);
 wire valid=dut.ocr_measurement;wire [39:0] word=dut.osd.label_word;
 initial begin
 repeat(3)@(negedge clk);rst=1;
 force dut.bv=1;force dut.measured=1;force dut.history_ready=1;force dut.locked=1;force dut.verified=1;
 force dut.l=256;force dut.r=1023;force dut.t=180;force dut.b=371;#2;if(!valid || word!="READY")$fatal(1,"valid ROI did not retain READY");
 force dut.t=0;#2;if(valid || word!="SCAN ")$fatal(1,"top-edge stale READY");
 force dut.t=180;force dut.b=719;#2;if(valid || word!="SCAN ")$fatal(1,"bottom-edge stale READY");
 force dut.b=371;force dut.r=400;#2;if(valid || word!="SCAN ")$fatal(1,"small ROI stale READY");
 force dut.r=1023;force dut.measured=0;#2;if(valid || word!="SCAN ")$fatal(1,"lost ROI stale READY");
 force dut.measured=1;force dut.verified=0;#2;if(!valid || word!="SCAN ")$fatal(1,"replacement review did not show SCAN");
 $display("PASS final SCAN gate: valid ROI, absent measurement, replacement review, too small and clipped borders");$finish;
 end
endmodule
