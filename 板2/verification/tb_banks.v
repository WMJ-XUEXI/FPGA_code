`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fs=0,pv=0,retry=0;reg [10:0] x=0;reg [9:0] y=0;reg [23:0] rgb=0;
 wire [9:0] gray={2'd0,rgb[23:16]}+{1'd0,rgb[15:8],1'b0}+{2'd0,rgb[7:0]};
 plate_capture dut(.clk(clk),.rst_n(rst),.frame_start(fs),.pixel_valid(pv),.x(x),.y(y),.gray(gray[9:2]),.rgb(rgb),
 .buttons_n(2'b11),.box_valid(1'b1),.enable(1'b1),.retry(retry),.box_l(11'd16),.box_r(11'd271),.box_t(10'd8),.box_b(10'd71));
 reg [23:0] photo[0:16383];reg [8:0] saved[0:16383];reg bank;
 reg [35:0] saved_metadata;
 wire [35:0] metadata={dut.threshold,dut.blue_left,dut.blue_right,dut.blue_top,dut.blue_bottom};
 integer f,xx,yy,i,writes=0;wire [5:0] main_state=dut.state;
 wire collision=dut.sample && main_state!=0 && dut.capture_bank==dut.read_bank;
 always @(posedge clk)begin
  if(rst && collision)$fatal(1,"producer overwrote a consumer-owned ROI bank");
  if(dut.sample)writes=writes+1;
 end
 task frame;input integer index;begin
  @(negedge clk);fs=1;pv=0;@(negedge clk);fs=0;repeat(4)@(negedge clk);
  for(yy=0;yy<80;yy=yy+1)begin
   for(xx=0;xx<288;xx=xx+1)begin
    @(negedge clk);pv=1;x=xx;y=yy;rgb=24'h202020;
    if(xx>=16 && xx<=271 && yy>=8 && yy<=71)rgb=index==0 ? photo[(yy-8)*256+xx-16]:24'h102080+index;
   end
   @(negedge clk);pv=0;
  end
 end endtask
 initial begin
  $readmemh("board1.hex",photo);repeat(4)@(negedge clk);rst=1;
  frame(0);repeat(300000)@(negedge clk);
  if(main_state!=14)$fatal(1,"did not hold the first ROI for CNN");
  bank=dut.read_bank;saved_metadata=metadata;
  for(i=0;i<16384;i=i+1)saved[i]=dut.bitmap[(bank ? 16384:0)+i];
  for(f=1;f<5;f=f+1)begin
   frame(f);
   if(metadata!==saved_metadata)$fatal(1,"producer changed the consumer's threshold/blue bounds");
   for(i=0;i<16384;i=i+1)if(dut.bitmap[(bank ? 16384:0)+i]!==saved[i])$fatal(1,"frozen ROI was modified frame=%0d address=%0d",f,i);
  end
  if(writes!=81920)$fatal(1,"producer did not acquire all five snapshots");
  @(negedge clk);retry=1;@(negedge clk);retry=0;#1;
  if(main_state!=0 || dut.roi_ready)$fatal(1,"retry did not discard bank ownership/pending ROI");
  $display("PASS two-bank ownership: 81920 writes, frozen consumer ROI unchanged through four producer overwrites, retry flush");$finish;
 end
endmodule
// Deliberately stall CNN forever to stress acquisition while its ROI is owned.
module cnn_serial_core(input clk,rst_n,input_wr,input [9:0] input_addr,input [6:0] input_pixel,input start,input [1:0] alphabet,output busy,done,output [6:0] prediction,output [15:0] margin);
 assign busy=1;assign done=0;assign prediction=127;assign margin=0;
endmodule
