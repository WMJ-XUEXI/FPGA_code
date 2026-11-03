`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fs=0,pv=0,retry=0,enable=1;reg [10:0] x=0,l=16,r=526;reg [9:0] y=0,t=8,b=134;
 reg [7:0] gray=0;wire done,good;wire [3:0] status;
 plate_capture dut(.clk(clk),.rst_n(rst),.frame_start(fs),.pixel_valid(pv),.x(x),.y(y),.gray(gray),.rgb(24'h1030a0),
 .buttons_n(2'b11),.box_valid(1'b1),.enable(enable),.retry(retry),.box_l(l),.box_r(r),.box_t(t),.box_b(b),
 .attempt_done(done),.attempt_confident(good),.capture_status(status));
 integer samples=0,xx,yy;reg check_grid=0;
 always @(posedge clk)if(dut.sample)begin
  if(check_grid && (x!=16+(samples%256)*511/256 || y!=8+(samples/256)*127/64))$fatal(1,"DDA coordinate mismatch");
  samples=samples+1;
 end
 task frame;begin @(negedge clk);pv=0;fs=1;@(negedge clk);fs=0;repeat(4)@(negedge clk);end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;frame;
  for(xx=16;xx<=526;xx=xx+1)begin @(negedge clk);x=xx;y=8;pv=1;end
  frame;repeat(4)@(negedge clk);
  if(good)$fatal(1,"incomplete capture accepted");
  retry=1;@(negedge clk);retry=0;samples=0;check_grid=1;frame;
  // Move the live box after ARM: the current snapshot must retain its original geometry.
  l=80;r=847;t=32;b=223;
  for(yy=8;yy<=134;yy=yy+1)begin
   for(xx=16;xx<=526;xx=xx+1)begin @(negedge clk);x=xx;y=yy;pv=1;gray=((xx+yy)%2) ? 8'd220:8'd20;end
   @(negedge clk);pv=0;
  end
  repeat(4)@(negedge clk);
  if(samples!=16384 || dut.threshold!=120)$fatal(1,"snapshot sample count or current-frame threshold failed");
  retry=1;@(negedge clk);retry=0;enable=0;samples=0;frame;
  repeat(100)@(negedge clk);
  if(dut.state!=0 || samples!=0)$fatal(1,"disabled capture ran");
  enable=1;l=10;r=100;t=8;b=134;frame;
  if(dut.state!=0)$fatal(1,"undersized plate not rejected");
  $display("PASS incomplete capture rejection, exact DDA, frozen geometry, current threshold, disable and small-box rejection");$finish;
 end
endmodule
// The protocol test intentionally isolates image capture from CNN arithmetic.
module cnn_serial_core(input clk,rst_n,input_wr,input [9:0] input_addr,input [6:0] input_pixel,input start,input [1:0] alphabet,output busy,done,output [6:0] prediction,output [15:0] margin);
 assign busy=0;assign done=0;assign prediction=127;assign margin=0;
endmodule
