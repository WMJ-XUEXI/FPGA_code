`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fs=0;wire bv,cv,mv,nt;wire [10:0] xl,xr,rl,rr;wire [9:0] yt,yb,rt,rb;wire [3:0] status;wire [15:0] seq;
 blue_plate_locator dut(clk,rst,fs,1'b0,1'b0,1'b0,11'd0,10'd0,24'd0,bv,cv,xl,xr,rl,rr,yt,yb,rt,rb,status,seq,mv,nt);
 task measure;input integer shift,background,weak;begin
  @(negedge clk);dut.active=background ? 4'b0011:4'b0001;
  dut.lefts[0]=50+shift;dut.rights[0]=149+shift;dut.tops[0]=60;dut.bottoms[0]=89;
  dut.blues[0]=weak ? 200:900;dut.edges[0]=weak ? 300:600;dut.text_rows[0]=12;
  dut.lefts[1]=180;dut.rights[1]=310;dut.tops[1]=10;dut.bottoms[1]=49;dut.blues[1]=5000;dut.edges[1]=1200;dut.text_rows[1]=12;
  repeat(9)@(negedge clk);fs=1;@(negedge clk);fs=0;
 end endtask
 task missing;begin @(negedge clk);dut.active=0;repeat(9)@(negedge clk);fs=1;@(negedge clk);fs=0;end endtask
 integer i;
 initial begin
  repeat(4)@(negedge clk);rst=1;
  measure(0,0,1);measure(0,0,1);if(bv)$fatal(1,"weak background acquired as plate");
  measure(0,0,0);measure(0,0,0);if(!bv || !mv || !nt || xl!=192)$fatal(1,"initial track failed");
  @(negedge clk);if(nt)$fatal(1,"new target is not one clock");
  measure(1,1,0);if(!bv || !mv || xl!=192)$fatal(1,"background stole target or jitter not filtered");
  measure(2,1,1);if(!bv || !mv || xl!=196)$fatal(1,"weak associated observation not retained");
  for(i=0;i<1;i=i+1)begin missing;if(!bv || mv)$fatal(1,"hold/fresh separation failed");end
  missing;if(bv || mv)$fatal(1,"lost track did not expire");
  measure(0,0,0);measure(0,0,0);if(!bv || !mv || !nt)$fatal(1,"track reacquisition failed");
  $display("PASS tracking association, background priority, weak evidence, jitter, hold and reacquisition");$finish;
 end
 initial begin #50000;$fatal(1,"timeout");end
endmodule
