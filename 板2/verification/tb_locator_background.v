`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,fs=0,pv=0,clear=0;
 reg [10:0] x=0;reg [9:0] y=0;reg [23:0] rgb=0;
 wire bv,cv,mv,nt;wire [10:0] l,r,rl,rr;wire [9:0] t,b,rt,rb;wire [3:0] status;wire [15:0] seq;
 blue_plate_locator dut(clk,rst,fs,pv,clear,1'b0,x,y,rgb,bv,cv,l,r,rl,rr,t,b,rt,rb,status,seq,mv,nt);
 integer xx,yy,k;
 task image;input integer pattern;begin
  @(negedge clk);fs=1;@(negedge clk);fs=0;pv=1;
  for(yy=0;yy<180;yy=yy+1)for(xx=0;xx<320;xx=xx+1)begin
   x=xx*4;y=yy*4;rgb=24'hffffff;
   if(xx>=60 && xx<=240 && yy>=60 && yy<=100)begin
    rgb=pattern==2 ? 24'hc8dcf0:24'h1038a0;
    if(pattern==1 && xx>=104 && xx<=120)rgb=24'hffffff;
    if(pattern==3 && yy>=68 && yy<=92 && xx>=72 && xx<=228 && (xx%22)<6)rgb=24'hffffff;
   end
   // Larger distractor nearby: blue field with no character texture.
   if(pattern==3 && xx>=255 && xx<=315 && yy>=20 && yy<=70)rgb=24'h1038a0;
   @(negedge clk);
  end
  pv=0;repeat(20)@(negedge clk);
 end endtask
 task reset_case;begin clear=1;@(negedge clk);clear=0;end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;
  for(k=0;k<3;k=k+1)begin
   reset_case;image(k);image(k);image(k);fs=1;@(negedge clk);fs=0;
   if(bv || cv)$fatal(1,"background accepted pattern=%0d",k);
  end
  reset_case;image(3);image(3);image(3);fs=1;@(negedge clk);fs=0;
  if(!bv || !mv || l<200 || r>1000 || t<210 || b>450)$fatal(1,"plate texture failed or distractor changed box l=%0d r=%0d t=%0d b=%0d",l,r,t,b);
  $display("PASS raw locator backgrounds: reject solid blue/white-border, blue regions bridged by white, near-neutral blue-tinted white; acquire seven-character texture beside blue distractor");$finish;
 end
 initial begin #10000000;$fatal(1,"timeout");end
endmodule
