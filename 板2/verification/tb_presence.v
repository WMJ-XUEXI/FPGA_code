`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,fs=0,pv=0,upd=0;reg [10:0] x=256;reg [9:0] y=200;reg [23:0] rgb=0;wire residual;
 plate_presence_guard g(clk,rst,1'b0,fs,pv,upd,x,11'd256,11'd1023,y,10'd180,10'd371,rgb,residual);
 reg toggle=0,button=1,daily_button=1;wire clear_daily;wire [1:0] page;wire clear_all;
 parking_controls #(.CLOCK_HZ(100)) c(clk,rst,fs,toggle,button,page,clear_all,daily_button,clear_daily);
 integer i,clears=0;always @(posedge clk)if(clear_all)clears=clears+1;
 task frame;begin @(negedge clk);pv=0;fs=1;@(negedge clk);fs=0;end endtask
 task pixels;input integer blues,whites;begin
  for(i=0;i<blues+whites;i=i+1)begin @(negedge clk);pv=1;x=256+i*4;rgb=i<blues ? 24'h102080:24'hffffff;end
  @(negedge clk);pv=0;frame;
 end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;upd=1;@(negedge clk);upd=0;
  pixels(32,8);if(!residual)$fatal(1,"partial plate was treated as departure");
  pixels(0,50);if(residual)$fatal(1,"white background kept false presence");
  pixels(50,0);if(residual)$fatal(1,"plain blue background kept false presence");
  pixels(0,0);if(residual)$fatal(1,"empty scene kept presence");
  toggle=1;repeat(4)@(negedge clk);if(page)$fatal(1,"page changed midframe");frame;if(page!=1)$fatal(1,"page button failed");
  button=0;repeat(100)@(negedge clk);button=1;repeat(2)@(negedge clk);if(clears)$fatal(1,"short press cleared records");
  button=0;repeat(210)@(negedge clk);if(clears!=1 || page!=0)$fatal(1,"two-second clear failed");repeat(210)@(negedge clk);if(clears!=1)$fatal(1,"held button repeated clear");
  button=1;repeat(2)@(negedge clk);daily_button=0;repeat(200)@(negedge clk);if(!clear_daily)$fatal(1,"daily reset long press failed");@(negedge clk);if(clear_daily)$fatal(1,"daily reset was not one shot");
  $display("PASS partial presence guard, white/blue background rejection, frame-aligned paging and two-second one-shot clear");$finish;
 end
endmodule
