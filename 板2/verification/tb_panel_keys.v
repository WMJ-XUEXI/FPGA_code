`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,fs=0,key_n=1;wire view,hidden;
 panel_controls #(.CLOCK_HZ(8)) dut(clk,rst,fs,key_n,view,hidden);
 task frame;begin @(negedge clk);fs=1;@(negedge clk);fs=0;end endtask
 task short;begin @(negedge clk);key_n=0;repeat(3)@(negedge clk);key_n=1;repeat(2)@(negedge clk);end endtask
 initial begin
  repeat(3)@(negedge clk);rst=1;
  short;if(view || hidden)$fatal(1,"view changed before frame boundary");frame;if(!view || hidden)$fatal(1,"short did not select time view");
  short;frame;if(view)$fatal(1,"third panel still reachable");
  key_n=0;repeat(7)@(negedge clk);frame;
  frame;if(!hidden || view)$fatal(1,"long press did not hide");
  repeat(20)@(negedge clk);frame;if(!hidden)$fatal(1,"hold repeatedly toggled hidden");
  key_n=1;repeat(3)@(negedge clk);frame;if(view || !hidden)$fatal(1,"long release became short press");
  short;frame;if(!view || !hidden)$fatal(1,"hidden view selection failed");
  key_n=0;repeat(9)@(negedge clk);frame;if(hidden || !view)$fatal(1,"long did not restore selected view");
  key_n=1;repeat(3)@(negedge clk);frame;if(hidden || !view)$fatal(1,"restore release changed view");
  $display("PASS panel short/long keys: two views, frame atomicity, hide/show once per hold, no short after long");$finish;
 end
endmodule
