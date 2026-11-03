`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,fs=0;reg [3:0] buttons=15;
 wire blue,toggle,unused,debug;wire [3:0] stable;wire [1:0] page;wire clear_all,clear_daily;reg [5:0] used=0;
 locator_keys #(.CLOCK_HZ(1000)) keys(clk,rst,buttons,blue,toggle,unused,debug,stable);
 parking_controls #(.CLOCK_HZ(1000)) controls(clk,rst,fs,toggle,stable[2],page,clear_all,stable[0],clear_daily,used);
 integer day_count=0,all_count=0;always @(posedge clk)begin if(clear_daily)day_count=day_count+1;if(clear_all)all_count=all_count+1;end
 task press;input integer key,clocks;begin @(negedge clk);buttons[key]=0;repeat(clocks)@(negedge clk);buttons[key]=1;repeat(50)@(negedge clk);end endtask
 task frame;begin fs=1;@(negedge clk);fs=0;@(negedge clk);end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;press(0,50);if(!blue || day_count)$fatal(1,"short K1 did not toggle only overlay");
  press(0,2050);if(!blue || day_count!=1)$fatal(1,"long K1 changed overlay or failed daily clear");
  press(1,50);frame;if(page!=1)$fatal(1,"K2 paging failed");
  press(2,50);frame;if(page!=1 || all_count)$fatal(1,"short K3 changed page or cleared data");
  press(2,2050);frame;if(page || all_count!=1)$fatal(1,"long K3 did not clear/reset page once");
  used=9;frame;if(page!=1)$fatal(1,"automatic second page failed");used=17;frame;if(page!=2)$fatal(1,"automatic third page failed");
  $display("PASS debounced physical keys: short K1 overlay, long K1 daily clear without overlay change, K2 page, short K3 no action, long K3 full reset");$finish;
 end
endmodule
