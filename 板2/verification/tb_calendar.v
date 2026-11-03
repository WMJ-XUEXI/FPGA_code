`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,rx=1;wire tx,valid,changed;wire [55:0] now;
 calendar_clock #(.CLOCK_HZ(10000),.BAUD(1000)) dut(clk,rst,rx,tx,now,valid,changed);
 integer days=0;reg [7:0] ack;integer acknowledgements=0,j;
 always @(posedge clk)if(changed)days=days+1;
 initial forever begin
  @(negedge tx);repeat(15)@(negedge clk);
  for(j=0;j<8;j=j+1)begin ack[j]=tx;repeat(10)@(negedge clk);end
  if(tx!==1)$fatal(1,"TX stop bit missing");acknowledgements=acknowledgements+1;
 end
 task byte;input [7:0] b;integer k;begin
  rx=0;repeat(10)@(negedge clk);
  for(k=0;k<8;k=k+1)begin rx=b[k];repeat(10)@(negedge clk);end
  rx=1;repeat(10)@(negedge clk);
 end endtask
 task settime;input [127:0] s;integer k;begin
  @(negedge clk);for(k=15;k>=0;k=k-1)byte(s[k*8 +:8]);repeat(120)@(negedge clk);
 end endtask
 task tick;begin @(negedge clk);dut.divider=9999;repeat(2)@(negedge clk);end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;
  repeat(10010)@(negedge clk);if(valid || now!=0)$fatal(1,"time invented before sync");
  settime("T20240228235959\n");if(!valid || now!==56'h20240228235959 || ack!="K")$fatal(1,"UART calibration failed %h",now);
  tick;if(now!==56'h20240229000000)$fatal(1,"leap day failed");
  settime("T20230228235959\n");tick;if(now!==56'h20230301000000)$fatal(1,"non-leap February failed");
  settime("T21000228235959\n");tick;if(now!==56'h21000301000000)$fatal(1,"2100 century rule failed");
  settime("T20000228235959\n");tick;if(now!==56'h20000229000000)$fatal(1,"2000 century rule failed");
  settime("T20261231235959\n");tick;if(now!==56'h20270101000000)$fatal(1,"year rollover failed");
  settime("T20260229000000\n");if(ack!="E" || now[55:24]!==32'h20270101)$fatal(1,"invalid day accepted");
  settime("T20261301000000\n");if(ack!="E")$fatal(1,"invalid month accepted");
  settime("T20261030240000\n");if(ack!="E")$fatal(1,"invalid hour accepted");
  settime("T20261030126000\n");if(ack!="E")$fatal(1,"invalid minute accepted");
  settime("T20261030123060\n");if(ack!="E")$fatal(1,"invalid second accepted");
  settime("T20261030123045\n");if(now!==56'h20261030123045)$fatal(1,"valid retry failed");
  @(negedge clk);dut.divider=0;repeat(9999)@(negedge clk);if(now!==56'h20261030123045)$fatal(1,"second period too short");
  repeat(2)@(negedge clk);if(now!==56'h20261030123046)$fatal(1,"second period too long");
  if(acknowledgements!=11 || days<6)$fatal(1,"ACK/day changes lost");
  $display("PASS UART calendar: calibration, invalid dates, leap years including 2000/2100, midnight/year rollover, 1Hz period, ACK and unsynced state");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
