`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,rx=1;wire tx,rd,vs,hs,de;wire [23:0] rgb;
 blueplate_video_mixer dut(clk,rst,1'b0,1'b0,1'b0,1'b0,1'b0,4'd0,4'b1111,rd,24'd0,vs,hs,de,rgb,rx,tx);
 wire [55:0] now=dut.now_bcd;wire valid=dut.clock_valid;wire [1:0] mode=dut.panel_mode;
 wire [15:0] ti=dut.total_in,to=dut.total_out,tf=dut.total_flow;wire [7:0] occ=dut.occupied;
 reg [48:0] value;
 task appear;input [48:0] p;begin
  @(negedge clk);value=p;force dut.confirmed=value;force dut.entry_event=1;
  @(negedge clk);release dut.entry_event;repeat(300)@(negedge clk);release dut.confirmed;
 end endtask
 task byte;input [7:0] b;integer k;begin
  rx=0;repeat(651)@(negedge clk);for(k=0;k<8;k=k+1)begin rx=b[k];repeat(651)@(negedge clk);end
  rx=1;repeat(651)@(negedge clk);
 end endtask
 task view;begin
  force dut.key_state=4'b0111;repeat(3)@(negedge clk);force dut.key_state=4'b1111;repeat(3)@(negedge clk);
  force dut.user2=1;@(negedge clk);release dut.user2;release dut.key_state;repeat(3)@(negedge clk);
 end endtask
 reg [127:0] message="T20261030235959\n";integer k;
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(100)@(negedge clk);
  appear(12345);if(ti!=1 || occ!=1 || valid)$fatal(1,"unsynced visit lost");
  for(k=15;k>=0;k=k-1)byte(message[k*8 +:8]);repeat(100)@(negedge clk);
  if(!valid || now!==56'h20261030235959 || ti || to || tf || occ!=1)$fatal(1,"UART->clock->daily-reset integration failed");
  appear(45678);if(ti!=1 || tf!=1 || occ!=2)$fatal(1,"dated entry not counted");
  @(negedge clk);dut.clock_unit.divider=74999999;repeat(4)@(negedge clk);
  if(now!==56'h20261031000000 || ti || to || tf || occ!=2 || dut.history_used!=2)$fatal(1,"midnight cleared occupancy/history or failed daily reset");
  view;if(mode!=1 || dut.frame_time!==now || !dut.frame_clock_valid)$fatal(1,"K4 date view not selected at frame boundary");
  view;if(mode!=0)$fatal(1,"K4 duration view missing");
  view;if(mode!=1)$fatal(1,"K4 two-view wrap failed");
  force dut.history_used=9;force dut.history_page=1;force dut.user2=1;@(negedge clk);release dut.user2;
  force dut.y2=624;repeat(5)@(negedge clk);
  if(dut.rows_top!=624 || dut.history_address!=8)$fatal(1,"last-page compact RAM address failed");
  force dut.history_used=0;force dut.history_page=0;force dut.user2=1;@(negedge clk);release dut.user2;repeat(3)@(negedge clk);
  if(dut.rows_top!=624)$fatal(1,"empty panel height failed");
  force dut.history_used=8;force dut.user2=1;@(negedge clk);release dut.user2;repeat(3)@(negedge clk);
  if(dut.rows_top!=400)$fatal(1,"full page height failed");
  force dut.history_used=4;force dut.user2=1;@(negedge clk);release dut.user2;repeat(3)@(negedge clk);
  if(dut.rows_top!=528)$fatal(1,"four-row compact panel height failed");
  force dut.history_used=7;repeat(3)@(negedge clk);if(dut.rows_top!=528)$fatal(1,"row count changed within video frame");
  release dut.history_used;release dut.history_page;release dut.y2;
  $display("PASS calendar integration: physical UART bytes, calibrated date, midnight daily reset with occupied/history carry-over, frame-stable date and two K4 views");$finish;
 end
 initial begin #3000000;$fatal(1,"timeout");end
endmodule
