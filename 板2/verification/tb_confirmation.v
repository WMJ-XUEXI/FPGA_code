`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fs=0,present=0,retry=0,done=0,good=0,new_target=0,measured=0,clear_all=0;reg [48:0] raw=12345;
 wire [48:0] plate;wire locked,enable,abort_capture,event_out,verified,exit_event,active;wire [15:0] visits;
 plate_confirmation #(.ABSENT_FRAMES(4)) dut(clk,rst,fs,present,retry,done,good,raw,plate,locked,enable,abort_capture,event_out,visits,new_target,verified,measured,clear_all,exit_event,active);
 task frame;begin @(negedge clk);fs=1;@(negedge clk);fs=0;end endtask
 task attempt;input [48:0] value;input quality;begin @(negedge clk);raw=value;good=quality;done=1;@(negedge clk);done=0;end endtask
 integer i;
 initial begin
  repeat(4)@(negedge clk);rst=1;present=1;measured=1;frame;
  attempt(12345,1);attempt(12346,1);attempt(12345,1);attempt(12345,1);
  if(locked)$fatal(1,"three votes locked too early");attempt(12345,1);
  if(!locked || plate!=12345 || visits!=1 || !enable)$fatal(1,"four of five confirmation failed");
  attempt(0,0);if(verified)$fatal(1,"failed quality retained READY");attempt(12345,1);if(!verified)$fatal(1,"same identity did not recover READY");
  attempt(12346,1);attempt(12345,1);attempt(12346,1);attempt(12345,1);
  if(plate!=12345 || visits!=1)$fatal(1,"isolated ambiguous character switched plate");
  repeat(5)attempt(54321,1);
  if(plate!=54321 || visits!=2)$fatal(1,"no-gap replacement failed");
  measured=0;present=1;repeat(20)frame;
  if(!locked || verified || visits!=2)$fatal(1,"visible but unreadable plate rearmed");
  measured=1;frame;repeat(5)attempt(54321,1);
  if(!locked || visits!=2 || !verified)$fatal(1,"same visible plate was registered twice after quality loss");
  measured=0;present=0;frame;frame;
  if(verified)$fatal(1,"stale READY did not expire promptly");
  frame;frame;if(locked)$fatal(1,"departure did not reset");
  present=1;measured=1;frame;repeat(4)attempt(12345,1);
  if(!locked || visits!=3)$fatal(1,"reappearance failed");
  clear_all=1;@(negedge clk);clear_all=0;
  frame;repeat(6)attempt(12345,1);if(locked || visits)$fatal(1,"clear immediately re-added visible plate");
  present=0;measured=0;repeat(4)frame;present=1;measured=1;frame;repeat(4)attempt(12345,1);
  if(!locked || visits!=1)$fatal(1,"clear did not rearm after actual departure");
  $display("PASS presence-aware confirmation: continuous inference, ambiguity rejection, no-gap replacement, fast departure and clear");$finish;
 end
 initial begin #100000;$fatal(1,"timeout");end
endmodule
