`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,fs=0,pv=0,update=0,measurement=1,done=0;
 reg [10:0] x=0;reg [9:0] y=0;reg [23:0] rgb=0;wire present;
 plate_presence_guard guard(clk,rst,1'b0,fs,pv,update,x,11'd24,11'd280,y,10'd24,10'd104,rgb,present);
 wire [48:0] plate;wire locked,enable,abort_capture,event_out,verified,exit_event,active;wire [15:0] visits;
 plate_confirmation #(.ABSENT_FRAMES(4)) session(clk,rst,fs,present,1'b0,done,1'b1,49'd12345,plate,locked,enable,abort_capture,event_out,visits,1'b0,verified,measurement,1'b0,exit_event,active);
 wire [86:0] record;wire history_ready,history_full;wire [5:0] history_used;
 wire [15:0] total_in,total_out,total_flow;wire [7:0] occupied;
 parking_records history(clk,rst,1'b0,event_out,plate,5'd0,record,history_ready,history_full,history_used,1'b0,total_in,total_out,total_flow,occupied,57'd0,,,,);
 integer xx,yy,i,events=0;
 always @(posedge clk)if(event_out)events=events+1;
 task frame;input integer pattern;begin
  @(negedge clk);fs=1;@(negedge clk);fs=0;pv=1;
  for(yy=0;yy<128;yy=yy+1)for(xx=0;xx<320;xx=xx+1)begin
   x=xx;y=yy;rgb=24'h183890;
   if(pattern==0 && xx>280)rgb=24'hffffff;
   if(pattern==1 && (xx<32 || xx>272 || yy<32 || yy>96))rgb=24'hffffff;
   if(pattern==2 && xx>=40 && xx<=264 && yy>=36 && yy<=92 && (xx%28)<8)rgb=24'hffffff;
   if(pattern==3)rgb=24'hffffff;
   if(pattern==4)rgb=24'hc8dcf0;
   @(negedge clk);
  end
  pv=0;fs=1;@(negedge clk);fs=0;
 end endtask
 task attempt;begin @(negedge clk);done=1;@(negedge clk);done=0;end endtask
 initial begin
  repeat(4)@(negedge clk);rst=1;update=1;@(negedge clk);update=0;
  frame(0);if(present)$fatal(1,"adjacent white background kept presence");
  frame(1);if(present)$fatal(1,"plain blue rectangle/white border kept presence");
  frame(4);if(present)$fatal(1,"near-neutral tinted white kept presence");
  frame(2);if(!present)$fatal(1,"blue plate text not detected");
  repeat(4)attempt;repeat(2)@(negedge clk);if(!locked || visits!=1)$fatal(1,"initial recognition failed");
  measurement=0;for(i=0;i<12;i=i+1)frame(2);
  if(!locked || verified || visits!=1)$fatal(1,"OCR loss reset physically visible plate");
  measurement=1;frame(2);repeat(5)attempt;repeat(2)@(negedge clk);
  repeat(150)@(negedge clk);
  if(total_in!=1 || total_out!=0 || total_flow!=1 || occupied!=1 || history_used!=1)$fatal(1,"quality loss caused false parking OUT or extra record");
  if(visits!=1 || events!=1 || !verified)$fatal(1,"same visible plate recorded twice");
  measurement=0;repeat(4)frame(3);repeat(5)begin @(negedge clk);fs=1;@(negedge clk);fs=0;end
  if(locked)$fatal(1,"actual absence did not rearm");
  measurement=1;frame(2);repeat(4)attempt;repeat(2)@(negedge clk);
  repeat(150)@(negedge clk);
  if(total_in!=1 || total_out!=1 || occupied!=0 || history_used!=1)$fatal(1,"true return failed parking OUT pairing");
  if(visits!=2 || events!=2)$fatal(1,"genuine reentry was incorrectly suppressed");
  $display("PASS local presence stability: reject adjacent white/blue/tinted background, retain unreadable visible plate, one registration after recovery, allow true reentry");$finish;
 end
 initial begin #12000000;$fatal(1,"timeout");end
endmodule
