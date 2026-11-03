`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,clear_all=0,clear_daily=0,event_in=0;reg [48:0] plate=12345;reg [4:0] addr=0;
 wire [86:0] rec;wire ready,full;wire [5:0] used;wire [15:0] total_in,total_out,total_flow;wire [7:0] occupied;
 parking_records #(.CLOCK_HZ(1000)) dut(clk,rst,clear_all,event_in,plate,addr,rec,ready,full,used,clear_daily,total_in,total_out,total_flow,occupied);
 wire [3:0] state=dut.state;wire pending=dut.event_pending;wire [11:0] count=rec[35:24];wire [23:0] duration=rec[23:0];wire running=rec[85];wire [48:0] number=rec[84:36];
 task idle;integer n;begin n=0;repeat(4)@(negedge clk);while(state!=1 || pending)begin @(negedge clk);n=n+1;if(n>2000)$fatal(1,"record controller stalled");end end endtask
 task enter;input [48:0] p;begin @(negedge clk);plate=p;event_in=1;@(negedge clk);event_in=0;idle;end endtask
 task readrow;input [4:0] a;begin addr=a;repeat(3)@(negedge clk);end endtask
 task totals;input [15:0] ti,to,tf;input [7:0] occ;begin if(total_in!=ti || total_out!=to || total_flow!=tf || occupied!=occ)$fatal(1,"traffic totals mismatch in=%h out=%h flow=%h park=%h",total_in,total_out,total_flow,occupied);end endtask
 reg [86:0] closed_a;integer i;
 initial begin
  repeat(4)@(negedge clk);rst=1;idle;
  enter(12345);readrow(0);if(used!=1 || count!=0 || !running)$fatal(1,"first A entry failed");totals(1,0,1,1);
  repeat(2200)@(negedge clk);enter(12345);readrow(0);closed_a=rec;if(count!=0 || running || duration<2)$fatal(1,"A exit failed");totals(1,1,2,0);
  enter(45678);readrow(1);if(used!=2 || number!=45678)$fatal(1,"B was not appended");totals(2,1,3,1);
  enter(12345);readrow(2);if(used!=3 || number!=12345 || count!=0 || !running || duration)$fatal(1,"A reentry did not append fresh row");totals(3,1,4,2);
  readrow(0);if(rec!==closed_a)$fatal(1,"A reentry rewrote previous completed visit");
  enter(12345);readrow(2);if(running || count!=0)$fatal(1,"A exit matched oldest row instead of active visit");totals(3,2,5,1);
  readrow(0);if(rec!==closed_a)$fatal(1,"second A exit rewrote first A visit");
  clear_daily=1;@(negedge clk);clear_daily=0;totals(0,0,0,1);if(used!=3)$fatal(1,"daily reset cleared records");
  enter(45678);totals(0,1,1,0);
  enter(12345);readrow(3);if(used!=4 || number!=12345 || count!=0)$fatal(1,"new visit cumulative appearance count failed");
  for(i=4;i<32;i=i+1)enter(100000+i);
  totals(16'h0029,1,16'h0030,8'h29);
  enter(999999);if(!full || used!=32)$fatal(1,"full list overwrote history");totals(16'h0029,1,16'h0030,8'h29);
  enter(12345);totals(16'h0029,2,16'h0031,8'h28);
  readrow(0);if(rec!==closed_a)$fatal(1,"full history changed oldest visit");
  clear_all=1;@(negedge clk);clear_all=0;idle;totals(0,0,0,0);if(used || full)$fatal(1,"full reset failed");
  // Reset-day concurrent with entry must include that event in the new day.
  @(negedge clk);plate=77777;event_in=1;@(negedge clk);event_in=0;
  while(state!=4)@(negedge clk);clear_daily=1;@(negedge clk);clear_daily=0;idle;totals(1,0,1,1);
  @(negedge clk);dut.total_out=16'h9999;dut.total_flow=16'h9999;
  enter(77777);totals(1,16'h9999,16'h9999,0);
  $display("PASS chronological visits: A-IN/A-OUT/B-IN/A-IN appends row 3, closed rows immutable, current active row exits, traffic totals, daily carry-over, full protection, decimal carry and saturation");$finish;
 end
 initial begin #500000;$fatal(1,"timeout");end
endmodule
