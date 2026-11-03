`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,clear_all=0,event_in=0;reg [48:0] plate=12345;reg [4:0] addr=0;
 reg [56:0] stamp=0;wire [86:0] row;wire [113:0] dates;wire ready,full,accepted,is_in;wire [48:0] accepted_plate;
 wire [5:0] used,unique_used;wire [15:0] ti,to,tf;wire [7:0] occ;wire [65:0] summary;
 parking_records #(.CLOCK_HZ(1000)) records(clk,rst,clear_all,event_in,plate,addr,row,ready,full,used,1'b0,ti,to,tf,occ,stamp,dates,accepted,is_in,accepted_plate);
 plate_totals totals(clk,rst,clear_all,accepted,is_in,accepted_plate,addr,summary,unique_used,);
 task settle;begin repeat(300)@(negedge clk);end endtask
 task enter;input [48:0] p;input [56:0] t;begin
  @(negedge clk);plate=p;stamp=t;event_in=1;@(negedge clk);event_in=0;stamp=0;settle;
 end endtask
 task read;input [4:0] a;begin addr=a;repeat(3)@(negedge clk);end endtask
 reg [113:0] closed;reg [23:0] duration;
 initial begin
  repeat(4)@(negedge clk);rst=1;settle;
  enter(12345,{1'b1,56'h20261030235950});read(0);
  if(dates!=={1'b1,56'h20261030235950,57'd0} || !row[85])$fatal(1,"IN timestamp snapshot lost");
  repeat(2200)@(negedge clk);
  enter(12345,{1'b1,56'h20261031000005});read(0);closed=dates;duration=row[23:0];
  if(dates!=={1'b1,56'h20261030235950,1'b1,56'h20261031000005} || row[85] || duration<2)$fatal(1,"OUT time/date or monotonic duration failed");
  enter(45678,{1'b1,56'h20261031000100});enter(12345,{1'b1,56'h20261031000200});
  read(0);if(dates!==closed)$fatal(1,"third appearance overwrote completed visit");
  if(summary!=={1'b1,49'd12345,8'h02,8'h01} || unique_used!=2)$fatal(1,"per-plate IN/OUT totals wrong");
  read(2);if(dates!=={1'b1,56'h20261031000200,57'd0} || !row[85])$fatal(1,"reentry did not append fresh dates");
  enter(12345,{1'b1,56'h20261030120000});read(2);
  if(row[85] || row[23:0]>24'h000002)$fatal(1,"backward clock correction changed elapsed duration");
  enter(78901,57'd0);read(3);if(dates!==0)$fatal(1,"unsynced entry fabricated a date");
  read(0);if(summary[15:0]!==16'h0202)$fatal(1,"second OUT total missing");
  clear_all=1;@(negedge clk);clear_all=0;settle;read(0);
  if(used || unique_used || dates || row || summary)$fatal(1,"clear left stale records");
  $display("PASS timestamp records: IN/OUT snapshots, cross-day visits, reentry append, immutable history, monotonic elapsed under backwards calibration, per-plate totals and unknown-time flags");$finish;
 end
 initial begin #200000;$fatal(1,"timeout");end
endmodule
