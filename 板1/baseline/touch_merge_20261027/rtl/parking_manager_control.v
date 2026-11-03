// Touch navigation adapted from the 1027 delivery. No mock identity source.
// All identity inputs are same-clock, stable, canonical, already validated by
// the future RX adapter (format/checksum/CNN-valid/timeout/message dedup).
module parking_manager_control #(parameter integer CLOCK_HZ=75000000)(
 input clk,rst_n,tap,input [8:0] x,y,
 input backend_ready,entry_valid,exit_valid,
 input [63:0] entry_plate,exit_plate,
 output reg page,output reg [1:0] pending_op,
 output reg enter_request,exit_request,
 output wire [63:0] plate,output reg [3:0] result,output wire [4:0] count,
 output selected_active,output [31:0] elapsed,output wire busy);
 reg core_request,waiting;
 reg [1:0] core_op;
 reg [63:0] core_plate;
 wire core_busy,core_done;wire [3:0] core_result;
 wire [31:0] seconds,last_duration;
 assign busy=waiting || core_busy || core_request;
 assign plate=core_plate;
 parking_session_core #(.CLOCK_HZ(CLOCK_HZ)) core(
 .clk(clk),.rst_n(rst_n),.request(core_request),.operation(core_op),.plate(core_plate),
 .busy(core_busy),.done(core_done),.result(core_result),.count(count),
 .selected_active(selected_active),.elapsed(elapsed),.seconds(seconds),.last_duration(last_duration));
 // Keep donor ENTRY/EXIT -> CONFIRM semantics. KEY1..4 never require confirmation.
 wire back_hit=x>=360 && x<472 && y>=264 && y<312;
 wire entry_hit=y>=40 && y<80 && x>=16 && x<152;
 wire exit_hit=y>=40 && y<80 && x>=168 && x<304;
 wire query_hit=y>=40 && y<80 && x>=320 && x<464;
 wire confirm_hit=y>=144 && y<184 && x>=240 && x<464;
 wire submit=tap && page && !busy && (query_hit || (confirm_hit && pending_op!=0));
 wire [1:0] submit_op=query_hit ? 2'd0:pending_op;
 wire id_valid=submit_op==1 ? entry_valid:submit_op==2 ? exit_valid:(exit_valid || entry_valid);
 wire [63:0] id_value=submit_op==1 ? entry_plate:submit_op==2 ? exit_plate:
                              exit_valid ? exit_plate:entry_plate;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   page<=0;pending_op<=0;enter_request<=0;exit_request<=0;
   core_request<=0;waiting<=0;core_op<=0;core_plate<=0;result<=0;
  end else begin
   enter_request<=0;exit_request<=0;core_request<=0;
   if(core_done && waiting)begin result<=core_result;waiting<=0;end
   if(tap && !busy)begin
    if(!page)begin if(back_hit)begin page<=1;pending_op<=0;end end
    else if(back_hit)begin page<=0;pending_op<=0;end
    else if(entry_hit)pending_op<=1;
    else if(exit_hit)pending_op<=2;
   end
   if(submit)begin
    pending_op<=0;
    enter_request<=submit_op==1;exit_request<=submit_op==2;
    if(!backend_ready)result<=8; // Explicit pending integration, never a fake success.
    else if(!id_valid)result<=submit_op==2 ? 10:9;
    else if(id_value==0)result<=6;
    else begin core_plate<=id_value;core_op<=submit_op;core_request<=1;waiting<=1;end
   end
  end
 end
endmodule
