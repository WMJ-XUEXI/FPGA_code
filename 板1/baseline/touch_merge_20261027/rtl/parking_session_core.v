`timescale 1ns/1ps
// Reused single-clock sequential session core. plate is an opaque canonical key.
// 64 bits is the existing internal carrier, NOT an agreed wire encoding.
// Upstream must supply validated identities; adapt width after protocol agreement.
module parking_session_core #(parameter integer CLOCK_HZ=75000000)(
 input clk,rst_n,input request,input [1:0] operation,input [63:0] plate,
 output reg busy,done,output reg [3:0] result,output reg [4:0] count,
 output wire selected_active,output wire [31:0] elapsed,
 output reg [31:0] seconds,output reg [31:0] last_duration);
 reg [31:0] divider;
 reg [63:0] plates[0:21]; reg [31:0] entered[0:21]; reg [21:0] active;
 reg [4:0] index,match_slot,free_slot,view_slot;
 reg found,free_found,view_valid,commit;
 reg [63:0] pending_plate; reg [1:0] pending_op;
 reg [31:0] event_time;
 assign selected_active=view_valid && active[view_slot];
 assign elapsed=selected_active ? seconds-entered[view_slot]:last_duration;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   divider<=0;seconds<=0;active<=0;busy<=0;done<=0;result<=0;count<=0;
   index<=0;match_slot<=0;free_slot<=0;found<=0;free_found<=0;commit<=0;
   pending_plate<=0;pending_op<=0;event_time<=0;view_slot<=0;view_valid<=0;last_duration<=0;
  end else begin
   done<=0;
   if(divider==CLOCK_HZ-1)begin divider<=0;seconds<=seconds+1'b1;end
   else divider<=divider+1'b1;
   if(!busy)begin
    if(request)begin
     busy<=1;commit<=0;index<=0;found<=0;free_found<=0;
     pending_plate<=plate;pending_op<=operation;event_time<=seconds;
    end
   end else if(!commit)begin
    if(active[index] && plates[index]==pending_plate)begin found<=1;match_slot<=index;end
    if(!active[index] && !free_found)begin free_found<=1;free_slot<=index;end
    if(index==21)commit<=1;else index<=index+1'b1;
   end else begin
    busy<=0;done<=1;view_valid<=0;last_duration<=0;
    if(pending_plate==0)result<=6;
    else if(pending_op==1)begin
     if(found)begin result<=3;view_valid<=1;view_slot<=match_slot;end
     else if(!free_found)result<=5;
     else begin
      active[free_slot]<=1;plates[free_slot]<=pending_plate;entered[free_slot]<=event_time;
      count<=count+1'b1;view_slot<=free_slot;view_valid<=1;result<=1;
     end
    end else if(pending_op==2)begin
     if(!found)result<=4;
     else begin
      active[match_slot]<=0;count<=count-1'b1;
      last_duration<=event_time-entered[match_slot];result<=2;
     end
    end else if(pending_op==0)begin
     if(found)begin view_valid<=1;view_slot<=match_slot;result<=7;end
     else result<=4;
    end else result<=6;
   end
  end
 end
endmodule
