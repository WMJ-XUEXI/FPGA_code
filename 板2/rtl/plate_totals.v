// Per distinct plate, not per OCR attempt. Updates only on accepted IN/OUT.
module plate_totals(
 input clk,rst_n,clear_all,event_valid,event_in,input [48:0] event_plate,
 input [4:0] display_address,output reg [65:0] display_record,
 output reg [5:0] used,output ready
);
 (* ram_style="bram" *)reg [65:0] records[0:31];
 reg [65:0] q;reg [4:0] address;reg [2:0] state;
 reg pending,pending_in;reg [48:0] pending_plate;
 localparam INIT=0,IDLE=1,READ=2,CHECK=3,WRITE=4;
 function [7:0] inc;input [7:0] v;begin
  inc=v==8'h99 ? v:v[3:0]==9 ? {v[7:4]+4'd1,4'd0}:v+1'b1;
 end endfunction
 wire found=q[65] && q[64:16]==pending_plate;
 wire [65:0] updated={1'b1,pending_plate,pending_in ? inc(found ? q[15:8]:8'd0):(found ? q[15:8]:8'd0),
  pending_in ? (found ? q[7:0]:8'd0):inc(found ? q[7:0]:8'd0)};
 assign ready=state!=INIT;
 always @(posedge clk)begin
  q<=records[address];display_record<=records[display_address];
  if(state==INIT)records[address]<=0;
  else if(state==WRITE)records[address]<=updated;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=INIT;address<=0;used<=0;pending<=0;pending_in<=0;pending_plate<=0;end
  else if(clear_all)begin state<=INIT;address<=0;used<=0;pending<=0;end
  else begin
   if(event_valid)begin pending<=1;pending_in<=event_in;pending_plate<=event_plate;end
   case(state)
    INIT:if(address==31)begin address<=0;state<=IDLE;end else address<=address+1'b1;
    IDLE:if(pending)begin address<=0;state<=READ;end
    READ:state<=CHECK;
    CHECK:if(found || !q[65])state<=WRITE;else if(address==31)begin pending<=0;state<=IDLE;end else begin address<=address+1'b1;state<=READ;end
    WRITE:begin if(!found)used<=used+1'b1;pending<=0;state<=IDLE;end
    default:state<=INIT;
   endcase
  end
 end
endmodule
