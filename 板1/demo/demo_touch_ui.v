module demo_touch_ui(
 input clk,rst_n,snapshot,page,input [2:0] active,input [47:0] ids,input [59:0] ids_decimal,input [5:0] codes,
 input [15:0] selected_id,input [3:0] result,input [8:0] x,y,output reg page_frame,output reg [15:0] rgb
);
 reg [2:0] active_q;reg [47:0] ids_q;reg [5:0] codes_q;reg [15:0] selected_q;reg [3:0] result_q;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin page_frame<=0;active_q<=0;ids_q<=0;codes_q<=0;selected_q<=0;result_q<=0;end
  else if(snapshot)begin page_frame<=page;active_q<=active;ids_q<=ids;codes_q<=codes;selected_q<=selected_id;result_q<=result;end
 end
function [255:0] message;input [3:0] n;begin case(n)
 4'd0:message={"READY / DONE",160'd0};
 4'd1:message={"SELECT VEHICLE",144'd0};
 4'd2:message={"OBSERVATION REQUIRED",96'd0};
 4'd3:message={"ALREADY REGISTERED",112'd0};
 4'd4:message={"NOT REGISTERED",144'd0};
 4'd5:message={"BUSY",224'd0};
 4'd6:message={"SELECT ONE VEHICLE",112'd0};
 4'd7:message={"TABLE FULL",176'd0};
 default:message=0;endcase end endfunction

 reg [255:0] text;reg [8:0] tx,ty;reg [7:0] ch;reg [15:0] bg;
 reg plate_area;reg [1:0] plate_code;reg [6:0] px;reg [4:0] py;
 reg [15:0] id;integer slot,col;
 wire ink,plate_ink;panel_font font(ch,ty[3:0],tx[2:0],ink);
 demo_plate_font plate_font(plate_code,px,py,plate_ink);
 always @*begin
  text=0;tx=0;ty=0;ch=32;bg=16'h0841;plate_area=0;plate_code=0;px=0;py=0;id=0;slot=-1;col=0;
  if(y<24)begin tx=x-16;ty=y;text={"DEMO VEHICLES",152'd0};end
  else if(y>=40 && y<80)begin
   ty=y-52;
   if(x>=16 && x<224)begin bg=16'h046c;tx=x-80;text={"ENTER",216'd0};end
   else if(x>=240 && x<464)begin bg=16'hb800;tx=x-312;text={"EXIT",224'd0};end
  end else if(y>=96 && y<128)begin slot=0;ty=y-100;end
  else if(y>=136 && y<168)begin slot=1;ty=y-140;end
  else if(y>=176 && y<208)begin slot=2;ty=y-180;end
  else if(y>=232 && y<248)begin tx=x-16;ty=y-232;text=message(result_q);end
  else if(y>=264 && y<312 && x>=360 && x<472)begin bg=16'h1949;tx=x-400;ty=y-280;text={"BACK",224'd0};end
  if(slot>=0)begin
   id=ids_q[slot*16+:16];
   bg=active_q[slot] && id==selected_q ? 16'h046c:16'h1949;tx=x-16;
   if(active_q[slot])begin
    if(x>=104 && x<192 && ty<16)begin plate_area=1;px=x-104;py=ty;plate_code=codes_q[slot*2+:2];end
   end else text={"EMPTY",216'd0};
  end
  col=tx>>3;if(col<32 && ty<16)ch=text[255-col*8 -:8];
 end
 always @(posedge clk)rgb<=plate_area ? (plate_ink ? 16'hf904:bg):(ink ? 16'hffff:bg);
endmodule
