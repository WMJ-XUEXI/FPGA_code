module parking_manager_ui(input clk,rst_n,snapshot,input page,
 input [1:0] pending_op,input [63:0] plate,input [3:0] result,
 input [4:0] count,input selected_active,input [31:0] elapsed,input busy,
 input [8:0] x,y,output reg page_frame,output [15:0] rgb,
 input backend_ready,identity_ready);
 reg [1:0] pending_q;reg [3:0] result_q;
 reg [4:0] count_q;reg active_q,busy_q;
 wire [3:0] h10,h1,m10,m1;wire time_over,time_busy;
 reg backend_q,identity_q;
 parking_hhmm time_text(clk,rst_n,snapshot,elapsed,h10,h1,m10,m1,time_over,time_busy);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin page_frame<=0;pending_q<=0;result_q<=0;
   count_q<=0;active_q<=0;busy_q<=0;backend_q<=0;identity_q<=0;end
  else if(snapshot)begin
   page_frame<=page;pending_q<=pending_op;result_q<=result;
   count_q<=count;active_q<=selected_active;busy_q<=busy;
   backend_q<=backend_ready;identity_q<=identity_ready;
  end
 end
 reg [255:0] text;reg [8:0] tx,ty;reg [15:0] bg;
 reg [7:0] ch;reg [7:0] ch_q;reg [3:0] row_q;reg [2:0] col_q;
 reg [15:0] bg_q;wire ink;
 panel_font font(ch_q,row_q,col_q,ink);
 assign rgb=ink ? 16'hffff:bg_q;
 always @(posedge clk)begin
  ch_q<=ch;row_q<=ty[3:0];col_q<=tx[2:0];bg_q<=bg;
 end
 function [7:0] digit;input [3:0] n;begin digit=8'd48+{4'd0,n};end endfunction
 always @* begin
  text=0;tx=0;ty=0;bg=16'h0841;ch=32;
  if(y<24)begin
   tx=x-8;ty=y;
   text={"PARKING MANAGER",136'd0};
  end
  if(y>=40 && y<80)begin
   ty=y-52;
   if(x>=16 && x<152)begin bg=pending_q==1 ? 16'h046c:16'h1949;tx=x-56;text={"ENTER",216'd0};end
   else if(x>=168 && x<304)begin bg=pending_q==2 ? 16'hb800:16'h1949;tx=x-216;text={"EXIT",224'd0};end
   else if(x>=320 && x<464)begin bg=16'h1949;tx=x-368;text={"QUERY",216'd0};end
  end
  if(y>=96 && y<112)begin ty=y-96;tx=x-24;
  if(!backend_q)text={"PLATE LINK PENDING",112'd0};
  else if(!identity_q)text={"NO VALID IDENTITY",120'd0};
  else text={"VALIDATED ID READY",112'd0};
 end
  if(y>=120 && y<136)begin ty=y-120;tx=x-24;
   if(pending_q==1)text={"CONFIRM ENTRY",152'd0};
   else if(pending_q==2)text={"CONFIRM EXIT",160'd0};
   else text={"SELECT ENTRY OR EXIT",96'd0};
  end
  if(y>=144 && y<184)begin ty=y-156;
   if(x>=16 && x<224)begin bg=16'h4208;tx=x-40;text={"IDENTITY",192'd0};end
   else if(x>=240 && x<464)begin bg=pending_q==0 ? 16'h4208:16'h046c;tx=x-312;text={"CONFIRM",200'd0};end
  end
  if(y>=200 && y<216)begin ty=y-200;tx=x-24;
   if(busy_q)text={"PROCESSING",176'd0};
   else case(result_q)
    1:text={"ENTRY OK",192'd0};2:text={"EXIT OK",200'd0};
    3:text={"ALREADY INSIDE",144'd0};4:text={"NOT INSIDE",176'd0};
    5:text={"TABLE FULL",176'd0};6:text={"INVALID REQUEST",136'd0};
    7:text={"IN PARK",200'd0};
    8:text={"BACKEND NOT READY",120'd0};
    9:text={"NO ENTRY IDENTITY",120'd0};
    10:text={"NO EXIT IDENTITY",128'd0};
    default:text={"READY",216'd0};endcase
  end
  if(y>=224 && y<240)begin ty=y-224;tx=x-24;
   text={"TIME ",digit(h10),digit(h1),":",digit(m10),digit(m1),time_over ? "+":" ",168'd0};
  end
  if(y>=248 && y<264)begin ty=y-248;tx=x-24;
   text={"INSIDE ",8'd48+(count_q>=20 ? 8'd2:count_q>=10 ? 8'd1:8'd0),8'd48+(count_q>=20 ? {3'd0,count_q}-8'd20:count_q>=10 ? {3'd0,count_q}-8'd10:{3'd0,count_q})," / 22",144'd0};
  end
  if(y>=280 && y<296)begin ty=y-280;tx=x-24;text={"FPGA TIME - NO HISTORY",80'd0};end
  if(y>=264 && y<312 && x>=360 && x<472)begin
   bg=16'h1949;tx=x-392;ty=y-280;text={"BACK",224'd0};
  end
  if(tx<256 && ty<16)ch=text[255-tx[7:3]*8-:8];
 end
endmodule
