// 32 chronological parking visits: closed rows never reopen; re-entry appends.
// One serial read/modify/write port plus an independent synchronous OSD read.
module parking_records #(parameter CLOCK_HZ=75000000)(
 input clk,rst_n,clear_all,entry_event,input [48:0] entry_plate,
 input [4:0] display_address,output [86:0] display_record,
 output ready,output reg full,output reg [5:0] used,
 input clear_daily,output reg [15:0] total_in,total_out,total_flow,output reg [7:0] occupied,
 input [56:0] current_stamp,output [113:0] display_stamp,
 output accepted_event,accepted_in,output [48:0] accepted_plate
);
 (* ram_style="bram" *)reg [74:0] records[0:31];
 (* ram_style="bram" *)reg [56:0] in_stamps[0:31],out_stamps[0:31];
 reg [74:0] core_q,display_core_q;reg [56:0] display_in_q,display_out_q;
 wire [86:0] q={core_q[74:24],12'd0,core_q[23:0]};
 assign display_record={display_core_q[74:24],12'd0,display_core_q[23:0]};
 assign display_stamp={display_in_q,display_out_q};reg [56:0] pending_stamp;reg [4:0] address;reg [3:0] state;
 reg [31:0] divider;reg tick_pending,event_pending;reg [48:0] pending_plate;
 reg [4:0] first_free;reg free_seen;
 localparam INIT=0,IDLE=1,FIND_REQ=2,FIND_CHECK=3,INSERT=4,TICK_REQ=5,TICK_WRITE=6;
 wire valid=q[86],running=q[85];wire [48:0] number=q[84:36];wire [11:0] count=q[35:24];wire [23:0] elapsed=q[23:0];
 assign ready=state!=INIT;
 function [15:0] total_next;input [15:0] c;begin
  if(c==16'h9999)total_next=c;
  else if(c[3:0]<9)total_next=c+1'b1;
  else if(c[7:4]<9)total_next={c[15:4]+12'd1,4'd0};
  else if(c[11:8]<9)total_next={c[15:8]+8'd1,8'd0};
  else total_next={c[15:12]+4'd1,12'd0};
 end endfunction
 wire entered=state==INSERT;
 wire departed=state==FIND_CHECK && valid && running && number==pending_plate;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin total_in<=0;total_out<=0;total_flow<=0;occupied<=0;end
  else if(clear_all)begin total_in<=0;total_out<=0;total_flow<=0;occupied<=0;end
  else begin
   if(clear_daily)begin total_in<=entered ? 16'd1:16'd0;total_out<=departed ? 16'd1:16'd0;total_flow<=entered || departed ? 16'd1:16'd0;end
   else if(entered || departed)begin
    total_flow<=total_next(total_flow);
    if(entered)total_in<=total_next(total_in);else total_out<=total_next(total_out);
   end
   if(entered)occupied<=occupied[3:0]==9 ? {occupied[7:4]+4'd1,4'd0}:occupied+1'b1;
   else if(departed && occupied!=0)occupied<=occupied[3:0]==0 ? {occupied[7:4]-4'd1,4'd9}:occupied-1'b1;
  end
 end
 function [23:0] time_next;input [23:0] t;begin
  if(t==24'h995959)time_next=t;
  else if(t[3:0]<9)time_next=t+1'b1;
  else if(t[7:4]<5)time_next={t[23:4]+20'd1,4'd0};
  else if(t[11:8]<9)time_next={t[23:8]+16'd1,8'd0};
  else if(t[15:12]<5)time_next={t[23:12]+12'd1,12'd0};
  else if(t[19:16]<9)time_next={t[23:16]+8'd1,16'd0};
  else time_next={t[23:20]+4'd1,20'd0};
 end endfunction
 // Keep RAM accesses synchronous and separate from state/BCD arithmetic.
 wire write_enable=state==INIT || state==INSERT || departed || (state==TICK_WRITE && valid && running);
 wire [4:0] write_address=state==INSERT ? first_free:address;
 wire [86:0] basic_write_data=state==INIT ? 87'd0:state==INSERT ? {1'b1,1'b1,pending_plate,12'd0,24'd0}:
  state==FIND_CHECK ? {1'b1,1'b0,number,12'd0,elapsed}:
  {valid,running,number,count,time_next(elapsed)};
 // Timestamps change only on accepted IN/OUT; second ticks update the compact core only.
 wire [74:0] core_write_data={basic_write_data[86:36],basic_write_data[23:0]};
 assign accepted_event=entered || departed;assign accepted_in=entered;assign accepted_plate=pending_plate;
 always @(posedge clk)begin
  core_q<=records[address];display_core_q<=records[display_address];
  if(write_enable)records[write_address]<=core_write_data;
 end
 always @(posedge clk)begin
  display_in_q<=in_stamps[display_address];display_out_q<=out_stamps[display_address];
  if(state==INIT || state==INSERT)in_stamps[write_address]<=state==INIT ? 57'd0:pending_stamp;
  if(state==INIT || state==INSERT || departed)out_stamps[write_address]<=departed ? pending_stamp:57'd0;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin state<=INIT;address<=0;divider<=0;tick_pending<=0;event_pending<=0;pending_plate<=0;pending_stamp<=0;first_free<=0;free_seen<=0;used<=0;full<=0;end
  else if(clear_all)begin state<=INIT;address<=0;divider<=0;tick_pending<=0;event_pending<=0;used<=0;full<=0;end
  else begin
   if(divider==CLOCK_HZ-1)begin divider<=0;tick_pending<=1;end else divider<=divider+1'b1;
   if(entry_event && ready)begin event_pending<=1;pending_plate<=entry_plate;pending_stamp<=current_stamp;end
   case(state)
    INIT:if(address==31)begin address<=0;state<=IDLE;end else address<=address+1'b1;
    IDLE:if(event_pending)begin address<=0;free_seen<=0;first_free<=0;state<=FIND_REQ;end
         else if(tick_pending)begin tick_pending<=0;address<=0;state<=TICK_REQ;end
    FIND_REQ:state<=FIND_CHECK;
    FIND_CHECK:begin
     if(departed)begin event_pending<=0;state<=IDLE;end
     else begin
      if(!valid && !free_seen)begin free_seen<=1;first_free<=address;end
      if(address==31)begin
       if(free_seen || !valid)begin if(!free_seen)first_free<=address;state<=INSERT;end
       else begin full<=1;event_pending<=0;state<=IDLE;end
      end else begin address<=address+1'b1;state<=FIND_REQ;end
     end
    end
    INSERT:begin event_pending<=0;used<=used+1'b1;state<=IDLE;end
    TICK_REQ:state<=TICK_WRITE;
    TICK_WRITE:if(address==31)state<=IDLE;else begin address<=address+1'b1;state<=TICK_REQ;end
    default:begin state<=INIT;address<=0;end
   endcase
  end
 end
endmodule

module parking_controls #(parameter CLOCK_HZ=75000000)(
 input clk,rst_n,frame_start,page_toggle,input clear_button_n,
 output reg [1:0] page,output reg clear_all,
 input daily_button_n,output reg clear_daily,input [5:0] history_used
);
 reg [5:0] used_q;wire [5:0] last_row=history_used-1'b1;reg page_q,page_pending;reg [31:0] held,daily_held;reg fired,daily_fired;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin used_q<=0;page_q<=0;page_pending<=0;page<=0;clear_all<=0;held<=0;fired<=0;daily_held<=0;daily_fired<=0;clear_daily<=0;end
  else begin
   if(frame_start)begin used_q<=history_used;if(history_used!=0 && history_used!=used_q)begin page<=last_row[4:3];page_pending<=0;end end
   page_q<=page_toggle;clear_all<=0;clear_daily<=0;
   if(daily_button_n)begin daily_held<=0;daily_fired<=0;end
   else if(!daily_fired)begin if(daily_held==2*CLOCK_HZ-1)begin clear_daily<=1;daily_fired<=1;end else daily_held<=daily_held+1'b1;end
   if(page_toggle!=page_q)page_pending<=1;
   if(frame_start && page_pending)begin page<=page+1'b1;page_pending<=0;end
   if(clear_button_n)begin held<=0;fired<=0;end
   else if(!fired)begin
    if(held==2*CLOCK_HZ-1)begin clear_all<=1;fired<=1;page<=0;page_pending<=0;end
    else held<=held+1'b1;
   end
  end
 end
endmodule

// K4 short release changes view; one-second hold toggles visibility once.
// Commands take effect only at a frame boundary; long release is not a short press.
module panel_controls #(parameter CLOCK_HZ=75000000)(
 input clk,rst_n,frame_start,key_n,output reg view,hidden
);
 reg [$clog2(CLOCK_HZ+1)-1:0] held;
 reg previous,fired,view_pending,hide_pending;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin held<=0;previous<=1;fired<=0;view_pending<=0;hide_pending<=0;view<=0;hidden<=0;end
  else begin
   previous<=key_n;
   if(!key_n)begin
    if(!fired)begin
     if(held==CLOCK_HZ-1)begin hide_pending<=~hide_pending;fired<=1;end
     else held<=held+1'b1;
    end
   end else begin
    held<=0;fired<=0;
    if(!previous && !fired)view_pending<=~view_pending;
   end
   if(frame_start)begin
    if(view_pending)begin view<=~view;view_pending<=0;end
    if(hide_pending)begin hidden<=~hidden;hide_pending<=0;end
   end
  end
 end
endmodule

// Local blue/white texture keeps a visit alive when OCR/box quality drops.
// Uses only the last measured plate interior; adjacent white surroundings are excluded.
module plate_presence_guard(
 input clk,rst_n,clear_all,frame_start,pixel_valid,update_box,
 input [10:0] x,box_l,box_r,input [9:0] y,box_t,box_b,
 input [23:0] rgb,output reg residual_present
);
 reg [10:0] left,right;reg [9:0] top,bottom;reg known;
 reg [15:0] blue_count,white_count;reg [7:0] text_rows;
 reg row_seen,row_blue,row_white;reg [9:0] row_y;reg [3:0] transitions;
 wire sample=pixel_valid && known && x[1:0]==0 && y[1:0]==0 && x>=left && x<=right && y>=top && y<=bottom;
 wire blue=rgb[7:0]>=48 && {1'b0,rgb[7:0]}>{1'b0,rgb[23:16]}+9'd16 && {1'b0,rgb[7:0]}>{1'b0,rgb[15:8]}+9'd8 &&
  {1'b0,rgb[7:0]}>={1'b0,rgb[23:16]}+{3'd0,rgb[7:2]} && {1'b0,rgb[7:0]}>={1'b0,rgb[15:8]}+{4'd0,rgb[7:3]};
 wire white=!blue && rgb[23:16]>=96 && rgb[15:8]>=96 && rgb[7:0]>=96;
 wire [8:0] total_rows={1'b0,text_rows}+((row_seen && transitions>=2) ? 9'd1:9'd0);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin left<=0;right<=0;top<=0;bottom<=0;known<=0;blue_count<=0;white_count<=0;text_rows<=0;row_seen<=0;row_blue<=0;row_white<=0;row_y<=0;transitions<=0;residual_present<=0;end
  else if(clear_all)begin known<=0;blue_count<=0;white_count<=0;text_rows<=0;row_seen<=0;row_blue<=0;row_white<=0;transitions<=0;residual_present<=0;end
  else begin
   if(update_box)begin
    known<=1;left<=box_l+((box_r-box_l)>>4);right<=box_r-((box_r-box_l)>>4);
    top<=box_t+((box_b-box_t)>>3);bottom<=box_b-((box_b-box_t)>>3);
   end
   if(frame_start)begin
    residual_present<=blue_count>=32 && white_count>=8 && total_rows>=2;
    blue_count<=0;white_count<=0;text_rows<=0;row_seen<=0;row_blue<=0;row_white<=0;transitions<=0;
   end else if(sample)begin
    if(blue && blue_count!=65535)blue_count<=blue_count+1'b1;
    if(white && white_count!=65535)white_count<=white_count+1'b1;
    if(!row_seen || row_y!=y)begin
     if(row_seen && transitions>=2 && text_rows!=255)text_rows<=text_rows+1'b1;
     row_seen<=1;row_y<=y;row_blue<=blue;row_white<=0;transitions<=0;
    end else if(blue)begin
     row_blue<=1;row_white<=0;if(row_white && transitions<15)transitions<=transitions+1'b1;
    end else if(white && row_blue)row_white<=1;
   end
  end
 end
endmodule
