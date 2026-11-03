module blueplate_video_mixer #(
 parameter H_OFFSET=0,V_OFFSET=0,IMG_WIDTH=1280,IMG_HEIGHT=720,DEBUG_MODE=0,
 parameter CLOCK_HZ=75000000
)(
 input I_clk,I_rst_n,I_video_vsync,I_video_hsync,I_video_de,I_video_user,I_video_last,
 input [3:0] I_debug_status,input [3:0] I_buttons_n,
 output O_video_rd_en,input [23:0] I_video_rd_data,
 output O_hdmi_vsync,O_hdmi_hsync,O_hdmi_de,output [23:0] O_hdmi_data,
 input I_uart_rx,output O_uart_tx
);
 reg [10:0] x,x1,x2;reg [9:0] y,y1,y2;
 reg vs1,vs2,hs1,hs2,de1,de2,user1,user2;reg [23:0] rgb1,rgb2;
 assign O_video_rd_en=I_video_de;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin
   x<=0;x1<=0;x2<=0;y<=0;y1<=0;y2<=0;vs1<=0;vs2<=0;hs1<=0;hs2<=0;de1<=0;de2<=0;user1<=0;user2<=0;rgb1<=0;rgb2<=0;
  end else begin
   if(I_video_user)begin x<=0;y<=0;end
   else if(I_video_de)begin if(I_video_last)begin x<=0;y<=y+1'b1;end else x<=x+1'b1;end
   x1<=x;x2<=x1;y1<=y;y2<=y1;rgb1<=I_video_rd_data;rgb2<=rgb1;
   vs1<=I_video_vsync;vs2<=vs1;hs1<=I_video_hsync;hs2<=hs1;de1<=I_video_de;de2<=de1;user1<=I_video_user;user2<=user1;
  end
 end
 wire show_blue,freeze,clear,show_debug;wire [3:0] key_state;
 locator_keys #(.CLOCK_HZ(CLOCK_HZ)) keys(I_clk,I_rst_n,I_buttons_n,show_blue,freeze,clear,show_debug,key_state);
 reg retry_toggle_q;
 always @(posedge I_clk or negedge I_rst_n)
  if(!I_rst_n)retry_toggle_q<=0;else retry_toggle_q<=freeze;
 wire clear_all,clear_daily,clock_daily;wire [15:0] total_in,total_out,total_flow;wire [7:0] occupied;wire [1:0] history_page;wire history_ready,history_full;wire [5:0] history_used;wire [86:0] history_record;
 wire [55:0] now_bcd;wire clock_valid;reg [55:0] frame_time;reg frame_clock_valid;
 calendar_clock #(.CLOCK_HZ(CLOCK_HZ)) clock_unit(I_clk,I_rst_n,I_uart_rx,O_uart_tx,now_bcd,clock_valid,clock_daily);
 wire panel_view,panel_hidden;wire [1:0] panel_mode={1'b0,panel_view};
 panel_controls #(.CLOCK_HZ(CLOCK_HZ)) panel_keys(I_clk,I_rst_n,user2,key_state[3],panel_view,panel_hidden);
 reg [5:0] display_used;reg [1:0] display_page;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin frame_time<=0;frame_clock_valid<=0;display_used<=0;display_page<=0;end
  else if(user2)begin frame_time<=now_bcd;frame_clock_valid<=clock_valid;display_used<=history_used;display_page<=history_page;end
 end
 parking_controls #(.CLOCK_HZ(CLOCK_HZ)) controls(I_clk,I_rst_n,user2,freeze,key_state[2],history_page,clear_all,key_state[0],clear_daily,history_used);
 wire retry_request=1'b0;
 wire bv,cv;wire [10:0] l,r,rl,rr;wire [9:0] t,b,rt,rb;wire [3:0] status;wire [15:0] sequence_id;
 wire measured,new_target,verified,residual_present;
 blue_plate_locator locator(I_clk,I_rst_n,user2,de2,clear_all,1'b0,x2,y2,rgb2,bv,cv,l,r,rl,rr,t,b,rt,rb,status,sequence_id,measured,new_target);
 wire [9:0] gray_sum={2'd0,rgb2[23:16]}+{1'd0,rgb2[15:8],1'b0}+{2'd0,rgb2[7:0]};
 wire ocr_measurement=bv && measured && t>=10'd4 && b<10'd716 && history_ready && r>=l+11'd255 && b>=t+10'd63;
 wire attempt_done,attempt_good,capture_busy,preview_wr,preview_pixel,locked,enable,abort_capture,entry_event;
 wire [9:0] preview_addr;wire [48:0] attempt_plate,confirmed;wire [3:0] capture_stage,char_count;wire [15:0] visits;
 wire [2:0] capture_reason;
 plate_capture capture(.clk(I_clk),.rst_n(I_rst_n),.frame_start(user2),.pixel_valid(de2),.x(x2),.y(y2),
 .gray(gray_sum[9:2]),.rgb(rgb2),.buttons_n(2'b11),.box_valid(bv && measured),.enable(enable),.retry(abort_capture),
 .box_l(l),.box_r(r),.box_t(t),.box_b(b),.busy(capture_busy),.attempt_done(attempt_done),
 .attempt_plate(attempt_plate),.attempt_confident(attempt_good),.capture_status(capture_stage),.detected_characters(char_count),
 .preview_wr(preview_wr),.preview_addr(preview_addr),.preview_pixel(preview_pixel),.capture_reason(capture_reason));
 plate_presence_guard presence_guard(.clk(I_clk),.rst_n(I_rst_n),.clear_all(clear_all),.frame_start(user2),.pixel_valid(de2),.update_box(bv && measured),
 .x(x2),.box_l(l),.box_r(r),.y(y2),.box_t(t),.box_b(b),.rgb(rgb2),.residual_present(residual_present));
 plate_confirmation confirm(.clk(I_clk),.rst_n(I_rst_n),.frame_start(user2),.present((bv && measured) || residual_present),.measurement_valid(ocr_measurement),.clear_all(clear_all),.exit_event(),.session_active(),.retry(retry_request),.new_target(new_target),.verified(verified),
 .attempt_done(attempt_done),.attempt_good(attempt_good),.attempt_plate(attempt_plate),.plate(confirmed),.locked(locked),
 .enable(enable),.abort_capture(abort_capture),.entry_event(entry_event),.visits(visits));
 wire [5:0] display_base={1'b0,display_page,3'd0};
 wire [5:0] page_left=display_used>display_base ? display_used-display_base:6'd0;
 wire [3:0] row_count=page_left>=8 ? 4'd8:page_left==0 ? 4'd1:page_left[3:0];
 wire [9:0] rows_top=10'd656-{1'b0,row_count,5'd0};
 wire [9:0] history_y=y2-rows_top;wire [4:0] history_address={display_page,history_y[7:5]};
 wire [113:0] history_stamp;wire accepted_event,accepted_in;wire [48:0] accepted_plate;
 parking_records #(.CLOCK_HZ(CLOCK_HZ)) history(I_clk,I_rst_n,clear_all,entry_event,confirmed,history_address,history_record,history_ready,history_full,history_used,clear_daily || clock_daily,total_in,total_out,total_flow,occupied,{clock_valid,now_bcd},history_stamp,accepted_event,accepted_in,accepted_plate);


 reg mix_vs,mix_hs,mix_de;reg [23:0] mix_rgb;reg [10:0] mix_x;reg [9:0] mix_y;
 recognition_osd osd(.clk(I_clk),.rst_n(I_rst_n),.vs(mix_vs),.hs(mix_hs),.de(mix_de),.rgb(mix_rgb),.x(mix_x),.y(mix_y),
 .plate(confirmed),.locked(locked && verified),.busy(capture_busy),.stage({locked && verified,capture_stage[2:0]}),.characters(char_count),
 .preview_wr(preview_wr),.preview_addr(preview_addr),.preview_pixel(preview_pixel),.invalidate(abort_capture || !bv),
 .reason(locked && !verified ? 3'd3:capture_reason),.present(ocr_measurement && (!locked || verified)),
 .history_record(history_record),.history_ready(history_ready),.history_full(history_full),.history_used(display_used),.history_page(display_page),.total_in(total_in),.total_out(total_out),.total_flow(total_flow),.occupied(occupied),
 .panel_mode(panel_mode),.now_bcd(frame_time),.clock_valid(frame_clock_valid),.history_stamp(history_stamp),.summary_record(66'd0),.panel_hidden(panel_hidden),
 .out_vs(O_hdmi_vsync),.out_hs(O_hdmi_hsync),.out_de(O_hdmi_de),.out_rgb(O_hdmi_data));
 wire [10:0] draw_l=bv ? l:rl,draw_r=bv ? r:rr;
 wire [9:0] draw_t=bv ? t:rt,draw_b=bv ? b:rb;
 wire border=(bv || cv) && x2>=draw_l && x2<=draw_r && y2>=draw_t && y2<=draw_b &&
      (x2<=draw_l+1'b1 || x2+1'b1>=draw_r || y2<=draw_t+1'b1 || y2+1'b1>=draw_b);
 wire blue_pixel=rgb2[7:0]>=48 && {1'b0,rgb2[7:0]}>{1'b0,rgb2[23:16]}+9'd16 && {1'b0,rgb2[7:0]}>{1'b0,rgb2[15:8]}+9'd8 && {1'b0,rgb2[7:0]}>={1'b0,rgb2[23:16]}+{3'd0,rgb2[7:2]} && {1'b0,rgb2[7:0]}>={1'b0,rgb2[15:8]}+{4'd0,rgb2[7:3]};
 wire [14:0] logo_address=y2*15'd160+x2;wire [24:0] logo_pixel;
 anlogic_logo_rom logo(logo_address,logo_pixel);
 wire logo_hit=x2<160 && y2<160 && logo_pixel[24];
 wire [7:0] marker_x=x2-11'd1008;
 wire status_area=1'b0;
 wire keys_area=1'b0;
 wire mode_area=1'b0;
 wire pulse_area=1'b0;
 always @(posedge I_clk or negedge I_rst_n)begin
  if(!I_rst_n)begin mix_vs<=0;mix_hs<=0;mix_de<=0;mix_rgb<=0;mix_x<=0;mix_y<=0;end
  else begin
   mix_x<=x2;mix_y<=y2;mix_vs<=vs2;mix_hs<=hs2;mix_de<=de2;
   if(!de2)mix_rgb<=0;
   else if(status_area)mix_rgb<=status[marker_x[6:5]] ? 24'h00ff40:24'hff8000;
   else if(keys_area)mix_rgb<=!key_state[marker_x[6:5]] ? 24'h00ffff:24'h404040;
   else if(mode_area)mix_rgb<=locked ? 24'h00ff40:24'h204020;
   else if(pulse_area)mix_rgb<=sequence_id[4] ? 24'h00ffff:24'h203030;
   else if(border)mix_rgb<=bv ? 24'h00ff40:24'hffff00;
   else if(logo_hit)mix_rgb<=logo_pixel[23:0];
   else if(show_blue && blue_pixel)mix_rgb<=24'h0080ff;
   else mix_rgb<=rgb2;
  end
 end
endmodule

module locator_keys #(parameter CLOCK_HZ=75000000,DEBOUNCE_MS=20)(
 input clk,rst_n,input [3:0] buttons_n,
 output reg show_blue,freeze,clear,show_debug,output reg [3:0] stable
);
 localparam LIMIT=(CLOCK_HZ/1000)*DEBOUNCE_MS;
 (* ASYNC_REG="TRUE" *)reg [3:0] meta,sync;
 reg [3:0] previous;reg [$clog2(LIMIT+1)-1:0] count;reg [31:0] blue_held;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin meta<=15;sync<=15;stable<=15;previous<=15;count<=0;blue_held<=0;show_blue<=0;freeze<=0;clear<=0;show_debug<=1;end
  else begin
   meta<=buttons_n;sync<=meta;previous<=stable;clear<=0;
   if(sync==stable)count<=0;
   else if(count>=LIMIT-1)begin stable<=sync;count<=0;end else count<=count+1'b1;
   if(stable[0])blue_held<=0;else if(blue_held<2*CLOCK_HZ)blue_held<=blue_held+1'b1;
   if(!previous[0] && stable[0] && blue_held<2*CLOCK_HZ)show_blue<=~show_blue;
   if(previous[1] && !stable[1])freeze<=~freeze;
   if(previous[2] && !stable[2])begin clear<=1;end
   if(previous[3] && !stable[3])show_debug<=~show_debug;
  end
 end
endmodule
