`timescale 1ns/1ps
// Passive probes only: never stall or change the video/DDR data path.
// All elapsed times use the board's 50MHz reference, in 0.1ms ticks.
module showcase_metrics(
 input wire clk50,rst_n,
 input wire camera_clk,camera_sof,camera_valid,
 input wire ddr_clk,write_sof,read_sof,
 input wire [1:0] write_bank,read_bank,
 input wire pixel_clk,display_vs,display_de,
 output reg [76:0] metrics,
 output reg update_toggle
);
 reg cap_toggle,complete_toggle,cap_started;
 reg [15:0] capture_id;
 reg [17:0] capture_beats;
 always @(posedge camera_clk or negedge rst_n) begin
  if(!rst_n) begin cap_toggle<=0;complete_toggle<=0;cap_started<=0;capture_id<=0;capture_beats<=0;end
  else if(camera_sof) begin
   cap_toggle<=~cap_toggle;capture_id<=capture_id+1'b1;cap_started<=1;
   if(cap_started && capture_beats==18'd230400) complete_toggle<=~complete_toggle;
   capture_beats<=camera_valid ? 18'd1 : 18'd0;
  end else if(camera_valid && cap_started && capture_beats<230401) capture_beats<=capture_beats+1'b1;
 end
 reg wr_toggle,rd_toggle;
 reg [1:0] wr_slot,rd_slot;
 always @(posedge ddr_clk or negedge rst_n) begin
  if(!rst_n) begin wr_toggle<=0;rd_toggle<=0;wr_slot<=0;rd_slot<=0;end
  else begin
   if(write_sof) begin wr_slot<=write_bank;wr_toggle<=~wr_toggle;end
   if(read_sof) begin rd_slot<=read_bank;rd_toggle<=~rd_toggle;end
  end
 end
 reg vs_q,display_armed,display_toggle;
 always @(posedge pixel_clk or negedge rst_n) begin
  if(!rst_n) begin vs_q<=0;display_armed<=0;display_toggle<=0;end
  else begin
   vs_q<=display_vs;
   if(display_vs && !vs_q) display_armed<=1;
   if(display_de && display_armed) begin display_toggle<=~display_toggle;display_armed<=0;end
  end
 end
 // Toggle synchronizers have one more settling clock than their stable
 // bundled bank/id buses. Events are separated by a camera/display frame.
 (* async_reg="true" *) reg [2:0] cap_sync,valid_sync,wr_sync,rd_sync,display_sync;
 reg cap_seen,valid_seen,wr_seen,rd_seen,display_seen;
 reg [15:0] id_meta,id_sync;
 reg [1:0] wr_meta,wr_bank_sync,rd_meta,rd_bank_sync;
 wire cap_event=cap_sync[2]!=cap_seen;
 wire valid_event=valid_sync[2]!=valid_seen;
 wire wr_event=wr_sync[2]!=wr_seen;
 wire rd_event=rd_sync[2]!=rd_seen;
 wire display_event=display_sync[2]!=display_seen;
 reg [12:0] tick_div;
 reg [23:0] ticks,captured_at,shown_at;
 reg [15:0] latest_id,shown_id,last_display_id;
 reg have_capture,shown_valid,have_display_id;
 reg [3:0] slot_valid;
 reg [39:0] slot_meta[0:3];
 reg [13:0] second_ticks;
 reg [7:0] capture_count,display_count,capture_fps,display_fps;
 wire unique_display=display_event && shown_valid && (!have_display_id || shown_id!=last_display_id);
 wire [23:0] elapsed=ticks-shown_at;
 integer s;
 always @(posedge clk50 or negedge rst_n) begin
  if(!rst_n) begin
   cap_sync<=0;valid_sync<=0;wr_sync<=0;rd_sync<=0;display_sync<=0;
   cap_seen<=0;valid_seen<=0;wr_seen<=0;rd_seen<=0;display_seen<=0;
   id_meta<=0;id_sync<=0;wr_meta<=0;wr_bank_sync<=0;rd_meta<=0;rd_bank_sync<=0;
   tick_div<=0;ticks<=0;captured_at<=0;shown_at<=0;latest_id<=0;shown_id<=0;
   have_capture<=0;shown_valid<=0;slot_valid<=0;last_display_id<=0;have_display_id<=0;
   second_ticks<=0;capture_count<=0;display_count<=0;capture_fps<=0;display_fps<=0;
   for(s=0;s<4;s=s+1) slot_meta[s]<=0;
  end else begin
   cap_sync<={cap_sync[1:0],cap_toggle};valid_sync<={valid_sync[1:0],complete_toggle};
   wr_sync<={wr_sync[1:0],wr_toggle};rd_sync<={rd_sync[1:0],rd_toggle};
   display_sync<={display_sync[1:0],display_toggle};
   cap_seen<=cap_sync[2];valid_seen<=valid_sync[2];wr_seen<=wr_sync[2];
   rd_seen<=rd_sync[2];display_seen<=display_sync[2];
   id_meta<=capture_id;id_sync<=id_meta;
   wr_meta<=wr_slot;wr_bank_sync<=wr_meta;rd_meta<=rd_slot;rd_bank_sync<=rd_meta;
   if(tick_div==4999) begin
    tick_div<=0;ticks<=ticks+1'b1;
    if(second_ticks==9999) begin
     second_ticks<=0;
     capture_fps<=capture_count<99 ? capture_count+valid_event : 8'd99;
     display_fps<=display_count<99 ? display_count+unique_display : 8'd99;
     capture_count<=0;display_count<=0;
    end else begin
     second_ticks<=second_ticks+1'b1;
     if(valid_event && capture_count<99) capture_count<=capture_count+1'b1;
     if(unique_display && display_count<99) display_count<=display_count+1'b1;
    end
   end else begin
    tick_div<=tick_div+1'b1;
    if(valid_event && capture_count<99) capture_count<=capture_count+1'b1;
    if(unique_display && display_count<99) display_count<=display_count+1'b1;
   end
   if(cap_event) begin captured_at<=ticks;latest_id<=id_sync;have_capture<=1;end
   // video_in supplies the OLD write pointer, exactly the slot whose base
   // address is loaded on its frame_start (including error rollback).
   // ISP streaming latency places RAW SOF before this DDR write event.
   if(wr_event) begin slot_meta[wr_bank_sync]<={latest_id,captured_at};slot_valid[wr_bank_sync]<=have_capture;end
   if(rd_event) begin
    {shown_id,shown_at}<=slot_meta[rd_bank_sync];
    shown_valid<=slot_valid[rd_bank_sync] && !(wr_event && wr_bank_sync==rd_bank_sync);
   end
   if(unique_display) begin last_display_id<=shown_id;have_display_id<=1;end
  end
 end
 // One 16-cycle double-dabble converter shared by IDs, delay and both FPS.
 // Payload settles before update_toggle; the pixel domain captures it only
 // after a three-stage toggle synchronizer, well before the text at y=40.
 reg busy,pending_valid;
 reg [2:0] job;
 reg [3:0] bit_index;
 reg [15:0] number,display_snapshot,delay_snapshot;
 reg [7:0] input_snapshot,output_snapshot;
 reg [19:0] bcd;
 reg [19:0] adjusted;
 integer digit;
 always @* begin
  adjusted=bcd;
  for(digit=0;digit<5;digit=digit+1)
   if(bcd[digit*4+:4]>=5) adjusted[digit*4+:4]=bcd[digit*4+:4]+4'd3;
 end
 wire [19:0] bcd_next={adjusted[18:0],number[15]};
 always @(posedge clk50 or negedge rst_n) begin
  if(!rst_n) begin
   busy<=0;pending_valid<=0;job<=0;bit_index<=0;number<=0;bcd<=0;
   display_snapshot<=0;delay_snapshot<=0;input_snapshot<=0;output_snapshot<=0;
   metrics<=0;update_toggle<=0;
  end else if(display_event && !busy) begin
   busy<=1;job<=0;bit_index<=0;bcd<=0;number<=latest_id;
   display_snapshot<=shown_id;delay_snapshot<=elapsed[23:16]!=0 ? 16'hffff : elapsed[15:0];
   pending_valid<=shown_valid && have_capture;
   input_snapshot<=capture_fps;output_snapshot<=display_fps;
  end else if(busy) begin
   number<=number<<1;bcd<=bcd_next;
   if(bit_index==15) begin
    case(job)
     0:begin metrics[75:56]<=bcd_next;number<=display_snapshot;end
     1:begin metrics[55:36]<=bcd_next;number<=delay_snapshot;end
     2:begin metrics[35:16]<=bcd_next;number<={8'd0,input_snapshot};end
     3:begin metrics[15:8]<=bcd_next[7:0];number<={8'd0,output_snapshot};end
     4:begin metrics[7:0]<=bcd_next[7:0];metrics[76]<=pending_valid;update_toggle<=~update_toggle;busy<=0;end
    endcase
    job<=job+1'b1;bit_index<=0;bcd<=0;
   end else bit_index<=bit_index+1'b1;
  end
 end
endmodule
