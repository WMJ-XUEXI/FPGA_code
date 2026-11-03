`timescale 1ns/1ps
// Compatibility adapter: one original single-car path plus one shared CCL.
module vehicle_tracker_multi(
 input wire clk,rst_n,enable,frame_start,geometry_valid,light_changed,
 input wire confirm,recalibrate,sample_valid,sample_hit,
 input wire [7:0] sample_x,input wire [6:0] sample_y,
 input wire [175:0] lefts,rights,input wire [153:0] tops,bottoms,
 input wire [11:0] display_x,display_y,
 output wire learning,background_ready,
 output reg overlay_hit,output reg [23:0] overlay_rgb,
 output wire buzzer_n,
     input wire demo_enter,demo_exit,input wire [15:0] demo_selected_id,
    output wire [47:0] demo_ids,output wire [59:0] demo_ids_decimal,output wire [2:0] demo_active,demo_observed,demo_registered,
    output wire [5:0] demo_codes,output wire demo_result_valid,output wire [3:0] demo_result,
 input wire sample_static
);
 wire old_overlay,old_buzzer,foreground_valid,foreground_hit;
 wire [23:0] old_rgb;
 wire selected_done,selected_found,selected_whole,selected_new,selected_warning;
 wire [7:0] selected_l,selected_r;wire [6:0] selected_t,selected_b;
 wire [2:0] selected_pose;
 vehicle_tracker_compat #(.EXTERNAL_MEASUREMENT(1)) compat(
  .clk(clk),.rst_n(rst_n),.enable(enable),.frame_start(frame_start),
  .geometry_valid(geometry_valid),.light_changed(light_changed),.confirm(confirm),.recalibrate(recalibrate),
  .sample_valid(sample_valid),.sample_hit(sample_hit),.sample_x(sample_x),.sample_y(sample_y),
  .lefts(lefts),.rights(rights),.tops(tops),.bottoms(bottoms),.display_x(display_x),.display_y(display_y),
  .learning(learning),.background_ready(background_ready),.overlay_hit(old_overlay),.overlay_rgb(old_rgb),
  .buzzer_n(old_buzzer),.foreground_valid(foreground_valid),.foreground_hit(foreground_hit),
  .measure_done(selected_done),.measure_found(selected_found),.measure_whole(selected_whole),.measure_new(selected_new),
  .measure_l(selected_l),.measure_r(selected_r),.measure_t(selected_t),.measure_b(selected_b),.measure_pose(selected_pose),
  .measurement_warning(selected_warning));
 wire [7:0] bay_w=rights[7:0]-lefts[7:0]+1'b1;
 wire [6:0] bay_h=tops[13:7]>tops[6:0] ? tops[13:7]-tops[6:0]:7'd1;
 wire [7:0] field_l=lefts[7:0]>(bay_w>>1) ? lefts[7:0]-(bay_w>>1):8'd0;
 wire [8:0] end_x={1'b0,rights[21*8+:8]}+({1'b0,bay_w}>>1);
 wire [7:0] field_r=end_x>159 ? 8'd159:end_x[7:0];
 wire [6:0] field_t=tops[6:0]>bay_h ? tops[6:0]-bay_h:7'd0;
 wire [8:0] end_y={2'b0,bottoms[5*7+:7]}+9'd1+{2'b0,bay_h};
 wire [6:0] field_b=end_y>89 ? 7'd89:end_y[6:0];
 wire cc_enable=enable && background_ready && geometry_valid && !light_changed && !recalibrate && !confirm;
 wire cc_done,cc_error,cc_more;
 wire [2:0] cc_valid,cc_whole;
 wire [23:0] cc_l,cc_r;wire [20:0] cc_t,cc_b;
 wire [29:0] cc_count;wire [8:0] cc_pose;
 vehicle_components components(clk,rst_n,cc_enable,frame_start,foreground_valid,foreground_hit,
  sample_x,bay_w,field_l,field_r,sample_y,bay_h,field_t,field_b,
  cc_done,cc_error,cc_more,cc_valid,cc_whole,cc_l,cc_r,cc_t,cc_b,cc_count,cc_pose,sample_static);
 wire tracks_done,association_ambiguous;
 wire [2:0] track_active,track_observed,track_new,track_whole;
 wire [59:0] track_ids_decimal;
 wire [47:0] track_ids;wire [23:0] track_l,track_r,track_cx;
 wire [20:0] track_t,track_b,track_cy;wire [8:0] track_pose;
 vehicle_tracks tracks(.clk(clk),.rst_n(rst_n),.enable(cc_enable),.start(cc_done),.frame_error(cc_error),.bay_w(bay_w),
  .candidate_valid(cc_valid),.candidate_whole(cc_whole),.candidate_l(cc_l),.candidate_r(cc_r),
  .candidate_t(cc_t),.candidate_b(cc_b),.candidate_pose(cc_pose),
  .done(tracks_done),.ambiguous(association_ambiguous),.active(track_active),.observed(track_observed),.new_target(track_new),
  .ids(track_ids),.decimal_ids(track_ids_decimal),.lefts(track_l),.rights(track_r),.centers_x(track_cx),.tops(track_t),.bottoms(track_b),.centers_y(track_cy),
  .poses(track_pose),.whole(track_whole));
 // Keep selection on the lowest live slot, including a short prediction.
 // Predictions never supply positive alarm evidence. A new identity resets
 // the selected-car speed and violation history instead of inheriting it.
 wire [1:0] selected_slot=track_active[0] ? 2'd0:track_active[1] ? 2'd1:2'd2;
 wire [15:0] selected_id=track_ids[selected_slot*16+:16];
 reg [15:0] previous_selected_id;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)previous_selected_id<=0;
  else if(!cc_enable)previous_selected_id<=0;
  else if(tracks_done)previous_selected_id<=track_active!=0 ? selected_id:16'd0;
 assign selected_done=tracks_done;
 assign selected_found=cc_enable && track_active[selected_slot] && track_observed[selected_slot];
 assign selected_whole=track_whole[selected_slot];
 assign selected_new=selected_id!=previous_selected_id;
 assign selected_l=track_l[selected_slot*8+:8];assign selected_r=track_r[selected_slot*8+:8];
 assign selected_t=track_t[selected_slot*7+:7];assign selected_b=track_b[selected_slot*7+:7];
 assign selected_pose=track_pose[selected_slot*3+:3];
 // The matcher owns its working table; only committed results cross SOF.
 // It completes in <100 clocks and holds its table until the next CCL result.
 reg pending,multi_seen;
 // Three consecutive multi observations enter; eight single observations
 // return. Predicted/ghost tracks cannot vote to enter multi display.
 reg [1:0] multi_votes;reg [2:0] single_votes;
 wire [2:0] measured=track_active & track_observed;
 wire several_measured=(measured & (measured-1'b1))!=0;
 reg [2:0] shown_valid,shown_observed;
 reg [47:0] shown_ids;
 assign demo_ids_decimal=60'd0; // Decimal ID display removed.
 reg [23:0] shown_l,shown_r;reg [20:0] shown_t,shown_b;
 assign demo_ids=shown_ids;assign demo_active=shown_valid;assign demo_observed=shown_observed;
 demo_registration registrations(.clk(clk),.rst_n(rst_n),.enable(cc_enable),.frame_start(frame_start),
  .active(shown_valid),.observed(shown_observed),.ids(shown_ids),.enter_request(demo_enter),.exit_request(demo_exit),.selected_id(demo_selected_id),
  .busy(),.result_valid(demo_result_valid),.result(demo_result),.registered(demo_registered),.plate_codes(demo_codes));

 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   pending<=0;shown_valid<=0;shown_observed<=0;multi_seen<=0;multi_votes<=0;single_votes<=0;shown_ids<=0;
   shown_l<=0;shown_r<=0;shown_t<=0;shown_b<=0;
  end else if(!cc_enable)begin pending<=0;shown_valid<=0;shown_observed<=0;multi_seen<=0;multi_votes<=0;single_votes<=0;end
  else begin
   if(tracks_done)pending<=1;
   if(frame_start && pending)begin
    pending<=0;shown_valid<=track_active;shown_observed<=track_observed;shown_ids<=track_ids;
    shown_l<=track_l;shown_r<=track_r;shown_t<=track_t;shown_b<=track_b;
    if(several_measured)begin
     single_votes<=0;
     if(multi_votes==2)multi_seen<=1;else multi_votes<=multi_votes+1'b1;
    end else begin
     multi_votes<=0;
     if(measured!=0)begin
      if(single_votes==7)multi_seen<=0;else single_votes<=single_votes+1'b1;
     end else single_votes<=0;
    end
   end
  end
 end
 // One shared selected-car feature engine, not per-ID alarms. Its measured
 // bounds now come from the same track table as the boxes, never a second finder.
 // In multi view mark the selected ID red whenever its alarm flag is set.
 assign buzzer_n=multi_seen ? 1'b1:old_buzzer;
 integer i;
 reg [11:0] l,r,t,b,label_top;
 reg border,label_hit;reg [1:0] label_slot;reg [6:0] label_x;reg [4:0] label_y;
 wire plate_ink;wire [1:0] label_code=demo_codes[label_slot*2+:2];
 demo_plate_font demo_label(label_code,label_x,label_y-5'd12,plate_ink);
 always @*begin
  overlay_hit=old_overlay;overlay_rgb=old_rgb;l=0;r=0;t=0;b=0;border=0;
  label_top=0;label_hit=0;label_slot=0;label_x=0;label_y=0;
  if(multi_seen)begin
   overlay_hit=0;overlay_rgb=0;
   for(i=0;i<3;i=i+1)begin
    l={1'b0,shown_l[i*8+:8],3'b0};r={1'b0,shown_r[i*8+:8],3'b111};
    t={2'b0,shown_t[i*7+:7],3'b0};b={2'b0,shown_b[i*7+:7],3'b111};
    border=shown_valid[i] && display_x>=l && display_x<=r && display_y>=t && display_y<=b &&
     (display_x-l<2 || r-display_x<2 || display_y-t<2 || b-display_y<2);
    if(enable && border && (shown_observed[i] || (display_x[2]^display_y[2])))begin
     overlay_hit=1;overlay_rgb=(demo_codes[i*2+:2]==0 || demo_codes[i*2+:2]==3) ? 24'hff2020:!shown_observed[i] ? 24'h888888:i==0 ? 24'h30ff30:i==1 ? 24'h00dfff:24'hffff30;
    end
    label_top=t>=30 ? t-30:b+2;
    if(enable && shown_valid[i] && display_x>=l && display_x<l+92 && display_y>=label_top && display_y<label_top+28)begin
     label_hit=1;label_slot=i;label_x=display_x-l;label_y=display_y-label_top;
    end
   end
   if(label_hit && label_y>=12 && plate_ink)begin
    overlay_hit=1;overlay_rgb=24'hff2020; // Same red for all plates and Unknown.
   end
  end
 end
endmodule
