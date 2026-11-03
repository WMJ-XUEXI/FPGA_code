// Horizontal blue/white plate: binary ROI -> row/column scans -> shared CNN.
// Two 16384x9 gray/chroma banks, plus one rectified binary ROI.
// No full-frame buffer, variable divider or second CNN.
module plate_capture(
 input clk,rst_n,frame_start,pixel_valid,input [10:0] x,input [9:0] y,
 input [7:0] gray,input [23:0] rgb,input [1:0] buttons_n,
 input box_valid,enable,retry,input [10:0] box_l,box_r,input [9:0] box_t,box_b,
 output reg [48:0] plate,output reg result_valid,digit_mode,output busy,
 output reg attempt_done,output clear_result,output reg [3:0] capture_status,output reg [3:0] detected_characters,output preview_wr,output [9:0] preview_addr,output preview_pixel,output reg [48:0] attempt_plate,output reg attempt_confident,output reg [2:0] capture_reason
);
 localparam WAIT_FRAME=0,CAPTURE=1,PREP=2,ROW_REQ=3,ROW_SUM=4,COL_PREP=5,
 COL_REQ=6,COL_SUM=7,COL_STEP=8,FLUSH=9,SELECT=10,LOAD_REQ=11,LOAD_WRITE=12,
 START=13,WAIT_CNN=14,PUBLISH=15;
 (* fsm_encoding="sequential" *)reg [5:0] state;
 localparam ARM=16,ANCHOR_REQ=17,ANCHOR_SUM=18,ANCHOR_COL=19,WARP_SETUP=20,
 WARP_MAP=21,WARP_RAW_REQ=22,WARP_STORE=23,WARP_FINISH=24,
 GLYPH_REQ=25,GLYPH_SUM=26,GLYPH_PREP=27,GLYPH_NORMALIZE=28,
 FILTER_REQ=29,FILTER_STEP=30,FILTER_FINISH=31,PROVINCE_FINAL=32;
 (* ram_style="bram" *)reg rectified[0:16383];
 (* ram_style="bram" *)reg filtered[0:16383];
 (* ram_style="dram" *)reg filter_line1[0:255],filter_line2[0:255];
 reg [7:0] filter_x;reg [5:0] filter_y;reg filter_q;
 reg [1:0] filter_top,filter_middle,filter_bottom;
 wire [3:0] filter_sum={3'd0,filter_top[1]}+{3'd0,filter_top[0]}+{3'd0,filter_line2[filter_x]}+
 {3'd0,filter_middle[1]}+{3'd0,filter_middle[0]}+{3'd0,filter_line1[filter_x]}+
 {3'd0,filter_bottom[1]}+{3'd0,filter_bottom[0]}+{3'd0,filter_q};
 wire [7:0] filter_center_x=filter_x-1'b1;wire [5:0] filter_center_y=filter_y-1'b1;
 reg [7:0] anchor_x,warp_x;reg [5:0] anchor_y,warp_y,warp_raw_y;
 reg column_seen,left_seen,middle_seen,right_seen,warp_inside_q;
 reg [5:0] column_top,column_bottom,left_top,left_bottom,middle_top,middle_bottom,right_top,right_bottom;
 reg [6:0] column_blues;
 reg [1:0] background_run;
 reg signed [15:0] top_acc,bottom_acc,top_delta,bottom_delta;
 reg [11:0] warp_scale;
 wire [6:0] measured_height={1'b0,column_bottom}-{1'b0,column_top}+7'd1;
 wire wide_pair=left_seen && right_seen;
 wire [5:0] edge_start_top=left_seen ? left_top:middle_top,edge_start_bottom=left_seen ? left_bottom:middle_bottom;
 wire [5:0] edge_end_top=right_seen ? right_top:middle_top,edge_end_bottom=right_seen ? right_bottom:middle_bottom;
 wire signed [15:0] top_difference_base=$signed({10'd0,edge_end_top})-$signed({10'd0,edge_start_top});
 wire signed [15:0] bottom_difference_base=$signed({10'd0,edge_end_bottom})-$signed({10'd0,edge_start_bottom});
 wire signed [15:0] top_difference=wide_pair ? top_difference_base:(top_difference_base<<<1);
 wire signed [15:0] bottom_difference=wide_pair ? bottom_difference_base:(bottom_difference_base<<<1);
 wire reliable_edges=((left_seen && right_seen)||(left_seen && middle_seen)||(middle_seen && right_seen)) && top_difference>=-24 && top_difference<=24 && bottom_difference>=-24 && bottom_difference<=24;
 wire qualified_edges=reliable_edges && (wide_pair || (top_difference>=-6 && top_difference<=6 && bottom_difference>=-6 && bottom_difference<=6));
 wire [15:0] top_absolute=top_difference<0 ? -top_difference:top_difference;
 wire [15:0] bottom_absolute=bottom_difference<0 ? -bottom_difference:bottom_difference;
 wire signed [15:0] top_inset=(top_absolute+7)>>3,bottom_inset=(bottom_absolute+7)>>3;
 wire signed [15:0] top_integer=top_acc>>>7,bottom_integer=bottom_acc>>>7;
 wire [5:0] warp_top=top_integer<0 ? 6'd0:top_integer>63 ? 6'd63:top_integer[5:0];
 wire [5:0] warp_bottom=bottom_integer<0 ? 6'd0:bottom_integer>63 ? 6'd63:bottom_integer[5:0];
 wire [6:0] warp_height={1'b0,warp_bottom}-{1'b0,warp_top}+7'd1;
 wire warp_column_valid=warp_bottom>=warp_top && warp_height>=16;
 wire [13:0] raw_address=state==ANCHOR_REQ ? {anchor_y,anchor_x}:{warp_raw_y,warp_x};
 reg row_running;reg [1:0] row_gap;reg [5:0] row_run_top;reg [6:0] row_best_height;

 localparam ACQ_IDLE=0,ACQ_ARM=1,ACQ_CAPTURE=2,ACQ_DONE=3;
 reg [1:0] acq_state;
 reg [10:0] roi_l,roi_w,target_x;reg [9:0] roi_t,roi_h,target_y;
 reg [18:0] accum_x;reg [15:0] accum_y;reg [7:0] cx;reg [5:0] cy;
 wire [19:0] next_ax={1'b0,accum_x}+roi_w;
 wire [16:0] next_ay={1'b0,accum_y}+roi_h;
 (* ram_style="bram" *)reg [8:0] bitmap[0:32767];
 reg [1:0] key_meta,key_sync,key_stable,key_previous;
 reg [20:0] key_count;
 wire clear_request=key_previous[0] && !key_stable[0];
 wire mode_request=key_previous[1] && !key_stable[1];
 wire restart=retry || clear_request || mode_request;
 assign clear_result=restart;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)attempt_done<=0;
  else attempt_done<=!restart && state==PUBLISH;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin key_meta<=3;key_sync<=3;key_stable<=3;key_previous<=3;key_count<=0;end
  else begin
   key_meta<=buttons_n;key_sync<=key_meta;key_previous<=key_stable;
   if(key_sync==key_stable)key_count<=0;
   else if(&key_count)begin key_stable<=key_sync;key_count<=0;end
   else key_count<=key_count+1'b1;
  end
 end
 wire sample=acq_state==ACQ_CAPTURE && pixel_valid && x==target_x && y==target_y;
 wire last_sample=cx==255 && cy==63;
 wire suitable=box_valid && box_r>=box_l+255 && box_b>=box_t+63 && box_t>=4 && box_b<716;
 reg [7:0] gray_lo,gray_hi,background,threshold;
 reg dark_ink;
 wire [8:0] threshold_sum={1'b0,gray_lo}+{1'b0,gray_hi};
 wire capture_ink=digit_mode && dark_ink ? gray<threshold : gray>threshold;
 // Blue bounds exclude surrounding white paper and the outer plate frame.
 // Store chroma separately; consumer rejects bright white pixels using the
 // CURRENT snapshot threshold, including under strong colour cast.
 wire blue=rgb[7:0]>=48 && {1'b0,rgb[7:0]}>{1'b0,rgb[23:16]}+9'd16 &&
                                          {1'b0,rgb[7:0]}>{1'b0,rgb[15:8]}+9'd8;
 reg [3:0] status_work;
 reg blue_seen;reg [7:0] blue_left,blue_right;reg [5:0] blue_top,blue_bottom;
 wire [6:0] blue_height={1'b0,blue_bottom}-{1'b0,blue_top}+7'd1;
 // Keep more of the Q tail and tall strokes than the old 1/8-height trim.
 wire [5:0] trim_y=blue_height>=24 ? {3'd0,blue_height[6:4]}+6'd2:6'd2;
 wire [8:0] blue_width={1'b0,blue_right}-{1'b0,blue_left}+9'd1;
 wire [7:0] trim_x=blue_width[8:5];
 // One bank is exclusively owned by segmentation/CNN until PUBLISH. The
 // producer may overwrite only the other bank. Pending snapshots are replaced
 // by newer complete images; incomplete/overwritten snapshots are never read.
 reg read_bank,capture_bank,ready_bank,roi_ready;
 wire take_snapshot=state==WAIT_FRAME && roi_ready && enable;
 reg [7:0] cap_lo,cap_hi,cap_left,cap_right,ready_lo,ready_hi,ready_left,ready_right;
 reg [5:0] cap_top,cap_bottom,ready_top,ready_bottom;
 reg cap_blue,ready_blue;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   acq_state<=ACQ_IDLE;capture_bank<=0;ready_bank<=0;roi_ready<=0;
   roi_l<=0;roi_t<=0;roi_w<=0;roi_h<=0;target_x<=0;target_y<=0;accum_x<=0;accum_y<=0;cx<=0;cy<=0;
   cap_lo<=255;cap_hi<=0;cap_left<=255;cap_right<=0;cap_top<=63;cap_bottom<=0;cap_blue<=0;
   ready_lo<=255;ready_hi<=0;ready_left<=255;ready_right<=0;ready_top<=63;ready_bottom<=0;ready_blue<=0;
  end else if(restart || !enable)begin acq_state<=ACQ_IDLE;roi_ready<=0;end
  else begin
   if(take_snapshot)roi_ready<=0;
   if(frame_start)begin acq_state<=ACQ_ARM;end
   else case(acq_state)
    ACQ_ARM:begin
     roi_ready<=0;
     if(suitable)begin
      capture_bank<=(state!=WAIT_FRAME || take_snapshot) ? !(take_snapshot ? ready_bank:read_bank):!capture_bank;
      roi_l<=box_l;roi_t<=box_t;roi_w<=box_r-box_l+1'b1;roi_h<=box_b-box_t+1'b1;
      target_x<=box_l;target_y<=box_t;accum_x<=0;accum_y<=0;cx<=0;cy<=0;
      cap_lo<=255;cap_hi<=0;cap_blue<=0;cap_left<=255;cap_right<=0;cap_top<=63;cap_bottom<=0;acq_state<=ACQ_CAPTURE;
     end else acq_state<=ACQ_IDLE;
    end
    ACQ_CAPTURE:if(sample)begin
     if(cx==255)begin cx<=0;accum_x<=0;target_x<=roi_l;cy<=cy+1'b1;accum_y<=next_ay;target_y<=roi_t+(next_ay>>6);end
     else begin cx<=cx+1'b1;accum_x<=next_ax;target_x<=roi_l+(next_ax>>8);end
     if(gray<cap_lo)cap_lo<=gray;if(gray>cap_hi)cap_hi<=gray;
     if(blue)begin cap_blue<=1;if(cx<cap_left)cap_left<=cx;if(cx>cap_right)cap_right<=cx;if(cy<cap_top)cap_top<=cy;if(cy>cap_bottom)cap_bottom<=cy;end
     if(last_sample)acq_state<=ACQ_DONE;
    end
    ACQ_DONE:begin
     ready_bank<=capture_bank;roi_ready<=1;ready_lo<=cap_lo;ready_hi<=cap_hi;
     ready_blue<=cap_blue;ready_left<=cap_left;ready_right<=cap_right;ready_top<=cap_top;ready_bottom<=cap_bottom;
     acq_state<=ACQ_IDLE;
    end
    default:acq_state<=ACQ_IDLE;
   endcase
  end
 end

 reg [7:0] left_bound,right_bound,sx;
 reg [5:0] top_bound,bottom_bound,sy,char_top,char_bottom;
 reg rows_seen;
 reg [8:0] count;
 // A pure synchronous byte output lets TD infer ERAM. Comparing inside the
 // RAM read assignment hid that register and expanded the ROI into LUT RAM.
 reg [8:0] bitmap_q;
 reg pixel_q;
 wire [8:0] count_next=count+{8'd0,pixel_q};
 wire [8:0] region_width={1'b0,right_bound}-{1'b0,left_bound}+9'd1;
 wire row_active=count_next>=(digit_mode ? 9'd1:9'd24) && count_next<(region_width-(region_width>>2));
 reg run_active;reg [3:0] gap;reg [7:0] run_left,run_right;reg [6:0] run_peak;
 reg [5:0] run_top,run_bottom;reg province_seen;reg [7:0] province_left,province_right;
 reg [7:0] box_left[0:6],box_right[0:6];
 reg [3:0] characters;reg overflow;
 wire [8:0] run_width={1'b0,run_right}-{1'b0,run_left}+9'd1;
 wire [6:0] text_height={1'b0,char_bottom}-{1'b0,char_top}+7'd1;
 wire province_remaining=!digit_mode && characters==6;
 wire column_active=count>=2;
 // The six rightmost connected glyphs determine the province boundary.
 // Merge ALL remaining left strokes, so 川 is never split into three letters.
 wire commit_run=run_active && ((state==COL_STEP && !column_active && gap==1) || state==FLUSH);
 wire [6:0] run_full_height={1'b0,run_bottom}-{1'b0,run_top}+7'd1;
 wire boundary_run=(run_right>=right_bound-2 || run_left<=left_bound+2) && run_width<=12 && run_full_height>=text_height+8;
 wire valid_run=!boundary_run && (province_remaining ? (run_width>=1 && run_width<=56 && run_peak>=3):
 (run_width>=(digit_mode ? 9'd2:9'd3) && run_width<=48 && run_peak>=5 && run_peak>=(text_height>>2)));
 wire [8:0] column_count_next=count+{8'd0,pixel_q && sy>=char_top && sy<=char_bottom};
 wire [2:0] store_index=digit_mode ? 3'd0:3'd6-characters[2:0];
 wire [6:0] row_run_height=row_running ? {1'b0,sy}-{1'b0,row_run_top}+7'd1:7'd1;
 wire [5:0] expanded_top=char_top>=4 ? char_top-2:6'd2;
 wire [5:0] expanded_bottom=char_bottom<=59 ? char_bottom+2:6'd61;
 reg [2:0] slot;
 reg [7:0] glyph_sx;reg [5:0] glyph_sy,glyph_top,glyph_bottom;
 wire [2:0] box_index=digit_mode ? 3'd0:slot;
 wire segmentation_valid=!overflow && rows_seen &&
                         (digit_mode ? characters==1:characters==7);
 wire [6:0] box_width={1'b0,box_right[box_index]}-{1'b0,box_left[box_index]}+7'd1;
 wire [6:0] box_height={1'b0,glyph_bottom}-{1'b0,glyph_top}+7'd1;
 wire province_glyph=!digit_mode && slot==0;
 wire [5:0] scan_top=province_glyph && char_top>=6 ? char_top-4:province_glyph ? 6'd2:char_top;
 wire [5:0] scan_bottom=province_glyph && char_bottom<=59 ? char_bottom+2:province_glyph ? 6'd61:char_bottom;
 wire glyph_row_active=count_next>=(province_glyph ? 7'd2:box_width>=16 ? (box_width>>3):7'd2);
 wire [6:0] glyph_run_height=row_running ? {1'b0,glyph_sy}-{1'b0,row_run_top}+7'd1:7'd1;
 wire [6:0] major=box_width>box_height ? box_width:box_height;
 wire [6:0] padded=major+(major>>2)+7'd2;
 wire [6:0] pad_x=(padded-box_width)>>1;
 wire [6:0] pad_y=(padded-box_height)>>1;
 // Province glyphs are physically narrow on a plate; training Chinese is square.
 // Normalize the two axes independently only for slot zero, preserving digit shapes.
 wire province=!digit_mode && slot==0;
 wire [6:0] padded_x=province ? box_width+(box_width>>2)+7'd2:padded;
 wire [6:0] province_pad_x=(padded_x-box_width)>>1;
 reg [6:0] span,span_x;
 reg signed [9:0] origin_x;
 reg signed [8:0] origin_y;
 reg [11:0] scale_x,scale_y;
 wire signed [9:0] source_x=origin_x+$signed({3'd0,scale_x[11:5]});
 wire signed [8:0] source_y=origin_y+$signed({2'd0,scale_y[11:5]});
 wire source_inside=source_x>=$signed({2'd0,box_left[box_index]}) &&
 source_x<=$signed({2'd0,box_right[box_index]}) &&
 source_y>=$signed({3'd0,glyph_top}) && source_y<=$signed({3'd0,glyph_bottom});
 reg source_inside_q;
 reg [9:0] load_index;reg [10:0] ink_count;
 wire [13:0] read_address=state==LOAD_REQ ? {source_y[5:0],source_x[7:0]}:state==GLYPH_REQ ? {glyph_sy,glyph_sx}:{sy,sx};
 wire read_request=state==ROW_REQ || state==COL_REQ || state==LOAD_REQ || state==GLYPH_REQ;
 always @(posedge clk)begin
  if(sample)bitmap[{capture_bank,cy,cx}]<={blue,gray};
  if(state==ANCHOR_REQ || state==WARP_RAW_REQ)bitmap_q<=bitmap[{read_bank,raw_address}];
  if(state==WARP_STORE)rectified[{warp_y,warp_x}]<=warp_inside_q && bitmap_q[7:0]>threshold;
  if(state==FILTER_REQ)filter_q<=rectified[{filter_y,filter_x}];
  if(state==FILTER_STEP)begin
   filter_line1[filter_x]<=filter_q;filter_line2[filter_x]<=filter_line1[filter_x];
   if(filter_x==0)begin filter_top<={1'b0,filter_line2[filter_x]};filter_middle<={1'b0,filter_line1[filter_x]};filter_bottom<={1'b0,filter_q};end
   else begin filter_top<={filter_top[0],filter_line2[filter_x]};filter_middle<={filter_middle[0],filter_line1[filter_x]};filter_bottom<={filter_bottom[0],filter_q};end
   if(filter_x>=2 && filter_y>=2)filtered[{filter_center_y,filter_center_x}]<=filter_sum>=5;
  end
  if(read_request)pixel_q<=filtered[read_address];
 end
 wire ink=pixel_q && source_inside_q;
 assign preview_wr=state==LOAD_WRITE && (digit_mode || slot==0);
 assign preview_addr=load_index;
 assign preview_pixel=ink;
 wire shape_valid=segmentation_valid && ink_count>=24 && ink_count<=800;
 wire core_busy,core_done;wire [6:0] prediction;wire [15:0] margin;wire classification_good;
 character_confidence confidence_filter(prediction,margin,!digit_mode && slot==0,!digit_mode,classification_good);
 wire [1:0] alphabet=digit_mode ? 2'd3:slot==0 ? 2'd0:slot==1 ? 2'd1:2'd2;
 reg working_confident;
 reg [48:0] working;
 wire [48:0] stable_next;
 assign busy=state!=WAIT_FRAME;
 cnn_serial_core core(.clk(clk),.rst_n(rst_n && !restart),
 .input_wr(state==LOAD_WRITE),.input_addr(load_index),.input_pixel(ink ? 7'd127:7'd0),
 .start(state==START && shape_valid),.alphabet(alphabet),.busy(core_busy),.done(core_done),.prediction(prediction),.margin(margin));
 // Per-character 3-of-5 consensus. Histories include rejected attempts.
 // Hold an established result through four inconsistent/rejected attempts;
 // fifth inconsistency expires it unless a new consensus has formed.
 genvar k;
 generate for(k=0;k<7;k=k+1)begin: results
  reg [6:0] h0,h1,h2,h3;
  reg [2:0] stale;
  wire [6:0] current=working[k*7+:7];
  wire [6:0] previous=plate[k*7+:7];
  wire [2:0] votes=3'd1+{2'd0,current==h0}+{2'd0,current==h1}+
                                {2'd0,current==h2}+{2'd0,current==h3};
  wire [2:0] required_votes=previous!=127 && current!=previous ? 3'd4:3'd3;
  wire accept=current!=127 && votes>=required_votes;
  assign stable_next[k*7+:7]=accept ? current :
       previous!=127 && (current==previous || stale<4) ? previous:7'd127;
  always @(posedge clk or negedge rst_n)begin
   if(!rst_n)begin
    working[k*7+:7]<=127;plate[k*7+:7]<=127;
    h0<=127;h1<=127;h2<=127;h3<=127;stale<=0;
   end else if(restart)begin
    working[k*7+:7]<=127;plate[k*7+:7]<=127;
    h0<=127;h1<=127;h2<=127;h3<=127;stale<=0;
   end else begin
    if(take_snapshot)working[k*7+:7]<=127;
    else if(slot==k && state==WAIT_CNN && core_done)working[k*7+:7]<=prediction;
    if(state==PUBLISH)begin
     h0<=current;h1<=h0;h2<=h1;h3<=h2;
     plate[k*7+:7]<=stable_next[k*7+:7];
     if(current==stable_next[k*7+:7] || stable_next[k*7+:7]==127)stale<=0;
     else if(stale<4)stale<=stale+1'b1;
    end
   end
  end
 end endgenerate
 wire stable_complete=digit_mode ? stable_next[27:21]!=127 :
 stable_next[6:0]!=127 && stable_next[13:7]!=127 && stable_next[20:14]!=127 &&
 stable_next[27:21]!=127 && stable_next[34:28]!=127 && stable_next[41:35]!=127 && stable_next[48:42]!=127;
 wire complete=digit_mode ? working[27:21]!=127 :
 working[6:0]!=127 && working[13:7]!=127 && working[20:14]!=127 && working[27:21]!=127 &&
 working[34:28]!=127 && working[41:35]!=127 && working[48:42]!=127;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   capture_reason<=0;attempt_plate<={49{1'b1}};attempt_confident<=0;working_confident<=0;capture_status<=0;detected_characters<=0;status_work<=0;state<=WAIT_FRAME;read_bank<=0;digit_mode<=0;result_valid<=0;slot<=0;load_index<=0;ink_count<=0;
   gray_lo<=255;gray_hi<=0;background<=0;threshold<=128;dark_ink<=0;
   blue_seen<=0;blue_left<=255;blue_right<=0;blue_top<=63;blue_bottom<=0;
   left_bound<=0;right_bound<=0;top_bound<=0;bottom_bound<=0;sx<=0;sy<=0;
   char_top<=0;char_bottom<=0;rows_seen<=0;row_running<=0;row_gap<=0;row_run_top<=0;row_best_height<=0;count<=0;run_active<=0;gap<=0;
   run_left<=0;run_right<=0;run_peak<=0;characters<=0;overflow<=0;
   span<=0;span_x<=0;origin_x<=0;origin_y<=0;scale_x<=0;scale_y<=0;source_inside_q<=0;
  end else if(restart)begin
   capture_reason<=0;attempt_plate<={49{1'b1}};attempt_confident<=0;working_confident<=0;capture_status<=0;detected_characters<=0;status_work<=0;state<=WAIT_FRAME;result_valid<=0;gray_lo<=255;gray_hi<=0;threshold<=128;dark_ink<=0;
   if(mode_request)digit_mode<=~digit_mode;
  end else begin
   if(state==WAIT_CNN && core_done && !classification_good)begin working_confident<=0;capture_reason<=3;end
   // Seven small bound pairs, no column histogram storage or parallel arithmetic.
   if(commit_run && valid_run)begin
    if(characters<6)begin
     box_left[store_index]<=run_left;box_right[store_index]<=run_right;
     characters<=characters+1'b1;
    end else if(characters==6)begin
     if(!province_seen)begin province_left<=run_left;province_right<=run_right;end
     else begin if(run_left<province_left)province_left<=run_left;if(run_right>province_right)province_right<=run_right;end
     province_seen<=1;
    end else overflow<=1;
   end
   case(state)
    WAIT_FRAME:if(take_snapshot)begin
     capture_reason<=0;read_bank<=ready_bank;state<=PREP;status_work<=0;working_confident<=1;
     gray_lo<=ready_lo;gray_hi<=ready_hi;
     blue_seen<=ready_blue;blue_left<=ready_left;blue_right<=ready_right;blue_top<=ready_top;blue_bottom<=ready_bottom;
    end
    PREP:begin
     threshold<=threshold_sum[8:1];
     status_work[0]<=digit_mode || (blue_seen && blue_right-blue_left>=64 && blue_bottom-blue_top>=12);
     status_work[1]<=({1'b0,gray_hi}-{1'b0,gray_lo})>=32;
     characters<=0;province_seen<=0;province_left<=255;province_right<=0;overflow<=0;rows_seen<=0;row_running<=0;row_gap<=0;row_run_top<=0;row_best_height<=0;run_active<=0;gap<=0;count<=0;
     if(({1'b0,gray_hi}-{1'b0,gray_lo})<32)begin capture_reason<=3;state<=PUBLISH;end
     else if(digit_mode)begin
      left_bound<=0;right_bound<=31;top_bound<=0;bottom_bound<=31;sx<=0;sy<=0;state<=ROW_REQ;
     end else if(blue_seen && blue_right-blue_left>=64 && blue_bottom-blue_top>=12)begin
      anchor_x<=48;anchor_y<=0;column_seen<=0;column_blues<=0;background_run<=0;column_top<=63;column_bottom<=0;
      left_seen<=0;middle_seen<=0;right_seen<=0;left_top<=63;middle_top<=63;right_top<=63;left_bottom<=0;middle_bottom<=0;right_bottom<=0;
      state<=ANCHOR_REQ;
     end else begin capture_reason<=1;state<=PUBLISH;end
    end
    ANCHOR_REQ:state<=ANCHOR_SUM;
    ANCHOR_SUM:begin
     if(bitmap_q[8] && bitmap_q[7:0]<threshold)begin
      if(background_run<3)background_run<=background_run+1'b1;
      if(background_run>=2)begin if(!column_seen)column_top<=anchor_y-2;column_bottom<=anchor_y;column_seen<=1;end
      column_blues<=column_blues+1'b1;
     end else background_run<=0;
     if(anchor_y==63)state<=ANCHOR_COL;
     else begin anchor_y<=anchor_y+1'b1;state<=ANCHOR_REQ;end
    end
    ANCHOR_COL:begin
     if(column_seen && column_blues>=8 && measured_height>=32)begin
      if(anchor_x<96)begin left_seen<=1;if(column_top<left_top)left_top<=column_top;if(column_bottom>left_bottom)left_bottom<=column_bottom;end
      else if(anchor_x<160)begin middle_seen<=1;if(column_top<middle_top)middle_top<=column_top;if(column_bottom>middle_bottom)middle_bottom<=column_bottom;end
      else begin right_seen<=1;if(column_top<right_top)right_top<=column_top;if(column_bottom>right_bottom)right_bottom<=column_bottom;end
     end
     column_seen<=0;column_blues<=0;background_run<=0;column_top<=63;column_bottom<=0;anchor_y<=0;
     if(anchor_x==207)state<=WARP_SETUP;
     else begin anchor_x<=anchor_x==79 ? 8'd112:anchor_x==143 ? 8'd176:anchor_x+1'b1;state<=ANCHOR_REQ;end
    end
    WARP_SETUP:begin
     if(!qualified_edges)begin capture_reason<=1;working_confident<=0;state<=PUBLISH;end
     else begin
      top_delta<=top_difference;bottom_delta<=bottom_difference;
      top_acc<=$signed({3'd0,edge_start_top,7'd0})-(left_seen ? (top_difference<<<6):(top_difference<<<7))+(top_inset<<<7);
      bottom_acc<=$signed({3'd0,edge_start_bottom,7'd0})-(left_seen ? (bottom_difference<<<6):(bottom_difference<<<7))-(bottom_inset<<<7);
      warp_x<=0;warp_y<=0;warp_scale<=0;state<=WARP_MAP;
     end
    end
    WARP_MAP:begin
     warp_raw_y<=warp_column_valid ? warp_top+warp_scale[11:6]:6'd0;
     warp_inside_q<=warp_column_valid && warp_y>=2 && warp_y<=61;state<=WARP_RAW_REQ;
    end
    WARP_RAW_REQ:state<=WARP_STORE;
    WARP_STORE:begin
     if(warp_y==63)begin
      warp_y<=0;warp_scale<=0;top_acc<=top_acc+top_delta;bottom_acc<=bottom_acc+bottom_delta;
      if(warp_x==255)state<=WARP_FINISH;
      else begin warp_x<=warp_x+1'b1;state<=WARP_MAP;end
     end else begin warp_y<=warp_y+1'b1;warp_scale<=warp_scale+{5'd0,warp_height};state<=WARP_MAP;end
    end
    WARP_FINISH:begin
     filter_x<=0;filter_y<=0;state<=FILTER_REQ;
    end
    FILTER_REQ:state<=FILTER_STEP;
    FILTER_STEP:begin
     if(filter_x==255)begin
      filter_x<=0;
      if(filter_y==63)state<=FILTER_FINISH;else begin filter_y<=filter_y+1'b1;state<=FILTER_REQ;end
     end else begin filter_x<=filter_x+1'b1;state<=FILTER_REQ;end
    end
    FILTER_FINISH:begin
     left_bound<=8;right_bound<=247;top_bound<=2;bottom_bound<=61;sx<=8;sy<=2;state<=ROW_REQ;
    end
    ROW_REQ:state<=ROW_SUM;
    ROW_SUM:begin
     if(sx==right_bound)begin
      count<=0;sx<=left_bound;
      if(row_active)begin
       if(!row_running)row_run_top<=sy;row_running<=1;row_gap<=0;
       if(row_run_height>row_best_height)begin row_best_height<=row_run_height;char_top<=row_running ? row_run_top:sy;char_bottom<=sy;rows_seen<=1;end
      end else if(row_running)begin if(row_gap==1)begin row_running<=0;row_gap<=0;end else row_gap<=row_gap+1'b1;end
      if(sy==bottom_bound)state<=COL_PREP;
      else begin sy<=sy+1'b1;state<=ROW_REQ;end
     end else begin count<=count_next;sx<=sx+1'b1;state<=ROW_REQ;end
    end
    COL_PREP:begin
     threshold<=threshold_sum[8:1];
     if(!rows_seen || char_bottom-char_top<10)begin capture_reason<=2;state<=PUBLISH;end
     else begin char_top<=expanded_top;char_bottom<=expanded_bottom;sx<=right_bound;sy<=2;column_seen<=0;column_top<=63;column_bottom<=0;count<=0;run_peak<=0;state<=COL_REQ;end
    end
    COL_REQ:state<=COL_SUM;
    COL_SUM:begin
     count<=column_count_next;
     if(pixel_q)begin if(!column_seen)column_top<=sy;column_bottom<=sy;column_seen<=1;end
     if(sy==61)state<=COL_STEP;
     else begin sy<=sy+1'b1;state<=COL_REQ;end
    end
    COL_STEP:begin
     if(column_active)begin
      if(!run_active)begin run_right<=sx;run_peak<=count[6:0];run_top<=column_top;run_bottom<=column_bottom;end
      else begin if(count>run_peak)run_peak<=count[6:0];if(column_top<run_top)run_top<=column_top;if(column_bottom>run_bottom)run_bottom<=column_bottom;end
      run_left<=sx;run_active<=1;gap<=0;
     end else if(run_active)begin
      if(gap==1)begin run_active<=0;gap<=0;run_peak<=0;end
      else gap<=gap+1'b1;
     end
     if(sx==left_bound)state<=FLUSH;
     else begin sx<=sx-1'b1;sy<=2;column_seen<=0;column_top<=63;column_bottom<=0;count<=0;state<=COL_REQ;end
    end
    FLUSH:state<=PROVINCE_FINAL;
    PROVINCE_FINAL:begin
     if(characters==6 && province_seen && province_right-province_left>=2 && province_right-province_left<=55)begin box_left[0]<=province_left;box_right[0]<=province_right;characters<=7;end
     slot<=0;state<=SELECT;
    end
    SELECT:begin
     status_work[2]<=segmentation_valid;
     if(!segmentation_valid)begin capture_reason<=2;state<=PUBLISH;end
     else begin
      glyph_top<=scan_top;glyph_bottom<=scan_bottom;glyph_sx<=box_left[box_index];glyph_sy<=scan_top;
      row_running<=0;row_gap<=0;row_best_height<=0;count<=0;state<=GLYPH_REQ;
     end
    end
    GLYPH_REQ:state<=GLYPH_SUM;
    GLYPH_SUM:begin
     if(glyph_sx==box_right[box_index])begin
      count<=0;glyph_sx<=box_left[box_index];
      if(glyph_row_active && province_glyph)begin
        if(row_best_height==0)glyph_top<=glyph_sy;glyph_bottom<=glyph_sy;
        row_best_height<=row_best_height==0 ? 7'd1:{1'b0,glyph_sy}-{1'b0,glyph_top}+7'd1;
       end else if(glyph_row_active)begin
       if(!row_running)row_run_top<=glyph_sy;row_running<=1;row_gap<=0;
       if(glyph_run_height>row_best_height)begin row_best_height<=glyph_run_height;glyph_top<=row_running ? row_run_top:glyph_sy;glyph_bottom<=glyph_sy;end
      end else if(row_running)begin if(row_gap==1)begin row_running<=0;row_gap<=0;end else row_gap<=row_gap+1'b1;end
      if(glyph_sy==scan_bottom)state<=GLYPH_PREP;
      else begin glyph_sy<=glyph_sy+1'b1;state<=GLYPH_REQ;end
     end else begin count<=count_next;glyph_sx<=glyph_sx+1'b1;state<=GLYPH_REQ;end
    end
    GLYPH_PREP:begin
     if(row_best_height<8)begin
      capture_reason<=2;working_confident<=0;
      if(slot==6)state<=PUBLISH;else begin slot<=slot+1'b1;state<=SELECT;end
     end else begin
      if(glyph_top>scan_top)glyph_top<=glyph_top-1'b1;
      if(glyph_bottom<scan_bottom)glyph_bottom<=glyph_bottom+1'b1;
      state<=GLYPH_NORMALIZE;
     end
    end
    GLYPH_NORMALIZE:begin
      span<=padded;span_x<=padded_x;
      origin_x<=$signed({2'd0,box_left[box_index]})-$signed({3'd0,province_pad_x});
      origin_y<=$signed({3'd0,glyph_top})-$signed({2'd0,pad_y});
      scale_x<=0;scale_y<=0;load_index<=0;ink_count<=0;state<=LOAD_REQ;
    end
    LOAD_REQ:begin source_inside_q<=source_inside;state<=LOAD_WRITE;end
    LOAD_WRITE:begin
     if(ink)ink_count<=ink_count+1'b1;
     if(load_index==1023)state<=START;
     else begin
      load_index<=load_index+1'b1;
      if(load_index[4:0]==31)begin scale_x<=0;scale_y<=scale_y+{5'd0,span};end
      else scale_x<=scale_x+{5'd0,span_x};
      state<=LOAD_REQ;
     end
    end
    START:begin
     if(shape_valid)state<=WAIT_CNN;
     else if(digit_mode || slot==6)state<=PUBLISH;
     else begin slot<=slot+1'b1;state<=SELECT;end
    end
    WAIT_CNN:if(core_done)begin
     if(digit_mode || slot==6)state<=PUBLISH;
     else begin slot<=slot+1'b1;state<=SELECT;end
    end
    PUBLISH:begin attempt_plate<=working;attempt_confident<=complete && working_confident;capture_status<={complete,status_work[2:0]};detected_characters<=characters;result_valid<=stable_complete;state<=WAIT_FRAME;end
    default:state<=WAIT_FRAME;
   endcase
  end
 end
endmodule

// Raw logit separation, not a probability. Ambiguous target classes wait for another sample.
module character_confidence(input [6:0] prediction,input [15:0] margin,input province,plate_mode,output confident);
 wire [15:0] minimum=plate_mode && (prediction==0 || prediction==21) ? 16'd256:
  plate_mode && (prediction==37 || prediction==57 || prediction==39 || prediction==42) ? 16'd192:province ? 16'd128:16'd64;
 assign confident=prediction<67 && margin>=minimum;
endmodule
