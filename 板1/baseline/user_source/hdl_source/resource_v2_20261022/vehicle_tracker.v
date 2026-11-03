`timescale 1ns/1ps
// Single vehicle, fixed camera, empty-scene calibration. Analysis is 160x90.
// Reuses parking_core's background RAM and calibrated bay coordinates.
module vehicle_tracker(
    input wire clk,rst_n,enable,frame_start,geometry_valid,light_changed,
    input wire confirm,recalibrate,sample_valid,sample_hit,
    input wire [7:0] sample_x,
    input wire [6:0] sample_y,
    input wire [175:0] lefts,rights,
    input wire [153:0] tops,bottoms,
    input wire [11:0] display_x,display_y,
    output reg learning,
    output wire background_ready,
    output reg overlay_hit,
    output reg [23:0] overlay_rgb,
    output wire buzzer_n
);
    // Finder orders bays by column: 6 left, 5+5 middle, 6 right.
    wire [7:0] origin_x=lefts[7:0];
    wire [7:0] lane_end=lefts[16*8+:8];
    wire [7:0] lane_span=lane_end>origin_x ? lane_end-origin_x : 8'd1;
    wire [7:0] bay_w=rights[7:0]-lefts[7:0]+1'b1;
    // Adjacent bay pitch is 3cm; interiors are eroded by the line finder.
    wire [6:0] bay_h=tops[13:7]>tops[6:0] ? tops[13:7]-tops[6:0] : 7'd1;
    wire [7:0] lane_top={1'b0,bottoms[5*7+:7]}+8'd1;
    wire [8:0] origin_y_sum={1'b0,lane_top}+{2'b0,bay_h};
    wire [6:0] origin_y=origin_y_sum>89 ? 7'd89 : origin_y_sum[6:0];
    wire [7:0] field_left=origin_x>(bay_w>>1) ? origin_x-(bay_w>>1) : 8'd0;
    wire [8:0] field_right_sum={1'b0,rights[21*8+:8]}+({1'b0,bay_w}>>1);
    wire [7:0] field_right=field_right_sum>159 ? 8'd159 : field_right_sum[7:0];
    wire [6:0] field_top=tops[6:0]>bay_h ? tops[6:0]-bay_h : 7'd0;
    wire [7:0] sx=sample_x;
    wire [6:0] sy=sample_y;
    reg enabled_q,armed,calibrated;
    reg [13:0] samples; // Resource: saturates at 14401.
    assign background_ready=calibrated && !learning && !armed;
    reg target_valid;
    reg [7:0] box_l,box_r;
    reg [6:0] box_t,box_b;
    reg [8:0] center_x2;
    reg [7:0] center_y2;
    wire [7:0] center_x=center_x2[8:1];
    wire [6:0] center_y=center_y2[7:1];
    wire [2:0] region_direction;
    reg [2:0] pose_q,pose_pending;
    reg pose_vote;
    reg [3:0] lost_frames;
    wire [7:0] min_x,max_x;
    wire [6:0] min_y,max_y;
    wire [9:0] region_hits;
    wire regions_done,region_found,region_complete;
    reg [11:0] hits;
    // No-target diagnostics are frame snapshots, never another image buffer.
    reg [11:0] difference_hits,last_difference,last_hits;
    reg [13:0] last_samples;
    reg [8:0] last_width;
    reg [7:0] last_height;
    reg [2:0] last_reject;
    // One bit per column, not a second frame buffer. A 2x2 changed patch
    // suppresses isolated sensor noise; every cell overwrites its column bit.
    (* ram_style="dram" *) reg previous_row[0:159];
    reg left_hit,top_left;
    wire in_field=sx>=field_left && sx<=field_right &&
                  sy>=field_top && sy<=origin_y;
    // Collect full-field evidence. Candidate association is applied only
    // while locked; loss releases it for full-field reacquisition.
    wire changed=calibrated && !learning && in_field && sample_hit;
    wire supported=changed && left_hit && previous_row[sx] && top_left &&
                   sx>field_left && sy>field_top;
    always @(posedge clk) if(sample_valid && sx<160) previous_row[sx]<=changed;

    // Independent overlapping local candidates replace the single scene-wide
    // min/max. Remote shadows or tape changes cannot stretch the car's box.
    vehicle_local_candidates u_local_candidates(
        .clk(clk),.rst_n(rst_n),.enable(enable && calibrated && !learning && !armed),
        .frame_start(frame_start),.point_valid(sample_valid && supported),.x(sx),.y(sy),
        .bay_w(bay_w),.bay_h(bay_h),.locked(target_valid),.track_x(center_x),.track_y(center_y),
        .done(regions_done),.found(region_found),.whole(region_complete),
        .left(min_x),.right(max_x),.top(min_y),.bottom(max_y),.count(region_hits),.direction(region_direction));

    wire [8:0] found_w={1'b0,max_x}-{1'b0,min_x}+9'd1;
    wire [7:0] found_h={1'b0,max_y}-{1'b0,min_y}+8'd1;
    // Red/black cars use a coherent new dark body, not tiny white-car windows.
    // Local candidates are already checked for size and spatial support.
    wire candidate=region_found;
    wire [2:0] rejection=last_samples!=14400 ? 3'd1 : last_hits<4 ? 3'd2 :
        (min_x>max_x || min_y>max_y) ? 3'd3 : !candidate ? 3'd4 : 3'd0;
    wire [8:0] midpoint_x={1'b0,min_x}+{1'b0,max_x};
    wire [7:0] midpoint_y={1'b0,min_y}+{1'b0,max_y};
    // Fixed 5x3cm display dimensions, scaled from the known 6x3cm bay.
    // Restore the finder erosion for horizontal scale. No sin/cos, variable multipliers or
    // trigonometric pipeline: eight small integer direction vectors suffice.
    reg [7:0] scale_w,scale_h,long_x,short_x,long_y;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin scale_w<=0;scale_h<=0;long_x<=0;short_x<=0;long_y<=0;end
        else begin
            if(frame_start) begin scale_w<=bay_w;scale_h<={1'b0,bay_h};end
            long_x<=(({2'b0,scale_w}+10'd2)*10'd5+10'd3)/10'd6;
            short_x<=({1'b0,scale_w}+9'd3)>>1;
            long_y<=({2'b0,scale_h}*10'd5+10'd1)/10'd3;
        end
    end
    reg [7:0] direction_x;
    reg [3:0] direction_frames;
    reg [5:0] wrong_hold;
    wire in_lane=target_valid && center_x>=origin_x && center_x<lane_end && center_y>=lane_top && center_y<=origin_y;
    reg [7:0] stop_x;
    reg [6:0] stop_y;
    reg [7:0] bad_frames;
    reg bad_parking;
    reg [2:0] bad_gap;
    wire moved=({1'b0,center_x}>{1'b0,stop_x}+9'd2) ||
               ({1'b0,stop_x}>{1'b0,center_x}+9'd2) ||
               ({1'b0,center_y}>{1'b0,stop_y}+8'd2) ||
               ({1'b0,stop_y}>{1'b0,center_y}+8'd2);
    // One bay at a time just after frame_start: do not instantiate 22 overlap units.
    reg [4:0] bay_index;
    reg scan_busy;
    reg near_bay,inside_bay;
    // Resource: word mux with constant offsets, not a 154-bit barrel
    // shifter driven by bay_index*7. No registers or scan clocks are added.
    reg [29:0] selected_geometry;
    always @* begin
        selected_geometry=30'd0;
        case(bay_index)
            5'd0:selected_geometry={lefts[0+:8],rights[0+:8],tops[0+:7],bottoms[0+:7]};
            5'd1:selected_geometry={lefts[8+:8],rights[8+:8],tops[7+:7],bottoms[7+:7]};
            5'd2:selected_geometry={lefts[16+:8],rights[16+:8],tops[14+:7],bottoms[14+:7]};
            5'd3:selected_geometry={lefts[24+:8],rights[24+:8],tops[21+:7],bottoms[21+:7]};
            5'd4:selected_geometry={lefts[32+:8],rights[32+:8],tops[28+:7],bottoms[28+:7]};
            5'd5:selected_geometry={lefts[40+:8],rights[40+:8],tops[35+:7],bottoms[35+:7]};
            5'd6:selected_geometry={lefts[48+:8],rights[48+:8],tops[42+:7],bottoms[42+:7]};
            5'd7:selected_geometry={lefts[56+:8],rights[56+:8],tops[49+:7],bottoms[49+:7]};
            5'd8:selected_geometry={lefts[64+:8],rights[64+:8],tops[56+:7],bottoms[56+:7]};
            5'd9:selected_geometry={lefts[72+:8],rights[72+:8],tops[63+:7],bottoms[63+:7]};
            5'd10:selected_geometry={lefts[80+:8],rights[80+:8],tops[70+:7],bottoms[70+:7]};
            5'd11:selected_geometry={lefts[88+:8],rights[88+:8],tops[77+:7],bottoms[77+:7]};
            5'd12:selected_geometry={lefts[96+:8],rights[96+:8],tops[84+:7],bottoms[84+:7]};
            5'd13:selected_geometry={lefts[104+:8],rights[104+:8],tops[91+:7],bottoms[91+:7]};
            5'd14:selected_geometry={lefts[112+:8],rights[112+:8],tops[98+:7],bottoms[98+:7]};
            5'd15:selected_geometry={lefts[120+:8],rights[120+:8],tops[105+:7],bottoms[105+:7]};
            5'd16:selected_geometry={lefts[128+:8],rights[128+:8],tops[112+:7],bottoms[112+:7]};
            5'd17:selected_geometry={lefts[136+:8],rights[136+:8],tops[119+:7],bottoms[119+:7]};
            5'd18:selected_geometry={lefts[144+:8],rights[144+:8],tops[126+:7],bottoms[126+:7]};
            5'd19:selected_geometry={lefts[152+:8],rights[152+:8],tops[133+:7],bottoms[133+:7]};
            5'd20:selected_geometry={lefts[160+:8],rights[160+:8],tops[140+:7],bottoms[140+:7]};
            5'd21:selected_geometry={lefts[168+:8],rights[168+:8],tops[147+:7],bottoms[147+:7]};
        endcase
    end
    wire [7:0] xl=selected_geometry[29:22],xr=selected_geometry[21:14];
    wire [6:0] yt=selected_geometry[13:7],yb=selected_geometry[6:0];
    wire [7:0] bay_l=xl==0 ? 8'd0 : xl-1'b1,bay_r=xr==159 ? 8'd159 : xr+1'b1;
    wire [6:0] bay_t=yt==0 ? 7'd0 : yt-1'b1,bay_b=yb==89 ? 7'd89 : yb+1'b1;
    // Test the measured new-black vehicle footprint, not its enlarged drawing.
    // A stopped car near a bay that is not contained in ANY one interior is
    // pressing a line / crossing a boundary. A car wholly in an aisle is exempt.
    wire next_near=near_bay || (box_r>=bay_l && box_l<=bay_r && box_b>=bay_t && box_t<=bay_b);
    // Undo exactly the finder's one-cell erosion; the enlarged display box
    // must never determine whether the measured vehicle presses a line.
    wire next_inside=inside_bay || (box_l>=bay_l && box_r<=bay_r && box_t>=bay_t && box_b<=bay_b);
    reg [4:0] beep_frames;
    wire alarm_audible=enable && calibrated && target_valid &&
                     (wrong_hold!=0 || bad_parking) && (beep_frames<8 || (beep_frames>=10 && beep_frames<18));
    // Selected test tone A: 50MHz / (8 * (16'h5997 + 1)) = 272.497Hz.
    // Here clk is 74.25MHz: round(91744 * 74.25/50) = 136240
    // clocks per half-period. One 18-bit counter; no new clock or PLL.
    reg [17:0] alarm_tone_count;
    reg alarm_tone;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin alarm_tone_count<=0;alarm_tone<=0;end
        else if(!alarm_audible) begin alarm_tone_count<=0;alarm_tone<=0;end
        else if(alarm_tone_count==18'd136239) begin alarm_tone_count<=0;alarm_tone<=~alarm_tone;end
        else alarm_tone_count<=alarm_tone_count+1'b1;
    end
    // Active low: high throughout reset/mute, square wave only in audible segments.
    assign buzzer_n=!rst_n || !alarm_audible || alarm_tone;

    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            enabled_q<=0;armed<=0;calibrated<=0;learning<=0;samples<=0;pose_q<=0;pose_pending<=0;pose_vote<=0;bad_gap<=0;
            target_valid<=0;box_l<=0;box_r<=0;box_t<=0;box_b<=0;center_x2<=0;center_y2<=0;
            hits<=0;left_hit<=0;top_left<=0;lost_frames<=0;
            difference_hits<=0;last_difference<=0;last_hits<=0;last_samples<=0;last_width<=0;last_height<=0;last_reject<=0;
            direction_x<=0;direction_frames<=0;wrong_hold<=0;stop_x<=0;stop_y<=0;
            bad_frames<=0;bad_parking<=0;bay_index<=0;scan_busy<=0;near_bay<=0;inside_bay<=0;beep_frames<=0;
        end else begin
            enabled_q<=enable;
            if(!enable || !enabled_q || !geometry_valid || light_changed || recalibrate) begin
                armed<=0;calibrated<=0;learning<=0;samples<=0;target_valid<=0;lost_frames<=0;pose_q<=0;pose_pending<=0;pose_vote<=0;bad_gap<=0;
                hits<=0;left_hit<=0;top_left<=0;
                difference_hits<=0;last_difference<=0;last_hits<=0;last_samples<=0;last_width<=0;last_height<=0;last_reject<=0;
                direction_frames<=0;wrong_hold<=0;bad_frames<=0;bad_parking<=0;scan_busy<=0;beep_frames<=0;
            end else begin
                if(confirm) begin armed<=1;calibrated<=0;target_valid<=0;wrong_hold<=0;bad_parking<=0;bad_frames<=0;bad_gap<=0;end
                if(sample_valid) begin
                    if(samples<14401) samples<=samples+1'b1;
                    if(sample_hit && difference_hits<4095) difference_hits<=difference_hits+1'b1;
                    left_hit<=changed;top_left<=previous_row[sx];
                    if(sx==159) begin left_hit<=0;top_left<=0;end
                    if(supported) begin
                        if(hits<4095) hits<=hits+1'b1;
                    end
                end
                if(scan_busy) begin
                    near_bay<=next_near;inside_bay<=next_inside;
                    if(bay_index==21) begin
                        scan_busy<=0;
                        if(!target_valid || moved) begin
                            bad_frames<=0;bad_parking<=0;bad_gap<=0;stop_x<=center_x;stop_y<=center_y;
                        end else if(next_near && !next_inside) begin
                            bad_gap<=0;
                            if(bad_frames<180) begin
                                bad_frames<=bad_frames+1'b1;
                                if(bad_frames==179) begin bad_parking<=1;beep_frames<=0;end
                            end
                        end else if(bad_gap==5) begin
                            bad_frames<=0;bad_parking<=0;
                        end else bad_gap<=bad_gap+1'b1;
                    end else bay_index<=bay_index+1'b1;
                end
                if(frame_start) begin
                    last_samples<=samples;last_difference<=difference_hits;last_hits<=hits;
                    difference_hits<=0;
                    samples<=0;hits<=0;left_hit<=0;top_left<=0;
                    if(armed) begin learning<=1;armed<=0;end
                    else if(learning && samples==14400) begin learning<=0;calibrated<=1;end
                    if(beep_frames==23) beep_frames<=0;else beep_frames<=beep_frames+1'b1;
                    if(wrong_hold!=0) wrong_hold<=wrong_hold-1'b1;
                end
                if(regions_done) begin
                    last_width<=found_w;last_height<=found_h;last_reject<=rejection;
                    if(calibrated && !learning && !armed && !confirm) begin
                        if(last_samples==14400 && candidate) begin
                            center_x2<=midpoint_x;center_y2<=midpoint_y;
                            box_l<=min_x;box_r<=max_x;box_t<=min_y;box_b<=max_y;
                            if(!target_valid || region_direction==pose_q) begin pose_q<=region_direction;pose_vote<=0;end
                            else if(pose_vote && pose_pending==region_direction) begin pose_q<=region_direction;pose_vote<=0;end
                            else begin pose_pending<=region_direction;pose_vote<=1;end
                            target_valid<=1;lost_frames<=0;
                            // One setup clock lets the new box reach the scan.
                            scan_busy<=region_complete;bay_index<=0;near_bay<=0;inside_bay<=0;
                            // A window-clipped body cannot prove a parking violation.
                            if(!region_complete) begin bad_frames<=0;bad_parking<=0;bad_gap<=0;end
                        end else begin
                            if(lost_frames==7) begin target_valid<=0;wrong_hold<=0;bad_parking<=0;end
                            else lost_frames<=lost_frames+1'b1;
                            // Pause brief dropouts; only confirmed cross-bay frames count.
                            if(bad_gap==5) begin bad_frames<=0;bad_parking<=0;end
                            else bad_gap<=bad_gap+1'b1;
                        end
                        if(!in_lane) begin direction_frames<=0;direction_x<=center_x;wrong_hold<=0;end
                        else if(direction_frames==9) begin
                            direction_frames<=0;
                            // Keep the anchor through sub-cell movement, so a
                            // slow reverse still accumulates enough evidence.
                            if({1'b0,center_x}+9'd2<={1'b0,direction_x}) begin wrong_hold<=30;direction_x<=center_x;if(wrong_hold==0)beep_frames<=0;end
                            else if({1'b0,direction_x}+9'd2<={1'b0,center_x}) begin wrong_hold<=0;direction_x<=center_x;end
                        end else direction_frames<=direction_frames+1'b1;
                    end
                end
            end
        end
    end

    // Shared restoring divider, 18 clocks/job, used once per frame for X/Y.
    // Coordinates are tenths of a centimetre. No pixel-rate division pipeline.
    // Resource: consumed numerator bits hold quotient bits in-place.
    reg [17:0] dividend;
    reg [8:0] remainder;
    reg [7:0] divisor;
    reg [4:0] divide_count;
    // Resource: eight math phases retain a compact three-bit encoding.
    (* fsm_encoding="sequential" *) reg [2:0] math_state;
    reg [11:0] x_cm10,y_cm10,speed_cm10;
    wire [9:0] trial={remainder,dividend[17]};
    // Resource: one subtractor supplies both the borrow and remainder.
    wire [10:0] trial_difference={1'b0,trial}-{3'd0,divisor};
    wire take=!trial_difference[10];
    wire [17:0] next_quotient={dividend[16:0],take};
    reg [4:0] speed_frames;
    reg speed_reference;
    reg [11:0] old_x,old_y;
    wire [11:0] dx=x_cm10>old_x ? x_cm10-old_x : old_x-x_cm10;
    wire [11:0] dy=y_cm10>old_y ? y_cm10-old_y : old_y-y_cm10;
    // Squaring uses the same small sequential shift/add unit for both axes.
    // Resource: square accumulator and root remainder share 28 registers.
    reg [27:0] math_acc;
    reg [25:0] square_term;
    reg [11:0] square_bits;
    reg [3:0] math_count;
    reg [25:0] radicand;
    reg [12:0] root;
    wire [27:0] root_trial={math_acc[25:0],radicand[25:24]};
    wire [27:0] root_test={13'd0,root,2'b01};
    // Resource: share the root comparison/subtraction carry chain too.
    wire root_phase=math_state==3'd6;
    wire [28:0] work_a=root_phase ? {1'b0,root_trial} : {1'b0,math_acc};
    wire [28:0] work_b=root_phase ? {1'b0,root_test} : {3'd0,square_term};
    // Resource: one carry chain adds square terms or subtracts a root trial.
    wire [28:0] work_result=work_a+(work_b^{29{root_phase}})+root_phase;
    wire [28:0] root_difference=work_result;
    wire root_take=!root_difference[28];
    wire [12:0] root_next={root[11:0],root_take};
    reg [15:0] x_bcd,y_bcd,speed_bcd,bcd;
    reg [11:0] binary_digits;
    reg [1:0] digit_job;
    reg [15:0] adjusted;
    integer digit;
    always @* begin
        adjusted=bcd;
        for(digit=0;digit<4;digit=digit+1)
            if(bcd[digit*4+:4]>=5) adjusted[digit*4+:4]=bcd[digit*4+:4]+4'd3;
    end
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            dividend<=0;remainder<=0;divisor<=1;divide_count<=0;math_state<=0;
            x_cm10<=0;y_cm10<=0;speed_cm10<=0;speed_frames<=0;speed_reference<=0;old_x<=0;old_y<=0;
            math_acc<=0;square_term<=0;square_bits<=0;math_count<=0;radicand<=0;root<=0;
            x_bcd<=0;y_bcd<=0;speed_bcd<=0;bcd<=0;binary_digits<=0;digit_job<=0;
        end else if(!enable || !calibrated || !target_valid || learning || armed) begin
            math_state<=0;math_count<=0;digit_job<=0;speed_frames<=0;speed_reference<=0;speed_cm10<=0;
            x_bcd<=0;y_bcd<=0;speed_bcd<=0;
        end else begin
            case(math_state)
                0:if(frame_start) begin
                    dividend<=center_x>origin_x ? (center_x-origin_x)*18'd420 : 18'd0;
                    divisor<=lane_span;remainder<=0;divide_count<=18;math_state<=1;
                end
                1,2:begin
                    dividend<=next_quotient;
                    remainder<=take ? trial_difference[8:0] : trial[8:0];
                    divide_count<=divide_count-1'b1;
                    if(divide_count==1) begin
                        if(math_state==1) begin
                            x_cm10<=next_quotient>4095 ? 12'd4095 : next_quotient[11:0];
                            dividend<=origin_y>center_y ? (origin_y-center_y)*18'd30 : 18'd0;
                            divisor<=bay_h==0 ? 8'd1 : {1'b0,bay_h};remainder<=0;divide_count<=18;math_state<=2;
                        end else begin
                            y_cm10<=next_quotient>4095 ? 12'd4095 : next_quotient[11:0];math_state<=3;
                        end
                    end
                end
                3:begin
                    if(!speed_reference) begin old_x<=x_cm10;old_y<=y_cm10;speed_reference<=1;speed_frames<=0;math_state<=7;end
                    else if(speed_frames==29) begin
                        speed_frames<=0;math_acc<=0;square_term<={14'd0,dx};square_bits<=dx;math_count<=0;math_state<=4;
                    end else begin speed_frames<=speed_frames+1'b1;math_state<=7;end
                end
                4,5:begin
                    if(square_bits[0]) math_acc<=work_result[27:0];
                    square_term<=square_term<<1;square_bits<=square_bits>>1;math_count<=math_count+1'b1;
                    if(math_count==11) begin
                        math_count<=0;
                        if(math_state==4) begin square_term<={14'd0,dy};square_bits<=dy;math_state<=5;end
                        else begin
                            radicand<=square_bits[0] ? work_result[25:0] : math_acc[25:0];
                            math_acc<=0;root<=0;math_state<=6;old_x<=x_cm10;old_y<=y_cm10;
                        end
                    end
                end
                6:begin
                    radicand<=radicand<<2;math_acc<=root_take ? root_difference[27:0] : root_trial;root<=root_next;
                    math_count<=math_count+1'b1;
                    if(math_count==12) begin
                        // 30 frames at 60fps = 0.5s. Euclidean displacement.
                        speed_cm10<=root_next>2047 ? 12'd4095 : (root_next<2 ? 12'd0 : {root_next[10:0],1'b0});
                        math_count<=0;math_state<=7;
                    end
                end
                7:begin
                    // Double dabble also shares one converter for all 3 fields.
                    if(math_count==0) begin bcd<=0;binary_digits<=digit_job==0 ? x_cm10 : digit_job==1 ? y_cm10 : speed_cm10;math_count<=1;end
                    else begin
                        bcd<={adjusted[14:0],binary_digits[11]};binary_digits<=binary_digits<<1;math_count<=math_count+1'b1;
                        if(math_count==12) begin
                            if(digit_job==0) x_bcd<={adjusted[14:0],binary_digits[11]};
                            else if(digit_job==1) y_bcd<={adjusted[14:0],binary_digits[11]};
                            else speed_bcd<={adjusted[14:0],binary_digits[11]};
                            math_count<=0;
                            if(digit_job==2) begin digit_job<=0;math_state<=0;end else digit_job<=digit_job+1'b1;
                        end
                    end
                end
            endcase
        end
    end

    function [34:0] font;
        input [7:0] c;
        begin case(c)
            "0":font=35'b01110_10001_10011_10101_11001_10001_01110;
            "1":font=35'b00100_01100_00100_00100_00100_00100_01110;
            "2":font=35'b01110_10001_00001_00010_00100_01000_11111;
            "3":font=35'b11110_00001_00001_01110_00001_00001_11110;
            "4":font=35'b00010_00110_01010_10010_11111_00010_00010;
            "5":font=35'b11111_10000_10000_11110_00001_00001_11110;
            "6":font=35'b01110_10000_10000_11110_10001_10001_01110;
            "7":font=35'b11111_00001_00010_00100_01000_01000_01000;
            "8":font=35'b01110_10001_10001_01110_10001_10001_01110;
            "9":font=35'b01110_10001_10001_01111_00001_00001_01110;
            "X":font=35'b10001_10001_01010_00100_01010_10001_10001;
            "Y":font=35'b10001_10001_01010_00100_00100_00100_00100;
            "V":font=35'b10001_10001_10001_10001_10001_01010_00100;
            "C":font=35'b01111_10000_10000_10000_10000_10000_01111;
            "M":font=35'b10001_11011_10101_10101_10001_10001_10001;
            "H":font=35'b10001_10001_10001_11111_10001_10001_10001;
            "S":font=35'b01111_10000_10000_01110_00001_00001_11110;
            "K":font=35'b10001_10010_10100_11000_10100_10010_10001;
            "E":font=35'b11111_10000_10000_11110_10000_10000_11111;
            "P":font=35'b11110_10001_10001_11110_10000_10000_10000;
            "T":font=35'b11111_00100_00100_00100_00100_00100_00100;
            "L":font=35'b10000_10000_10000_10000_10000_10000_11111;
            "A":font=35'b01110_10001_10001_11111_10001_10001_10001;
            "B":font=35'b11110_10001_10001_11110_10001_10001_11110;
            "D":font=35'b11110_10001_10001_10001_10001_10001_11110;
            "F":font=35'b11111_10000_10000_11110_10000_10000_10000;
            "R":font=35'b11110_10001_10001_11110_10100_10010_10001;
            "N":font=35'b10001_11001_10101_10011_10001_10001_10001;
            "O":font=35'b01110_10001_10001_10001_10001_10001_01110;
            "W":font=35'b10001_10001_10001_10101_10101_10101_01010;
            "G":font=35'b01111_10000_10000_10111_10001_10001_01111;
            ".":font=35'b00000_00000_00000_00000_00000_00110_00110;
            "/":font=35'b00001_00001_00010_00100_01000_10000_10000;
            default:font=0;
        endcase end
    endfunction
    function [7:0] hex_character;
        input [3:0] value;
        begin hex_character=value<10 ? 8'd48+{4'd0,value} : 8'd55+{4'd0,value};end
    endfunction
    wire [11:0] pixel_l={1'b0,box_l,3'b000};
    wire [11:0] pixel_t={2'b0,box_t,3'b000},pixel_b={2'b0,box_b,3'b111};
    wire [11:0] pixel_cx={1'b0,center_x2,2'b00}+12'd4,pixel_cy={2'b00,center_y2,2'b00}+12'd4;
    wire signed [12:0] draw_dx=$signed({1'b0,display_x})-$signed({1'b0,pixel_cx});
    wire signed [12:0] draw_dy=$signed({1'b0,display_y})-$signed({1'b0,pixel_cy});
    // Two shared add/subtract paths, not eight parallel direction datapaths.
    wire axis_direction=pose_q[1:0]==0;
    wire x_double=axis_direction || pose_q[1:0]==1;
    wire y_double=pose_q[1:0]==3;
    wire signed [13:0] xu=x_double ? (draw_dx<<<1) : draw_dx;
    wire signed [13:0] yu=axis_direction ? 14'sd0 : y_double ? (draw_dy<<<1) : draw_dy;
    wire signed [13:0] yv=x_double ? (draw_dy<<<1) : draw_dy;
    wire signed [13:0] xv=axis_direction ? 14'sd0 : y_double ? (draw_dx<<<1) : draw_dx;
    wire signed [13:0] base_u=xu+yu,base_v=yv-xv;
    wire [13:0] base_abs_u=base_u<0 ? -base_u : base_u,base_abs_v=base_v<0 ? -base_v : base_v;
    wire [13:0] abs_u=pose_q[2] ? base_abs_v : base_abs_u,abs_v=pose_q[2] ? base_abs_u : base_abs_v;
    wire [8:0] diag_long={1'b0,long_x}+{1'b0,long_y},diag_short={1'b0,short_x}+{2'b0,scale_h[6:0]};
    function [11:0] diagonal_limit;
        input [8:0] span;
        input oblique;
        reg [15:0] term;
        begin
            term={7'd0,span};
            if(oblique) term=(term<<6)+(term<<3);
            else term=(term<<5)+(term<<3)+(term<<2)+(term<<1);
            diagonal_limit=term>>4;
        end
    endfunction
    wire [11:0] limit_u=pose_q==0 ? {1'b0,long_x,3'b000} : pose_q==4 ? {1'b0,long_y,3'b000} : diagonal_limit(diag_long,pose_q[0]);
    wire [11:0] limit_v=pose_q==0 ? {2'b00,scale_h[6:0],3'b000} : pose_q==4 ? {1'b0,short_x,3'b000} : diagonal_limit(diag_short,pose_q[0]);
    wire [2:0] edge_band=pose_q[0] ? 3'd5 : pose_q[1] ? 3'd3 : 3'd4;
    wire border=target_valid && abs_u<=limit_u && abs_v<=limit_v &&
                (abs_u+edge_band>=limit_u || abs_v+edge_band>=limit_v);
    wire [11:0] text_x=target_valid ? (pixel_l>964 ? 12'd964 : pixel_l) : 12'd16;
    wire [11:0] text_y=target_valid ? (pixel_t>=90 ? pixel_t-54 : pixel_b<660 ? pixel_b+20 : 12'd40) : 12'd40;
    wire text_area=display_x>=text_x && display_x<text_x+(target_valid ? 12'd300 : 12'd336) && display_y>=text_y && display_y<text_y+32;
    // Resource/timing: only local text coordinates are divided/indexed.
    // text_area bounds tx to 0..335 and ty to 0..31; no pixel delay is added.
    wire [8:0] tx=display_x-text_x;
    wire [4:0] ty=display_y-text_y;
    wire [4:0] column=tx/12;
    // Fixed 16x16 glyphs: parking warning plus two speed-label characters.
    function [15:0] warning_row;
        input [6:0] address;
        begin case(address)
            7'd0:warning_row=16'h1860;
            7'd1:warning_row=16'h17fe;
            7'd2:warning_row=16'h1000;
            7'd3:warning_row=16'h33f8;
            7'd4:warning_row=16'h3208;
            7'd5:warning_row=16'h53f8;
            7'd6:warning_row=16'h5000;
            7'd7:warning_row=16'h17fe;
            7'd8:warning_row=16'h1402;
            7'd9:warning_row=16'h13fc;
            7'd10:warning_row=16'h1020;
            7'd11:warning_row=16'h1020;
            7'd12:warning_row=16'h1020;
            7'd13:warning_row=16'h10e0;
            7'd14:warning_row=16'h0080;
            7'd16:warning_row=16'h0100;
            7'd17:warning_row=16'h0300;
            7'd18:warning_row=16'h3ffc;
            7'd19:warning_row=16'h0600;
            7'd20:warning_row=16'h0480;
            7'd21:warning_row=16'h0880;
            7'd22:warning_row=16'h1080;
            7'd23:warning_row=16'h1ffc;
            7'd24:warning_row=16'h0080;
            7'd25:warning_row=16'h0080;
            7'd26:warning_row=16'h7ffe;
            7'd27:warning_row=16'h0080;
            7'd28:warning_row=16'h0080;
            7'd29:warning_row=16'h0080;
            7'd30:warning_row=16'h0080;
            7'd32:warning_row=16'h1230;
            7'd33:warning_row=16'h7fbe;
            7'd34:warning_row=16'h1244;
            7'd35:warning_row=16'h3fa8;
            7'd36:warning_row=16'h7cb8;
            7'd37:warning_row=16'h271c;
            7'd38:warning_row=16'h3c66;
            7'd39:warning_row=16'h7ffe;
            7'd41:warning_row=16'h1ff8;
            7'd42:warning_row=16'h1ff8;
            7'd44:warning_row=16'h1ff8;
            7'd45:warning_row=16'h1ff8;
            7'd46:warning_row=16'h1008;
            7'd48:warning_row=16'h0080;
            7'd49:warning_row=16'h0880;
            7'd50:warning_row=16'h1080;
            7'd51:warning_row=16'h3ffc;
            7'd52:warning_row=16'h2080;
            7'd53:warning_row=16'h0080;
            7'd54:warning_row=16'h7ffe;
            7'd56:warning_row=16'h1ff8;
            7'd57:warning_row=16'h1008;
            7'd58:warning_row=16'h1008;
            7'd59:warning_row=16'h1008;
            7'd60:warning_row=16'h1ff8;
            7'd61:warning_row=16'h1008;
            7'd66:warning_row=16'h0c00;
            7'd67:warning_row=16'h0c00;
            7'd68:warning_row=16'h0c00;
            7'd69:warning_row=16'h0c00;
            7'd70:warning_row=16'h0c00;
            7'd71:warning_row=16'h0800;
            7'd72:warning_row=16'h0800;
            7'd75:warning_row=16'h0c00;
            7'd76:warning_row=16'h0c00;
            7'd80:warning_row=16'h0040;
            7'd81:warning_row=16'h27fe;
            7'd82:warning_row=16'h37fe;
            7'd83:warning_row=16'h1040;
            7'd84:warning_row=16'h03fc;
            7'd85:warning_row=16'h0244;
            7'd86:warning_row=16'h7244;
            7'd87:warning_row=16'h37fc;
            7'd88:warning_row=16'h30d0;
            7'd89:warning_row=16'h3158;
            7'd90:warning_row=16'h3246;
            7'd91:warning_row=16'h3440;
            7'd92:warning_row=16'h3800;
            7'd93:warning_row=16'hc7fe;
            7'd94:warning_row=16'h4000;
            7'd96:warning_row=16'h0080;
            7'd97:warning_row=16'h0080;
            7'd98:warning_row=16'h3ffc;
            7'd99:warning_row=16'h2210;
            7'd100:warning_row=16'h2210;
            7'd101:warning_row=16'h2ffc;
            7'd102:warning_row=16'h2210;
            7'd103:warning_row=16'h23f0;
            7'd104:warning_row=16'h2000;
            7'd105:warning_row=16'h2ff8;
            7'd106:warning_row=16'h2310;
            7'd107:warning_row=16'h21a0;
            7'd108:warning_row=16'h60c0;
            7'd109:warning_row=16'h473e;
            7'd110:warning_row=16'h0802;
            default:warning_row=0;
        endcase end
    endfunction
    wire [4:0] speed_x=tx-12'd144;
    wire [6:0] cjk_address=ty[4] ? {tx[6:4],ty[3:0]} : 7'd80+{2'd0,speed_x[4],4'd0}+{3'd0,ty[3:0]};
    wire [15:0] warning_bitmap=warning_row(cjk_address);
    reg [7:0] character;
    reg [15:0] number;
    reg [34:0] glyph;
    reg [3:0] n;
    always @* begin
        character=" ";number=0;n=0;
        overlay_hit=enable && border;overlay_rgb=0;
        if(border) overlay_rgb=(bad_parking || wrong_hold!=0) ? 24'hff3030 : 24'h30ff30;
        if(text_area) begin
            if(calibrated && !target_valid) begin
                // F=frame samples (3840 hex=14400), D=raw differences,
                // S=2x2-supported evidence, E=rejection, W/H=coarse box size.
                if(!ty[4]) case(column)
                    0:character="N";1:character="O";3:character="C";4:character="A";5:character="R";
                    7:character="F";
                    8:character=hex_character({2'b00,last_samples[13:12]});
                    9:character=hex_character(last_samples[11:8]);10:character=hex_character(last_samples[7:4]);11:character=hex_character(last_samples[3:0]);
                    13:character="D";14:character=hex_character(last_difference[11:8]);15:character=hex_character(last_difference[7:4]);16:character=hex_character(last_difference[3:0]);
                    18:character="S";19:character=hex_character(last_hits[11:8]);20:character=hex_character(last_hits[7:4]);21:character=hex_character(last_hits[3:0]);
                endcase
                else case(column)
                    0:character="E";1:character=hex_character({1'b0,last_reject});
                    3:character="W";4:character=hex_character({3'd0,last_width[8]});5:character=hex_character(last_width[7:4]);6:character=hex_character(last_width[3:0]);
                    8:character="H";9:character=hex_character(last_height[7:4]);10:character=hex_character(last_height[3:0]);
                endcase
            end
            else if(!target_valid && ty[4]) character=" ";
            else if(!geometry_valid) begin case(column)0:character="P";1:character="A";2:character="R";3:character="K";5:character="C";6:character="A";7:character="L";endcase end
            else if(!calibrated) begin
                if(learning) begin case(column)0:character="L";1:character="E";2:character="A";3:character="R";4:character="N";endcase end
                else begin case(column)0:character="E";1:character="M";2:character="P";3:character="T";4:character="Y";6:character="K";7:character="1";endcase end
            end else if(!target_valid) begin case(column)0:character="N";1:character="O";3:character="C";4:character="A";5:character="R";endcase end
            else if(!ty[4]) begin
                // X00.0 Y00.0, fixed two-character speed label, 000.0 CM/S.
                case(column)
                    0:character="X";6:character="Y";
                    21:character="C";22:character="M";23:character="/";24:character="S";
                    default:begin
                        if(column>=1 && column<=4) begin number=x_bcd;n=column;end
                        else if(column>=7 && column<=10) begin number=y_bcd;n=column-6;end
                        else if(column>=15 && column<=19) begin number=speed_bcd;n=column-15;end
                        else n=15;
                        case(n)
                            0:character="0"+number[15:12];1:character="0"+number[11:8];
                            2:character="0"+number[7:4];3:character=".";4:character="0"+number[3:0];
                        endcase
                    end
                endcase
            end else if(wrong_hold!=0 && !bad_parking) begin case(column)0:character="W";1:character="R";2:character="O";3:character="N";4:character="G";6:character="W";7:character="A";8:character="Y";endcase end
            glyph=font(character);
            if(tx%12<10 && ty[3:1]<7 && glyph[34-ty[3:1]*5-((tx%12)>>1)]) begin
                overlay_hit=enable;
                overlay_rgb=(bad_parking || wrong_hold!=0) ? 24'hff3030 : 24'h0040c0;
            end
            if(target_valid && !ty[4] && tx>=144 && tx<176 && warning_bitmap[15-speed_x[3:0]]) begin
                overlay_hit=enable;overlay_rgb=(bad_parking || wrong_hold!=0) ? 24'hff3030 : 24'h0040c0;
            end
            if(target_valid && bad_parking && ty[4] && tx<80 && warning_bitmap[15-tx[3:0]]) begin
                overlay_hit=enable;overlay_rgb=24'hff3030;
            end
        end else glyph=0;
    end
endmodule

// Sixty overlapping 32x32 windows, stride 16. Local 5/6-bit coordinates
// pack count, XY and both diagonal extents in 54 bits/window (no image RAM).
// Four serial updates per point; the six-step area unit is shared by all windows.
module vehicle_local_candidates(
    input wire clk,rst_n,enable,frame_start,point_valid,locked,
    input wire [7:0] x,bay_w,
    input wire [6:0] y,bay_h,
    input wire [7:0] track_x,
    input wire [6:0] track_y,
    output reg done,found,whole,
    output reg [7:0] left,right,
    output reg [6:0] top,bottom,
    output reg [9:0] count,
    output reg [2:0] direction
);
    localparam IDLE=0,UPDATE=1,SCAN_READ=2,SCAN_USE=3,SCAN_AREA=4;
    localparam [53:0] EMPTY={10'd0,5'd31,5'd0,5'd31,5'd0,6'd63,6'd0,6'd63,6'd0};
    (* ram_style="bram" *) reg [53:0] windows[0:63];
    reg [53:0] read_q;
    reg [29:0] best;
    reg [2:0] best_direction;
    (* fsm_encoding="sequential" *) reg [2:0] state;
    reg [2:0] area_step,scan_row;
    reg [1:0] step;
    // 33x33 maximum support extent needs 11 bits, not 12.
    reg [10:0] area,area_term;
    reg [5:0] area_bits,scan_index;
    reg [3:0] scan_col;
    reg [7:0] point_x;
    reg [6:0] point_y;
    reg [3:0] best_col;
    reg [2:0] best_row;
    reg best_found,best_complete,scan_enabled;
    function [5:0] window_address;
        input [7:0] px;
        input [6:0] py;
        input [1:0] offset;
        reg [3:0] col,row;
        begin
            col=px[7:4]-(offset[0] ? 4'd1 : 4'd0);
            row={1'b0,py[6:4]}-(offset[1] ? 4'd1 : 4'd0);
            window_address=row*6'd10+col;
        end
    endfunction
    wire update_valid=(!step[0] || point_x[7:4]!=0) && (!step[1] || point_y[6:4]!=0);
    wire start_point=state==IDLE && point_valid && enable && !frame_start;
    wire read_enable=start_point || (state==UPDATE && step!=3) || state==SCAN_READ;
    wire [5:0] read_address=start_point ? window_address(x,y,0) :
        state==UPDATE ? window_address(point_x,point_y,step+2'd1) : scan_index;
    wire [5:0] write_address=state==UPDATE ? window_address(point_x,point_y,step) : scan_index;
    wire [9:0] old_count=read_q[53:44];
    wire [4:0] old_l=read_q[43:39],old_r=read_q[38:34],old_t=read_q[33:29],old_b=read_q[28:24];
    wire [5:0] old_u0=read_q[23:18],old_u1=read_q[17:12],old_v0=read_q[11:6],old_v1=read_q[5:0];
    wire [4:0] lx={step[0],point_x[3:0]},ly={step[1],point_y[3:0]};
    wire [5:0] lu={1'b0,lx}+{1'b0,ly},lv={1'b0,lx}+6'd31-{1'b0,ly};
    wire [4:0] updated_l=old_count==0 || lx<old_l ? lx : old_l;
    wire [4:0] updated_r=old_count==0 || lx>old_r ? lx : old_r;
    wire [4:0] updated_t=old_count==0 || ly<old_t ? ly : old_t;
    wire [4:0] updated_b=old_count==0 || ly>old_b ? ly : old_b;
    wire [5:0] updated_u0=old_count==0 || lu<old_u0 ? lu : old_u0;
    wire [5:0] updated_u1=old_count==0 || lu>old_u1 ? lu : old_u1;
    wire [5:0] updated_v0=old_count==0 || lv<old_v0 ? lv : old_v0;
    wire [5:0] updated_v1=old_count==0 || lv>old_v1 ? lv : old_v1;
    wire [9:0] updated_count=old_count==1023 ? old_count : old_count+1'b1;
    wire write_enable=(state==UPDATE && update_valid) || state==SCAN_USE;
    wire [53:0] write_data=state==SCAN_USE ? EMPTY :
        {updated_count,updated_l,updated_r,updated_t,updated_b,updated_u0,updated_u1,updated_v0,updated_v1};
    always @(posedge clk) begin
        if(read_enable) read_q<=windows[read_address];
        if(write_enable) windows[write_address]<=write_data;
    end
    // Recover the full 2x2 support extent around the stored bottom-right points.
    wire [8:0] width={4'd0,old_r}-{4'd0,old_l}+9'd2;
    wire [7:0] height={3'd0,old_b}-{3'd0,old_t}+8'd2;
    wire shape_ok=width<=({1'b0,bay_w}+({1'b0,bay_w}>>2)) && height<=({1'b0,bay_h}<<1) &&
        width<=({3'd0,height}*11'd6) && height<=({2'd0,width}*11'd6);
    wire dense_enough={old_count,2'b00}>=area;
    wire [7:0] min_hits=bay_w<8 ? 8'd4 : bay_w>>1;
    // Associate only after acquisition. Eight missing frames release the lock
    // in the caller, allowing full-field reacquisition instead of a permanent ROI.
    wire [8:0] local_cx={1'b0,scan_col,4'b0}+{4'd0,old_l}+({4'd0,old_r}-{4'd0,old_l}>>1);
    wire [7:0] local_cy={1'b0,scan_row,4'b0}+{3'd0,old_t}+({3'd0,old_b}-{3'd0,old_t}>>1);
    wire [8:0] dx=local_cx>{1'b0,track_x} ? local_cx-{1'b0,track_x} : {1'b0,track_x}-local_cx;
    wire [7:0] dy=local_cy>{1'b0,track_y} ? local_cy-{1'b0,track_y} : {1'b0,track_y}-local_cy;
    wire [8:0] track_radius=({1'b0,bay_w}>>1)+9'd4;
    wire associated=!locked || (dx<=track_radius && {1'b0,dy}<=track_radius);
    wire eligible=associated && width>=4 && height>=4 && scan_enabled && scan_index<60 && old_count>=4 &&
        old_l<=old_r && old_t<=old_b && shape_ok && dense_enough && old_count>=min_hits &&
        {1'b0,old_count}<=({3'd0,bay_w}<<3);
    // Support count comes first: a tiny complete reflection/shadow must not
    // beat the car. Prefer an unclipped window only when evidence counts tie.
    wire complete=(old_l!=0 || scan_col==0) && (old_r!=31 || scan_col>=8) &&
                  (old_t!=0 || scan_row==0) && (old_b!=31 || scan_row>=4);
    wire replace_best=(!best_found && eligible) ||
        (eligible==best_found && ((!eligible && old_count>best[29:20]) ||
        (eligible && (old_count>best[29:20] ||
         (old_count==best[29:20] && complete && !best_complete)))));
    wire [29:0] chosen=replace_best ? read_q[53:24] : best;
    wire chosen_found=best_found || eligible;
    wire [7:0] chosen_x=replace_best ? {scan_col,4'b0000} : {best_col,4'b0000};
    wire [6:0] chosen_y=replace_best ? {scan_row,4'b0000} : {best_row,4'b0000};
    wire [8:0] chosen_l={1'b0,chosen_x}+{4'd0,chosen[19:15]};
    wire [7:0] chosen_t={1'b0,chosen_y}+{3'd0,chosen[9:5]};
    wire [6:0] cw={2'd0,old_r}-{2'd0,old_l}+7'd2;
    wire [6:0] ch={2'd0,old_b}-{2'd0,old_t}+7'd2;
    wire [6:0] du={1'b0,old_u1}-{1'b0,old_u0}+7'd3;
    wire [6:0] dv={1'b0,old_v1}-{1'b0,old_v0}+7'd3;
    // For a rotated rectangle, the difference between the two diagonal
    // spans cancels body thickness. Unlike a bounding-box aspect ratio it
    // is zero for a horizontal/vertical body, even after symmetric clipping.
    // Quantize the angle with shift/add comparisons (no atan or multiplier).
    wire [6:0] axis_delta=cw>=ch ? cw-ch : ch-cw;
    wire [6:0] diagonal_delta=du>=dv ? du-dv : dv-du;
    wire [9:0] axis_five={axis_delta,2'b00}+{3'd0,axis_delta};
    wire axis_pose={diagonal_delta,3'b000}<=axis_five;
    wire diagonal_pose={3'd0,diagonal_delta}>=axis_five;
    wire [2:0] current_direction=axis_pose ? (cw>=ch ? 3'd0 : 3'd4) :
        diagonal_pose ? (du>=dv ? 3'd2 : 3'd6) :
        cw>=ch ? (du>=dv ? 3'd1 : 3'd7) : (du>=dv ? 3'd3 : 3'd5);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            state<=IDLE;step<=0;scan_index<=0;scan_col<=0;scan_row<=0;point_x<=0;point_y<=0;
            area_step<=0;area<=0;area_term<=0;area_bits<=0;
            best<=EMPTY[53:24];best_found<=0;best_complete<=0;scan_enabled<=0;best_col<=0;best_row<=0;best_direction<=0;
            done<=0;found<=0;whole<=0;left<=159;right<=0;top<=89;bottom<=0;count<=0;direction<=0;
        end else begin
            done<=0;
            case(state)
                IDLE:begin
                    if(frame_start) begin
                        scan_index<=0;scan_col<=0;scan_row<=0;state<=SCAN_READ;best<=EMPTY[53:24];
                        best_found<=0;best_complete<=0;scan_enabled<=enable;
                    end else if(start_point) begin point_x<=x;point_y<=y;step<=0;state<=UPDATE;end
                end
                UPDATE:if(step==3) state<=IDLE;else step<=step+1'b1;
                SCAN_READ:begin state<=SCAN_AREA;area_step<=0;end
                SCAN_AREA:begin
                    if(area_step==0) begin area<=0;area_term<={5'd0,width[5:0]};area_bits<=height[5:0];end
                    else begin
                        if(area_bits[0]) area<=area+area_term;
                        area_term<=area_term<<1;area_bits<=area_bits>>1;
                    end
                    if(area_step==6) state<=SCAN_USE;else area_step<=area_step+1'b1;
                end
                SCAN_USE:begin
                    best<=chosen;best_found<=chosen_found;
                    if(replace_best) begin best_col<=scan_col;best_row<=scan_row;best_direction<=current_direction;best_complete<=eligible && complete;end
                    if(scan_index==63) begin
                        left<=chosen_l==0 ? 8'd0 : chosen_l-1'b1;
                        right<=chosen_x+{3'd0,chosen[14:10]};
                        top<=chosen_t==0 ? 7'd0 : chosen_t-1'b1;
                        bottom<=chosen_y+{2'd0,chosen[4:0]};count<=chosen[29:20];
                        direction<=replace_best ? current_direction : best_direction;found<=chosen_found;
                        whole<=chosen_found && (replace_best ? complete : best_complete);done<=1;state<=IDLE;
                    end else begin
                        scan_index<=scan_index+1'b1;state<=SCAN_READ;
                        if(scan_col==9) begin scan_col<=0;scan_row<=scan_row+1'b1;end else scan_col<=scan_col+1'b1;
                    end
                end
            endcase
        end
    end
endmodule
