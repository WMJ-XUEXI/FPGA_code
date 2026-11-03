`timescale 1ns/1ps
module parking_hdmi_legacy #(
    parameter H_OFFSET = 128,
    parameter V_OFFSET = 60,
    parameter IMG_WIDTH = 1920,
    parameter IMG_HEIGHT = 1080,
    parameter integer DEBUG_MODE = 0,
    parameter integer GAMMA_ENABLE = 1,
    parameter integer GAMMA_MIX = 4,
    parameter integer SATURATION_Q8 = 256,
    parameter integer CAL_WARM_FRAMES = 120
)(
    input wire demo_enter,demo_exit,input wire [15:0] demo_selected_id,
    output wire [47:0] demo_ids,output wire [59:0] demo_ids_decimal,output wire [2:0] demo_active,demo_observed,demo_registered,
    output wire [5:0] demo_codes,output wire demo_result_valid,output wire [3:0] demo_result,
    input wire        I_clk,
    input wire        I_rst_n,
    input wire [95:0] I_diag_data,
    input wire I_diag_req,
    output reg O_diag_ack,

    input wire        I_video_vsync,
    input wire        I_video_hsync,
    input wire        I_video_de,
    input wire        I_video_user,
    input wire        I_video_last,

    input wire[3:0]   I_debug_status,
    input wire[3:0]   I_cal_buttons_n,
    input wire [1:0] I_touch_keys,
    output wire [21:0] O_park_occupied,
    output wire O_park_valid,
    output wire [3:0] O_park_status,
    input wire        I_light_mode,
    input wire        I_tracking,
    input wire [7:0]  I_vehicle_threshold,
    output wire       O_buzzer_n,
    output reg [23:0] O_fill_rgb,
    output wire O_fill_ready,
    output wire [2:0] O_exposure_gray,
    output wire       O_uart_tx,

    output wire       O_video_rd_en,
    input wire[23:0]  I_video_rd_data,

    output reg        O_hdmi_vsync,
    output reg        O_hdmi_hsync,
    output reg        O_hdmi_de,
    output reg[23:0]  O_hdmi_data
);

    wire [23:0] view_rgb;
    wire view_de,view_user,view_last,view_vs,view_hs,view_edge;
    parking_edge_stream u_edge_stream(I_clk,I_rst_n,I_video_rd_data,
        I_video_de,I_video_user,I_video_last,I_video_vsync,I_video_hsync,I_light_mode,
        view_rgb,view_de,view_user,view_last,view_vs,view_hs,view_edge);
    reg        S_video_vsync_1d;
    reg        S_video_vsync_2d;
    reg        S_video_hsync_1d;
    reg        S_video_hsync_2d;
    reg        S_video_de_1d;
    reg        S_video_de_2d;
    reg [23:0] S_video_data_1d;
    reg [23:0] S_video_data_2d;
    wire [23:0] camera_display_rgb;
    wire [23:0] camera_tone_rgb;
    camera_gamma_rgb #(.ENABLE(GAMMA_ENABLE),.MIX(GAMMA_MIX)) u_camera_gamma (
        .rgb_in(S_video_data_2d), .rgb_out(camera_tone_rgb)
    );
    camera_saturation_rgb #(.SAT_Q8(SATURATION_Q8)) u_camera_saturation (
        .rgb_in(camera_tone_rgb), .rgb_out(camera_display_rgb)
    );
    reg [11:0] S_x;
    reg [11:0] S_y;

    // Mode 4: RGB888 output-chain test. Pipeline matches DE (two stages
    // before the output register); independent of camera/DDR pixel values.
    reg [11:0] diag_x, diag_y;
    wire [11:0] test_x = view_user ? 12'd0 : diag_x;
    wire [11:0] test_y = view_user ? 12'd0 : diag_y;
    reg [23:0] test_rgb;
    reg [7:0] test_gray;
    reg [23:0] test_rgb_d1, test_rgb_d2;
    integer band;

    // Analysis is a passive tap of pre-Gamma pixels; video latency is unchanged.
    reg [11:0] roi_x_d1,roi_x_d2,roi_y_d1,roi_y_d2;
    reg roi_user_d1,roi_user_d2,roi_user_d3;
    wire [2:0] park_state;wire [21:0] park_occupied;
    wire [1:0] board_press;
    wire [1:0] key_press=board_press | I_touch_keys;
    wire [4:0] park_free,park_used,park_total,park_detected;
    wire [7:0] block_x,block_level;
    wire [6:0] block_y;
    wire [175:0] park_lefts,park_rights;
    wire [153:0] park_tops,park_bottoms;
    wire block_valid,block_ink_raw,park_ambiguous,park_failed;
    wire stable_block_valid,stable_block_ink;
    wire [7:0] stable_block_x,stable_block_level;
    wire [6:0] stable_block_y;
    reg edge_d1,edge_d2,diff_d2,mask_d2;
    reg mode_meta,mode_sync,mode_prev,mode_recal_required;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            mode_meta<=1;mode_sync<=1;mode_prev<=1;mode_recal_required<=0;
        end else begin
            mode_meta<=I_light_mode;mode_sync<=mode_meta;mode_prev<=mode_sync;
            // Restart WARM/SCAN on a lighting change; never reuse the old background.
            mode_recal_required<=(mode_sync!=mode_prev);
        end
    end
    wire [2:0] exposure_step=3'd7; // Sensor exposure stays fixed in both lighting modes.
    wire [7:0] left_bright_cells;
    parking_exposure_meter u_exposure_meter(
        .clk(I_clk),.rst_n(I_rst_n),.adjust(1'b0),.frame_start(roi_user_d2),
        .cell_valid(block_valid),.x(block_x),.y(block_y),.level(block_level),
        .step(),.left_bright(left_bright_cells));
    assign O_exposure_gray=exposure_step^(exposure_step>>1);
    // Temporary board diagnostics: key events and complete analysis cells/frame.
    reg [3:0] key1_count,key2_count;
    reg [15:0] frame_cells,last_frame_cells;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin key1_count<=0;key2_count<=0;frame_cells<=0;last_frame_cells<=0;end
        else begin
            if(key_press[0]) key1_count<=key1_count+1'b1;
            if(key_press[1]) key2_count<=key2_count+1'b1;
            if(roi_user_d2) begin last_frame_cells<=frame_cells;frame_cells<=0;end
            else if(block_valid) frame_cells<=frame_cells+1'b1;
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            roi_x_d1<=0;roi_x_d2<=0;roi_y_d1<=0;roi_y_d2<=0;
            roi_user_d1<=0;roi_user_d2<=0;roi_user_d3<=0;
        end else begin
            roi_x_d1<=test_x;roi_x_d2<=roi_x_d1;
            roi_y_d1<=test_y;roi_y_d2<=roi_y_d1;
            roi_user_d1<=view_user;roi_user_d2<=roi_user_d1;roi_user_d3<=roi_user_d2;
        end
    end
    // Same rounded luma as the previous ISP grayscale path, before display gamma.
    wire [15:0] analysis_r={8'd0,S_video_data_2d[23:16]};
    wire [15:0] analysis_g={8'd0,S_video_data_2d[15:8]};
    wire [15:0] analysis_b={8'd0,S_video_data_2d[7:0]};
    wire [15:0] analysis_sum=analysis_r*16'd77+analysis_g*16'd150+analysis_b*16'd29+16'd128;
    wire [7:0] analysis_gray=analysis_sum[15:8];
    // The GRAY page is for observation.  Show luma after the existing camera
    // tone enhancement so a valid but dark sensor image remains visible.
    wire [7:0] display_gray={2'b0,camera_display_rgb[23:18]}+
                            {1'b0,camera_display_rgb[15:9]}+
                            {2'b0,camera_display_rgb[7:2]};
    // Calibration must use the camera luma before display gamma/saturation.
    // Display enhancement raises dark sensor noise and made the old mask follow
    // illumination rather than the black tape.
    wire [7:0] detector_gray=analysis_gray;
    parking_blocks u_parking_blocks (
        .clk(I_clk),.rst_n(I_rst_n),.valid(S_video_de_2d),
        .x(roi_x_d2),.y(roi_y_d2),.gray(detector_gray),.edge_pixel(edge_d2),
        .cell_valid(block_valid),.cx(block_x),.cy(block_y),.level(block_level),.ink(block_ink_raw)
    );
    // EDGE evidence, rather than absolute brightness, forms the calibration
    // mask. The finder flood-fills the interiors enclosed by these line cells.
    parking_temporal_mask u_parking_temporal_mask(
        I_clk,I_rst_n,block_valid,block_y*160+block_x,block_ink_raw,
        block_x,block_y,block_level,
        stable_block_valid,stable_block_ink,stable_block_x,stable_block_y,stable_block_level);
    parking_keys u_parking_keys (I_clk,I_rst_n,I_cal_buttons_n[1:0],board_press);
    wire diff_valid,diff_hit,diff_pixel,diff_ready;
    wire track_learning,track_ready,track_overlay;wire diff_static;
    reg [7:0] track_sample_x;
    reg [6:0] track_sample_y;
    always @(posedge I_clk) if(stable_block_valid) begin
        track_sample_x<=stable_block_x;track_sample_y<=stable_block_y;
    end
    wire [23:0] track_rgb;
    vehicle_tracker_multi u_vehicle_tracker(
        .demo_enter(demo_enter),.demo_exit(demo_exit),.demo_selected_id(demo_selected_id),
        .demo_ids(demo_ids),.demo_ids_decimal(demo_ids_decimal),.demo_active(demo_active),.demo_observed(demo_observed),.demo_registered(demo_registered),
        .demo_codes(demo_codes),.demo_result_valid(demo_result_valid),.demo_result(demo_result),

        .clk(I_clk),.rst_n(I_rst_n),.enable(I_tracking),.frame_start(roi_user_d3),
        .geometry_valid(park_total==22 && park_state>=3 && (&I_debug_status)),
        .light_changed(mode_recal_required),.confirm(key_press[0]),.recalibrate(key_press[1]),
        .sample_valid(diff_valid),.sample_hit(diff_hit),.sample_x(track_sample_x),.sample_y(track_sample_y),
        .lefts(park_lefts),.rights(park_rights),.tops(park_tops),.bottoms(park_bottoms),
        .display_x(test_x),.display_y(test_y),.learning(track_learning),.background_ready(track_ready),
        .overlay_hit(track_overlay),.overlay_rgb(track_rgb),.buzzer_n(O_buzzer_n),.sample_static(diff_static));
    wire [13:0] diff_address;
    parking_core #(.WARM_FRAMES(CAL_WARM_FRAMES)) u_parking (
        .clk(I_clk),.rst_n(I_rst_n),.ready((&I_debug_status) && !mode_recal_required),.frame_start(roi_user_d3),
        .cell_valid(stable_block_valid),.ink(stable_block_ink),.cx(stable_block_x),.cy(stable_block_y),.level(stable_block_level),
        .confirm(key_press[0] && !I_tracking),.recalibrate(key_press[1] && !I_tracking),
        .dark_threshold(I_vehicle_threshold),.tracking(I_tracking),.track_learning(track_learning),.track_ready(track_ready),.weak_light(!mode_sync),.state(park_state),.occupied(park_occupied),
        .lefts(park_lefts),.rights(park_rights),.tops(park_tops),.bottoms(park_bottoms),
        .free_count(park_free),.used_count(park_used),.total_count(park_total),.detected_count(park_detected),.search_ambiguous(park_ambiguous),.search_failed(park_failed),
        .diff_valid(diff_valid),.diff_address(diff_address),.diff_hit(diff_hit),.diff_static(diff_static)
    );
    reg [2:0] parking_resume;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) parking_resume<=0;
        else if(I_tracking) parking_resume<=6;
        else if(view_user && parking_resume!=0) parking_resume<=parking_resume-1'b1;
    end
    parking_uart_status u_parking_uart_status (
        .clk(I_clk), .rst_n(I_rst_n), .frame_start(view_user), .occupied(park_occupied),
        .detected(park_detected), .used(park_used),
        .valid(!I_tracking && parking_resume==0 && park_state == 3'd5 && park_total == 5'd22 && !mode_recal_required),
        .strong_light(mode_sync), .recal_required(mode_recal_required),
        .tx(O_uart_tx)
    );
    // Passive status tap only. Invalid/learning/tracking is never shown as free.
    assign O_park_occupied=park_occupied;
    assign O_park_valid=!I_tracking && parking_resume==0 && park_state==5 &&
        park_total==22 && !mode_recal_required && (&I_debug_status);
    assign O_park_status=!(&I_debug_status) ? 4'd10 :
        I_tracking ? (track_ready ? 4'd8 : 4'd7) :
        (park_failed || park_ambiguous) ? 4'd9 :
        park_state==0 ? 4'd1 : park_state==1 ? 4'd2 :
        park_state==2 ? 4'd3 : park_state==3 ? 4'd4 :
        park_state==4 ? 4'd5 : 4'd6;
    wire [1:0] view_press;
    reg [2:0] requested_view,active_view;
    wire [2:0] next_view=(key_press[1] || view_press[0]) ? 0 :
        view_press[1] ? (requested_view==4 ? 1 : requested_view+1'b1) : requested_view;
    parking_keys u_view_keys(I_clk,I_rst_n,I_cal_buttons_n[3:2],view_press);
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin requested_view<=0;active_view<=0;end
        else begin requested_view<=next_view;if(view_user)active_view<=next_view;end
    end
    wire [13:0] display_cell=(test_y>>3)*160+(test_x>>3);
    wire [13:0] block_address=stable_block_y*160+stable_block_x;
    wire mask_pixel,mask_ready;
    parking_mask_view u_mask_view(I_clk,I_rst_n,roi_user_d3,
        stable_block_valid,block_address,stable_block_ink,display_cell,mask_pixel,mask_ready);
    parking_diff_view u_diff_view(I_clk,I_rst_n,park_state==5 && !key_press[1],roi_user_d3,
        diff_valid,diff_address,diff_hit,display_cell,diff_pixel,diff_ready);
    always @(posedge I_clk or negedge I_rst_n)begin
        if(!I_rst_n)begin edge_d1<=0;edge_d2<=0;diff_d2<=0;mask_d2<=0;end
        else begin edge_d1<=view_edge;edge_d2<=edge_d1;diff_d2<=diff_pixel;mask_d2<=mask_pixel;end
    end
    wire [23:0] selected_video=active_view==1 ? {display_gray,display_gray,display_gray} :
        active_view==2 ? {24{edge_d2}} :
        active_view==3 ? {24{mask_d2 && mask_ready}} :
        active_view==4 ? {24{diff_d2 && diff_ready && park_state==5}} : camera_display_rgb;
    wire [23:0] park_color=park_state==5 ? 24'h30ff30 : park_state==4 ? 24'h00bfff : 24'hffff00;

    // Bundled-data handshake: source holds the whole bus until ACK.
    reg [1:0] diag_req_sync;
    reg [95:0] diag_pending, diag_display;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            diag_req_sync<=0;O_diag_ack<=0;diag_pending<=0;diag_display<=0;
        end else begin
            diag_req_sync<={diag_req_sync[0],I_diag_req};
            if(diag_req_sync[1]!=O_diag_ack) begin
                diag_pending<=I_diag_data;
                O_diag_ack<=diag_req_sync[1];
            end
            if(view_user) diag_display<=diag_pending;
        end
    end
    // 5x7 hex font plus labels. All displayed numeric fields are hexadecimal.
    function [34:0] glyph;
        input integer c;
        begin case(c)
        0:glyph=35'b01110_10001_10011_10101_11001_10001_01110;
        1:glyph=35'b00100_01100_00100_00100_00100_00100_01110;
        2:glyph=35'b01110_10001_00001_00010_00100_01000_11111;
        3:glyph=35'b11110_00001_00001_01110_00001_00001_11110;
        4:glyph=35'b00010_00110_01010_10010_11111_00010_00010;
        5:glyph=35'b11111_10000_10000_11110_00001_00001_11110;
        6:glyph=35'b01110_10000_10000_11110_10001_10001_01110;
        7:glyph=35'b11111_00001_00010_00100_01000_01000_01000;
        8:glyph=35'b01110_10001_10001_01110_10001_10001_01110;
        9:glyph=35'b01110_10001_10001_01111_00001_00001_01110;
        10:glyph=35'b01110_10001_10001_11111_10001_10001_10001;
        11:glyph=35'b11110_10001_10001_11110_10001_10001_11110;
        12:glyph=35'b01111_10000_10000_10000_10000_10000_01111;
        13:glyph=35'b11110_10001_10001_10001_10001_10001_11110;
        14:glyph=35'b11111_10000_10000_11110_10000_10000_11111;
        15:glyph=35'b11111_10000_10000_11110_10000_10000_10000;
        16:glyph=35'b11110_10001_10001_11110_10000_10000_10000; // P
        17:glyph=35'b11110_10001_10001_11110_10100_10010_10001; // R
        18:glyph=35'b01110_10001_10001_10001_10001_10001_01110; // O
        19:glyph=35'b01111_10000_10000_01110_00001_00001_11110; // S
        20:glyph=35'b11111_00100_00100_00100_00100_00100_00100; // T
        21:glyph=35'b10001_11001_11001_10101_10011_10011_10001; // N
        22:glyph=35'b10001_10001_10001_10001_10001_01010_00100; // V
        23:glyph=35'b10001_10001_10001_10101_10101_10101_01010; // W
        24:glyph=35'b01110_00100_00100_00100_00100_00100_01110; // I
        25:glyph=35'b10001_10001_10001_10001_10001_10001_01110; // U
        26:glyph=35'b10001_10001_01010_00100_00100_00100_00100; // Y
        27:glyph=35'b10000_10000_10000_10000_10000_10000_11111; // L
        28:glyph=35'b10001_11011_10101_10101_10001_10001_10001; // M
        29:glyph=35'b10001_10001_10001_11111_10001_10001_10001; // H
        30:glyph=35'b10001_10010_10100_11000_10100_10010_10001; // K
        32:glyph=35'b01110_10001_10000_10111_10001_10001_01110; // G
        default:glyph=0;
        endcase end
    endfunction
    wire [21:0] bay_outline,busy_outline,bay_inside;
    assign O_fill_ready=park_total==22 && park_state>=3 && !mode_recal_required;
    genvar ob;
    generate for(ob=0;ob<22;ob=ob+1)begin: outlines
        wire [11:0] x0={1'b0,park_lefts[ob*8+:8],3'b100};
        wire [11:0] x1={1'b0,park_rights[ob*8+:8],3'b100};
        wire [11:0] y0={2'b0,park_tops[ob*7+:7],3'b100};
        wire [11:0] y1={2'b0,park_bottoms[ob*7+:7],3'b100};
        // Do not flash isolated noise candidates.  Partial geometry is shown
        // only when it is already close to the required 22-bay structure.
        wire in_x=test_x>=x0 && test_x<=x1;
        wire in_y=test_y>=y0 && test_y<=y1;
        assign bay_outline[ob]=(ob<park_detected) && (park_state>=3 || park_detected>=18) &&
            (((test_x==x0 || test_x==x1) && in_y) || ((test_y==y0 || test_y==y1) && in_x));
        assign bay_inside[ob]=O_fill_ready && in_x && in_y;
        assign busy_outline[ob]=bay_outline[ob] && park_occupied[ob];
    end endgenerate
    // The two mask clocks match camera_display_rgb; the final register
    // matches this module's existing RGB output and the main mixer pipeline.
    reg [1:0] fill_mask_delay,fill_edge_delay;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin fill_mask_delay<=0;fill_edge_delay<=0;O_fill_rgb<=0;end
        else begin
            fill_mask_delay<={fill_mask_delay[0],(|bay_inside)};
            fill_edge_delay<={fill_edge_delay[0],((|bay_outline) && O_fill_ready)};
            if(fill_edge_delay[1]) O_fill_rgb<=24'h30ff30;
            else if(fill_mask_delay[1]) O_fill_rgb<={1'b0,camera_display_rgb[23:17],1'b1,camera_display_rgb[15:9],1'b0,camera_display_rgb[7:1]};
            else O_fill_rgb<=camera_display_rgb;
        end
    end
    wire stats_hit;wire [23:0] stats_rgb;
    parking_osd22 u_stats(I_clk,I_rst_n,test_x,test_y,park_state,park_total,park_used,park_free,park_detected,park_ambiguous,stats_hit,stats_rgb);
    reg overlay_on;
    reg [23:0] overlay_rgb;
    reg overlay_d1,overlay_d2;
    reg [23:0] overlay_rgb_d1,overlay_rgb_d2;
    integer row,col,gx,gy,ch;
    integer bay,bx0,bx1,by0,by1;
    reg [23:0] bay_color;
    reg [23:0] field;
    reg [34:0] bits_font;
    always @* begin
        overlay_on=0;overlay_rgb=0;row=0;col=0;gx=0;gy=0;ch=31;field=0;bits_font=0;
        bx0=0;bx1=0;by0=0;by1=0;bay_color=0;
        if(I_tracking) begin
            overlay_on=track_overlay;overlay_rgb=track_rgb;
        end else if(test_x>=800 && test_x<944 && test_y>=16 && test_y<36)begin
            overlay_on=1;overlay_rgb=0;
            col=(test_x-800)/16;gx=((test_x-800)%16)/2;gy=(test_y-16)/2;
            case(col)
                0:ch=10;1:ch=14;3:ch=exposure_step;
                5:ch=29;6:ch=left_bright_cells/10;7:ch=left_bright_cells%10;
            endcase
            bits_font=glyph(ch);
            if(gx<5 && gy<7 && bits_font[34-gy*5-gx])overlay_rgb=24'hffffff;
        end else if(active_view!=0 && test_x>=544 && test_x<752 && test_y>=16 && test_y<36)begin
            overlay_on=1;overlay_rgb=0;
            col=(test_x-544)/16;gx=((test_x-544)%16)/2;gy=(test_y-16)/2;
            case(active_view)
            1:case(col)0:ch=32;1:ch=17;2:ch=10;3:ch=26;endcase
            2:case(col)0:ch=14;1:ch=13;2:ch=32;3:ch=14;endcase
            3:begin
                case(col)0:ch=28;1:ch=10;2:ch=19;3:ch=30;endcase
                if(!mask_ready)case(col)5:ch=23;6:ch=10;7:ch=24;8:ch=20;endcase
            end
            4:begin
                case(col)0:ch=13;1:ch=24;2:ch=15;3:ch=15;endcase
                if(!diff_ready || park_state!=5)case(col)5:ch=23;6:ch=10;7:ch=24;8:ch=20;endcase
            end
            endcase
            bits_font=glyph(ch);
            if(gx<5 && gy<7 && bits_font[34-gy*5-gx])overlay_rgb=24'hffffff;
        end else if(park_detected!=0 && (|bay_outline))begin
            overlay_on=1;
            overlay_rgb=park_state==5 ? ((|busy_outline) ? 24'hff3030 : 24'h30ff30) : park_color;
        end
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin overlay_d1<=0;overlay_d2<=0;overlay_rgb_d1<=0;overlay_rgb_d2<=0;end
        else begin
            overlay_d1<=overlay_on;overlay_d2<=overlay_d1;
            overlay_rgb_d1<=overlay_rgb;overlay_rgb_d2<=overlay_rgb_d1;
        end
    end

    always @* begin
        test_rgb = 24'h000000;
        test_gray = 8'd0;
        band = 0;
        if (view_de) begin
            if (test_y < IMG_HEIGHT/2) begin
                band = test_x / (IMG_WIDTH/8);
                case (band)
                    0: test_rgb = 24'hff0000;
                    1: test_rgb = 24'h00ff00;
                    2: test_rgb = 24'h0000ff;
                    3: test_rgb = 24'hffff00;
                    4: test_rgb = 24'h00ffff;
                    5: test_rgb = 24'hff00ff;
                    6: test_rgb = 24'hffffff;
                    default: test_rgb = 24'h000000;
                endcase
            end else begin
                band = test_x / (IMG_WIDTH/16);
                if (test_y < (IMG_HEIGHT*3)/4)
                    test_gray = band * 17; // 0,17,...,255
                else if (test_y < (IMG_HEIGHT*7)/8)
                    test_gray = band * 2;  // near black: 0,2,...,30
                else
                    test_gray = 225 + band * 2; // near white: 225,...,255
                test_rgb = {test_gray, test_gray, test_gray};
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            diag_x <= 0;
            diag_y <= 0;
            test_rgb_d1 <= 0;
            test_rgb_d2 <= 0;
        end else begin
            test_rgb_d1 <= test_rgb;
            test_rgb_d2 <= test_rgb_d1;
            if (view_user) begin
                diag_x <= view_de ? 12'd1 : 12'd0;
                diag_y <= 0;
            end else if (view_de) begin
                if (view_last) begin
                    diag_x <= 0;
                    diag_y <= diag_y + 1'b1;
                end else
                    diag_x <= diag_x + 1'b1;
            end
        end
    end

    assign O_video_rd_en = I_video_de;

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(view_user) begin
            S_x <= 12'd0;
            S_y <= 12'd0;
        end
        else if(view_de) begin
            if(view_last) begin
                S_x <= 12'd0;
                S_y <= S_y + 12'd1;
            end
            else begin
                S_x <= S_x + 12'd1;
            end
        end
    end

    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            S_video_vsync_1d <= 1'b0;
            S_video_vsync_2d <= 1'b0;
            S_video_hsync_1d <= 1'b0;
            S_video_hsync_2d <= 1'b0;
            S_video_de_1d    <= 1'b0;
            S_video_de_2d    <= 1'b0;
            O_hdmi_vsync     <= 1'b0;
            O_hdmi_hsync     <= 1'b0;
            O_hdmi_de        <= 1'b0;
            O_hdmi_data      <= 24'd0;
        end
        else begin
            S_video_vsync_1d <= view_vs;
            S_video_vsync_2d <= S_video_vsync_1d;
            O_hdmi_vsync     <= S_video_vsync_2d;

            S_video_hsync_1d <= view_hs;
            S_video_hsync_2d <= S_video_hsync_1d;
            O_hdmi_hsync     <= S_video_hsync_2d;

            S_video_de_1d    <= view_de;
            S_video_de_2d    <= S_video_de_1d;
            O_hdmi_de        <= S_video_de_2d;
            S_video_data_1d  <= view_rgb;
            S_video_data_2d  <= S_video_data_1d;

            if(DEBUG_MODE == 5) begin
                O_hdmi_data <= S_video_de_2d ? ((stats_hit && !I_tracking) ? stats_rgb : overlay_d2 ? overlay_rgb_d2 : selected_video) : 24'd0;
            end
            else if(DEBUG_MODE == 4) begin
                O_hdmi_data <= S_video_de_2d ? test_rgb_d2 : 24'd0;
            end
            else if(DEBUG_MODE == 1) begin
                if(S_video_de_2d) begin
                    if(S_x < 12'd128)
                        O_hdmi_data <= 24'hff0000;
                    else if(S_x < 12'd256)
                        O_hdmi_data <= 24'h00ff00;
                    else if(S_x < 12'd384)
                        O_hdmi_data <= 24'h0000ff;
                    else if(S_x < 12'd512)
                        O_hdmi_data <= 24'hffff00;
                    else if(S_x < 12'd640)
                        O_hdmi_data <= 24'h00ffff;
                    else if(S_x < 12'd768)
                        O_hdmi_data <= 24'hff00ff;
                    else if(S_x < 12'd896)
                        O_hdmi_data <= 24'hffffff;
                    else
                        O_hdmi_data <= 24'h202020;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 2) begin
                if(S_video_de_2d) begin
                    case (I_debug_status)
                        4'b0000: O_hdmi_data <= 24'h000000;
                        4'b0001: O_hdmi_data <= 24'hff0000;
                        4'b0011: O_hdmi_data <= 24'hffff00;
                        4'b0111: O_hdmi_data <= 24'h00ff00;
                        4'b1111: O_hdmi_data <= view_rgb;
                        default: O_hdmi_data <= 24'h0000ff;
                    endcase
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else if(DEBUG_MODE == 3) begin
                if(S_video_de_2d) begin
                    if(S_x < (IMG_WIDTH/4))
                        O_hdmi_data <= I_debug_status[0] ? 24'hff8000 : 24'h404040;
                    else if(S_x < (IMG_WIDTH/2))
                        O_hdmi_data <= I_debug_status[1] ? 24'hffff00 : 24'h404040;
                    else if(S_x < ((IMG_WIDTH*3)/4))
                        O_hdmi_data <= I_debug_status[2] ? 24'hff0000 : 24'h404040;
                    else
                        O_hdmi_data <= I_debug_status[3] ? 24'h00ffff : 24'h404040;
                end
                else begin
                    O_hdmi_data <= 24'd0;
                end
            end
            else begin
                O_hdmi_data <= camera_display_rgb;
            end
        end
    end

endmodule

// Camera-only display transfer: round(255*(input/255)^(1/2.2)).
// Same monotonic curve on R/G/B; black=0, white=255; no added latency.
// Test pattern and diagnostic text bypass this stage.
module camera_gamma_rgb #(parameter integer ENABLE=1, parameter integer MIX=4) (
    input [23:0] rgb_in,
    output [23:0] rgb_out
);
    function [7:0] encode;
        input [7:0] value;
        begin case(value)
            8'd0: encode = 8'd0;
            8'd1: encode = 8'd21;
            8'd2: encode = 8'd28;
            8'd3: encode = 8'd34;
            8'd4: encode = 8'd39;
            8'd5: encode = 8'd43;
            8'd6: encode = 8'd46;
            8'd7: encode = 8'd50;
            8'd8: encode = 8'd53;
            8'd9: encode = 8'd56;
            8'd10: encode = 8'd59;
            8'd11: encode = 8'd61;
            8'd12: encode = 8'd64;
            8'd13: encode = 8'd66;
            8'd14: encode = 8'd68;
            8'd15: encode = 8'd70;
            8'd16: encode = 8'd72;
            8'd17: encode = 8'd74;
            8'd18: encode = 8'd76;
            8'd19: encode = 8'd78;
            8'd20: encode = 8'd80;
            8'd21: encode = 8'd82;
            8'd22: encode = 8'd84;
            8'd23: encode = 8'd85;
            8'd24: encode = 8'd87;
            8'd25: encode = 8'd89;
            8'd26: encode = 8'd90;
            8'd27: encode = 8'd92;
            8'd28: encode = 8'd93;
            8'd29: encode = 8'd95;
            8'd30: encode = 8'd96;
            8'd31: encode = 8'd98;
            8'd32: encode = 8'd99;
            8'd33: encode = 8'd101;
            8'd34: encode = 8'd102;
            8'd35: encode = 8'd103;
            8'd36: encode = 8'd105;
            8'd37: encode = 8'd106;
            8'd38: encode = 8'd107;
            8'd39: encode = 8'd109;
            8'd40: encode = 8'd110;
            8'd41: encode = 8'd111;
            8'd42: encode = 8'd112;
            8'd43: encode = 8'd114;
            8'd44: encode = 8'd115;
            8'd45: encode = 8'd116;
            8'd46: encode = 8'd117;
            8'd47: encode = 8'd118;
            8'd48: encode = 8'd119;
            8'd49: encode = 8'd120;
            8'd50: encode = 8'd122;
            8'd51: encode = 8'd123;
            8'd52: encode = 8'd124;
            8'd53: encode = 8'd125;
            8'd54: encode = 8'd126;
            8'd55: encode = 8'd127;
            8'd56: encode = 8'd128;
            8'd57: encode = 8'd129;
            8'd58: encode = 8'd130;
            8'd59: encode = 8'd131;
            8'd60: encode = 8'd132;
            8'd61: encode = 8'd133;
            8'd62: encode = 8'd134;
            8'd63: encode = 8'd135;
            8'd64: encode = 8'd136;
            8'd65: encode = 8'd137;
            8'd66: encode = 8'd138;
            8'd67: encode = 8'd139;
            8'd68: encode = 8'd140;
            8'd69: encode = 8'd141;
            8'd70: encode = 8'd142;
            8'd71: encode = 8'd143;
            8'd72: encode = 8'd144;
            8'd73: encode = 8'd144;
            8'd74: encode = 8'd145;
            8'd75: encode = 8'd146;
            8'd76: encode = 8'd147;
            8'd77: encode = 8'd148;
            8'd78: encode = 8'd149;
            8'd79: encode = 8'd150;
            8'd80: encode = 8'd151;
            8'd81: encode = 8'd151;
            8'd82: encode = 8'd152;
            8'd83: encode = 8'd153;
            8'd84: encode = 8'd154;
            8'd85: encode = 8'd155;
            8'd86: encode = 8'd156;
            8'd87: encode = 8'd156;
            8'd88: encode = 8'd157;
            8'd89: encode = 8'd158;
            8'd90: encode = 8'd159;
            8'd91: encode = 8'd160;
            8'd92: encode = 8'd160;
            8'd93: encode = 8'd161;
            8'd94: encode = 8'd162;
            8'd95: encode = 8'd163;
            8'd96: encode = 8'd164;
            8'd97: encode = 8'd164;
            8'd98: encode = 8'd165;
            8'd99: encode = 8'd166;
            8'd100: encode = 8'd167;
            8'd101: encode = 8'd167;
            8'd102: encode = 8'd168;
            8'd103: encode = 8'd169;
            8'd104: encode = 8'd170;
            8'd105: encode = 8'd170;
            8'd106: encode = 8'd171;
            8'd107: encode = 8'd172;
            8'd108: encode = 8'd173;
            8'd109: encode = 8'd173;
            8'd110: encode = 8'd174;
            8'd111: encode = 8'd175;
            8'd112: encode = 8'd175;
            8'd113: encode = 8'd176;
            8'd114: encode = 8'd177;
            8'd115: encode = 8'd178;
            8'd116: encode = 8'd178;
            8'd117: encode = 8'd179;
            8'd118: encode = 8'd180;
            8'd119: encode = 8'd180;
            8'd120: encode = 8'd181;
            8'd121: encode = 8'd182;
            8'd122: encode = 8'd182;
            8'd123: encode = 8'd183;
            8'd124: encode = 8'd184;
            8'd125: encode = 8'd184;
            8'd126: encode = 8'd185;
            8'd127: encode = 8'd186;
            8'd128: encode = 8'd186;
            8'd129: encode = 8'd187;
            8'd130: encode = 8'd188;
            8'd131: encode = 8'd188;
            8'd132: encode = 8'd189;
            8'd133: encode = 8'd190;
            8'd134: encode = 8'd190;
            8'd135: encode = 8'd191;
            8'd136: encode = 8'd192;
            8'd137: encode = 8'd192;
            8'd138: encode = 8'd193;
            8'd139: encode = 8'd194;
            8'd140: encode = 8'd194;
            8'd141: encode = 8'd195;
            8'd142: encode = 8'd195;
            8'd143: encode = 8'd196;
            8'd144: encode = 8'd197;
            8'd145: encode = 8'd197;
            8'd146: encode = 8'd198;
            8'd147: encode = 8'd199;
            8'd148: encode = 8'd199;
            8'd149: encode = 8'd200;
            8'd150: encode = 8'd200;
            8'd151: encode = 8'd201;
            8'd152: encode = 8'd202;
            8'd153: encode = 8'd202;
            8'd154: encode = 8'd203;
            8'd155: encode = 8'd203;
            8'd156: encode = 8'd204;
            8'd157: encode = 8'd205;
            8'd158: encode = 8'd205;
            8'd159: encode = 8'd206;
            8'd160: encode = 8'd206;
            8'd161: encode = 8'd207;
            8'd162: encode = 8'd207;
            8'd163: encode = 8'd208;
            8'd164: encode = 8'd209;
            8'd165: encode = 8'd209;
            8'd166: encode = 8'd210;
            8'd167: encode = 8'd210;
            8'd168: encode = 8'd211;
            8'd169: encode = 8'd212;
            8'd170: encode = 8'd212;
            8'd171: encode = 8'd213;
            8'd172: encode = 8'd213;
            8'd173: encode = 8'd214;
            8'd174: encode = 8'd214;
            8'd175: encode = 8'd215;
            8'd176: encode = 8'd215;
            8'd177: encode = 8'd216;
            8'd178: encode = 8'd217;
            8'd179: encode = 8'd217;
            8'd180: encode = 8'd218;
            8'd181: encode = 8'd218;
            8'd182: encode = 8'd219;
            8'd183: encode = 8'd219;
            8'd184: encode = 8'd220;
            8'd185: encode = 8'd220;
            8'd186: encode = 8'd221;
            8'd187: encode = 8'd221;
            8'd188: encode = 8'd222;
            8'd189: encode = 8'd223;
            8'd190: encode = 8'd223;
            8'd191: encode = 8'd224;
            8'd192: encode = 8'd224;
            8'd193: encode = 8'd225;
            8'd194: encode = 8'd225;
            8'd195: encode = 8'd226;
            8'd196: encode = 8'd226;
            8'd197: encode = 8'd227;
            8'd198: encode = 8'd227;
            8'd199: encode = 8'd228;
            8'd200: encode = 8'd228;
            8'd201: encode = 8'd229;
            8'd202: encode = 8'd229;
            8'd203: encode = 8'd230;
            8'd204: encode = 8'd230;
            8'd205: encode = 8'd231;
            8'd206: encode = 8'd231;
            8'd207: encode = 8'd232;
            8'd208: encode = 8'd232;
            8'd209: encode = 8'd233;
            8'd210: encode = 8'd233;
            8'd211: encode = 8'd234;
            8'd212: encode = 8'd234;
            8'd213: encode = 8'd235;
            8'd214: encode = 8'd235;
            8'd215: encode = 8'd236;
            8'd216: encode = 8'd236;
            8'd217: encode = 8'd237;
            8'd218: encode = 8'd237;
            8'd219: encode = 8'd238;
            8'd220: encode = 8'd238;
            8'd221: encode = 8'd239;
            8'd222: encode = 8'd239;
            8'd223: encode = 8'd240;
            8'd224: encode = 8'd240;
            8'd225: encode = 8'd241;
            8'd226: encode = 8'd241;
            8'd227: encode = 8'd242;
            8'd228: encode = 8'd242;
            8'd229: encode = 8'd243;
            8'd230: encode = 8'd243;
            8'd231: encode = 8'd244;
            8'd232: encode = 8'd244;
            8'd233: encode = 8'd245;
            8'd234: encode = 8'd245;
            8'd235: encode = 8'd246;
            8'd236: encode = 8'd246;
            8'd237: encode = 8'd247;
            8'd238: encode = 8'd247;
            8'd239: encode = 8'd248;
            8'd240: encode = 8'd248;
            8'd241: encode = 8'd249;
            8'd242: encode = 8'd249;
            8'd243: encode = 8'd249;
            8'd244: encode = 8'd250;
            8'd245: encode = 8'd250;
            8'd246: encode = 8'd251;
            8'd247: encode = 8'd251;
            8'd248: encode = 8'd252;
            8'd249: encode = 8'd252;
            8'd250: encode = 8'd253;
            8'd251: encode = 8'd253;
            8'd252: encode = 8'd254;
            8'd253: encode = 8'd254;
            8'd254: encode = 8'd255;
            8'd255: encode = 8'd255;
            default: encode=0;
        endcase end
    endfunction
    // MIX=0: linear; 3: 75% encoded + 25% linear; 4: original full curve.
    function [7:0] tone;
        input [7:0] value;
        integer mixed;
        begin
            mixed=(MIX*encode(value)+(4-MIX)*value+2)/4;
            tone=mixed[7:0];
        end
    endfunction
    assign rgb_out = ENABLE ? {tone(rgb_in[23:16]),tone(rgb_in[15:8]),tone(rgb_in[7:0])} : rgb_in;
endmodule

// Display-domain chroma scaling about approximate Rec.601 luma.
// SAT_Q8=256 is exact bypass; 320 is 1.25x. Neutral gray stays neutral.
module camera_saturation_rgb #(parameter integer SAT_Q8=256) (
    input [23:0] rgb_in,
    output [23:0] rgb_out
);
    wire [7:0] r=rgb_in[23:16],g=rgb_in[15:8],b=rgb_in[7:0];
    wire [15:0] luma_sum=77*r+150*g+29*b+16'd128;
    wire [7:0] y=luma_sum[15:8];
    function [7:0] adjust;
        input [7:0] c,luma;
        integer delta,scaled;
        begin
            delta=c;delta=delta-luma;
            scaled=luma; // Keep the full expression signed, including negative chroma.
            scaled=(256*scaled+SAT_Q8*delta+128) >>> 8;
            if(scaled<0)adjust=0;
            else if(scaled>255)adjust=255;
            else adjust=scaled[7:0];
        end
    endfunction
    assign rgb_out=SAT_Q8==256 ? rgb_in : {adjust(r,y),adjust(g,y),adjust(b,y)};
endmodule

`timescale 1ns/1ps
`timescale 1ns/1ps
`timescale 1ns/1ps
`timescale 1ns/1ps
`timescale 1ns/1ps
`timescale 1ns/1ps
`timescale 1ns/1ps
`timescale 1ns/1ps
`timescale 1ns/1ps
// Average disjoint 2x2 pixels before contrast extraction. All pixels contribute;
// isolated sensor/Bayer noise no longer sets the extrema of an entire 8x8 cell.
module parking_blocks(input clk,rst_n,valid,input [11:0] x,y,input [7:0] gray,input edge_pixel,
 output cell_valid,output [7:0] cx,output [6:0] cy,output [7:0] level,output ink);
 reg [8:0] line_pairs[0:639];
 reg [7:0] left_pixel;
 wire [8:0] pair_sum={1'b0,left_pixel}+{1'b0,gray};
 reg [8:0] above_pair,current_pair;
 reg pair_valid;
 reg [11:0] pair_x,pair_y;
 wire [9:0] smooth_sum={1'b0,above_pair}+{1'b0,current_pair}+10'd2;
 reg [3:0] edge_horizontal_sum;
 reg [6:0] edge_vertical_sum[0:159];
 reg [6:0] edge_total;
 wire [3:0] next_edge_horizontal=x[2:0]==0 ? {3'b0,edge_pixel} :
                                     edge_horizontal_sum+edge_pixel;
 wire [6:0] next_edge_vertical=y[2:0]==0 ? {3'b0,next_edge_horizontal} :
                              edge_vertical_sum[x[10:3]]+next_edge_horizontal;
 always @(posedge clk) begin
  if(valid && x<1280 && y<720 && x[0]) begin
   if(!y[0]) line_pairs[x[10:1]]<=pair_sum;
   above_pair<=line_pairs[x[10:1]];
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin left_pixel<=0;current_pair<=0;pair_valid<=0;pair_x<=0;pair_y<=0;
   edge_horizontal_sum<=0;edge_total<=0;end
  else begin
   pair_valid<=valid && x<1280 && y<720 && x[0] && y[0];
   if(valid) left_pixel<=gray;
   if(valid && x[0]) begin current_pair<=pair_sum;pair_x<=x;pair_y<=y;end
   if(valid)begin
    edge_horizontal_sum<=next_edge_horizontal;
    if(x[2:0]==7)begin
     if(y[2:0]!=7)edge_vertical_sum[x[10:3]]<=next_edge_vertical;
     else edge_total<=next_edge_vertical;
    end
   end
  end
 end
 wire brightness_ink;
 parking_cells cells(clk,rst_n,pair_valid,pair_x,pair_y,smooth_sum[9:2],cell_valid,cx,cy,level,brightness_ink);
 // Require several Sobel pixels in an 8x8 cell. Isolated sensor speckles are
 // rejected while every real tape edge contributes a continuous cell chain.
 assign ink=cell_valid && edge_total>=7'd3;
endmodule

// Average all sixteen 2x2-filtered samples in each 8x8 source cell.  The old
// min/max detector turned one noisy pixel into a full white mask cell.  Here a
// slowly decaying white reference is propagated from the left and above; a
// cell is ink only when its mean is materially darker than that local reference.
module parking_cells(input clk,rst_n,valid,input [11:0] x,y,input [7:0] gray,
 output reg cell_valid,output reg [7:0] cx,output reg [6:0] cy,
 output reg [7:0] level,output reg ink);
 reg [9:0] horizontal_sum;
 reg [11:0] vertical_sum[0:159];
 reg [7:0] above_reference[0:159];
 reg [7:0] row_reference;
 reg [7:0] cc;
 reg [6:0] yy;
 reg [11:0] total_sum;
 reg pending;
 wire [9:0] next_horizontal=x[2:0]==1 ? {2'b0,gray} : horizontal_sum+gray;
 wire [11:0] next_vertical=y[2:0]==1 ? {2'b0,next_horizontal} :
                            vertical_sum[x[10:3]]+next_horizontal;
 wire [7:0] cell_mean=(total_sum+12'd8)>>4;
 wire [7:0] local_reference=yy==0 ? (cc==0 ? cell_mean:row_reference) :
                            cc==0 ? above_reference[cc] :
                            (above_reference[cc]>row_reference ? above_reference[cc]:row_reference);
 wire [7:0] reference_drop=local_reference>8'd63 ? 8'd3 :
                           local_reference>8'd31 ? 8'd2 : 8'd1;
 wire [7:0] decayed_reference=local_reference>reference_drop ?
                              local_reference-reference_drop:8'd0;
 wire [7:0] updated_reference=cell_mean>decayed_reference ?
                              cell_mean:decayed_reference;
 // Averaging removes pixel noise, so a modest contrast works in dim scenes.
 // The proportional term prevents bright scenes from accepting small ripples.
 wire [7:0] adaptive_gap=8'd6+{4'b0000,local_reference[7:4]};
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin horizontal_sum<=0;pending<=0;cell_valid<=0;cx<=0;cy<=0;
   level<=0;ink<=0;cc<=0;yy<=0;total_sum<=0;row_reference<=0;end
  else begin
   pending<=0;cell_valid<=0;
   if(valid && x<1280 && y<720) begin
    horizontal_sum<=next_horizontal;
    if(x[2:0]==7) begin
     if(y[2:0]!=7) vertical_sum[x[10:3]]<=next_vertical;
     else begin pending<=1;cc<=x[10:3];yy<=y[9:3];total_sum<=next_vertical;end
    end
   end
   if(pending) begin
    cell_valid<=1;cx<=cc;cy<=yy;level<=cell_mean;
    if(cc==0) row_reference<=cell_mean; else row_reference<=updated_reference;
    above_reference[cc]<=updated_reference;
    ink<=cc!=0 && yy!=0 && ({1'b0,cell_mean}+{1'b0,adaptive_gap}<{1'b0,local_reference});
   end
  end
 end
endmodule

// Per-cell majority over the current and previous two complete frames.
// WARM_FRAMES guarantees every history entry is valid before calibration.
module parking_temporal_mask(input clk,rst_n,input sample_valid,input [13:0] sample_address,
 input sample_hit,input [7:0] sample_x,input [6:0] sample_y,input [7:0] sample_level,
 output reg valid,output reg hit,output reg [7:0] x,output reg [6:0] y,output reg [7:0] level);
 // Synchronous read followed by update: infer ERAM rather than 2048 DRAM
 // LUTs. Cells arrive at least eight clocks apart. Coordinates, level and
 // valid travel with the read; the caller delays analysis frame_start too.
 (* ram_style="bram" *) reg [1:0] history[0:16383];
 reg [1:0] old_history;
 reg pending,hit_q;
 reg [13:0] address_q;
 reg [7:0] x_q,level_q;
 reg [6:0] y_q;
 // No reset on RAM/read register: calibration warm-up validates history.
 always @(posedge clk)begin
  if(rst_n && sample_valid) old_history<=history[sample_address];
  if(rst_n && pending) history[address_q]<={old_history[0],hit_q};
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   valid<=0;hit<=0;x<=0;y<=0;level<=0;
   pending<=0;hit_q<=0;address_q<=0;x_q<=0;y_q<=0;level_q<=0;
  end else begin
   pending<=sample_valid;
   if(sample_valid)begin
    address_q<=sample_address;hit_q<=sample_hit;
    x_q<=sample_x;y_q<=sample_y;level_q<=sample_level;
   end
   valid<=pending;
   if(pending)begin
    hit<=(hit_q & old_history[0]) | (hit_q & old_history[1]) |
         (old_history[0] & old_history[1]);
    x<=x_q;y<=y_q;level<=level_q;
   end
  end
 end
endmodule

// Meter the raw camera luma before display enhancement. The leftmost part of
// the sand table is where direct daylight previously flooded the mask.
// One sample per 8x8-cell grid point keeps the counters small. Exposure is
// changed one Gray-coded step at a time and held once calibration succeeds.
module parking_exposure_meter(input clk,rst_n,adjust,frame_start,
 input cell_valid,input [7:0] x,input [6:0] y,input [7:0] level,
 output reg [2:0] step,output reg [7:0] left_bright);
 reg [7:0] sample_count,bright_count,dark_count,left_count;
 reg [2:0] bright_frames,dark_frames;
 wire take=cell_valid && x[2:0]==0 && y[2:0]==0;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   step<=7;left_bright<=0;sample_count<=0;bright_count<=0;
   dark_count<=0;left_count<=0;bright_frames<=0;dark_frames<=0;
  end else if(frame_start)begin
   if(sample_count>=200)begin
    left_bright<=left_count;
    if(adjust && (left_count>=6 || bright_count>=12))begin
     dark_frames<=0;
     if(bright_frames==3)begin
      bright_frames<=0;if(step!=0)step<=step-1'b1;
     end else bright_frames<=bright_frames+1'b1;
    end else if(adjust && dark_count>=150 && bright_count==0)begin
     bright_frames<=0;
     if(dark_frames==7)begin
      dark_frames<=0;if(step!=7)step<=step+1'b1;
     end else dark_frames<=dark_frames+1'b1;
    end else begin bright_frames<=0;dark_frames<=0;end
   end
   sample_count<=0;bright_count<=0;dark_count<=0;left_count<=0;
  end else if(take)begin
   sample_count<=sample_count+1'b1;
   if(level>=235)begin
    bright_count<=bright_count+1'b1;
    if(x<56)left_count<=left_count+1'b1;
   end
   if(level<70)dark_count<=dark_count+1'b1;
  end
 end
endmodule

module parking_keys #(parameter integer DEBOUNCE=1500000)(input clk,rst_n,input [1:0] keys_n,output reg [1:0] press);
 reg [1:0] s1,s2,stable;reg [20:0] count0,count1;
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin s1<=3;s2<=3;stable<=3;count0<=0;count1<=0;press<=0;end
  else begin
   s1<=keys_n;s2<=s1;press<=0;
   if(s2[0]==stable[0]) count0<=0;
   else if(count0==DEBOUNCE-1) begin stable[0]<=s2[0];count0<=0;press[0]<=!s2[0];end else count0<=count0+1'b1;
   if(s2[1]==stable[1]) count1<=0;
   else if(count1==DEBOUNCE-1) begin stable[1]<=s2[1];count1<=0;press[1]<=!s2[1];end else count1<=count1+1'b1;
  end
 end
endmodule
`timescale 1ns/1ps
// Three adjacent bays, with shared boundaries or separate close double lines.
// Horizontal bands allow hand-drawn slope/curvature; each bay gets its own
// inscribed rectangle rather than forcing a common top and bottom coordinate.
`timescale 1ns/1ps
// Find enclosed light components in the black-line mask, then validate 6/5/5/6 columns.
// Coordinates come only from the image. No parking position is hard-coded.
module parking_finder(input clk,rst_n,start,output reg [13:0] address,input pixel,
 output reg busy,done,found,ambiguous,output reg [175:0] lefts,rights,
 output reg [153:0] tops,bottoms,output reg [4:0] count);
 localparam IDLE=0,C_REQ=1,C_WAIT=2,C_USE=3,S_REQ=4,S_WAIT=5,S_USE=6,SEED=7,
  POP_REQ=8,POP_WAIT=9,POP_USE=10,N_REQ=11,N_WAIT=12,N_USE=13,MARK=14,N_NEXT=15,
  FILTER=16,ADVANCE=17,SORT_X=18,SORT_Y=19,VALIDATE=20,FINISH=21,C_NREQ=22;
 reg [4:0] state;
 reg visited[0:16383];reg vq;reg [13:0] va;
 reg [14:0] stack[0:16383];reg [14:0] sq;reg [13:0] sp;
 reg [13:0] scan,area;
 reg [7:0] sx,px,nx,xlo,xhi;
 reg [6:0] sy,py,ny,ylo,yhi;
 reg [2:0] direction;
 reg boundary,bad,dilated;
 // One physical table. Read-only aliases retain the original index widths.
 wire [7:0] l[0:21],r[0:21];wire [6:0] t[0:21],b[0:21];
 genvar bounds_index;
 generate for(bounds_index=0;bounds_index<22;bounds_index=bounds_index+1)begin: bounds_alias
  assign l[bounds_index]=lefts[bounds_index*8+:8];
  assign r[bounds_index]=rights[bounds_index*8+:8];
  assign t[bounds_index]=tops[bounds_index*7+:7];
  assign b[bounds_index]=bottoms[bounds_index*7+:7];
 end endgenerate
 reg [4:0] pass,index;
 reg [7:0] base_width,base_height,col_center;
 wire [7:0] width=xhi-xlo+1'b1,height=yhi-ylo+1'b1;
 wire [15:0] box_area=width*height;
 wire candidate=!boundary && width>=6 && width<=36 && height>=3 && height<=18 &&
    width>=height && {1'b0,width}<=(height*5) && {area,1'b0}>=box_area;
 wire [8:0] center={1'b0,l[index]}+{1'b0,r[index]};
 wire [8:0] next_center={1'b0,l[index+1'b1]}+{1'b0,r[index+1'b1]};
 wire [7:0] current_width=r[index]-l[index]+1'b1;
 wire [7:0] current_height=b[index]-t[index]+1'b1;
 wire column_start=index==0 || index==6 || index==11 || index==16;
 wire center_near=center[8:1]>=col_center ? center[8:1]-col_center<=6 : col_center-center[8:1]<=6;
 wire stack_write=state==SEED || (state==MARK && sp<14400);
 wire [13:0] stack_waddr=state==SEED ? 14'd0 : sp;
 wire [14:0] stack_wdata=state==SEED ? {sy,sx} : {ny,nx};
 always @(posedge clk)begin
  vq<=visited[va];
  if(state==C_USE && direction==4)visited[scan]<=dilated|pixel;
  else if(state==SEED)visited[scan]<=1;
  else if(state==MARK)visited[va]<=1;
  if(stack_write)stack[stack_waddr]<=stack_wdata;
  if(state==POP_REQ && sp!=0)sq<=stack[sp-1'b1];
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;address<=0;busy<=0;done<=0;found<=0;ambiguous<=0;lefts<=0;rights<=0;tops<=0;bottoms<=0;count<=0;
   va<=0;sp<=0;scan<=0;area<=0;sx<=0;sy<=0;px<=0;py<=0;nx<=0;ny<=0;xlo<=0;xhi<=0;ylo<=0;yhi<=0;
   direction<=0;boundary<=0;bad<=0;dilated<=0;pass<=0;index<=0;base_width<=0;base_height<=0;col_center<=0;
  end else begin
   done<=0;
   case(state)
    IDLE:if(start)begin busy<=1;found<=0;ambiguous<=0;count<=0;bad<=0;scan<=0;sx<=0;sy<=0;state<=C_REQ;end
    C_REQ:begin address<=scan;direction<=0;dilated<=0;state<=C_WAIT;end
    C_WAIT:state<=C_USE;
    C_USE:begin
     dilated<=dilated|pixel;
     if(direction!=4)begin direction<=direction+1'b1;state<=C_NREQ;end
     else if(scan==14399)begin scan<=0;sx<=0;sy<=0;state<=S_REQ;end
     else begin
      scan<=scan+1'b1;
      if(sx==159)begin sx<=0;sy<=sy+1'b1;end else sx<=sx+1'b1;
      state<=C_REQ;
     end
    end
    // Read the four direct neighbours. OR-ing them with the current mask
    // dilates one coarse cell and closes isolated one-cell breaks in tape edges.
    C_NREQ:begin
     case(direction)
      1:address<=sx==0 ? scan : scan-1'b1;
      2:address<=sx==159 ? scan : scan+1'b1;
      3:address<=sy==0 ? scan : scan-14'd160;
      default:address<=sy==89 ? scan : scan+14'd160;
     endcase
     state<=C_WAIT;
    end
    S_REQ:begin va<=scan;state<=S_WAIT;end
    S_WAIT:state<=S_USE;
    S_USE:if(vq)state<=ADVANCE;else begin
      sp<=1;area<=0;xlo<=sx;xhi<=sx;ylo<=sy;yhi<=sy;boundary<=0;state<=SEED;end
    SEED:state<=POP_REQ;
    POP_REQ:if(sp==0)state<=FILTER;else begin sp<=sp-1'b1;state<=POP_WAIT;end
    POP_WAIT:state<=POP_USE;
    POP_USE:begin
     px<=sq[7:0];py<=sq[14:8];area<=area+1'b1;direction<=0;
     if(sq[7:0]<xlo)xlo<=sq[7:0];if(sq[7:0]>xhi)xhi<=sq[7:0];
     if(sq[14:8]<ylo)ylo<=sq[14:8];if(sq[14:8]>yhi)yhi<=sq[14:8];
     if(sq[7:0]==0 || sq[7:0]==159 || sq[14:8]==0 || sq[14:8]==89)boundary<=1;
     state<=N_REQ;
    end
    N_REQ:begin
     state<=N_WAIT;
     case(direction)
      0:if(px==0)state<=N_NEXT;else begin nx<=px-1'b1;ny<=py;va<=py*160+px-1'b1;end
      1:if(px==159)state<=N_NEXT;else begin nx<=px+1'b1;ny<=py;va<=py*160+px+1'b1;end
      2:if(py==0)state<=N_NEXT;else begin nx<=px;ny<=py-1'b1;va<=py*160+px-160;end
      3:if(py==89)state<=N_NEXT;else begin nx<=px;ny<=py+1'b1;va<=py*160+px+160;end
     endcase
    end
    N_WAIT:state<=N_USE;
    N_USE:if(vq)state<=N_NEXT;else state<=MARK;
    MARK:begin if(sp<14400)sp<=sp+1'b1;else bad<=1;state<=N_NEXT;end
    N_NEXT:if(direction==3)state<=POP_REQ;else begin direction<=direction+1'b1;state<=N_REQ;end
    FILTER:begin
     if(candidate)begin
      if(count<22)begin
       lefts[count*8+:8]<=xlo;rights[count*8+:8]<=xhi;
       tops[count*7+:7]<=ylo;bottoms[count*7+:7]<=yhi;
       count<=count+1'b1;
      end
      else begin count<=23;bad<=1;end
     end
     state<=ADVANCE;
    end
    ADVANCE:if(scan==14399)begin
     pass<=0;index<=0;
     if(count==22 && !bad)state<=SORT_X;else state<=FINISH;
    end else begin
     scan<=scan+1'b1;if(sx==159)begin sx<=0;sy<=sy+1'b1;end else sx<=sx+1'b1;state<=S_REQ;
    end
    SORT_X:begin
     if(center>next_center)begin
      lefts[(index)*8+:8]<=l[index+1'b1];rights[(index)*8+:8]<=r[index+1'b1];tops[(index)*7+:7]<=t[index+1'b1];bottoms[(index)*7+:7]<=b[index+1'b1];
      lefts[(index+1'b1)*8+:8]<=l[index];rights[(index+1'b1)*8+:8]<=r[index];tops[(index+1'b1)*7+:7]<=t[index];bottoms[(index+1'b1)*7+:7]<=b[index];
     end
     if(index==20)begin index<=0;if(pass==20)begin pass<=0;state<=SORT_Y;end else pass<=pass+1'b1;end else index<=index+1'b1;
    end
    SORT_Y:begin
     if(index!=5 && index!=10 && index!=15 && t[index]>t[index+1'b1])begin
      lefts[(index)*8+:8]<=l[index+1'b1];rights[(index)*8+:8]<=r[index+1'b1];tops[(index)*7+:7]<=t[index+1'b1];bottoms[(index)*7+:7]<=b[index+1'b1];
      lefts[(index+1'b1)*8+:8]<=l[index];rights[(index+1'b1)*8+:8]<=r[index];tops[(index+1'b1)*7+:7]<=t[index];bottoms[(index+1'b1)*7+:7]<=b[index];
     end
     if(index==20)begin index<=0;if(pass==5)begin base_width<=r[0]-l[0]+1'b1;base_height<=b[0]-t[0]+1'b1;state<=VALIDATE;end else pass<=pass+1'b1;end else index<=index+1'b1;
    end
    VALIDATE:begin
     if(column_start)begin
      if(index!=0 && lefts[(index)*8+:8]<=r[index-1'b1])bad<=1;
      col_center<=center[8:1];
     end else if(!center_near || tops[(index)*7+:7]<=b[index-1'b1])bad<=1;
     if(current_width*2<base_width || current_width>base_width*2 || current_height*2<base_height || current_height>base_height*2)bad<=1;
     if(index==21)state<=FINISH;else index<=index+1'b1;
    end
    FINISH:begin found<=count==22 && !bad;ambiguous<=bad;done<=1;busy<=0;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule

module parking_core #(parameter integer WARM_FRAMES=120,parameter integer LEARN_FRAMES=32)(
 input clk,rst_n,ready,frame_start,cell_valid,ink,confirm,recalibrate,
 input tracking,track_learning,track_ready,weak_light,
 input [7:0] cx,level,dark_threshold,input [6:0] cy,
 output reg [2:0] state,output reg [21:0] occupied,
 output reg [175:0] lefts,rights,output reg [153:0] tops,bottoms,
 output [4:0] free_count,used_count,output reg [4:0] total_count,detected_count,output reg search_ambiguous,output reg search_failed,
 output diff_valid,output reg [13:0] diff_address,output diff_hit,output diff_static
);
 localparam WARM=0,SCAN=1,ANALYZE=2,PREVIEW=3,LEARN=4,RUN=5;
 reg mask[0:14399];
 (* ram_style="bram" *) reg [7:0] background[0:14399];
 wire [13:0] cell_address=cy*160+cx;
 wire [13:0] read_address;reg mask_q;
 reg finder_start;wire finder_busy,finder_done,finder_found,finder_ambiguous;
 wire [175:0] fl,fr;wire [153:0] ft,fb;wire [4:0] finder_count;
 reg [15:0] warm_count,cell_count;
 reg [5:0] learn_count;
 reg confirm_pending;
 reg result_pending,result_valid;
 reg [1:0] candidate_count;
 function near8;input [7:0] a,b;begin near8=(a>=b?a-b:b-a)<=2;end endfunction
 wire [21:0] same_bay,selected;
 wire same_candidate=&same_bay;
 genvar bay;
 generate for(bay=0;bay<22;bay=bay+1)begin: bay_regions
  wire [7:0] xl=lefts[bay*8+:8],xr=rights[bay*8+:8];
  wire [6:0] yt=tops[bay*7+:7],yb=bottoms[bay*7+:7];
  assign selected[bay]=cy>yt && cy<yb && cx>xl && cx<xr;
  assign same_bay[bay]=near8(xl,fl[bay*8+:8]) && near8(xr,fr[bay*8+:8]) &&
   near8({1'b0,yt},{1'b0,ft[bay*7+:7]}) && near8({1'b0,yb},{1'b0,fb[bay*7+:7]});
 end endgenerate
 reg [7:0] bg_q,level_q;reg compare_valid;reg [21:0] selected_q;
 // Once geometry is found its mask RAM is free. Reuse it for the learned
 // binary empty scene instead of allocating another frame buffer.
 reg [7:0] tracking_threshold;
 wire binary_reference=tracking && state>=PREVIEW;
 wire background_white=!mask_q && {1'b0,bg_q}>={1'b0,tracking_threshold}+9'd8;
 // Track the common illumination shift with a fractional IIR accumulator.
 // A lamp changes nearly every empty bay in the same direction; a vehicle
 // changes only one bay.  Occupancy therefore uses the residual after this
 // common component is removed.  Already occupied bays do not train it.
 reg signed [17:0] illumination_acc;
 wire signed [9:0] signed_delta=$signed({1'b0,level_q})-$signed({1'b0,bg_q});
 wire signed [9:0] illumination_offset=illumination_acc>>>8;
 wire signed [10:0] corrected_level=$signed({3'b000,level_q})-
                                    $signed({illumination_offset[9],illumination_offset});
 wire current_black=corrected_level<$signed({3'b000,tracking_threshold})-11'sd8;
 wire signed [10:0] residual=$signed({signed_delta[9],signed_delta})-
                              $signed({illumination_offset[9],illumination_offset});
 wire [10:0] corrected_difference=residual<0 ? -residual : residual;
 wire illumination_sample=compare_valid && |(selected_q & ~occupied) &&
     (!tracking || corrected_difference<64);
 wire [21:0] samples_present;
 genvar j;
  generate for(j=0;j<22;j=j+1)begin: occupancy
   reg [9:0] changes,strong_changes,samples;
  reg [1:0] on_count,off_count;
  assign samples_present[j]=samples!=0;
  always @(posedge clk or negedge rst_n)begin
    if(!rst_n)begin changes<=0;strong_changes<=0;samples<=0;on_count<=0;off_count<=0;occupied[j]<=0;end
    else if(!ready || recalibrate || tracking)begin changes<=0;strong_changes<=0;samples<=0;on_count<=0;off_count<=0;occupied[j]<=0;end
   else begin
    if(cell_valid && selected[j])samples<=samples+1'b1;
     if(compare_valid && selected_q[j] && corrected_difference>=14)changes<=changes+1'b1;
     if(compare_valid && selected_q[j] && corrected_difference>=40)strong_changes<=strong_changes+1'b1;
     if(frame_start)begin
      changes<=0;strong_changes<=0;samples<=0;
     if(state!=RUN)begin occupied[j]<=0;on_count<=0;off_count<=0;end
     else if(cell_count==14400 && (&samples_present))begin
       if({changes,3'b0}>=samples || {strong_changes,4'b0}>=samples || strong_changes>=2)begin
        off_count<=0;if(on_count==2)begin occupied[j]<=1;on_count<=0;end else on_count<=on_count+1'b1;
       end else if({changes,4'b0}<=samples && {strong_changes,5'b0}<=samples)begin
       on_count<=0;if(off_count==2)begin occupied[j]<=0;off_count<=0;end else off_count<=off_count+1'b1;
      end else begin on_count<=0;off_count<=0;end
     end
    end
   end
  end
 end endgenerate
 assign diff_valid=compare_valid && (state==RUN || tracking);
  // The learned black tape is always excluded. Freeze the empty-scene
  // threshold so adding a car cannot redefine which old cells are black.
  assign diff_static=tracking && mask_q;
  assign diff_hit=tracking ? (background_white && current_black && residual<=(weak_light ? -11'sd24 : -11'sd20)) : ((|selected_q) && corrected_difference>=14);
 always @(posedge clk)if(cell_valid)diff_address<=cell_address;
 wire [4:0] sum0,sum1,sum2,sum3;
 assign sum0={4'b0,occupied[0]}+occupied[1]+occupied[2]+occupied[3]+occupied[4]+occupied[5];
 assign sum1={4'b0,occupied[6]}+occupied[7]+occupied[8]+occupied[9]+occupied[10]+occupied[11];
 assign sum2={4'b0,occupied[12]}+occupied[13]+occupied[14]+occupied[15]+occupied[16]+occupied[17];
 assign sum3={4'b0,occupied[18]}+occupied[19]+occupied[20]+occupied[21];
 assign used_count=(sum0+sum1)+(sum2+sum3);
 assign free_count=total_count-used_count;
 parking_finder finder(clk,rst_n && !recalibrate,finder_start,read_address,mask_q,
  finder_busy,finder_done,finder_found,finder_ambiguous,fl,fr,ft,fb,finder_count);
 always @(posedge clk) begin
  mask_q<=mask[binary_reference ? cell_address : read_address];
  if(cell_valid && rst_n && ready) begin
   if(tracking && track_learning) mask[cell_address]<=level<tracking_threshold;
   else if(state==SCAN) mask[cell_address]<=ink;
   if((!tracking && state==LEARN) || (tracking && track_learning)) background[cell_address]<=level;
   bg_q<=background[cell_address];
  end
 end
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n) begin
   state<=WARM;total_count<=0;detected_count<=0;lefts<=0;rights<=0;tops<=0;bottoms<=0;
   warm_count<=0;cell_count<=0;learn_count<=0;confirm_pending<=0;finder_start<=0;search_ambiguous<=0;
    result_pending<=0;result_valid<=0;candidate_count<=0;search_failed<=0;illumination_acc<=0;
   compare_valid<=0;selected_q<=0;level_q<=0;tracking_threshold<=128;

  end else if(!ready || recalibrate) begin
   state<=recalibrate && ready ? SCAN : WARM;total_count<=0;detected_count<=0;warm_count<=0;cell_count<=0;learn_count<=0;confirm_pending<=0;finder_start<=0;
   compare_valid<=0;search_ambiguous<=0;
    result_pending<=0;result_valid<=0;candidate_count<=0;search_failed<=0;illumination_acc<=0;

  end else begin
    // An interrupted empty-scene capture must not become a valid parking
    // reference. Returning before calibration completes requires normal K1.
    if(tracking && state>=PREVIEW) state<=track_ready ? RUN : PREVIEW;
    if(tracking && !track_learning && !track_ready) tracking_threshold<=dark_threshold;
    finder_start<=0;compare_valid<=cell_valid && (state==RUN || (tracking && state>=PREVIEW));
    if(state!=RUN || (tracking && !track_ready)) illumination_acc<=0;
    else if(illumination_sample)
     illumination_acc<=illumination_acc+signed_delta-illumination_offset;
   if(confirm && !tracking && state==PREVIEW) confirm_pending<=1;
   if(cell_valid) begin
    level_q<=level;selected_q<=selected;
    if(cell_count<14401) cell_count<=cell_count+1'b1;
   end
   if(state==ANALYZE && finder_done) begin
    search_ambiguous<=finder_ambiguous;detected_count<=finder_count;
    search_failed<=!finder_found;
    lefts<=fl;rights<=fr;tops<=ft;bottoms<=fb;
    result_pending<=1;result_valid<=finder_found;
   end
   if(frame_start) begin
    cell_count<=0;compare_valid<=0;
    if(!tracking || state<PREVIEW) case(state)
     WARM:if(cell_count==14400) begin
      if(warm_count>=WARM_FRAMES-1) begin state<=SCAN;warm_count<=0;end else warm_count<=warm_count+1'b1;
     end else warm_count<=0;
     SCAN:if(cell_count==14400) begin state<=ANALYZE;finder_start<=1;end
     ANALYZE:if(result_pending) begin
      result_pending<=0;state<=SCAN;
      // A complete 6+5+5+6 result has already passed the strict geometry
      // checks in the finder.  Latch that first complete result so a later
      // exposure fluctuation cannot make K1 unavailable again.
      if(result_valid) begin
       candidate_count<=3;total_count<=finder_count;state<=PREVIEW;
      end else candidate_count<=0;
     end
     PREVIEW:if(confirm_pending || confirm) begin state<=LEARN;learn_count<=0;confirm_pending<=0;end
     // Capture an empty reference for several complete frames.  The final
     // frame becomes the reference, after camera exposure has had time to
     // settle; occupancy remains forced clear throughout this interval.
     LEARN:if(cell_count==14400 && (&samples_present)) begin
      if(learn_count>=LEARN_FRAMES-1) begin state<=RUN;learn_count<=0;end
      else learn_count<=learn_count+1'b1;
     end else if(cell_count!=14400) learn_count<=0;
     RUN:if(cell_count!=14400 || !(&samples_present))begin state<=PREVIEW;confirm_pending<=0;end
     default:state<=WARM;
    endcase
   end
  end
 end
endmodule

module anlogic_logo_rom(input wire I_clk,input wire [14:0] I_addr,output wire [24:0] O_pixel);
    // Lossless cropped index ROM. Nonzero pixels lie in [4096,20480).
    // The low 14 bits are one-to-one over that interval; other addresses
    // retain transparency. No color, alpha, size or throughput is changed.
    wire [9:0] raw_index;
    reg index_valid;
    always @(posedge I_clk) index_valid<=I_addr>=15'd4096 && I_addr<15'd20480;
    wire [9:0] palette_address=index_valid ? raw_index:10'd0;
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(10),.ADDR_WIDTH_A(14),.DATA_DEPTH_A(16384),
        .DATA_WIDTH_B(10),.ADDR_WIDTH_B(14),.DATA_DEPTH_B(16384),
        .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .IMPLEMENT("20K(FAST)"),.INIT_FILE("D:/Desktop/FPGA/code/111/baseline/touch_merge_20261027/rtl/logo_data/index.dath"),.FORCE_KEEP("ON")
    ) index_memory(.dia(10'd0),.addra(I_addr[13:0]),.cea(1'b1),.ocea(1'b0),
        .clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(raw_index),
        .dib(10'd0),.addrb(14'd0),.ceb(1'b0),.oceb(1'b0),.clkb(I_clk),
        .web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(),
        .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr());
    // A second synchronous ROM replaces the caller's old alignment stage.
    PH1P_LOGIC_ERAM #(
        .DATA_WIDTH_A(25),.ADDR_WIDTH_A(10),.DATA_DEPTH_A(1024),
        .DATA_WIDTH_B(25),.ADDR_WIDTH_B(10),.DATA_DEPTH_B(1024),
        .MODE("SP"),.REGMODE_A("NOREG"),.REGMODE_B("NOREG"),
        .IMPLEMENT("20K(FAST)"),.INIT_FILE("D:/Desktop/FPGA/code/111/baseline/touch_merge_20261027/rtl/logo_data/palette.dath"),.FORCE_KEEP("ON")
    ) palette_memory(.dia(25'd0),.addra(palette_address),.cea(1'b1),.ocea(1'b0),
        .clka(I_clk),.wea(1'b0),.rsta(1'b0),.bea(1'b0),.doa(O_pixel),
        .dib(25'd0),.addrb(10'd0),.ceb(1'b0),.oceb(1'b0),.clkb(I_clk),
        .web(1'b0),.rstb(1'b0),.beb(1'b0),.dob(),
        .ecc_sbiterrinj(1'b0),.ecc_dbiterrinj(1'b0),.ecc_sbiterr(),.ecc_dbiterr());
endmodule




`timescale 1ns/1ps
// Pack four HDMI pixels for the shared, noise-gated Sobel module.
// Six clocks align the live RGB, timing and serialized edge result.
module parking_edge_stream(input clk,rst_n,input [23:0] rgb,
 input de,user,last,vs,hs,strong_light,output [23:0] rgb_out,
 output de_out,user_out,last_out,vs_out,hs_out,output reg edge_out);
 reg [1:0] phase;
 reg [71:0] pack_rgb;
 reg pack_user;
 reg [95:0] group_rgb;
 reg group_valid,group_user,group_last;
 wire [95:0] edge_rgb;
 wire edge_valid,edge_user,edge_last,unused_ready;
 reg [2:0] edge_tail;
 reg [28:0] delay_pipe[0:5];
 integer i;
 assign {vs_out,hs_out,de_out,user_out,last_out,rgb_out}=delay_pipe[5];
 edge_detect_96 #(.IMG_WIDTH(1280),.IMG_HEIGHT(720),.EDGE_TH(11'd80),.WEAK_EDGE_TH(11'd112)) original_edge(
  .I_clk(clk),.I_rst_n(rst_n),.I_strong_light(strong_light),.I_tlast(group_last),.I_tuser(group_user),
  .I_tdata(group_rgb),.I_tvalid(group_valid),.I_tready(1'b1),
  .O_tlast(edge_last),.O_tuser(edge_user),.O_tdata(edge_rgb),.O_tvalid(edge_valid),.O_tready(unused_ready));
 always @(posedge clk or negedge rst_n) begin
  if(!rst_n)begin
   phase<=0;pack_rgb<=0;pack_user<=0;group_rgb<=0;group_valid<=0;group_user<=0;group_last<=0;
   edge_tail<=0;edge_out<=0;
   for(i=0;i<6;i=i+1)delay_pipe[i]<=0;
  end else begin
   delay_pipe[0]<={vs,hs,de,user,last,rgb};
   for(i=1;i<6;i=i+1)delay_pipe[i]<=delay_pipe[i-1];
   group_valid<=0;group_user<=0;group_last<=0;
   if(de)begin
    pack_rgb<={pack_rgb[47:0],rgb};
    if(user || phase==0)pack_user<=user;
    if(phase==3 && !user)begin
     group_rgb<={pack_rgb,rgb};group_valid<=1;group_user<=pack_user;group_last<=last;
    end
    phase<=last ? 0 : user ? 1 : phase+1'b1;
   end
   if(edge_valid)begin edge_out<=edge_rgb[95];edge_tail<={edge_rgb[71],edge_rgb[47],edge_rgb[23]};end
   else begin edge_out<=edge_tail[2];edge_tail<={edge_tail[1:0],1'b0};end
  end
 end
endmodule

// Two complete 160x90 difference maps. Read the previous complete frame,
// never the bank currently being written. Unselected cells are written zero.
// Double-buffer the exact 160x90 mask consumed by the calibration finder.
// This is deliberately separate from the Sobel demonstration view.
module parking_mask_view(input clk,rst_n,frame_start,
 input sample_valid,input [13:0] sample_address,input sample_hit,
 input [13:0] display_address,output reg pixel,output reg ready);
 (* ram_style="block" *) reg maps[0:32767];
 reg write_bank;
 reg [14:0] samples;
 always @(posedge clk)begin
  pixel<=maps[{!write_bank,display_address}];
  if(rst_n && sample_valid)maps[{write_bank,sample_address}]<=sample_hit;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin write_bank<=0;samples<=0;ready<=0;end
  else if(frame_start)begin
   samples<=0;
   if(samples==14400)begin write_bank<=!write_bank;ready<=1;end
  end else if(sample_valid && samples<14401)samples<=samples+1'b1;
 end
endmodule

module parking_diff_view(input clk,rst_n,run,frame_start,
 input sample_valid,input [13:0] sample_address,input sample_hit,
 input [13:0] display_address,output reg pixel,output reg ready);
 (* ram_style="block" *) reg maps[0:32767];
 reg write_bank;
 reg [14:0] samples;
 always @(posedge clk)begin
  pixel<=maps[{!write_bank,display_address}];
  if(rst_n && run && sample_valid)maps[{write_bank,sample_address}]<=sample_hit;
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin write_bank<=0;samples<=0;ready<=0;end
  else if(!run)begin write_bank<=0;samples<=0;ready<=0;end
  else if(frame_start)begin
   samples<=0;
   if(samples==14400)begin write_bank<=!write_bank;ready<=1;end
   else ready<=0;
  end else if(sample_valid && samples<14401)samples<=samples+1'b1;
 end
endmodule

module edge_detect_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_TH    = 11'd80,
    parameter WEAK_EDGE_TH = 11'd112
)(
    input               I_clk,
    input               I_rst_n,
    input               I_strong_light,
    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    input               I_tready,
    output reg          O_tlast,
    output reg          O_tuser,
    output reg [95:0]   O_tdata,
    output reg          O_tvalid,
    output              O_tready
);

assign O_tready = I_tready;

wire [23:0] pix0 = I_tdata[95:72];
wire [23:0] pix1 = I_tdata[71:48];
wire [23:0] pix2 = I_tdata[47:24];
wire [23:0] pix3 = I_tdata[23:0];

// Low-cost luminance approximation: R/4 + G/2 + B/4.  Its maximum is
// 253, so it cannot overflow an 8-bit result and requires no divider.
wire [7:0] gray0 = {2'b0,pix0[23:18]} + {1'b0,pix0[15:9]} + {2'b0,pix0[7:2]};
wire [7:0] gray1 = {2'b0,pix1[23:18]} + {1'b0,pix1[15:9]} + {2'b0,pix1[7:2]};
wire [7:0] gray2 = {2'b0,pix2[23:18]} + {1'b0,pix2[15:9]} + {2'b0,pix2[7:2]};
wire [7:0] gray3 = {2'b0,pix3[23:18]} + {1'b0,pix3[15:9]} + {2'b0,pix3[7:2]};

localparam IMG_WIDTH_GRP = (IMG_WIDTH >> 2);

// Use two grayscale samples per 4-pixel input beat.  Each sample represents
// a neighbouring pair of pixels, providing 640 horizontal edge samples while
// keeping the two line buffers small enough for this device.
wire [8:0] pair_sum0 = {1'b0,gray0} + {1'b0,gray1};
wire [8:0] pair_sum1 = {1'b0,gray2} + {1'b0,gray3};
wire [7:0] pair0 = pair_sum0[8:1];
wire [7:0] pair1 = pair_sum1[8:1];

(* ram_style = "dram" *) reg [15:0] line1_mem [0:IMG_WIDTH_GRP-1];
(* ram_style = "dram" *) reg [15:0] line2_mem [0:IMG_WIDTH_GRP-1];

reg [10:0] x_grp;
reg [10:0] y_pos;

wire [15:0] top_word = line2_mem[x_grp];
wire [15:0] mid_word = line1_mem[x_grp];
wire [15:0] bot_word = {pair0,pair1};

wire [7:0] top0 = top_word[15:8];
wire [7:0] top1 = top_word[7:0];
wire [7:0] mid0 = mid_word[15:8];
wire [7:0] mid1 = mid_word[7:0];

reg [7:0] top_prev2, top_prev1;
reg [7:0] mid_prev2, mid_prev1;
reg [7:0] bot_prev2, bot_prev1;

function [13:0] sobel_mag;
    input [7:0] a00, a01, a02;
    input [7:0] a10, a11, a12;
    input [7:0] a20, a21, a22;
    reg signed [12:0] gx_f;
    reg signed [12:0] gy_f;
    reg [12:0] abs_gx_f;
    reg [12:0] abs_gy_f;
    begin
        gx_f = -$signed({1'b0,a00}) + $signed({1'b0,a02})
             - ($signed({1'b0,a10}) <<< 1) + ($signed({1'b0,a12}) <<< 1)
             - $signed({1'b0,a20}) + $signed({1'b0,a22});
        gy_f =  $signed({1'b0,a00}) + ($signed({1'b0,a01}) <<< 1) + $signed({1'b0,a02})
             - $signed({1'b0,a20}) - ($signed({1'b0,a21}) <<< 1) - $signed({1'b0,a22});
        abs_gx_f = gx_f[12] ? (~gx_f + 1'b1) : gx_f;
        abs_gy_f = gy_f[12] ? (~gy_f + 1'b1) : gy_f;
        sobel_mag = abs_gx_f + abs_gy_f;
    end
endfunction

// Two Sobel windows are evaluated per input beat. Each result is duplicated
// to its corresponding pair of output pixels.
wire [13:0] mag0 = sobel_mag(top_prev2,top_prev1,top0,
                             mid_prev2,mid_prev1,mid0,
                             bot_prev2,bot_prev1,pair0);
wire [13:0] mag1 = sobel_mag(top_prev1,top0,top1,
                             mid_prev1,mid0,mid1,
                             bot_prev1,pair0,pair1);

wire window_valid = (y_pos >= 11'd2) && (x_grp != 11'd0) && !I_tuser;
// Keep a strong response only when the adjacent window supplies support.
// Higher weak-light threshold rejects amplified sensor noise. No extra rows,
// frame buffers or output cycles are introduced.
wire [10:0] edge_threshold=I_strong_light ? EDGE_TH : WEAK_EDGE_TH;
wire [10:0] support_threshold=edge_threshold>>1;
wire edge0 = window_valid && (mag0 >= edge_threshold) && (mag1 >= support_threshold);
wire edge1 = window_valid && (mag1 >= edge_threshold) && (mag0 >= support_threshold);

wire [23:0] out0 = edge0 ? 24'hffffff : 24'h000000;
wire [23:0] out1 = edge1 ? 24'hffffff : 24'h000000;

always @(posedge I_clk or negedge I_rst_n) begin
    if(!I_rst_n) begin
        O_tlast  <= 1'b0;
        O_tuser  <= 1'b0;
        O_tdata  <= 96'd0;
        O_tvalid <= 1'b0;
        x_grp    <= 11'd0;
        y_pos    <= 11'd0;
        top_prev2 <= 8'd0; top_prev1 <= 8'd0;
        mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
        bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
        // Do not clear line memories here.  A reset loop turns the 10-Kbit
        // buffers into slice registers on this device.  The first two rows
        // are masked by window_valid and naturally initialize both buffers.
    end else begin
        O_tlast  <= I_tlast;
        O_tuser  <= I_tuser;
        O_tvalid <= I_tvalid;

        if(I_tvalid) begin
            O_tdata <= {out0,out0,out1,out1};

            line2_mem[x_grp] <= mid_word;
            line1_mem[x_grp] <= bot_word;

            top_prev2 <= top0; top_prev1 <= top1;
            mid_prev2 <= mid0; mid_prev1 <= mid1;
            bot_prev2 <= pair0; bot_prev1 <= pair1;

            if(I_tuser) begin
                x_grp <= 11'd1;
                y_pos <= 11'd0;
            end else if(x_grp == IMG_WIDTH_GRP - 1) begin
                x_grp <= 11'd0;
                if(y_pos == IMG_HEIGHT - 1)
                    y_pos <= 11'd0;
                else
                    y_pos <= y_pos + 1'b1;
                top_prev2 <= 8'd0; top_prev1 <= 8'd0;
                mid_prev2 <= 8'd0; mid_prev1 <= 8'd0;
                bot_prev2 <= 8'd0; bot_prev1 <= 8'd0;
            end else begin
                x_grp <= x_grp + 1'b1;
            end
        end else begin
            O_tdata <= 96'd0;
        end
    end
end

endmodule

module parking_osd22(input clk,rst_n,input [11:0] x,y,input [2:0] state,
 input [4:0] total,used,free_slots,detected,input ambiguous,output reg hit,output reg [23:0] rgb);
 wire area=x>=1040 && x<1264 && y>=4 && y<100;
 wire [11:0] dx=x-1040,dy=y-4;
 wire [2:0] col=dx[7:5];wire [1:0] row=dy[6:5];
 reg [5:0] character;reg [4:0] number;reg [23:0] color;
 reg area_d;reg [9:0] font_address;reg [3:0] bit_column;reg [23:0] color_d;
 function [15:0] font_row;input [9:0] address;begin case(address)
 19:font_row=16'h1800;
 20:font_row=16'h2400;
 21:font_row=16'h4200;
 22:font_row=16'h4200;
 23:font_row=16'h4200;
 24:font_row=16'h4200;
 25:font_row=16'h4200;
 26:font_row=16'h4200;
 27:font_row=16'h4200;
 28:font_row=16'h2400;
 29:font_row=16'h1800;
 35:font_row=16'h0800;
 36:font_row=16'h3800;
 37:font_row=16'h0800;
 38:font_row=16'h0800;
 39:font_row=16'h0800;
 40:font_row=16'h0800;
 41:font_row=16'h0800;
 42:font_row=16'h0800;
 43:font_row=16'h0800;
 44:font_row=16'h0800;
 45:font_row=16'h3e00;
 51:font_row=16'h3c00;
 52:font_row=16'h4200;
 53:font_row=16'h4200;
 54:font_row=16'h4200;
 55:font_row=16'h0200;
 56:font_row=16'h0400;
 57:font_row=16'h0800;
 58:font_row=16'h1000;
 59:font_row=16'h2000;
 60:font_row=16'h4200;
 61:font_row=16'h7e00;
 67:font_row=16'h3c00;
 68:font_row=16'h4200;
 69:font_row=16'h4200;
 70:font_row=16'h0200;
 71:font_row=16'h0400;
 72:font_row=16'h1800;
 73:font_row=16'h0400;
 74:font_row=16'h0200;
 75:font_row=16'h4200;
 76:font_row=16'h4200;
 77:font_row=16'h3c00;
 83:font_row=16'h0400;
 84:font_row=16'h0c00;
 85:font_row=16'h0c00;
 86:font_row=16'h1400;
 87:font_row=16'h2400;
 88:font_row=16'h2400;
 89:font_row=16'h4400;
 90:font_row=16'h7f00;
 91:font_row=16'h0400;
 92:font_row=16'h0400;
 93:font_row=16'h1f00;
 99:font_row=16'h7e00;
 100:font_row=16'h4000;
 101:font_row=16'h4000;
 102:font_row=16'h4000;
 103:font_row=16'h7800;
 104:font_row=16'h4400;
 105:font_row=16'h0200;
 106:font_row=16'h0200;
 107:font_row=16'h4200;
 108:font_row=16'h4400;
 109:font_row=16'h3800;
 115:font_row=16'h1800;
 116:font_row=16'h2400;
 117:font_row=16'h4000;
 118:font_row=16'h4000;
 119:font_row=16'h5c00;
 120:font_row=16'h6200;
 121:font_row=16'h4200;
 122:font_row=16'h4200;
 123:font_row=16'h4200;
 124:font_row=16'h2200;
 125:font_row=16'h1c00;
 131:font_row=16'h7e00;
 132:font_row=16'h4200;
 133:font_row=16'h0400;
 134:font_row=16'h0400;
 135:font_row=16'h0800;
 136:font_row=16'h0800;
 137:font_row=16'h1000;
 138:font_row=16'h1000;
 139:font_row=16'h1000;
 140:font_row=16'h1000;
 141:font_row=16'h1000;
 147:font_row=16'h3c00;
 148:font_row=16'h4200;
 149:font_row=16'h4200;
 150:font_row=16'h4200;
 151:font_row=16'h2400;
 152:font_row=16'h1800;
 153:font_row=16'h2400;
 154:font_row=16'h4200;
 155:font_row=16'h4200;
 156:font_row=16'h4200;
 157:font_row=16'h3c00;
 163:font_row=16'h3800;
 164:font_row=16'h4400;
 165:font_row=16'h4200;
 166:font_row=16'h4200;
 167:font_row=16'h4200;
 168:font_row=16'h4600;
 169:font_row=16'h3a00;
 170:font_row=16'h0200;
 171:font_row=16'h0200;
 172:font_row=16'h2400;
 173:font_row=16'h1800;
 185:font_row=16'h3000;
 186:font_row=16'h3000;
 188:font_row=16'h3000;
 189:font_row=16'h3000;
 192:font_row=16'h1010;
 193:font_row=16'h0820;
 194:font_row=16'h0440;
 196:font_row=16'h1ff0;
 197:font_row=16'h1010;
 198:font_row=16'h1010;
 199:font_row=16'h1010;
 200:font_row=16'h1ff0;
 201:font_row=16'h1010;
 202:font_row=16'h0100;
 203:font_row=16'h0884;
 204:font_row=16'h4892;
 205:font_row=16'h4812;
 206:font_row=16'h87f0;
 208:font_row=16'h0040;
 209:font_row=16'h2040;
 210:font_row=16'h1040;
 211:font_row=16'h1040;
 212:font_row=16'h0040;
 213:font_row=16'h0040;
 214:font_row=16'hf7fe;
 215:font_row=16'h1040;
 216:font_row=16'h1040;
 217:font_row=16'h1040;
 218:font_row=16'h1040;
 219:font_row=16'h1040;
 220:font_row=16'h1440;
 221:font_row=16'h1840;
 222:font_row=16'h1040;
 223:font_row=16'h0040;
 224:font_row=16'h0200;
 225:font_row=16'h0200;
 226:font_row=16'h0200;
 227:font_row=16'h7ffc;
 228:font_row=16'h0400;
 229:font_row=16'h0900;
 230:font_row=16'h1100;
 231:font_row=16'h2100;
 232:font_row=16'h3ff8;
 233:font_row=16'h0100;
 234:font_row=16'h0100;
 235:font_row=16'hfffe;
 236:font_row=16'h0100;
 237:font_row=16'h0100;
 238:font_row=16'h0100;
 239:font_row=16'h0100;
 240:font_row=16'h0880;
 241:font_row=16'h0840;
 242:font_row=16'h0840;
 243:font_row=16'h1000;
 244:font_row=16'h17fc;
 245:font_row=16'h3000;
 246:font_row=16'h3008;
 247:font_row=16'h5208;
 248:font_row=16'h9208;
 249:font_row=16'h1110;
 250:font_row=16'h1110;
 251:font_row=16'h1110;
 252:font_row=16'h1120;
 253:font_row=16'h1020;
 254:font_row=16'h1ffe;
 255:font_row=16'h1000;
 256:font_row=16'h0100;
 257:font_row=16'h0100;
 258:font_row=16'h0100;
 259:font_row=16'h01fe;
 260:font_row=16'h0100;
 261:font_row=16'h0100;
 262:font_row=16'h0100;
 263:font_row=16'h0100;
 264:font_row=16'h3ff8;
 265:font_row=16'h2008;
 266:font_row=16'h2008;
 267:font_row=16'h2008;
 268:font_row=16'h2008;
 269:font_row=16'h2008;
 270:font_row=16'h3ff8;
 271:font_row=16'h2008;
 273:font_row=16'h3ff8;
 274:font_row=16'h2108;
 275:font_row=16'h2108;
 276:font_row=16'h2108;
 277:font_row=16'h3ff8;
 278:font_row=16'h2108;
 279:font_row=16'h2108;
 280:font_row=16'h2108;
 281:font_row=16'h3ff8;
 282:font_row=16'h2108;
 283:font_row=16'h2108;
 284:font_row=16'h2108;
 285:font_row=16'h4108;
 286:font_row=16'h4128;
 287:font_row=16'h8010;
 288:font_row=16'h0200;
 289:font_row=16'h0100;
 290:font_row=16'h7ffe;
 291:font_row=16'h4002;
 292:font_row=16'h8824;
 293:font_row=16'h1010;
 294:font_row=16'h2008;
 296:font_row=16'h1ff0;
 297:font_row=16'h0100;
 298:font_row=16'h0100;
 299:font_row=16'h0100;
 300:font_row=16'h0100;
 301:font_row=16'h0100;
 302:font_row=16'h7ffc;
 304:font_row=16'h2000;
 305:font_row=16'h17fc;
 306:font_row=16'h0004;
 307:font_row=16'h4104;
 308:font_row=16'h4104;
 309:font_row=16'h4104;
 310:font_row=16'h5ff4;
 311:font_row=16'h4104;
 312:font_row=16'h4384;
 313:font_row=16'h4544;
 314:font_row=16'h4924;
 315:font_row=16'h5114;
 316:font_row=16'h4104;
 317:font_row=16'h4104;
 318:font_row=16'h4014;
 319:font_row=16'h4008;
 321:font_row=16'h3ff0;
 322:font_row=16'h0010;
 323:font_row=16'h0010;
 324:font_row=16'h0010;
 325:font_row=16'h2010;
 326:font_row=16'h2010;
 327:font_row=16'h3ff0;
 328:font_row=16'h2000;
 329:font_row=16'h2000;
 330:font_row=16'h2000;
 331:font_row=16'h2004;
 332:font_row=16'h2004;
 333:font_row=16'h2004;
 334:font_row=16'h1ffc;
 337:font_row=16'h2000;
 338:font_row=16'h11fc;
 339:font_row=16'h1104;
 340:font_row=16'h0104;
 341:font_row=16'h0104;
 342:font_row=16'hf104;
 343:font_row=16'h1104;
 344:font_row=16'h11fc;
 345:font_row=16'h1104;
 346:font_row=16'h1000;
 347:font_row=16'h1490;
 348:font_row=16'h1888;
 349:font_row=16'h1104;
 350:font_row=16'h0202;
 351:font_row=16'h0402;
 352:font_row=16'h0004;
 353:font_row=16'h7f04;
 354:font_row=16'h4104;
 355:font_row=16'h4124;
 356:font_row=16'h4124;
 357:font_row=16'h7f24;
 358:font_row=16'h1024;
 359:font_row=16'h1024;
 360:font_row=16'hff24;
 361:font_row=16'h1124;
 362:font_row=16'h1124;
 363:font_row=16'h1124;
 364:font_row=16'h2104;
 365:font_row=16'h2104;
 366:font_row=16'h4a14;
 367:font_row=16'h8408;
 368:font_row=16'h2040;
 369:font_row=16'h3f7e;
 370:font_row=16'h4890;
 371:font_row=16'h8508;
 372:font_row=16'h0100;
 373:font_row=16'h3ff8;
 374:font_row=16'h0100;
 375:font_row=16'h0100;
 376:font_row=16'hfffe;
 378:font_row=16'h0020;
 379:font_row=16'h7ffc;
 380:font_row=16'h0820;
 381:font_row=16'h0420;
 382:font_row=16'h04a0;
 383:font_row=16'h0040;
 384:font_row=16'h0840;
 385:font_row=16'h0840;
 386:font_row=16'h1040;
 387:font_row=16'h23fc;
 388:font_row=16'h4840;
 389:font_row=16'h0840;
 390:font_row=16'h17fe;
 391:font_row=16'h3010;
 392:font_row=16'h5010;
 393:font_row=16'h97fe;
 394:font_row=16'h1010;
 395:font_row=16'h1210;
 396:font_row=16'h1110;
 397:font_row=16'h1110;
 398:font_row=16'h1050;
 399:font_row=16'h1020;
 401:font_row=16'hfffe;
 404:font_row=16'h4fe4;
 405:font_row=16'h4924;
 406:font_row=16'h4924;
 407:font_row=16'h4fe4;
 408:font_row=16'h4924;
 409:font_row=16'h4924;
 410:font_row=16'h4924;
 411:font_row=16'h4fe4;
 412:font_row=16'h4004;
 413:font_row=16'h4004;
 414:font_row=16'h7ffc;
 415:font_row=16'h0004;
 417:font_row=16'hfffe;
 418:font_row=16'h0200;
 419:font_row=16'h0200;
 420:font_row=16'h0400;
 421:font_row=16'h3ff8;
 422:font_row=16'h2448;
 423:font_row=16'h2448;
 424:font_row=16'h27c8;
 425:font_row=16'h2448;
 426:font_row=16'h2448;
 427:font_row=16'h27c8;
 428:font_row=16'h2448;
 429:font_row=16'h2448;
 430:font_row=16'h3ff8;
 431:font_row=16'h2008;
 433:font_row=16'h7ffc;
 434:font_row=16'h0100;
 435:font_row=16'h0100;
 436:font_row=16'h0100;
 437:font_row=16'h0100;
 438:font_row=16'h1100;
 439:font_row=16'h11f8;
 440:font_row=16'h1100;
 441:font_row=16'h1100;
 442:font_row=16'h1100;
 443:font_row=16'h1100;
 444:font_row=16'h1100;
 445:font_row=16'h1100;
 446:font_row=16'hfffe;
 448:font_row=16'h0200;
 449:font_row=16'h0200;
 450:font_row=16'h0400;
 451:font_row=16'hfffe;
 452:font_row=16'h0800;
 453:font_row=16'h0840;
 454:font_row=16'h1040;
 455:font_row=16'h3040;
 456:font_row=16'h57fc;
 457:font_row=16'h9040;
 458:font_row=16'h1040;
 459:font_row=16'h1040;
 460:font_row=16'h1040;
 461:font_row=16'h1040;
 462:font_row=16'h1ffe;
 463:font_row=16'h1000;
 464:font_row=16'h1000;
 465:font_row=16'h11fc;
 466:font_row=16'h1000;
 467:font_row=16'h1000;
 468:font_row=16'hfc00;
 469:font_row=16'h13fe;
 470:font_row=16'h3020;
 471:font_row=16'h3820;
 472:font_row=16'h5528;
 473:font_row=16'h5524;
 474:font_row=16'h9224;
 475:font_row=16'h1222;
 476:font_row=16'h1422;
 477:font_row=16'h1020;
 478:font_row=16'h10a0;
 479:font_row=16'h1040;
 480:font_row=16'h0200;
 481:font_row=16'h0100;
 482:font_row=16'h7ffe;
 483:font_row=16'h4002;
 484:font_row=16'h8004;
 486:font_row=16'h3ff8;
 487:font_row=16'h0100;
 488:font_row=16'h0100;
 489:font_row=16'h1100;
 490:font_row=16'h11f8;
 491:font_row=16'h1100;
 492:font_row=16'h1100;
 493:font_row=16'h2900;
 494:font_row=16'h47fe;
 495:font_row=16'h8000;
 496:font_row=16'h0040;
 497:font_row=16'h0040;
 498:font_row=16'hfc7c;
 499:font_row=16'h1084;
 500:font_row=16'h1108;
 501:font_row=16'h22fe;
 502:font_row=16'h3c92;
 503:font_row=16'h6492;
 504:font_row=16'h64fe;
 505:font_row=16'ha492;
 506:font_row=16'h2492;
 507:font_row=16'h24fe;
 508:font_row=16'h3c92;
 509:font_row=16'h2512;
 510:font_row=16'h210a;
 511:font_row=16'h0204;
 512:font_row=16'h0040;
 513:font_row=16'h2040;
 514:font_row=16'h1040;
 515:font_row=16'h1040;
 516:font_row=16'h0040;
 517:font_row=16'h0040;
 518:font_row=16'hf040;
 519:font_row=16'h1040;
 520:font_row=16'h10a0;
 521:font_row=16'h10a0;
 522:font_row=16'h10a0;
 523:font_row=16'h1510;
 524:font_row=16'h1910;
 525:font_row=16'h1208;
 526:font_row=16'h0404;
 527:font_row=16'h0802;
 528:font_row=16'h2208;
 529:font_row=16'h1108;
 530:font_row=16'h1110;
 531:font_row=16'h0020;
 532:font_row=16'h7ffe;
 533:font_row=16'h4002;
 534:font_row=16'h8004;
 535:font_row=16'h1fe0;
 536:font_row=16'h0040;
 537:font_row=16'h0180;
 538:font_row=16'hfffe;
 539:font_row=16'h0100;
 540:font_row=16'h0100;
 541:font_row=16'h0100;
 542:font_row=16'h0500;
 543:font_row=16'h0200;
 545:font_row=16'h7ff8;
 546:font_row=16'h0008;
 547:font_row=16'h0008;
 548:font_row=16'h0808;
 549:font_row=16'h0408;
 550:font_row=16'h0208;
 551:font_row=16'h0208;
 552:font_row=16'h0068;
 553:font_row=16'h0188;
 554:font_row=16'h0e08;
 555:font_row=16'h7008;
 556:font_row=16'h2008;
 557:font_row=16'h0008;
 558:font_row=16'h0050;
 559:font_row=16'h0020;
 560:font_row=16'h0480;
 561:font_row=16'h0498;
 562:font_row=16'h7ce0;
 563:font_row=16'h0484;
 564:font_row=16'h1c84;
 565:font_row=16'he47c;
 566:font_row=16'h4000;
 567:font_row=16'h1ff0;
 568:font_row=16'h1010;
 569:font_row=16'h1ff0;
 570:font_row=16'h1010;
 571:font_row=16'h1ff0;
 572:font_row=16'h1010;
 573:font_row=16'h1010;
 574:font_row=16'h1050;
 575:font_row=16'h1020;
 576:font_row=16'h1ff0;
 577:font_row=16'h1010;
 578:font_row=16'h1ff0;
 579:font_row=16'h1010;
 580:font_row=16'h1ff0;
 581:font_row=16'h0100;
 582:font_row=16'hfffe;
 584:font_row=16'h1ff0;
 585:font_row=16'h1010;
 586:font_row=16'h1010;
 587:font_row=16'h1ff0;
 588:font_row=16'h0100;
 589:font_row=16'h2108;
 590:font_row=16'h4504;
 591:font_row=16'h0200;
 592:font_row=16'h0800;
 593:font_row=16'h0bf8;
 594:font_row=16'h0a08;
 595:font_row=16'h1208;
 596:font_row=16'h1208;
 597:font_row=16'h33f8;
 598:font_row=16'h3040;
 599:font_row=16'h5040;
 600:font_row=16'h97fc;
 601:font_row=16'h10e0;
 602:font_row=16'h1150;
 603:font_row=16'h1248;
 604:font_row=16'h1444;
 605:font_row=16'h1842;
 606:font_row=16'h1040;
 607:font_row=16'h1040;
 608:font_row=16'h1020;
 609:font_row=16'h1020;
 610:font_row=16'h1020;
 611:font_row=16'h11fc;
 612:font_row=16'hfc20;
 613:font_row=16'h1020;
 614:font_row=16'h13fe;
 615:font_row=16'h1008;
 616:font_row=16'h1808;
 617:font_row=16'h33fe;
 618:font_row=16'hd008;
 619:font_row=16'h1088;
 620:font_row=16'h1048;
 621:font_row=16'h1008;
 622:font_row=16'h5028;
 623:font_row=16'h2010;
 625:font_row=16'h3ff8;
 626:font_row=16'h2008;
 627:font_row=16'h2008;
 628:font_row=16'h2008;
 629:font_row=16'h3ff8;
 630:font_row=16'h2008;
 631:font_row=16'h2008;
 632:font_row=16'h2008;
 633:font_row=16'h3ff8;
 634:font_row=16'h2008;
 635:font_row=16'h2008;
 636:font_row=16'h2008;
 637:font_row=16'h2008;
 638:font_row=16'h3ff8;
 639:font_row=16'h2008;
 640:font_row=16'h0200;
 641:font_row=16'h0200;
 642:font_row=16'h0200;
 643:font_row=16'hfffe;
 644:font_row=16'h0400;
 645:font_row=16'h0900;
 646:font_row=16'h1100;
 647:font_row=16'h3ff8;
 648:font_row=16'h5108;
 649:font_row=16'h9108;
 650:font_row=16'h1108;
 651:font_row=16'h1108;
 652:font_row=16'h1128;
 653:font_row=16'h1110;
 654:font_row=16'h0100;
 655:font_row=16'h0100;
 657:font_row=16'h3ff8;
 658:font_row=16'h2008;
 659:font_row=16'h2008;
 660:font_row=16'h3ff8;
 661:font_row=16'h2000;
 662:font_row=16'h2000;
 663:font_row=16'h3ffc;
 664:font_row=16'h2004;
 665:font_row=16'h27e4;
 666:font_row=16'h2424;
 667:font_row=16'h2424;
 668:font_row=16'h47e4;
 669:font_row=16'h4004;
 670:font_row=16'h8028;
 671:font_row=16'h0010;
 673:font_row=16'h7ffc;
 674:font_row=16'h0080;
 675:font_row=16'h0080;
 676:font_row=16'h0100;
 677:font_row=16'h0100;
 678:font_row=16'h0340;
 679:font_row=16'h0520;
 680:font_row=16'h0910;
 681:font_row=16'h1108;
 682:font_row=16'h2104;
 683:font_row=16'h4104;
 684:font_row=16'h8100;
 685:font_row=16'h0100;
 686:font_row=16'h0100;
 687:font_row=16'h0100;
 688:font_row=16'h1040;
 689:font_row=16'h1040;
 690:font_row=16'h3f7e;
 691:font_row=16'h4890;
 692:font_row=16'h8508;
 693:font_row=16'h0810;
 694:font_row=16'h0810;
 695:font_row=16'h13fe;
 696:font_row=16'h3010;
 697:font_row=16'h5110;
 698:font_row=16'h9090;
 699:font_row=16'h1090;
 700:font_row=16'h1010;
 701:font_row=16'h1010;
 702:font_row=16'h1050;
 703:font_row=16'h1020;
 704:font_row=16'h0100;
 705:font_row=16'h0100;
 706:font_row=16'h0100;
 707:font_row=16'h3ff8;
 708:font_row=16'h0100;
 709:font_row=16'h0100;
 710:font_row=16'h0100;
 711:font_row=16'hfffe;
 712:font_row=16'h0380;
 713:font_row=16'h0540;
 714:font_row=16'h0920;
 715:font_row=16'h1110;
 716:font_row=16'h2108;
 717:font_row=16'hc106;
 718:font_row=16'h0100;
 719:font_row=16'h0100;
 720:font_row=16'h0100;
 721:font_row=16'h0100;
 722:font_row=16'h0280;
 723:font_row=16'h0440;
 724:font_row=16'h0820;
 725:font_row=16'h1010;
 726:font_row=16'h2fe8;
 727:font_row=16'hc106;
 728:font_row=16'h0100;
 729:font_row=16'h0100;
 730:font_row=16'h1ff0;
 731:font_row=16'h0100;
 732:font_row=16'h0100;
 733:font_row=16'h0100;
 734:font_row=16'h7ffc;
 739:font_row=16'hee00;
 740:font_row=16'h4400;
 741:font_row=16'h4800;
 742:font_row=16'h5000;
 743:font_row=16'h7000;
 744:font_row=16'h5000;
 745:font_row=16'h4800;
 746:font_row=16'h4800;
 747:font_row=16'h4400;
 748:font_row=16'h4400;
 749:font_row=16'hee00;
 default:font_row=0;endcase end endfunction
 wire [15:0] bitmap=font_row(font_address);
 always @* begin
  character=0;number=0;color=24'hffff00;
  if(state==5)begin
   case(row)
    0:begin number=total;color=24'hffffff;case(col) 0:character=6'd12; 1:character=6'd13; 2:character=6'd14; 3:character=6'd15; 4:character=6'd11; endcase end
    1:begin number=used;color=24'hff3030;case(col) 0:character=6'd16; 1:character=6'd17; 2:character=6'd14; 3:character=6'd15; 4:character=6'd11; endcase end
    2:begin number=free_slots;color=24'h30ff30;case(col) 0:character=6'd18; 1:character=6'd19; 2:character=6'd14; 3:character=6'd15; 4:character=6'd11; endcase end
   endcase
   if(col==5)character=number>=20 ? 3 : number>=10 ? 2 : 0;
   if(col==6)character=1+(number>=20 ? number-20 : number>=10 ? number-10 : number);
  end else begin
   if(row==0)begin
case(col) 0:character=6'd20; 1:character=6'd21; 2:character=6'd22; 3:character=6'd11; endcase
    if(col==4)character=detected>=20 ? 3 : detected>=10 ? 2 : 0;
    if(col==5)character=1+(detected>=20 ? detected-20 : detected>=10 ? detected-10 : detected);
   end else if(row==1)begin
    case(state)
    0:begin case(col) 0:character=6'd23; 1:character=6'd24; 2:character=6'd25; 3:character=6'd26; endcase end
    1:begin case(col) 0:character=6'd27; 1:character=6'd28; 2:character=6'd29; 3:character=6'd30; endcase end
    2:begin case(col) 0:character=6'd27; 1:character=6'd28; 2:character=6'd29; 3:character=6'd30; endcase end
    3:begin case(col) 0:character=6'd46; 1:character=6'd2; 2:character=6'd31; 3:character=6'd32; endcase end
    4:begin case(col) 0:character=6'd33; 1:character=6'd34; 2:character=6'd35; 3:character=6'd36; endcase end
    endcase
   end else if(row==2)begin
    if(state>=3)begin
case(col) 0:character=6'd37; 1:character=6'd38; 2:character=6'd14; 3:character=6'd15; 4:character=6'd18; 5:character=6'd19; endcase end
    else if(ambiguous)begin case(col) 0:character=6'd40; 1:character=6'd41; 2:character=6'd42; 3:character=6'd43; endcase end
    else begin case(col) 0:character=6'd39; 1:character=6'd29; 2:character=6'd11; 3:character=6'd3; 4:character=6'd3; endcase end
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin area_d<=0;font_address<=0;bit_column<=0;color_d<=0;hit<=0;rgb<=0;end
  else begin
   area_d<=area;font_address<={character,dy[4:1]};bit_column<=dx[4:1];color_d<=color;
   hit<=area_d;rgb<=bitmap[15-bit_column] ? color_d : 24'h000000;
  end
 end
endmodule
