// Display-side algorithms. Every mode receives calibrated ISP RGB.
// Logo and text are composed only at the final display output.
module algorithm_hdmi_switch #(
    parameter H_OFFSET=0, V_OFFSET=0, IMG_WIDTH=1280, IMG_HEIGHT=720,
    parameter integer DEBUG_MODE=0
)(
    input wire I_clk, I_rst_n,
    input wire I_video_vsync, I_video_hsync, I_video_de, I_video_user, I_video_last,
    input wire [3:0] I_debug_status,
    input wire [2:0] I_mode,
    input wire I_source_ready,
    input wire I_strong_light,
    input wire I_key3_n,
    output wire O_buzzer_n,
    input wire [1:0] I_parking_keys_n,
    output wire O_uart_tx,
    output wire O_video_rd_en,
    input wire [23:0] I_video_rd_data,
    output reg O_hdmi_vsync, O_hdmi_hsync, O_hdmi_de,
    output reg [23:0] O_hdmi_data
);
    // Reuse the established six-clock streaming Sobel and RGB alignment.
    wire [23:0] rgb;
    wire de, user, last, vs, hs, edge_pixel;
    parking_edge_stream u_algorithm_edge(
        I_clk,I_rst_n,I_video_rd_data,I_video_de,I_video_user,I_video_last,
        I_video_vsync,I_video_hsync,I_strong_light,rgb,de,user,last,vs,hs,edge_pixel);
    assign O_video_rd_en = I_video_de;

    // Debounce K3 continuously, accept presses only in PARKING, commit at VS.
    wire [1:0] tracking_press;
    reg tracking_request,tracking_active;
    parking_keys u_tracking_key(I_clk,I_rst_n,{1'b1,I_key3_n},tracking_press);
    wire tracking_next=(I_mode==6 && I_source_ready) ?
                       (tracking_press[0] ? !tracking_request : tracking_request) : 1'b0;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin tracking_request<=0;tracking_active<=0;end
        else begin
            tracking_request<=tracking_next;
            if(I_mode!=6 || !I_source_ready) tracking_active<=0;
            else if(I_video_user) tracking_active<=tracking_next;
        end
    end
    wire tracking_enabled=tracking_active && I_mode==6 && I_source_ready;
    reg [7:0] binary_threshold;
    // Preserve the existing parking recognition and original overlay layout.
    // Disable its separate gamma and saturation: input is already corrected.
    wire [23:0] parking_rgb;
    parking_hdmi_legacy #(
        .H_OFFSET(0),.V_OFFSET(0),.IMG_WIDTH(1280),.IMG_HEIGHT(720),
        .DEBUG_MODE(5),.GAMMA_ENABLE(0),.GAMMA_MIX(0),.SATURATION_Q8(256)
    ) u_parking_view (
        .I_clk(I_clk),.I_rst_n(I_rst_n),.I_diag_data(96'd0),.I_diag_req(1'b0),.O_diag_ack(),
        .I_video_vsync(I_video_vsync),.I_video_hsync(I_video_hsync),
        .I_video_de(I_video_de),.I_video_user(I_video_user),.I_video_last(I_video_last),
        .I_debug_status(I_source_ready ? I_debug_status : 4'd0),
        .I_cal_buttons_n({2'b11,(I_mode==6 ? I_parking_keys_n : 2'b11)}),
        .I_vehicle_threshold(binary_threshold),.I_light_mode(I_strong_light),.I_tracking(tracking_enabled),.O_buzzer_n(O_buzzer_n),.O_exposure_gray(),.O_uart_tx(O_uart_tx),
        .O_video_rd_en(),.I_video_rd_data(I_video_rd_data),
        .O_hdmi_vsync(),.O_hdmi_hsync(),.O_hdmi_de(),.O_hdmi_data(parking_rgb)
    );

    reg [10:0] x;
    reg [9:0] y;
    wire [10:0] px = user ? 11'd0 : x;
    wire [9:0] py = user ? 10'd0 : y;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin x<=0; y<=0; end
        else if (user) begin x<=de ? 11'd1 : 11'd0; y<=0; end
        else if (de) begin
            if (last) begin x<=0; y<=y==719 ? 10'd0 : y+10'd1; end
            else x<=x+11'd1;
        end
    end
    wire [15:0] gray_sum = {8'd0,rgb[23:16]}*16'd77
                          + {8'd0,rgb[15:8]}*16'd150
                          + {8'd0,rgb[7:0]}*16'd29 + 16'd128;
    wire [7:0] gray = gray_sum[15:8];

    // Frame-adaptive binary: threshold is the previous complete frame's mean.
    // All 921600 calibrated luminance samples contribute; incomplete frames
    // never replace the threshold. First frame uses the documented value 128.
    reg [27:0] luma_sum;
    reg [19:0] luma_count;
    // Divide once per frame during blanking, instead of a wide combinational
    // constant divider in the pixel clock path. Eight subtract/shift steps give
    // the EXACT floor(sum/921600), with the same next-frame threshold update.
    reg [27:0] mean_remainder,mean_divisor;
    reg [7:0] mean_quotient,frame_mean;
    reg [2:0] mean_bit;
    reg mean_busy,mean_ready;
    wire [28:0] mean_subtract = {1'b0,mean_remainder}-{1'b0,mean_divisor};
    wire mean_take = !mean_subtract[28];
    wire [7:0] mean_next = {mean_quotient[6:0],mean_take};
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) begin
            mean_remainder<=0;mean_divisor<=0;mean_quotient<=0;frame_mean<=128;
            mean_bit<=0;mean_busy<=0;mean_ready<=0;
        end else if(!I_source_ready) begin
            mean_remainder<=0;mean_divisor<=0;mean_quotient<=0;frame_mean<=128;
            mean_bit<=0;mean_busy<=0;mean_ready<=0;
        end else if(de && !user && luma_count==20'd921599) begin
            mean_remainder<=luma_sum+gray;
            mean_divisor<=28'd117964800; // 921600 << 7
            mean_quotient<=0;mean_bit<=7;mean_busy<=1;mean_ready<=0;
        end else if(mean_busy) begin
            if(mean_take) mean_remainder<=mean_subtract[27:0];
            mean_divisor<=mean_divisor>>1;
            mean_quotient<=mean_next;
            mean_bit<=mean_bit-1'b1;
            if(mean_bit==0) begin
                frame_mean<=mean_next;mean_busy<=0;mean_ready<=1;
            end
        end else if(user) mean_ready<=0;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin luma_sum<=0; luma_count<=0; binary_threshold<=128; end
        else if (!I_source_ready) begin luma_sum<=0; luma_count<=0; binary_threshold<=128; end
        else if (user) begin
            if (luma_count==20'd921600 && mean_ready)
                binary_threshold<=frame_mean<16 ? 8'd16 : frame_mean>240 ? 8'd240 : frame_mean[7:0];
            luma_sum<=de ? {20'd0,gray} : 28'd0;
            luma_count<=de ? 20'd1 : 20'd0;
        end else if (de) begin
            luma_sum<=luma_sum+gray; luma_count<=luma_count+20'd1;
        end
    end

    // Frame-difference motion at 160x90 sample resolution (8x8 display blocks).
    // Read-before-write holds the previous frame's grayscale. Two mask banks
    // ensure display reads only the last COMPLETE difference map.
    (* ram_style="bram" *) reg [7:0] previous_gray[0:14399];
    (* ram_style="bram" *) reg motion_maps[0:32767];
    wire [13:0] cell_address = (py>>3)*14'd160 + (px>>3);
    wire cell_sample = de && px[2:0]==3'd4 && py[2:0]==3'd4;
    reg map_bank, background_ready, maps_ready;
    reg [14:0] motion_samples;
    reg sample_q, sample_bg_ready;
    reg [13:0] sample_address;
    reg [7:0] previous_q,current_q;
    wire [7:0] motion_difference = previous_q>current_q ? previous_q-current_q : current_q-previous_q;
    // Memories are not asynchronously reset; validity guards hide cold data.
    always @(posedge I_clk) begin
        if (I_rst_n && I_source_ready && cell_sample) begin
            previous_q<=previous_gray[cell_address];
            current_q<=gray;
            previous_gray[cell_address]<=gray;
        end
        if (I_rst_n && I_source_ready && sample_q)
            motion_maps[{map_bank,sample_address}]<=sample_bg_ready && motion_difference>=8'd20;
    end
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            map_bank<=0;background_ready<=0;maps_ready<=0;motion_samples<=0;
            sample_q<=0;sample_bg_ready<=0;sample_address<=0;
        end else if (!I_source_ready) begin
            map_bank<=0;background_ready<=0;maps_ready<=0;motion_samples<=0;
            sample_q<=0;sample_bg_ready<=0;sample_address<=0;
        end else begin
            sample_q<=cell_sample;
            if (cell_sample) begin
                sample_address<=cell_address;
                sample_bg_ready<=background_ready;
                motion_samples<=motion_samples+15'd1;
            end
            if (user) begin
                maps_ready<=background_ready && motion_samples==15'd14400;
                background_ready<=motion_samples==15'd14400;
                if (motion_samples==15'd14400) map_bank<=~map_bank;
                motion_samples<=0;
            end
        end
    end

    // These three registers match the three registers in parking_hdmi_legacy.
    reg [28:0] video_pipe[0:2];
    reg [10:0] x_pipe[0:2];
    reg [9:0] y_pipe[0:2];
    reg [7:0] gray_pipe[0:2];
    reg [2:0] edge_pipe,binary_pipe,motion_ready_pipe;
    reg [2:1] motion_pipe;
    reg motion_read;
    always @(posedge I_clk) begin
        // Hold on blanking. Clearing this RAM output makes TD infer a sync
        // reset, which prevents ERAM mapping. HDMI DE masks blanking pixels.
        if (de && py<720 && px<1280) motion_read<=motion_maps[{!map_bank,cell_address}];
    end
    integer stage;
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            for(stage=0;stage<3;stage=stage+1) begin
                video_pipe[stage]<=0;x_pipe[stage]<=0;y_pipe[stage]<=0;gray_pipe[stage]<=0;
            end
            edge_pipe<=0;binary_pipe<=0;motion_pipe<=0;motion_ready_pipe<=0;
        end else begin
            video_pipe[0]<={vs,hs,de,user,last,rgb};
            x_pipe[0]<=px;y_pipe[0]<=py;gray_pipe[0]<=gray;
            for(stage=1;stage<3;stage=stage+1) begin
                video_pipe[stage]<=video_pipe[stage-1];
                x_pipe[stage]<=x_pipe[stage-1];y_pipe[stage]<=y_pipe[stage-1];
                gray_pipe[stage]<=gray_pipe[stage-1];
            end
            edge_pipe<={edge_pipe[1:0],edge_pixel};
            binary_pipe<={binary_pipe[1:0],gray>=binary_threshold};
            motion_ready_pipe<={motion_ready_pipe[1:0],maps_ready && I_source_ready};
            motion_pipe[2]<=motion_pipe[1];
            motion_pipe[1]<=motion_read;
        end
    end
    // Logo is a display-only overlay for mode 00 ISP, never algorithm input.
    // Stage-0 coordinates + synchronous ROM + this register align with stage 2.
    wire logo_area=x_pipe[0]<11'd160 && y_pipe[0]<10'd160;
    wire [14:0] logo_addr=logo_area ? y_pipe[0]*15'd160+x_pipe[0] : 15'd0;
    wire [24:0] logo_pixel;
    reg [24:0] logo_aligned;
    anlogic_logo_rom u_isp_logo(I_clk,logo_addr,logo_pixel);
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n) logo_aligned<=0;
        else logo_aligned<=logo_pixel;
    end
    // ENHANCE is a separate, modest contrast expansion on corrected RGB.
    function [7:0] enhance_channel;
        input [7:0] c;
        reg signed [12:0] v;
        begin
            v=($signed({5'd0,c}) <<< 3)+$signed({5'd0,c})-13'sd128+13'sd4;
            v=v>>>3;
            enhance_channel=v<0 ? 8'd0 : v>255 ? 8'd255 : v[7:0];
        end
    endfunction
    wire [23:0] aligned_rgb=video_pipe[2][23:0];
    wire [23:0] enhanced_rgb={enhance_channel(aligned_rgb[23:16]),
                             enhance_channel(aligned_rgb[15:8]),enhance_channel(aligned_rgb[7:0])};
    reg [23:0] selected_rgb;
    always @* begin
        case (I_mode)
            1: selected_rgb={3{gray_pipe[2]}};
            2: selected_rgb={24{edge_pipe[2]}};
            3: selected_rgb={24{binary_pipe[2]}};
            4: selected_rgb={24{motion_pipe[2] && motion_ready_pipe[2]}};
            5: selected_rgb=enhanced_rgb;
            6: selected_rgb=parking_rgb;
            default: selected_rgb=aligned_rgb;
        endcase
    end

    // The mode title is composed after all algorithms and never enters analysis.
    function [7:0] mode_character;
        input [2:0] mode;
        input integer index;
        input strong_light;
        begin
            mode_character=8'd32;
            if(index==0) mode_character=8'd48;
            else if(index==1) mode_character=8'd48+mode;
            else if(index==11) mode_character=strong_light ? "H" : "L";
            else if(index>=3) begin
                case(mode)
                    0: case(index) 3:mode_character="I";4:mode_character="S";5:mode_character="P";endcase
                    1: case(index) 3:mode_character="G";4:mode_character="R";5:mode_character="A";6:mode_character="Y";endcase
                    2: case(index) 3:mode_character="S";4:mode_character="O";5:mode_character="B";6:mode_character="E";7:mode_character="L";endcase
                    3: case(index) 3:mode_character="B";4:mode_character="I";5:mode_character="N";6:mode_character="A";7:mode_character="R";8:mode_character="Y";endcase
                    4: case(index) 3:mode_character="M";4:mode_character="O";5:mode_character="T";6:mode_character="I";7:mode_character="O";8:mode_character="N";endcase
                    5: case(index) 3:mode_character="E";4:mode_character="N";5:mode_character="H";6:mode_character="A";7:mode_character="N";8:mode_character="C";9:mode_character="E";endcase
                    6: case(index) 3:mode_character="P";4:mode_character="A";5:mode_character="R";6:mode_character="K";7:mode_character="I";8:mode_character="N";9:mode_character="G";endcase
                endcase
            end
        end
    endfunction
    function [7:0] tracking_character;
        input integer index;
        begin case(index)
            3:tracking_character="T";4:tracking_character="R";5:tracking_character="A";
            6:tracking_character="C";7:tracking_character="K";default:tracking_character=" ";
        endcase end
    endfunction
    function [34:0] title_font;
        input [7:0] c;
        begin
            case(c)
                8'd48: title_font=35'b01110100011001110101110011000101110;
                8'd49: title_font=35'b00100011000010000100001000010001110;
                8'd50: title_font=35'b01110100010000100010001000100011111;
                8'd51: title_font=35'b11110000010000101110000010000111110;
                8'd52: title_font=35'b00010001100101010010111110001000010;
                8'd53: title_font=35'b11111100001000011110000010000111110;
                8'd54: title_font=35'b01110100001000011110100011000101110;
                8'd55: title_font=35'b11111000010001000100010000100001000;
                8'd65: title_font=35'b01110100011000111111100011000110001;
                8'd66: title_font=35'b11110100011000111110100011000111110;
                8'd67: title_font=35'b01111100001000010000100001000001111;
                8'd69: title_font=35'b11111100001000011110100001000011111;
                8'd71: title_font=35'b01111100001000010111100011000101111;
                8'd72: title_font=35'b10001100011000111111100011000110001;
                8'd73: title_font=35'b01110001000010000100001000010001110;
                8'd75: title_font=35'b10001100101010011000101001001010001;
                8'd76: title_font=35'b10000100001000010000100001000011111;
                8'd77: title_font=35'b10001110111010110101100011000110001;
                8'd78: title_font=35'b10001110011010110011100011000110001;
                8'd79: title_font=35'b01110100011000110001100011000101110;
                8'd80: title_font=35'b11110100011000111110100001000010000;
                8'd82: title_font=35'b11110100011000111110101001001010001;
                8'd83: title_font=35'b01111100001000001110000010000111110;
                8'd84: title_font=35'b11111001000010000100001000010000100;
                8'd87: title_font=35'b10001100011000110101101011010101010;
                8'd89: title_font=35'b10001100010101000100001000010000100;
                default: title_font=35'd0;
            endcase
        end
    endfunction
    wire title_area=x_pipe[2]>=480 && x_pipe[2]<624 && y_pipe[2]>=8 && y_pipe[2]<24;
    wire [10:0] title_x=x_pipe[2]-11'd480;
    wire [9:0] title_y=y_pipe[2]-10'd8;
    wire [34:0] title_glyph=title_font((I_mode==6 && tracking_enabled && title_x/12>=3 && title_x/12<=9) ?
        tracking_character(title_x/12) : mode_character(I_mode,title_x/12,I_strong_light));
    wire [3:0] glyph_x=(title_x%12)>>1;
    wire [2:0] glyph_y=title_y[3:1];
    wire title_ink=title_area && glyph_x<5 && glyph_y<7
                 && title_glyph[34-glyph_y*5-glyph_x];
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin O_hdmi_vsync<=0;O_hdmi_hsync<=0;O_hdmi_de<=0;O_hdmi_data<=0; end
        else begin
            O_hdmi_vsync<=video_pipe[2][28];
            O_hdmi_hsync<=video_pipe[2][27];
            O_hdmi_de<=video_pipe[2][26];
            if(!video_pipe[2][26]) O_hdmi_data<=0;
            else if(title_area) O_hdmi_data<=title_ink ? 24'hffffff : 24'h000000;
            else if(I_source_ready && I_mode==3'd0 && x_pipe[2]<11'd160
                    && y_pipe[2]<10'd160 && logo_aligned[24])
                O_hdmi_data<=logo_aligned[23:0];
            else O_hdmi_data<=I_source_ready ? selected_rgb : 24'h000000;
        end
    end
endmodule
