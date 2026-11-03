/*****************************************************************
Company : MiLianKe Electronic Technology Co., Ltd.
WebSite:https://www.milianke.com
TechWeb:https://www.uisrc.com
tmall-shop:https://milianke.tmall.com
jd-shop:https://milianke.jd.com
taobao-shop: https://milianke.taobao.com
Description: 
The reference demo provided by Milianke is only used for learning. 
We cannot ensure that the demo itself is free of bugs, so users 
should be responsible for the technical problems and consequences
caused by the use of their own products.
@Author      :   XiaoQingquan 
@Time        :   2024/10 
version:     :   
@Description :   在原基础上删去了一些赘余的代码，并且优化了时序，加入BRAM IP核，
                 让从除法器出来的每一个数据都可以对上相应的像素数据进行增益，使
                 得它的适用行更加广泛了。
*****************************************************************/
module awb  #(   
    parameter IMG_HEIGHT = 1080,
    parameter IMG_WIDTH  = 1920,
    parameter ENABLE_AWB = 1,
    parameter FIXED_WHITE_REFERENCE = 0
) 
(
    input                   I_clk  ,
    input                   I_rst_n,


    input                   I_tlast  ,
    input                   I_tuser  ,
    input [95:0]            I_tdata  ,
    input                   I_tvalid , 
    output                  I_tready ,

    output                  O_tlast  ,
    output                  O_tuser  ,
    output [95:0]           O_tdata  ,
    output                  O_tvalid ,
    input                   O_tready,
    output [19:0]           O_debug_r_gain,
    output [19:0]           O_debug_b_gain,
    output reg [19:0]       O_debug_count
);

/*****************************************************************
                          RGB565תRGB888                                        
*****************************************************************/

    // //RGB565??????888
    // wire [4:0]  rgb565_r   = I_st_data[11+:5];
    // wire [5:0]  rgb565_g   = I_st_data[5+:6];
    // wire [4:0]  rgb565_b   = I_st_data[0+:5];
    
    // wire [23:0] rgb888     = {{rgb565_r[4:0],rgb565_r[2:0]},{rgb565_g[5:0],rgb565_g[1:0]},{rgb565_b[4:0],rgb565_b[2:0]}};
    
    // wire [7:0]  rgb888_r   = rgb888[16+:8];
    // wire [7:0]  rgb888_g   = rgb888[8+:8] ;
    // wire [7:0]  rgb888_b   = rgb888[0+:8] ;
    
  /*****************************************************************
                          中间信号                                        
*****************************************************************/


  reg         I_tvalid_r0,I_tvalid_r1;
  reg  [95:0] I_tdata_d0,I_tdata_d1;


  //累加RGB通道的�??
  reg  [31:0]         sum_r[0:3];
  reg  [31:0]         sum_g[0:3];
  reg  [31:0]         sum_b[0:3];

  
  wire  [95:0]        data_d0 ;
  wire                valid_d0;
  wire                I_tuser_r0;    

  reg                 I_tuser_r1;

  reg   [95:0]        data_d1 ;
  reg                 valid_d1;
  /*****************************************************************
                            平均值计�??                                      
*****************************************************************/

// 对输入数据和有效信号进行寄存，形成流水线
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        I_tdata_d0  <= 0;
        I_tdata_d1  <= 0;
        {I_tvalid_r0,I_tvalid_r1} <= 0;
    end else begin
        I_tdata_d0  <= I_tdata   ;
        I_tdata_d1  <= I_tdata_d0;
        I_tvalid_r0 <= I_tvalid;
        I_tvalid_r1 <= I_tvalid_r0;
    end
end

// Accumulate all four lanes. Snapshot complete frames for gray-world AWB.
// Exclude near-black and clipped samples; this is not a calibrated color matrix.
reg stats_tuser;
reg stats_start;
reg [31:0] stats_count;
reg [31:0] frame_sum_r, frame_sum_g, frame_sum_b;
wire [9:0] lane_r[0:3], lane_g[0:3], lane_b[0:3];
wire [2:0] lane_count[0:3];
genvar stats_lane;
generate for (stats_lane = 0; stats_lane < 4; stats_lane = stats_lane + 1) begin : awb_statistics
    wire [7:0] r = I_tdata_d0[stats_lane*24+16 +: 8];
    wire [7:0] g = I_tdata_d0[stats_lane*24+8 +: 8];
    wire [7:0] b = I_tdata_d0[stats_lane*24 +: 8];
    wire [9:0] brightness = {2'b0,r} + {2'b0,g} + {2'b0,b};
    wire [7:0] rg_max = (r > g) ? r : g;
    wire [7:0] rg_min = (r < g) ? r : g;
    wire [7:0] channel_max = (rg_max > b) ? rg_max : b;
    wire [7:0] channel_min = (rg_min < b) ? rg_min : b;
    // Measured white after BLC is about (R,G,B)=(15,23,16) in RAW8.
    // Allow min/max >= 0.6 so that reference remains eligible, while
    // rejecting strongly colored samples. This is a tuning threshold.
    wire [10:0] neutral_min_times5 = {1'b0,channel_min,2'b00} + {3'b000,channel_min};
    wire [10:0] neutral_max_times3 = {2'b00,channel_max,1'b0} + {3'b000,channel_max};
    wire usable = brightness >= 10'd48 && r > 8'd3 && g > 8'd3 && b > 8'd3
                  && channel_max < 8'd250 && neutral_min_times5 >= neutral_max_times3;
    assign lane_r[stats_lane] = usable ? {2'b0,r} : 10'd0;
    assign lane_g[stats_lane] = usable ? {2'b0,g} : 10'd0;
    assign lane_b[stats_lane] = usable ? {2'b0,b} : 10'd0;
    assign lane_count[stats_lane] = usable ? 3'd1 : 3'd0;
end endgenerate
wire [9:0] beat_r = lane_r[0] + lane_r[1] + lane_r[2] + lane_r[3];
wire [9:0] beat_g = lane_g[0] + lane_g[1] + lane_g[2] + lane_g[3];
wire [9:0] beat_b = lane_b[0] + lane_b[1] + lane_b[2] + lane_b[3];
wire [2:0] beat_count = lane_count[0] + lane_count[1] + lane_count[2] + lane_count[3];
always @(posedge I_clk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        stats_tuser <= 0;
        stats_start <= 0;
        stats_count <= 0;
        O_debug_count <= 0;
        sum_r[0] <= 1; sum_g[0] <= 1; sum_b[0] <= 1;
        frame_sum_r <= 1; frame_sum_g <= 1; frame_sum_b <= 1;
    end else begin
        stats_tuser <= I_tuser;
        stats_start <= 0;
        if (stats_tuser) begin
            O_debug_count <= stats_count[19:0];
            frame_sum_r <= sum_r[0];
            frame_sum_g <= sum_g[0];
            frame_sum_b <= sum_b[0];
            stats_start <= (stats_count >= 32'd1024);
            sum_r[0] <= 32'd1 + (I_tvalid_r0 ? beat_r : 10'd0);
            sum_g[0] <= 32'd1 + (I_tvalid_r0 ? beat_g : 10'd0);
            sum_b[0] <= 32'd1 + (I_tvalid_r0 ? beat_b : 10'd0);
            stats_count <= I_tvalid_r0 ? {29'd0,beat_count} : 32'd0;
        end else if (I_tvalid_r0) begin
            sum_r[0] <= sum_r[0] + beat_r;
            sum_g[0] <= sum_g[0] + beat_g;
            sum_b[0] <= sum_b[0] + beat_b;
            stats_count <= stats_count + beat_count;
        end
    end
end

  /*****************************************************************
                           计算增益                                       
*****************************************************************/

  wire [47:0] dout_R[3:0];
  wire [47:0] dout_B[3:0];

  wire done[3:0];
  wire blue_gain_done;

  wire [47:0] in_r_x1;
  wire [47:0] in_b_x1;
  assign in_r_x1 = dout_R[0][47:0];
  assign in_b_x1 = dout_B[0][47:0];



/*****************************************************************
                   IP 调用                                    
*****************************************************************/
    //为了保证合理的延迟，这里使用了除法器

      divider div_gen_G_R_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(stats_start),
          .denominator(frame_sum_r),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({frame_sum_g,16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_R[0]),  // output wire [79 : 0] m_axis_dout_tdata
          .done(done[0])
      );

      divider divide_G_B_r1 (
          .clk(I_clk),  // input wire aclk
          .rst(!I_rst_n),
          .start(stats_start),
          .denominator(frame_sum_b),  // input wire [31 : 0] s_axis_divisor_tdata
          .numerator({frame_sum_g,16'd0}),  // input wire [47 : 0] s_axis_dividend_tdata
          .quotient(dout_B[0]),  // output wire [79 : 0] m_axis_dout_tdata
          .done(blue_gain_done)
      );

     
    signal_delay #(
        .IMG_HEIGHT   (IMG_HEIGHT),
        .IMG_WIDTH    (IMG_WIDTH),
        .DATA_WIDTH   (96),
        .DELAY_CYCLE  (50)
    ) signal_delay_d (
        .I_clk   (I_clk),
        .I_rst_n (I_rst_n),
        .I_tuser (I_tuser),
        .I_valid (I_tvalid_r1),
        .I_data  (I_tdata_d1),
        .O_valid (valid_d0),
        .O_data  (data_d0),
        .O_tuser (I_tuser_r0)
    );

    reg [19:0] in_r_r1, in_b_r1;
    // Fixed-gain comparison using the measured white-paper OSD values.
    // OSD Q8 1C0/18B -> Q16 1C000/18B00; calibration is already included.
    // Do not apply WHITE_TRIM again to these measured, final gains.
    localparam [19:0] WHITE_REFERENCE_R = 20'h1C000;
    localparam [19:0] WHITE_REFERENCE_B = 20'h18B00;
    wire [19:0] applied_r_gain = !ENABLE_AWB ? 20'h10000
                              : FIXED_WHITE_REFERENCE ? WHITE_REFERENCE_R : in_r_r1;
    wire [19:0] applied_b_gain = !ENABLE_AWB ? 20'h10000
                              : FIXED_WHITE_REFERENCE ? WHITE_REFERENCE_B : in_b_r1;
    assign O_debug_r_gain = applied_r_gain;
    assign O_debug_b_gain = applied_b_gain;
    reg [19:0] pending_r_gain, pending_b_gain;
    reg pending_gain_valid;
    function [19:0] bounded_gain;
        input [47:0] quotient;
        begin
            if (quotient > 48'h000000040000) bounded_gain = 20'h40000;
            else if (quotient < 48'h000000004000) bounded_gain = 20'h04000;
            else bounded_gain = quotient[19:0];
        end
    endfunction
    // White-reference trial under the current illumination (post-AWB ROI):
    // R=0x018, G=0x01D, B=0x01A. Q8 trims approximate G/R and G/B.
    // Apply once to each fresh AWB target, never recursively to held gains.
    localparam [8:0] WHITE_TRIM_R_Q8 = 9'd309; // 1.20703125
    localparam [8:0] WHITE_TRIM_B_Q8 = 9'd286; // 1.11718750
    wire [19:0] base_r_target = bounded_gain(in_r_x1);
    wire [19:0] base_b_target = bounded_gain(in_b_x1);
    wire [28:0] trimmed_r_target = base_r_target * WHITE_TRIM_R_Q8 + 29'd128;
    wire [28:0] trimmed_b_target = base_b_target * WHITE_TRIM_B_Q8 + 29'd128;
    // First-order temporal smoothing: 75% old gain + 25% new target.
    // Widen before addition so gain limits cannot overflow the accumulator.
    wire [21:0] smooth_r_gain = ({2'b00,in_r_r1} << 1) + {2'b00,in_r_r1}
                               + {2'b00,pending_r_gain} + 22'd2;
    wire [21:0] smooth_b_gain = ({2'b00,in_b_r1} << 1) + {2'b00,in_b_r1}
                               + {2'b00,pending_b_gain} + 22'd2;
    // Keep displayed gains constant within each frame, including its first lines.
    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            in_r_r1 <= 20'h10000;
            in_b_r1 <= 20'h10000;
            pending_r_gain <= 20'h10000;
            pending_b_gain <= 20'h10000;
            pending_gain_valid <= 0;
        end else begin
            if (done[0] && blue_gain_done) begin
                pending_r_gain <= bounded_gain({27'd0,trimmed_r_target[28:8]});
                pending_b_gain <= bounded_gain({27'd0,trimmed_b_target[28:8]});
                pending_gain_valid <= 1;
            end
            if (I_tuser && pending_gain_valid) begin
                in_r_r1 <= smooth_r_gain[21:2];
                in_b_r1 <= smooth_b_gain[21:2];
            end
        end
    end

    ////信号同步
    always @(posedge I_clk or negedge I_rst_n) begin
      if (!I_rst_n) begin
          valid_d1 <= 0;
          data_d1  <= 0;
          I_tuser_r1 <= 0;
      end else begin
          valid_d1 <= valid_d0;
          data_d1  <= data_d0 ;
          I_tuser_r1 <= I_tuser_r0;
      end
    end
/*****************************************************************
                           ��������                                       
*****************************************************************/

    reg [27:0] R_new_r1;
    reg [27:0] G_new_r1;
    reg [27:0] B_new_r1;

    reg [27:0] R_new_r2;
    reg [27:0] G_new_r2;
    reg [27:0] B_new_r2;

    reg [27:0] R_new_r3;
    reg [27:0] G_new_r3;
    reg [27:0] B_new_r3;

    reg [27:0] R_new_r4;
    reg [27:0] G_new_r4;
    reg [27:0] B_new_r4;


    wire   [7:0] rgb888_r_d1[0:3];
    wire   [7:0] rgb888_g_d1[0:3];
    wire   [7:0] rgb888_b_d1[0:3];

    assign  rgb888_r_d1[0] = data_d1[16+:8];
    assign  rgb888_g_d1[0] = data_d1[8+:8];
    assign  rgb888_b_d1[0] = data_d1[0+:8];

    assign  rgb888_r_d1[1] = data_d1[40+:8];
    assign  rgb888_g_d1[1] = data_d1[32+:8];
    assign  rgb888_b_d1[1] = data_d1[24+:8];

    assign  rgb888_r_d1[2] = data_d1[64+:8];
    assign  rgb888_g_d1[2] = data_d1[56+:8];
    assign  rgb888_b_d1[2] = data_d1[48+:8];

    assign  rgb888_r_d1[3] = data_d1[88+:8];
    assign  rgb888_g_d1[3] = data_d1[80+:8];
    assign  rgb888_b_d1[3] = data_d1[72+:8];
    always @(posedge I_clk or negedge I_rst_n)begin 
        if(!I_rst_n)begin
            R_new_r1 <= 0;
            G_new_r1 <= 0;
            B_new_r1 <= 0;

            R_new_r2 <= 0;
            G_new_r2 <= 0;
            B_new_r2 <= 0;

            R_new_r3 <= 0;
            G_new_r3 <= 0;
            B_new_r3 <= 0;

            R_new_r4 <= 0;
            G_new_r4 <= 0;
            B_new_r4 <= 0;
        end
        else if(valid_d1) begin
            R_new_r1 <= applied_r_gain * rgb888_r_d1[0];
            G_new_r1 <=           rgb888_g_d1[0];
            B_new_r1 <= applied_b_gain * rgb888_b_d1[0];

            R_new_r2 <= applied_r_gain * rgb888_r_d1[1];
            G_new_r2 <=           rgb888_g_d1[1];
            B_new_r2 <= applied_b_gain * rgb888_b_d1[1];

            R_new_r3 <= applied_r_gain * rgb888_r_d1[2];
            G_new_r3 <=           rgb888_g_d1[2];
            B_new_r3 <= applied_b_gain * rgb888_b_d1[2];

            R_new_r4 <= applied_r_gain * rgb888_r_d1[3];
            G_new_r4 <=           rgb888_g_d1[3];
            B_new_r4 <= applied_b_gain * rgb888_b_d1[3];
        end
    end

/*****************************************************************
                            �������???                                      
*****************************************************************/

    wire [7:0] R_new_x1;
    wire [7:0] G_new_x1;
    wire [7:0] B_new_x1;

    wire [7:0] R_new_x2;
    wire [7:0] G_new_x2;
    wire [7:0] B_new_x2;

    wire [7:0] R_new_x3;
    wire [7:0] G_new_x3;
    wire [7:0] B_new_x3;

    wire [7:0] R_new_x4;
    wire [7:0] G_new_x4;
    wire [7:0] B_new_x4;

    assign R_new_x1 = (R_new_r1[27:24])?255:R_new_r1[23:16];
    assign G_new_x1 =  G_new_r1;
    assign B_new_x1 = (B_new_r1[27:24])?255:B_new_r1[23:16];

    assign R_new_x2 = (R_new_r2[27:24])?255:R_new_r2[23:16];
    assign G_new_x2 =  G_new_r2;
    assign B_new_x2 = (B_new_r2[27:24])?255:B_new_r2[23:16];

    assign R_new_x3 = (R_new_r3[27:24])?255:R_new_r3[23:16];
    assign G_new_x3 =  G_new_r3;
    assign B_new_x3 = (B_new_r3[27:24])?255:B_new_r3[23:16];

    assign R_new_x4 = (R_new_r4[27:24])?255:R_new_r4[23:16];
    assign G_new_x4 =  G_new_r4;
    assign B_new_x4 = (B_new_r4[27:24])?255:B_new_r4[23:16];
/*****************************************************************
                           ͬ���ź�                                       
*****************************************************************/

    reg valid_d2,I_tuser_r2;
    always @(posedge I_clk or negedge I_rst_n) begin
        if(!I_rst_n)begin
            valid_d2 <= 0;
            I_tuser_r2 <= 0;
        end
        else begin
            valid_d2 <= valid_d1;
            I_tuser_r2 <= I_tuser_r1;
        end
    end


    assign  O_tlast  = (valid_d1 == 0) && (valid_d2 == 1);   
    assign  O_tuser  = I_tuser ;   
    assign  O_tvalid = valid_d2;

    assign  O_tdata = O_tvalid?{R_new_x4,G_new_x4,B_new_x4,
                       R_new_x3,G_new_x3,B_new_x3,
                       R_new_x2,G_new_x2,B_new_x2,
                       R_new_x1,G_new_x1,B_new_x1}:0;
    // assign O_t_data = {R_new[7-:5],G_new[7-:6],B_new[7-:5]};
    assign  I_tready = O_tready;
 
endmodule



