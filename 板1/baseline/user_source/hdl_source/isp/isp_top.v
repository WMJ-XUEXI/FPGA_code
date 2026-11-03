module isp_top #(parameter ENABLE_DIAGNOSTICS = 1) (
    input 			axi4s_video_aclk,
    input 			I_rst_n			,
    input               I_strong_light,
    output wire         O_strong_light,
    input 			I_tlast			,
    input 			I_tuser			,
    input [39:0] 	I_tdata			,
    input 			I_tvalid		,
    input [9:0] 	I_tdest			,
    input 			O_tready		,
    output [127:0] 	O_tdata			,
    output 			O_tlast			,
    output 			O_tuser			,
    output 			O_tvalid		,
    output 			I_tready
);
 
wire         m_aixs_tvalid;  //synthesis keep 
  wire [127:0] m_aixs_tdata;  //synthesis keep 
  wire         m_aixs_tuser;  //synthesis keep 
  wire         m_aixs_tlast;  //synthesis keep 
  wire         m_aixs_tready;  //synthesis keep 
  
  wire [ 95:0] m_aixs_tdata_96;  //synthesis keep 


  wire         awb_O_tlast;  //synthesis keep 
  wire         awb_O_tuser;  //synthesis keep 
  wire [ 95:0] awb_O_tdata;  //synthesis keep 
  wire         awb_O_tvalid;  //synthesis keep 
  wire         awb_O_tready;  //synthesis keep 

  wire         awb_O_tuser_128;  //synthesis keep 
  wire         awb_O_tvalid_128;  //synthesis keep 
  wire [127:0] awb_O_tdata_128;  //synthesis keep 

  assign awb_O_tready = O_tready;
  assign O_tdata  = awb_O_tdata_128;
  assign O_tlast  = awb_O_tlast    ;
  assign O_tuser  = awb_O_tuser_128    ;
  assign O_tvalid = awb_O_tvalid_128   ;
  
  //assign m_aixs_tready = O_tready;
  //assign O_tdata  = m_aixs_tdata;
  //assign O_tlast  = m_aixs_tlast    ;
  //assign O_tuser  = m_aixs_tuser    ;
  //assign O_tvalid = m_aixs_tvalid   ;
 
 
// Stage 4: synthetic BGGR RAW10, using unchanged camera stream timing.
localparam TEST_RAW_PATH = 0; // Live SC500 RAW input; synthetic source disabled.
reg [8:0] test_raw_beat;
reg test_raw_odd_row;
reg [23:0] test_raw_color;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        test_raw_beat <= 0;
        test_raw_odd_row <= 0;
    end else if (I_tuser) begin
        test_raw_beat <= I_tvalid ? 9'd1 : 9'd0;
        test_raw_odd_row <= 0;
    end else if (I_tvalid) begin
        if (I_tlast) begin
            test_raw_beat <= 0;
            test_raw_odd_row <= ~test_raw_odd_row;
        end else begin
            test_raw_beat <= test_raw_beat + 9'd1;
        end
    end
end
always @* begin
    if      (test_raw_beat < 9'd40)  test_raw_color = 24'hff0000;
    else if (test_raw_beat < 9'd80)  test_raw_color = 24'h00ff00;
    else if (test_raw_beat < 9'd120) test_raw_color = 24'h0000ff;
    else if (test_raw_beat < 9'd160) test_raw_color = 24'hffff00;
    else if (test_raw_beat < 9'd200) test_raw_color = 24'h00ffff;
    else if (test_raw_beat < 9'd240) test_raw_color = 24'hff00ff;
    else if (test_raw_beat < 9'd280) test_raw_color = 24'hffffff;
    else                           test_raw_color = 24'h202020;
end
// RAW sample 0 occupies [9:0]; rows alternate B,G,B,G and G,R,G,R.
wire [9:0] test_raw_r = {test_raw_color[23:16],2'b00};
wire [9:0] test_raw_g = {test_raw_color[15:8],2'b00};
wire [9:0] test_raw_b = {test_raw_color[7:0],2'b00};
wire [39:0] test_raw_data = test_raw_odd_row
                           ? {test_raw_r,test_raw_g,test_raw_r,test_raw_g}
                           : {test_raw_g,test_raw_b,test_raw_g,test_raw_b};
// Diagnostic phase sweep: 240 frames per phase (about 4 seconds at 60 fps).
localparam TEST_BAYER_PHASES = 0; // Hold original phase; color-phase diagnosis is not conclusive.
reg [7:0] bayer_phase_frames;
reg [1:0] bayer_test_phase;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        bayer_phase_frames <= 0;
        bayer_test_phase <= 0;
    end else if (I_tuser && TEST_BAYER_PHASES) begin
        if (bayer_phase_frames == 8'd239) begin
            bayer_phase_frames <= 0;
            bayer_test_phase <= bayer_test_phase + 2'd1;
        end else begin
            bayer_phase_frames <= bayer_phase_frames + 8'd1;
        end
    end
end
wire [1:0] bayer_phase = TEST_BAYER_PHASES ? bayer_test_phase : 2'b00;
// Measured dark-frame means: all four RAW10 positions are 0x040 (64).
// Correct before demosaic/AWB; keep the meter connected to the original RAW.
localparam [9:0] RAW_BLACK_LEVEL = 10'd64;
wire [39:0] black_corrected_raw;
genvar black_sample;
generate for (black_sample = 0; black_sample < 4; black_sample = black_sample + 1) begin : raw_black_correction
    wire [9:0] sample_value = I_tdata[black_sample*10 +: 10];
    assign black_corrected_raw[black_sample*10 +: 10] = (sample_value > RAW_BLACK_LEVEL)
                           ? sample_value - RAW_BLACK_LEVEL : 10'd0;
end endgenerate
wire [39:0] phased_camera_raw = bayer_phase[0]
                             ? {black_corrected_raw[29:20],black_corrected_raw[39:30],black_corrected_raw[9:0],black_corrected_raw[19:10]}
                             : black_corrected_raw;
wire [39:0] demosaic_input_raw = TEST_RAW_PATH ? test_raw_data
                               : phased_camera_raw;

demosaic #(
    .IMG_HEIGHT          (720 ), // 图像高度
    .IMG_WIDTH           (1280), // 图像宽度
    .data_complete_delay (50  ),
    .BAYER_MODE          ("BGGR")
)
u_demosaic
(
    .I_clk   			(axi4s_video_aclk)	,   // 时钟信号
    .I_rst_n 			(I_rst_n)	,   // 复位信号，低有效
    .I_bayer_y_flip       (bayer_phase[1]),
    .axi4s_video_tdata 	(demosaic_input_raw)	,  // AXI4-Stream视频数据
    .axi4s_video_tdest 	(I_tdest)	,
    .axi4s_video_tlast 	(I_tlast)	,  // 行结束信号
    .axi4s_video_tvalid	(I_tvalid)	,  // 数据有效信号
    .axi4s_video_tuser 	(I_tuser)	,  // 帧开始信号
    .axi4s_video_tready	(I_tready)	,  // 从模块准备好接受数据
    .O_tlast  			(m_aixs_tlast)	,  // 输出行结束信号
    .O_tuser  			(m_aixs_tuser)	,  // 输出帧开始信号
    .O_tdata  			(m_aixs_tdata)	,  // 输出数据
    .O_tvalid 			(m_aixs_tvalid)	,  // 输出数据有效信号
    .O_tready   		(m_aixs_tready)	   // 输出数据准备好信号
);
 
data128_96 u_data128_96 (
    .I_tdata(m_aixs_tdata),
    .O_tdata(m_aixs_tdata_96)
);

// One to four white blocks identify the active phase; no brightness correction.
reg [8:0] phase_marker_x;
reg [9:0] phase_marker_y;
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        phase_marker_x <= 0;
        phase_marker_y <= 0;
    end else if (m_aixs_tuser) begin
        phase_marker_x <= m_aixs_tvalid ? 9'd1 : 9'd0;
        phase_marker_y <= 0;
    end else if (m_aixs_tvalid) begin
        if (phase_marker_x == 9'd319) begin
            phase_marker_x <= 0;
            phase_marker_y <= phase_marker_y + 10'd1;
        end else phase_marker_x <= phase_marker_x + 9'd1;
    end
end
wire phase_marker_area = TEST_BAYER_PHASES && (phase_marker_y < 10'd24) && (phase_marker_x < 9'd32);
wire phase_marker_white = (phase_marker_y >= 10'd4) && (phase_marker_y < 10'd20)
                         && (phase_marker_x[4:3] <= bayer_phase) && (phase_marker_x[2:0] < 3'd4);
wire [95:0] phase_compare_rgb = phase_marker_area
                            ? {4{phase_marker_white ? 24'hffffff : 24'h000000}}
                            : m_aixs_tdata_96;

// Diagnostics only: applied Q16 gains and eligible samples from the last frame.
wire [19:0] awb_debug_r_gain, awb_debug_b_gain, awb_debug_count;
awb #(
    .IMG_HEIGHT(720 ),
    .IMG_WIDTH (1280),
    .ENABLE_AWB(1),
    .FIXED_WHITE_REFERENCE(1) // Hold measured white-paper gains for the color comparison.
) u_awb (
    .I_clk   (axi4s_video_aclk),
    .I_rst_n (I_rst_n),
    .I_tlast (m_aixs_tlast),
    .I_tuser (m_aixs_tuser),
    .I_tdata (phase_compare_rgb),
    .I_tvalid(m_aixs_tvalid),
    .I_tready(m_aixs_tready),
    .O_tlast (awb_O_tlast),
    .O_tuser (awb_O_tuser),
    .O_tdata (awb_O_tdata),
    .O_tvalid(awb_O_tvalid),
    .O_tready(awb_O_tready),
    .O_debug_r_gain(awb_debug_r_gain),
    .O_debug_b_gain(awb_debug_b_gain),
    .O_debug_count(awb_debug_count)
);

  // Display LUT: 4x/soft shoulder/sRGB/contrast; shadow toe below input 24.
  // LUT only: no new pixel or frame-control latency.
  function [7:0] linear_to_srgb;
      input [7:0] linear_value;
      begin
          case (linear_value)
            8'd0: linear_to_srgb = 8'd0; 8'd1: linear_to_srgb = 8'd22; 8'd2: linear_to_srgb = 8'd34; 8'd3: linear_to_srgb = 8'd43;
            8'd4: linear_to_srgb = 8'd51; 8'd5: linear_to_srgb = 8'd59; 8'd6: linear_to_srgb = 8'd66; 8'd7: linear_to_srgb = 8'd73;
            8'd8: linear_to_srgb = 8'd80; 8'd9: linear_to_srgb = 8'd86; 8'd10: linear_to_srgb = 8'd92; 8'd11: linear_to_srgb = 8'd98;
            8'd12: linear_to_srgb = 8'd104; 8'd13: linear_to_srgb = 8'd111; 8'd14: linear_to_srgb = 8'd116; 8'd15: linear_to_srgb = 8'd121;
            8'd16: linear_to_srgb = 8'd127; 8'd17: linear_to_srgb = 8'd133; 8'd18: linear_to_srgb = 8'd138; 8'd19: linear_to_srgb = 8'd142;
            8'd20: linear_to_srgb = 8'd149; 8'd21: linear_to_srgb = 8'd153; 8'd22: linear_to_srgb = 8'd160; 8'd23: linear_to_srgb = 8'd164;
            8'd24: linear_to_srgb = 8'd169; 8'd25: linear_to_srgb = 8'd173; 8'd26: linear_to_srgb = 8'd176; 8'd27: linear_to_srgb = 8'd179;
            8'd28: linear_to_srgb = 8'd182; 8'd29: linear_to_srgb = 8'd185; 8'd30: linear_to_srgb = 8'd188; 8'd31: linear_to_srgb = 8'd191;
            8'd32: linear_to_srgb = 8'd194; 8'd33: linear_to_srgb = 8'd195; 8'd34: linear_to_srgb = 8'd196; 8'd35: linear_to_srgb = 8'd197;
            8'd36: linear_to_srgb = 8'd198; 8'd37: linear_to_srgb = 8'd199; 8'd38: linear_to_srgb = 8'd200; 8'd39: linear_to_srgb = 8'd201;
            8'd40: linear_to_srgb = 8'd202; 8'd41: linear_to_srgb = 8'd203; 8'd42: linear_to_srgb = 8'd204; 8'd43: linear_to_srgb = 8'd205;
            8'd44: linear_to_srgb = 8'd206; 8'd45: linear_to_srgb = 8'd206; 8'd46: linear_to_srgb = 8'd207; 8'd47: linear_to_srgb = 8'd208;
            8'd48: linear_to_srgb = 8'd209; 8'd49: linear_to_srgb = 8'd210; 8'd50: linear_to_srgb = 8'd210; 8'd51: linear_to_srgb = 8'd211;
            8'd52: linear_to_srgb = 8'd212; 8'd53: linear_to_srgb = 8'd212; 8'd54: linear_to_srgb = 8'd213; 8'd55: linear_to_srgb = 8'd214;
            8'd56: linear_to_srgb = 8'd215; 8'd57: linear_to_srgb = 8'd215; 8'd58: linear_to_srgb = 8'd216; 8'd59: linear_to_srgb = 8'd216;
            8'd60: linear_to_srgb = 8'd217; 8'd61: linear_to_srgb = 8'd218; 8'd62: linear_to_srgb = 8'd218; 8'd63: linear_to_srgb = 8'd219;
            8'd64: linear_to_srgb = 8'd219; 8'd65: linear_to_srgb = 8'd220; 8'd66: linear_to_srgb = 8'd221; 8'd67: linear_to_srgb = 8'd221;
            8'd68: linear_to_srgb = 8'd222; 8'd69: linear_to_srgb = 8'd222; 8'd70: linear_to_srgb = 8'd223; 8'd71: linear_to_srgb = 8'd223;
            8'd72: linear_to_srgb = 8'd224; 8'd73: linear_to_srgb = 8'd224; 8'd74: linear_to_srgb = 8'd225; 8'd75: linear_to_srgb = 8'd225;
            8'd76: linear_to_srgb = 8'd225; 8'd77: linear_to_srgb = 8'd225; 8'd78: linear_to_srgb = 8'd226; 8'd79: linear_to_srgb = 8'd226;
            8'd80: linear_to_srgb = 8'd227; 8'd81: linear_to_srgb = 8'd227; 8'd82: linear_to_srgb = 8'd227; 8'd83: linear_to_srgb = 8'd228;
            8'd84: linear_to_srgb = 8'd228; 8'd85: linear_to_srgb = 8'd229; 8'd86: linear_to_srgb = 8'd229; 8'd87: linear_to_srgb = 8'd229;
            8'd88: linear_to_srgb = 8'd230; 8'd89: linear_to_srgb = 8'd230; 8'd90: linear_to_srgb = 8'd231; 8'd91: linear_to_srgb = 8'd231;
            8'd92: linear_to_srgb = 8'd231; 8'd93: linear_to_srgb = 8'd232; 8'd94: linear_to_srgb = 8'd232; 8'd95: linear_to_srgb = 8'd232;
            8'd96: linear_to_srgb = 8'd233; 8'd97: linear_to_srgb = 8'd233; 8'd98: linear_to_srgb = 8'd234; 8'd99: linear_to_srgb = 8'd234;
            8'd100: linear_to_srgb = 8'd234; 8'd101: linear_to_srgb = 8'd235; 8'd102: linear_to_srgb = 8'd235; 8'd103: linear_to_srgb = 8'd235;
            8'd104: linear_to_srgb = 8'd235; 8'd105: linear_to_srgb = 8'd235; 8'd106: linear_to_srgb = 8'd235; 8'd107: linear_to_srgb = 8'd235;
            8'd108: linear_to_srgb = 8'd236; 8'd109: linear_to_srgb = 8'd236; 8'd110: linear_to_srgb = 8'd236; 8'd111: linear_to_srgb = 8'd237;
            8'd112: linear_to_srgb = 8'd237; 8'd113: linear_to_srgb = 8'd237; 8'd114: linear_to_srgb = 8'd237; 8'd115: linear_to_srgb = 8'd238;
            8'd116: linear_to_srgb = 8'd238; 8'd117: linear_to_srgb = 8'd238; 8'd118: linear_to_srgb = 8'd239; 8'd119: linear_to_srgb = 8'd239;
            8'd120: linear_to_srgb = 8'd239; 8'd121: linear_to_srgb = 8'd239; 8'd122: linear_to_srgb = 8'd240; 8'd123: linear_to_srgb = 8'd240;
            8'd124: linear_to_srgb = 8'd240; 8'd125: linear_to_srgb = 8'd240; 8'd126: linear_to_srgb = 8'd241; 8'd127: linear_to_srgb = 8'd241;
            8'd128: linear_to_srgb = 8'd241; 8'd129: linear_to_srgb = 8'd241; 8'd130: linear_to_srgb = 8'd241; 8'd131: linear_to_srgb = 8'd241;
            8'd132: linear_to_srgb = 8'd241; 8'd133: linear_to_srgb = 8'd241; 8'd134: linear_to_srgb = 8'd241; 8'd135: linear_to_srgb = 8'd242;
            8'd136: linear_to_srgb = 8'd242; 8'd137: linear_to_srgb = 8'd242; 8'd138: linear_to_srgb = 8'd242; 8'd139: linear_to_srgb = 8'd243;
            8'd140: linear_to_srgb = 8'd243; 8'd141: linear_to_srgb = 8'd243; 8'd142: linear_to_srgb = 8'd243; 8'd143: linear_to_srgb = 8'd243;
            8'd144: linear_to_srgb = 8'd244; 8'd145: linear_to_srgb = 8'd244; 8'd146: linear_to_srgb = 8'd244; 8'd147: linear_to_srgb = 8'd244;
            8'd148: linear_to_srgb = 8'd244; 8'd149: linear_to_srgb = 8'd245; 8'd150: linear_to_srgb = 8'd245; 8'd151: linear_to_srgb = 8'd245;
            8'd152: linear_to_srgb = 8'd245; 8'd153: linear_to_srgb = 8'd245; 8'd154: linear_to_srgb = 8'd246; 8'd155: linear_to_srgb = 8'd246;
            8'd156: linear_to_srgb = 8'd246; 8'd157: linear_to_srgb = 8'd246; 8'd158: linear_to_srgb = 8'd246; 8'd159: linear_to_srgb = 8'd246;
            8'd160: linear_to_srgb = 8'd246; 8'd161: linear_to_srgb = 8'd246; 8'd162: linear_to_srgb = 8'd246; 8'd163: linear_to_srgb = 8'd246;
            8'd164: linear_to_srgb = 8'd246; 8'd165: linear_to_srgb = 8'd246; 8'd166: linear_to_srgb = 8'd247; 8'd167: linear_to_srgb = 8'd247;
            8'd168: linear_to_srgb = 8'd247; 8'd169: linear_to_srgb = 8'd247; 8'd170: linear_to_srgb = 8'd247; 8'd171: linear_to_srgb = 8'd247;
            8'd172: linear_to_srgb = 8'd248; 8'd173: linear_to_srgb = 8'd248; 8'd174: linear_to_srgb = 8'd248; 8'd175: linear_to_srgb = 8'd248;
            8'd176: linear_to_srgb = 8'd248; 8'd177: linear_to_srgb = 8'd248; 8'd178: linear_to_srgb = 8'd248; 8'd179: linear_to_srgb = 8'd249;
            8'd180: linear_to_srgb = 8'd249; 8'd181: linear_to_srgb = 8'd249; 8'd182: linear_to_srgb = 8'd249; 8'd183: linear_to_srgb = 8'd249;
            8'd184: linear_to_srgb = 8'd249; 8'd185: linear_to_srgb = 8'd249; 8'd186: linear_to_srgb = 8'd250; 8'd187: linear_to_srgb = 8'd250;
            8'd188: linear_to_srgb = 8'd250; 8'd189: linear_to_srgb = 8'd250; 8'd190: linear_to_srgb = 8'd250; 8'd191: linear_to_srgb = 8'd250;
            8'd192: linear_to_srgb = 8'd250; 8'd193: linear_to_srgb = 8'd250; 8'd194: linear_to_srgb = 8'd250; 8'd195: linear_to_srgb = 8'd250;
            8'd196: linear_to_srgb = 8'd250; 8'd197: linear_to_srgb = 8'd250; 8'd198: linear_to_srgb = 8'd250; 8'd199: linear_to_srgb = 8'd250;
            8'd200: linear_to_srgb = 8'd250; 8'd201: linear_to_srgb = 8'd251; 8'd202: linear_to_srgb = 8'd251; 8'd203: linear_to_srgb = 8'd251;
            8'd204: linear_to_srgb = 8'd251; 8'd205: linear_to_srgb = 8'd251; 8'd206: linear_to_srgb = 8'd251; 8'd207: linear_to_srgb = 8'd251;
            8'd208: linear_to_srgb = 8'd251; 8'd209: linear_to_srgb = 8'd251; 8'd210: linear_to_srgb = 8'd252; 8'd211: linear_to_srgb = 8'd252;
            8'd212: linear_to_srgb = 8'd252; 8'd213: linear_to_srgb = 8'd252; 8'd214: linear_to_srgb = 8'd252; 8'd215: linear_to_srgb = 8'd252;
            8'd216: linear_to_srgb = 8'd252; 8'd217: linear_to_srgb = 8'd252; 8'd218: linear_to_srgb = 8'd252; 8'd219: linear_to_srgb = 8'd253;
            8'd220: linear_to_srgb = 8'd253; 8'd221: linear_to_srgb = 8'd253; 8'd222: linear_to_srgb = 8'd253; 8'd223: linear_to_srgb = 8'd253;
            8'd224: linear_to_srgb = 8'd253; 8'd225: linear_to_srgb = 8'd253; 8'd226: linear_to_srgb = 8'd253; 8'd227: linear_to_srgb = 8'd253;
            8'd228: linear_to_srgb = 8'd253; 8'd229: linear_to_srgb = 8'd253; 8'd230: linear_to_srgb = 8'd253; 8'd231: linear_to_srgb = 8'd253;
            8'd232: linear_to_srgb = 8'd253; 8'd233: linear_to_srgb = 8'd253; 8'd234: linear_to_srgb = 8'd253; 8'd235: linear_to_srgb = 8'd253;
            8'd236: linear_to_srgb = 8'd253; 8'd237: linear_to_srgb = 8'd253; 8'd238: linear_to_srgb = 8'd253; 8'd239: linear_to_srgb = 8'd254;
            8'd240: linear_to_srgb = 8'd254; 8'd241: linear_to_srgb = 8'd254; 8'd242: linear_to_srgb = 8'd254; 8'd243: linear_to_srgb = 8'd254;
            8'd244: linear_to_srgb = 8'd254; 8'd245: linear_to_srgb = 8'd254; 8'd246: linear_to_srgb = 8'd254; 8'd247: linear_to_srgb = 8'd254;
            8'd248: linear_to_srgb = 8'd254; 8'd249: linear_to_srgb = 8'd254; 8'd250: linear_to_srgb = 8'd254; 8'd251: linear_to_srgb = 8'd254;
            8'd252: linear_to_srgb = 8'd254; 8'd253: linear_to_srgb = 8'd254; 8'd254: linear_to_srgb = 8'd254; 8'd255: linear_to_srgb = 8'd255;
            default: linear_to_srgb = 8'd0;
          endcase
      end
  endfunction
  // Empirical linear-RGB matrix trial, NOT a color-chart calibration.
  // Q3 rows: [10,-2,0], [-1,9,0], [0,-2,10]. Each row sums to 8.
  // Neutral RGB is unchanged. Clip signed results before the existing LUT.
  function [7:0] clamp_matrix_q3;
      input signed [12:0] numerator;
      reg signed [12:0] rounded_value;
      begin
          rounded_value = (numerator + 13'sd4) >>> 3;
          if (rounded_value < 13'sd0) clamp_matrix_q3 = 8'd0;
          else if (rounded_value > 13'sd255) clamp_matrix_q3 = 8'd255;
          else clamp_matrix_q3 = rounded_value[7:0];
      end
  endfunction
  wire [95:0] corrected_linear_rgb;
  genvar matrix_pixel;
  generate for (matrix_pixel = 0; matrix_pixel < 4; matrix_pixel = matrix_pixel + 1) begin : linear_color_matrix
      wire signed [12:0] r = $signed({5'd0,awb_O_tdata[matrix_pixel*24+16 +: 8]});
      wire signed [12:0] g = $signed({5'd0,awb_O_tdata[matrix_pixel*24+8 +: 8]});
      wire signed [12:0] b = $signed({5'd0,awb_O_tdata[matrix_pixel*24 +: 8]});
      wire signed [12:0] red_numerator = (r <<< 3) + (r <<< 1) - (g <<< 1);
      wire signed [12:0] green_numerator = (g <<< 3) + g - r;
      wire signed [12:0] blue_numerator = (b <<< 3) + (b <<< 1) - (g <<< 1);
      assign corrected_linear_rgb[matrix_pixel*24 +: 24] = {
          clamp_matrix_q3(red_numerator), clamp_matrix_q3(green_numerator),
          clamp_matrix_q3(blue_numerator)};
  end endgenerate
  // Weak-light 1.5x LINEAR gain composed into a constant LUT.
  // Strong light uses the accepted LUT unchanged; black stays zero.
  reg strong_frame;
  always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
      if(!I_rst_n) strong_frame<=1;
      else if(awb_O_tuser) strong_frame<=I_strong_light;
  end
  wire strong_encode=awb_O_tuser ? I_strong_light : strong_frame;
  assign O_strong_light=strong_frame;
  function [7:0] weak_linear_to_srgb;
      input [7:0] value;
      begin
          case(value)
              8'd0: weak_linear_to_srgb=8'd0;
              8'd1: weak_linear_to_srgb=8'd34;
              8'd2: weak_linear_to_srgb=8'd43;
              8'd3: weak_linear_to_srgb=8'd59;
              8'd4: weak_linear_to_srgb=8'd66;
              8'd5: weak_linear_to_srgb=8'd80;
              8'd6: weak_linear_to_srgb=8'd86;
              8'd7: weak_linear_to_srgb=8'd98;
              8'd8: weak_linear_to_srgb=8'd104;
              8'd9: weak_linear_to_srgb=8'd116;
              8'd10: weak_linear_to_srgb=8'd121;
              8'd11: weak_linear_to_srgb=8'd133;
              8'd12: weak_linear_to_srgb=8'd138;
              8'd13: weak_linear_to_srgb=8'd149;
              8'd14: weak_linear_to_srgb=8'd153;
              8'd15: weak_linear_to_srgb=8'd164;
              8'd16: weak_linear_to_srgb=8'd169;
              8'd17: weak_linear_to_srgb=8'd176;
              8'd18: weak_linear_to_srgb=8'd179;
              8'd19: weak_linear_to_srgb=8'd185;
              8'd20: weak_linear_to_srgb=8'd188;
              8'd21: weak_linear_to_srgb=8'd194;
              8'd22: weak_linear_to_srgb=8'd195;
              8'd23: weak_linear_to_srgb=8'd197;
              8'd24: weak_linear_to_srgb=8'd198;
              8'd25: weak_linear_to_srgb=8'd200;
              8'd26: weak_linear_to_srgb=8'd201;
              8'd27: weak_linear_to_srgb=8'd203;
              8'd28: weak_linear_to_srgb=8'd204;
              8'd29: weak_linear_to_srgb=8'd206;
              8'd30: weak_linear_to_srgb=8'd206;
              8'd31: weak_linear_to_srgb=8'd208;
              8'd32: weak_linear_to_srgb=8'd209;
              8'd33: weak_linear_to_srgb=8'd210;
              8'd34: weak_linear_to_srgb=8'd211;
              8'd35: weak_linear_to_srgb=8'd212;
              8'd36: weak_linear_to_srgb=8'd213;
              8'd37: weak_linear_to_srgb=8'd215;
              8'd38: weak_linear_to_srgb=8'd215;
              8'd39: weak_linear_to_srgb=8'd216;
              8'd40: weak_linear_to_srgb=8'd217;
              8'd41: weak_linear_to_srgb=8'd218;
              8'd42: weak_linear_to_srgb=8'd219;
              8'd43: weak_linear_to_srgb=8'd220;
              8'd44: weak_linear_to_srgb=8'd221;
              8'd45: weak_linear_to_srgb=8'd222;
              8'd46: weak_linear_to_srgb=8'd222;
              8'd47: weak_linear_to_srgb=8'd223;
              8'd48: weak_linear_to_srgb=8'd224;
              8'd49: weak_linear_to_srgb=8'd225;
              8'd50: weak_linear_to_srgb=8'd225;
              8'd51: weak_linear_to_srgb=8'd225;
              8'd52: weak_linear_to_srgb=8'd226;
              8'd53: weak_linear_to_srgb=8'd227;
              8'd54: weak_linear_to_srgb=8'd227;
              8'd55: weak_linear_to_srgb=8'd228;
              8'd56: weak_linear_to_srgb=8'd228;
              8'd57: weak_linear_to_srgb=8'd229;
              8'd58: weak_linear_to_srgb=8'd229;
              8'd59: weak_linear_to_srgb=8'd230;
              8'd60: weak_linear_to_srgb=8'd231;
              8'd61: weak_linear_to_srgb=8'd231;
              8'd62: weak_linear_to_srgb=8'd232;
              8'd63: weak_linear_to_srgb=8'd232;
              8'd64: weak_linear_to_srgb=8'd233;
              8'd65: weak_linear_to_srgb=8'd234;
              8'd66: weak_linear_to_srgb=8'd234;
              8'd67: weak_linear_to_srgb=8'd235;
              8'd68: weak_linear_to_srgb=8'd235;
              8'd69: weak_linear_to_srgb=8'd235;
              8'd70: weak_linear_to_srgb=8'd235;
              8'd71: weak_linear_to_srgb=8'd235;
              8'd72: weak_linear_to_srgb=8'd236;
              8'd73: weak_linear_to_srgb=8'd236;
              8'd74: weak_linear_to_srgb=8'd237;
              8'd75: weak_linear_to_srgb=8'd237;
              8'd76: weak_linear_to_srgb=8'd237;
              8'd77: weak_linear_to_srgb=8'd238;
              8'd78: weak_linear_to_srgb=8'd238;
              8'd79: weak_linear_to_srgb=8'd239;
              8'd80: weak_linear_to_srgb=8'd239;
              8'd81: weak_linear_to_srgb=8'd240;
              8'd82: weak_linear_to_srgb=8'd240;
              8'd83: weak_linear_to_srgb=8'd240;
              8'd84: weak_linear_to_srgb=8'd241;
              8'd85: weak_linear_to_srgb=8'd241;
              8'd86: weak_linear_to_srgb=8'd241;
              8'd87: weak_linear_to_srgb=8'd241;
              8'd88: weak_linear_to_srgb=8'd241;
              8'd89: weak_linear_to_srgb=8'd241;
              8'd90: weak_linear_to_srgb=8'd242;
              8'd91: weak_linear_to_srgb=8'd242;
              8'd92: weak_linear_to_srgb=8'd242;
              8'd93: weak_linear_to_srgb=8'd243;
              8'd94: weak_linear_to_srgb=8'd243;
              8'd95: weak_linear_to_srgb=8'd243;
              8'd96: weak_linear_to_srgb=8'd244;
              8'd97: weak_linear_to_srgb=8'd244;
              8'd98: weak_linear_to_srgb=8'd244;
              8'd99: weak_linear_to_srgb=8'd245;
              8'd100: weak_linear_to_srgb=8'd245;
              8'd101: weak_linear_to_srgb=8'd245;
              8'd102: weak_linear_to_srgb=8'd245;
              8'd103: weak_linear_to_srgb=8'd246;
              8'd104: weak_linear_to_srgb=8'd246;
              8'd105: weak_linear_to_srgb=8'd246;
              8'd106: weak_linear_to_srgb=8'd246;
              8'd107: weak_linear_to_srgb=8'd246;
              8'd108: weak_linear_to_srgb=8'd246;
              8'd109: weak_linear_to_srgb=8'd246;
              8'd110: weak_linear_to_srgb=8'd246;
              8'd111: weak_linear_to_srgb=8'd247;
              8'd112: weak_linear_to_srgb=8'd247;
              8'd113: weak_linear_to_srgb=8'd247;
              8'd114: weak_linear_to_srgb=8'd247;
              8'd115: weak_linear_to_srgb=8'd248;
              8'd116: weak_linear_to_srgb=8'd248;
              8'd117: weak_linear_to_srgb=8'd248;
              8'd118: weak_linear_to_srgb=8'd248;
              8'd119: weak_linear_to_srgb=8'd249;
              8'd120: weak_linear_to_srgb=8'd249;
              8'd121: weak_linear_to_srgb=8'd249;
              8'd122: weak_linear_to_srgb=8'd249;
              8'd123: weak_linear_to_srgb=8'd249;
              8'd124: weak_linear_to_srgb=8'd250;
              8'd125: weak_linear_to_srgb=8'd250;
              8'd126: weak_linear_to_srgb=8'd250;
              8'd127: weak_linear_to_srgb=8'd250;
              8'd128: weak_linear_to_srgb=8'd250;
              8'd129: weak_linear_to_srgb=8'd250;
              8'd130: weak_linear_to_srgb=8'd250;
              8'd131: weak_linear_to_srgb=8'd250;
              8'd132: weak_linear_to_srgb=8'd250;
              8'd133: weak_linear_to_srgb=8'd250;
              8'd134: weak_linear_to_srgb=8'd251;
              8'd135: weak_linear_to_srgb=8'd251;
              8'd136: weak_linear_to_srgb=8'd251;
              8'd137: weak_linear_to_srgb=8'd251;
              8'd138: weak_linear_to_srgb=8'd251;
              8'd139: weak_linear_to_srgb=8'd251;
              8'd140: weak_linear_to_srgb=8'd252;
              8'd141: weak_linear_to_srgb=8'd252;
              8'd142: weak_linear_to_srgb=8'd252;
              8'd143: weak_linear_to_srgb=8'd252;
              8'd144: weak_linear_to_srgb=8'd252;
              8'd145: weak_linear_to_srgb=8'd252;
              8'd146: weak_linear_to_srgb=8'd253;
              8'd147: weak_linear_to_srgb=8'd253;
              8'd148: weak_linear_to_srgb=8'd253;
              8'd149: weak_linear_to_srgb=8'd253;
              8'd150: weak_linear_to_srgb=8'd253;
              8'd151: weak_linear_to_srgb=8'd253;
              8'd152: weak_linear_to_srgb=8'd253;
              8'd153: weak_linear_to_srgb=8'd253;
              8'd154: weak_linear_to_srgb=8'd253;
              8'd155: weak_linear_to_srgb=8'd253;
              8'd156: weak_linear_to_srgb=8'd253;
              8'd157: weak_linear_to_srgb=8'd253;
              8'd158: weak_linear_to_srgb=8'd253;
              8'd159: weak_linear_to_srgb=8'd254;
              8'd160: weak_linear_to_srgb=8'd254;
              8'd161: weak_linear_to_srgb=8'd254;
              8'd162: weak_linear_to_srgb=8'd254;
              8'd163: weak_linear_to_srgb=8'd254;
              8'd164: weak_linear_to_srgb=8'd254;
              8'd165: weak_linear_to_srgb=8'd254;
              8'd166: weak_linear_to_srgb=8'd254;
              8'd167: weak_linear_to_srgb=8'd254;
              8'd168: weak_linear_to_srgb=8'd254;
              8'd169: weak_linear_to_srgb=8'd254;
              8'd170: weak_linear_to_srgb=8'd255;
              8'd171: weak_linear_to_srgb=8'd255;
              8'd172: weak_linear_to_srgb=8'd255;
              8'd173: weak_linear_to_srgb=8'd255;
              8'd174: weak_linear_to_srgb=8'd255;
              8'd175: weak_linear_to_srgb=8'd255;
              8'd176: weak_linear_to_srgb=8'd255;
              8'd177: weak_linear_to_srgb=8'd255;
              8'd178: weak_linear_to_srgb=8'd255;
              8'd179: weak_linear_to_srgb=8'd255;
              8'd180: weak_linear_to_srgb=8'd255;
              8'd181: weak_linear_to_srgb=8'd255;
              8'd182: weak_linear_to_srgb=8'd255;
              8'd183: weak_linear_to_srgb=8'd255;
              8'd184: weak_linear_to_srgb=8'd255;
              8'd185: weak_linear_to_srgb=8'd255;
              8'd186: weak_linear_to_srgb=8'd255;
              8'd187: weak_linear_to_srgb=8'd255;
              8'd188: weak_linear_to_srgb=8'd255;
              8'd189: weak_linear_to_srgb=8'd255;
              8'd190: weak_linear_to_srgb=8'd255;
              8'd191: weak_linear_to_srgb=8'd255;
              8'd192: weak_linear_to_srgb=8'd255;
              8'd193: weak_linear_to_srgb=8'd255;
              8'd194: weak_linear_to_srgb=8'd255;
              8'd195: weak_linear_to_srgb=8'd255;
              8'd196: weak_linear_to_srgb=8'd255;
              8'd197: weak_linear_to_srgb=8'd255;
              8'd198: weak_linear_to_srgb=8'd255;
              8'd199: weak_linear_to_srgb=8'd255;
              8'd200: weak_linear_to_srgb=8'd255;
              8'd201: weak_linear_to_srgb=8'd255;
              8'd202: weak_linear_to_srgb=8'd255;
              8'd203: weak_linear_to_srgb=8'd255;
              8'd204: weak_linear_to_srgb=8'd255;
              8'd205: weak_linear_to_srgb=8'd255;
              8'd206: weak_linear_to_srgb=8'd255;
              8'd207: weak_linear_to_srgb=8'd255;
              8'd208: weak_linear_to_srgb=8'd255;
              8'd209: weak_linear_to_srgb=8'd255;
              8'd210: weak_linear_to_srgb=8'd255;
              8'd211: weak_linear_to_srgb=8'd255;
              8'd212: weak_linear_to_srgb=8'd255;
              8'd213: weak_linear_to_srgb=8'd255;
              8'd214: weak_linear_to_srgb=8'd255;
              8'd215: weak_linear_to_srgb=8'd255;
              8'd216: weak_linear_to_srgb=8'd255;
              8'd217: weak_linear_to_srgb=8'd255;
              8'd218: weak_linear_to_srgb=8'd255;
              8'd219: weak_linear_to_srgb=8'd255;
              8'd220: weak_linear_to_srgb=8'd255;
              8'd221: weak_linear_to_srgb=8'd255;
              8'd222: weak_linear_to_srgb=8'd255;
              8'd223: weak_linear_to_srgb=8'd255;
              8'd224: weak_linear_to_srgb=8'd255;
              8'd225: weak_linear_to_srgb=8'd255;
              8'd226: weak_linear_to_srgb=8'd255;
              8'd227: weak_linear_to_srgb=8'd255;
              8'd228: weak_linear_to_srgb=8'd255;
              8'd229: weak_linear_to_srgb=8'd255;
              8'd230: weak_linear_to_srgb=8'd255;
              8'd231: weak_linear_to_srgb=8'd255;
              8'd232: weak_linear_to_srgb=8'd255;
              8'd233: weak_linear_to_srgb=8'd255;
              8'd234: weak_linear_to_srgb=8'd255;
              8'd235: weak_linear_to_srgb=8'd255;
              8'd236: weak_linear_to_srgb=8'd255;
              8'd237: weak_linear_to_srgb=8'd255;
              8'd238: weak_linear_to_srgb=8'd255;
              8'd239: weak_linear_to_srgb=8'd255;
              8'd240: weak_linear_to_srgb=8'd255;
              8'd241: weak_linear_to_srgb=8'd255;
              8'd242: weak_linear_to_srgb=8'd255;
              8'd243: weak_linear_to_srgb=8'd255;
              8'd244: weak_linear_to_srgb=8'd255;
              8'd245: weak_linear_to_srgb=8'd255;
              8'd246: weak_linear_to_srgb=8'd255;
              8'd247: weak_linear_to_srgb=8'd255;
              8'd248: weak_linear_to_srgb=8'd255;
              8'd249: weak_linear_to_srgb=8'd255;
              8'd250: weak_linear_to_srgb=8'd255;
              8'd251: weak_linear_to_srgb=8'd255;
              8'd252: weak_linear_to_srgb=8'd255;
              8'd253: weak_linear_to_srgb=8'd255;
              8'd254: weak_linear_to_srgb=8'd255;
              8'd255: weak_linear_to_srgb=8'd255;
              default: weak_linear_to_srgb=0;
          endcase
      end
  endfunction
  wire [95:0] srgb_output;
  genvar srgb_channel;
  generate for (srgb_channel = 0; srgb_channel < 12; srgb_channel = srgb_channel + 1) begin : encode_srgb
      assign srgb_output[srgb_channel*8 +: 8] = strong_encode ? linear_to_srgb(corrected_linear_rgb[srgb_channel*8 +: 8])
          : weak_linear_to_srgb(corrected_linear_rgb[srgb_channel*8 +: 8]);
  end endgenerate

// RAW10 meter: positional planes, independent of the assumed Bayer color names.
// Every 1280x720 frame contributes 230400 samples per plane.
reg meter_raw_odd_row;
reg [31:0] meter_raw_beats;
wire meter_complete = (meter_raw_beats == 32'd230400);
reg meter_ready;
reg [31:0] meter_sum[0:3];
reg [9:0] meter_min[0:3], meter_max[0:3];
reg [9:0] meter_mean_out[0:3], meter_min_out[0:3], meter_max_out[0:3];
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        meter_raw_odd_row <= 0;
        meter_raw_beats <= 0;
        meter_ready <= 0;
    end else if (I_tuser) begin
        meter_raw_odd_row <= 0;
        meter_raw_beats <= I_tvalid ? 32'd1 : 32'd0;
        meter_ready <= meter_complete;
    end else if (I_tvalid) begin
        meter_raw_beats <= meter_raw_beats + 32'd1;
        if (I_tlast) meter_raw_odd_row <= ~meter_raw_odd_row;
    end
end
genvar meter_plane;
generate for (meter_plane = 0; meter_plane < 4; meter_plane = meter_plane + 1) begin : raw_plane_meter
    wire [9:0] a = I_tdata[(meter_plane%2)*10 +: 10];
    wire [9:0] b = I_tdata[(meter_plane%2)*10+20 +: 10];
    wire [10:0] pair_sum = {1'b0,a} + {1'b0,b};
    wire [9:0] pair_min = (a < b) ? a : b;
    wire [9:0] pair_max = (a > b) ? a : b;
    wire active_plane = I_tuser ? (meter_plane < 2) : (meter_raw_odd_row == (meter_plane/2));
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            meter_sum[meter_plane] <= 0;
            meter_min[meter_plane] <= 10'd1023;
            meter_max[meter_plane] <= 0;
            meter_mean_out[meter_plane] <= 0;
            meter_min_out[meter_plane] <= 0;
            meter_max_out[meter_plane] <= 0;
        end else if (I_tuser) begin
            if (meter_complete) begin
                meter_mean_out[meter_plane] <= meter_sum[meter_plane] / 32'd230400;
                meter_min_out[meter_plane] <= meter_min[meter_plane];
                meter_max_out[meter_plane] <= meter_max[meter_plane];
            end
            meter_sum[meter_plane] <= (I_tvalid && active_plane) ? {21'd0,pair_sum} : 32'd0;
            meter_min[meter_plane] <= (I_tvalid && active_plane) ? pair_min : 10'd1023;
            meter_max[meter_plane] <= (I_tvalid && active_plane) ? pair_max : 10'd0;
        end else if (I_tvalid && active_plane) begin
            meter_sum[meter_plane] <= meter_sum[meter_plane] + pair_sum;
            if (pair_min < meter_min[meter_plane]) meter_min[meter_plane] <= pair_min;
            if (pair_max > meter_max[meter_plane]) meter_max[meter_plane] <= pair_max;
        end
    end
end endgenerate

// Center 128x128 patch: compare AWB linear RGB with displayed RGB.
// Power-of-two sample count keeps averaging to a shift; no pixel-path delay.
reg [8:0] meter_display_x;
reg [9:0] meter_display_y;
wire color_patch_active = meter_display_x >= 9'd144 && meter_display_x < 9'd176
                       && meter_display_y >= 10'd296 && meter_display_y < 10'd424;
reg [12:0] color_patch_beats;
reg color_patch_ready;
reg [7:0] color_linear_mean[0:2], color_display_mean[0:2];
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        color_patch_beats <= 0;
        color_patch_ready <= 0;
    end else if (awb_O_tuser) begin
        color_patch_ready <= (color_patch_beats == 13'd4096);
        color_patch_beats <= 0;
    end else if (awb_O_tvalid && color_patch_active) begin
        color_patch_beats <= color_patch_beats + 13'd1;
    end
end
genvar color_channel;
generate for (color_channel = 0; color_channel < 3; color_channel = color_channel + 1) begin : color_patch_meter
    wire [9:0] linear_sum = {2'b00,awb_O_tdata[color_channel*8 +: 8]}
                         + {2'b00,awb_O_tdata[24+color_channel*8 +: 8]}
                         + {2'b00,awb_O_tdata[48+color_channel*8 +: 8]}
                         + {2'b00,awb_O_tdata[72+color_channel*8 +: 8]};
    wire [9:0] display_sum = {2'b00,srgb_output[color_channel*8 +: 8]}
                          + {2'b00,srgb_output[24+color_channel*8 +: 8]}
                          + {2'b00,srgb_output[48+color_channel*8 +: 8]}
                          + {2'b00,srgb_output[72+color_channel*8 +: 8]};
    reg [21:0] linear_total, display_total;
    always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            linear_total <= 0; display_total <= 0;
            color_linear_mean[color_channel] <= 0;
            color_display_mean[color_channel] <= 0;
        end else if (awb_O_tuser) begin
            if (color_patch_beats == 13'd4096) begin
                color_linear_mean[color_channel] <= linear_total[21:14];
                color_display_mean[color_channel] <= display_total[21:14];
            end
            linear_total <= 0; display_total <= 0;
        end else if (awb_O_tvalid && color_patch_active) begin
            linear_total <= linear_total + linear_sum;
            display_total <= display_total + display_sum;
        end
    end
end endgenerate

// Compact hexadecimal OSD, after sRGB so its black/white is not tone-mapped.
function [34:0] raw_meter_font;
    input [7:0] character;
    begin
        case (character)
            8'd48: raw_meter_font = 35'b01110100011001110101110011000101110;
            8'd49: raw_meter_font = 35'b00100011000010000100001000010001110;
            8'd50: raw_meter_font = 35'b01110100010000100010001000100011111;
            8'd51: raw_meter_font = 35'b11110000010000101110000010000111110;
            8'd52: raw_meter_font = 35'b00010001100101010010111110001000010;
            8'd53: raw_meter_font = 35'b11111100001000011110000010000111110;
            8'd54: raw_meter_font = 35'b01110100001000011110100011000101110;
            8'd55: raw_meter_font = 35'b11111000010001000100010000100001000;
            8'd56: raw_meter_font = 35'b01110100011000101110100011000101110;
            8'd57: raw_meter_font = 35'b01110100011000101111000010000101110;
            8'd65: raw_meter_font = 35'b01110100011000111111100011000110001;
            8'd66: raw_meter_font = 35'b11110100011000111110100011000111110;
            8'd67: raw_meter_font = 35'b01111100001000010000100001000001111;
            8'd68: raw_meter_font = 35'b11110100011000110001100011000111110;
            8'd69: raw_meter_font = 35'b11111100001000011110100001000011111;
            8'd70: raw_meter_font = 35'b11111100001000011110100001000010000;
            8'd80: raw_meter_font = 35'b11110100011000111110100001000010000;
            8'd86: raw_meter_font = 35'b10001100011000110001100010101000100;
            8'd71: raw_meter_font = 35'b01111100001000010111100011000101111;
            8'd77: raw_meter_font = 35'b10001110111010110101100011000110001;
            8'd73: raw_meter_font = 35'b01110001000010000100001000010001110;
            8'd78: raw_meter_font = 35'b10001110011010110011100011000110001;
            8'd88: raw_meter_font = 35'b10001100010101000100010101000110001;
            8'd76: raw_meter_font = 35'b10000100001000010000100001000011111;
            8'd79: raw_meter_font = 35'b01110100011000110001100011000101110;
            8'd82: raw_meter_font = 35'b11110100011000111110101001001010001;
            8'd84: raw_meter_font = 35'b11111001000010000100001000010000100;
            8'd85: raw_meter_font = 35'b10001100011000110001100011000101110;
            8'd87: raw_meter_font = 35'b10001100011000110101101011010101010;
            8'd45: raw_meter_font = 35'b00000000000000011111000000000000000;
            default: raw_meter_font = 35'd0;
        endcase
    end
endfunction
function [7:0] raw_meter_hex;
    input [3:0] digit;
    begin raw_meter_hex = (digit < 10) ? 8'd48 + digit : 8'd65 + digit - 8'd10; end
endfunction
function [23:0] raw_meter_pixel;
    input [10:0] x;
    input [9:0] y;
    reg [7:0] character;
    reg [34:0] glyph;
    reg [19:0] number;
    integer text_row, text_column, glyph_x, glyph_y;
    begin
        text_row = y / 16;
        text_column = x / 12;
        glyph_x = (x % 12) / 2;
        glyph_y = (y % 16) / 2;
        character = 8'd32;
        number = 0;
        if (text_row == 0) begin
            case (text_column)
                    0: character = 8'd80;
                    1: character = 8'd32;
                    2: character = 8'd65;
                    3: character = 8'd86;
                    4: character = 8'd71;
                    5: character = 8'd32;
                    6: character = 8'd77;
                    7: character = 8'd73;
                    8: character = 8'd78;
                    9: character = 8'd32;
                    10: character = 8'd77;
                    11: character = 8'd65;
                    12: character = 8'd88;
                default: character = 8'd32;
            endcase
        end else if (text_row < 5) begin
            if (text_column == 0) character = 8'd48 + text_row - 1;
            else if (text_column != 1 && text_column != 5 && text_column != 9) begin
                if (text_column < 5) number = meter_mean_out[text_row-1];
                else if (text_column < 9) number = meter_min_out[text_row-1];
                else number = meter_max_out[text_row-1];
                if (!meter_ready) character = 8'd45;
                else case (text_column)
                    2,6,10: character = raw_meter_hex({2'b00,number[9:8]});
                    3,7,11: character = raw_meter_hex(number[7:4]);
                    4,8,12: character = raw_meter_hex(number[3:0]);
                    default: character = 8'd32;
                endcase
            end
        end
        if (text_row == 6) begin
            case (text_column)
                0: character = 8'd67; // C LIN OUT
                2: character = 8'd76;
                3: character = 8'd73;
                4: character = 8'd78;
                6: character = 8'd79;
                7: character = 8'd85;
                8: character = 8'd84;
                default: character = 8'd32;
            endcase
        end else if (text_row >= 7 && text_row <= 9) begin
            if (text_column == 0) begin
                case (text_row)
                    7: character = 8'd82;
                    8: character = 8'd71;
                    9: character = 8'd66;
                endcase
            end else if ((text_column >= 2 && text_column <= 4)
                      || (text_column >= 6 && text_column <= 8)) begin
                if (text_column < 5) number = {2'b00,color_linear_mean[9-text_row]};
                else number = {2'b00,color_display_mean[9-text_row]};
                if (!color_patch_ready) character = 8'd45;
                else case (text_column)
                    2,6: character = 8'd48;
                    3,7: character = raw_meter_hex(number[7:4]);
                    4,8: character = raw_meter_hex(number[3:0]);
                endcase
            end
        end
        if (text_row == 11) begin
            case (text_column)
                0: character = 8'd87; // W GAIN COUNT
                2: character = 8'd71;
                3: character = 8'd65;
                4: character = 8'd73;
                5: character = 8'd78;
                7: character = 8'd67;
                8: character = 8'd79;
                9: character = 8'd85;
                10: character = 8'd78;
                11: character = 8'd84;
                default: character = 8'd32;
            endcase
        end else if (text_row == 12 || text_row == 13) begin
            number = (text_row == 12) ? awb_debug_r_gain : awb_debug_b_gain;
            case (text_column)
                0: character = (text_row == 12) ? 8'd82 : 8'd66;
                2: character = raw_meter_hex(number[19:16]);
                3: character = raw_meter_hex(number[15:12]);
                4: character = raw_meter_hex(number[11:8]);
                default: character = 8'd32;
            endcase
            if (text_row == 12) begin
                case (text_column)
                    7: character = raw_meter_hex(awb_debug_count[19:16]);
                    8: character = raw_meter_hex(awb_debug_count[15:12]);
                    9: character = raw_meter_hex(awb_debug_count[11:8]);
                    10: character = raw_meter_hex(awb_debug_count[7:4]);
                    11: character = raw_meter_hex(awb_debug_count[3:0]);
                endcase
            end
        end
        glyph = raw_meter_font(character);
        raw_meter_pixel = 24'h000000;
        if (glyph_x < 5 && glyph_y < 7)
            if (glyph[34-glyph_y*5-glyph_x]) raw_meter_pixel = 24'hffffff;
    end
endfunction
always @(posedge axi4s_video_aclk or negedge I_rst_n) begin
    if (!I_rst_n) begin
        meter_display_x <= 0; meter_display_y <= 0;
    end else if (awb_O_tuser) begin
        meter_display_x <= awb_O_tvalid ? 9'd1 : 9'd0;
        meter_display_y <= 0;
    end else if (awb_O_tvalid) begin
        if (meter_display_x == 9'd319) begin
            meter_display_x <= 0;
            meter_display_y <= (meter_display_y == 10'd719) ? 10'd0 : meter_display_y + 10'd1;
        end else meter_display_x <= meter_display_x + 9'd1;
    end
end
wire [95:0] display_input_rgb = srgb_output;
wire [95:0] metered_output;
genvar meter_pixel;
generate for (meter_pixel = 0; meter_pixel < 4; meter_pixel = meter_pixel + 1) begin : raw_meter_overlay
    wire [10:0] x = {meter_display_x,2'b00} + (3-meter_pixel);
    // Two-pixel yellow border OUTSIDE the measured x=576..703, y=296..423.
    // Statistics read srgb_output before this overlay, so the marker is excluded.
    wire color_patch_border = x >= 11'd574 && x < 11'd706
                           && meter_display_y >= 10'd294 && meter_display_y < 10'd426
                           && (x < 11'd576 || x >= 11'd704
                            || meter_display_y < 10'd296 || meter_display_y >= 10'd424);
    assign metered_output[meter_pixel*24 +: 24] = (ENABLE_DIAGNOSTICS && x < 11'd156 && meter_display_y < 10'd224)
                        ? raw_meter_pixel(x,meter_display_y)
                        : (ENABLE_DIAGNOSTICS && color_patch_border) ? 24'hffff00 : display_input_rgb[meter_pixel*24 +: 24];
end endgenerate

    data_96bit_to_128bit u_data_96bit_to_128bit(
        .I_clk              ( axi4s_video_aclk  ),
        .I_rst_n            ( I_rst_n           ),
	
        .I_96b_frame_start  ( awb_O_tuser	    ),
        .I_96b_valid        ( awb_O_tvalid 		),
        .I_96b_data         ( metered_output		),
	
        .O_128b_frame_start ( awb_O_tuser_128  	),
        .O_128b_valid       ( awb_O_tvalid_128 	),
        .O_128b_data        ( awb_O_tdata_128  	)
    );



//data96_128 u_data96_128 (
//    .I_tdata(awb_O_tdata),
//    .O_tdata(awb_O_tdata_128)
//);
	
endmodule
