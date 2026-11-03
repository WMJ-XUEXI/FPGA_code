`timescale 1ns/1ps
// No interactive calibration. Adjust these fixed board/panel settings if needed.
// One shared iterative divider maps both axes without a frame buffer.
module parking_touch_control #(
 parameter SWAP_XY=1, INVERT_X=1, INVERT_Y=0,
 parameter integer RAW_X_MIN=0, RAW_X_MAX=4095,
 parameter integer RAW_Y_MIN=0, RAW_Y_MAX=4095,
 // 0: TP_IRQ low; 1: pressure hint; 2: IRQ low OR pressure hint.
 parameter integer CONTACT_MODE=1
)(input clk,rst_n,poll_done,raw_valid,irq_n,enable,
 input [11:0] raw_x,raw_y,
 output wire [1:0] calibration,output wire detail,output wire [4:0] selected,
 output wire [43:0] sensitivity,
 output reg tap,output reg [8:0] pixel_x,pixel_y,
 output wire debug_contact,debug_held,output wire [1:0] debug_state,debug_stable,debug_released,debug_contact_mode);
 // Legacy ports retained only to avoid disturbing the main design interface.
 assign calibration=2'd3;assign detail=0;assign selected=0;
 assign sensitivity={22{2'd1}};
 localparam [11:0] X_LO=RAW_X_MIN,X_HI=RAW_X_MAX,Y_LO=RAW_Y_MIN,Y_HI=RAW_Y_MAX;
 localparam [11:0] X_SPAN=RAW_X_MAX>RAW_X_MIN ? RAW_X_MAX-RAW_X_MIN:1;
 localparam [11:0] Y_SPAN=RAW_Y_MAX>RAW_Y_MIN ? RAW_Y_MAX-RAW_Y_MIN:1;
 wire [11:0] axis_x=SWAP_XY ? raw_y:raw_x,axis_y=SWAP_XY ? raw_x:raw_y;
 wire [11:0] dx=axis_x<=X_LO ? 12'd0:axis_x>=X_HI ? X_SPAN:axis_x-X_LO;
 wire [11:0] dy=axis_y<=Y_LO ? 12'd0:axis_y>=Y_HI ? Y_SPAN:axis_y-Y_LO;
 wire [21:0] ext_x={10'd0,dx},ext_y={10'd0,dy};
 // 479 = 512-32-1; 319 = 256+64-1. Explicit shifts/adds, no '*' or '/'.
 wire [21:0] scaled_x=(ext_x<<9)-(ext_x<<5)-ext_x;
 wire [21:0] scaled_y=(ext_y<<8)+(ext_y<<6)-ext_y;
 wire contact=CONTACT_MODE==0 ? !irq_n:CONTACT_MODE==1 ? raw_valid:(!irq_n || raw_valid);
 reg held;reg [1:0] stable,released,state;
 assign debug_contact=contact;assign debug_held=held;assign debug_state=state;
 assign debug_stable=stable;assign debug_released=released;assign debug_contact_mode=CONTACT_MODE;
 reg [21:0] dividend,quotient,pending_y;
 reg [12:0] remainder;reg [11:0] divisor;reg [4:0] bits_left;
 wire [13:0] trial={remainder,dividend[21]};
 wire subtract_ok=trial>={2'b0,divisor};
 wire [21:0] next_quotient={quotient[20:0],subtract_ok};
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
 tap<=0;pixel_x<=0;pixel_y<=0;held<=0;stable<=0;released<=0;state<=0;
 dividend<=0;quotient<=0;pending_y<=0;remainder<=0;divisor<=1;bits_left<=0;
 end else begin
 tap<=0;
 if(poll_done)begin
 if(!contact)begin
 stable<=0;
 if(released==2)begin held<=0;released<=0;end else released<=released+1'b1;
 end else begin
 released<=0;
 if(!held && state==0 && enable)begin
 if(stable!=0)begin
 held<=1;stable<=0;dividend<=scaled_x;pending_y<=scaled_y;
 divisor<=X_SPAN;remainder<=0;quotient<=0;bits_left<=22;state<=1;
 end else stable<=1;
 end
 end
 end
 if(state==1 || state==2)begin
 dividend<={dividend[20:0],1'b0};
 remainder<=subtract_ok ? trial-{2'b0,divisor}:trial;
 quotient<=next_quotient;bits_left<=bits_left-1'b1;
 if(bits_left==1)begin
 if(state==1)begin
 pixel_x<=INVERT_X ? 9'd479-next_quotient[8:0]:next_quotient[8:0];
 dividend<=pending_y;divisor<=Y_SPAN;remainder<=0;quotient<=0;bits_left<=22;state<=2;
 end else begin
 pixel_y<=INVERT_Y ? 9'd319-next_quotient[8:0]:next_quotient[8:0];state<=3;
 end
 end
 end
 if(state==3)begin state<=0;tap<=1;end
 end
 end
endmodule
