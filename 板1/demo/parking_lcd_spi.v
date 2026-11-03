`timescale 1ns/1ps
// MHS3528 serial-to-parallel module: 16 clocks per command, parameter,
// or RGB565 pixel. This is NOT the native 8-bit ILI9486 serial protocol.
// Factory MHS35 overlay: buswidth=8, regwidth=16, COLMOD=0x55.
module parking_lcd_spi #(
 parameter integer CLK_HZ=75000000,
 parameter integer TOUCH_DEBUG=0,
 parameter integer DEBUG_APP=0,
 parameter integer SPI_HALF=4,
 parameter [7:0] MADCTL=8'h28
)(input clk,rst_n,input [21:0] occupied,input status_valid,input [3:0] app_status,
 input tp_miso,tp_irq_n,touch_enable,
 input [2:0] active_mode,input tracking,
 input [59:0] demo_ids_decimal,input [47:0] demo_ids,input [2:0] demo_active,demo_observed,demo_registered,input [5:0] demo_codes,
 output [15:0] demo_selected_id,input demo_result_valid,input [3:0] demo_result,
 output enter_request,exit_request,output [3:0] manager_status,output [1:0] key12,
 output mode_set,output [2:0] mode_value,output [3:0] actions,output track_tap,
 input [9:0] metric_changes,metric_strong,metric_samples,
 output [43:0] sensitivity,output [4:0] selected_bay,
 output wire sck,mosi,cs_n,dc,output reg lcd_rst_n,output wire tp_cs_n);
 reg [3:0] state;
 wire lcd_sck,lcd_mosi,lcd_cs_n;
 wire tp_sck,tp_mosi,tp_busy,tp_done,tp_valid;
 reg tp_start;
 wire [11:0] raw_x,raw_y;
 reg last_contact;
 always @(posedge clk or negedge rst_n)if(!rst_n)last_contact<=0;else if(tp_done)last_contact<=tp_valid;
 wire [1:0] calibration;wire detail;
 wire tap;wire [8:0] touch_x,touch_y;
 reg [1:0] frame_calibration,frame_sensitivity;
 reg frame_detail,frame_enable;
 reg [4:0] frame_selected;
 reg [9:0] frame_changes,frame_strong,frame_samples;
 (* async_reg="true" *) reg [1:0] pen_sync;
 always @(posedge clk or negedge rst_n)
  if(!rst_n)pen_sync<=3;else pen_sync<={pen_sync[0],tp_irq_n};
 wire touch_owns=state==10 || state==11;
 assign sck=touch_owns ? tp_sck:lcd_sck;
 assign mosi=touch_owns ? tp_mosi:lcd_mosi;
 assign cs_n=touch_owns ? 1'b1:lcd_cs_n;
 wire [11:0] raw_z1,raw_z2;
 wire control_contact,control_held;wire [1:0] control_state,control_stable,control_released,control_contact_mode;
 parking_touch_spi #(.HALF_PERIOD(CLK_HZ/500000)) touch_bus(
 .clk(clk),.rst_n(rst_n),.start(tp_start),.miso(tp_miso),.irq_n(tp_irq_n),
 .busy(tp_busy),.done(tp_done),.valid(tp_valid),.raw_x(raw_x),.raw_y(raw_y),
 .sck(tp_sck),.mosi(tp_mosi),.cs_n(tp_cs_n),.diag_z1(raw_z1),.diag_z2(raw_z2));
 parking_touch_control touch_control(
 .clk(clk),.rst_n(rst_n),.poll_done(tp_done),.raw_valid(tp_valid),.irq_n(pen_sync[1]),.enable(touch_enable),
 .raw_x(raw_x),.raw_y(raw_y),.calibration(calibration),.detail(detail),.selected(selected_bay),
 .sensitivity(sensitivity),.tap(tap),.pixel_x(touch_x),.pixel_y(touch_y),
 .debug_contact(control_contact),.debug_held(control_held),.debug_state(control_state),
 .debug_stable(control_stable),.debug_released(control_released),.debug_contact_mode(control_contact_mode));
 localparam RESET=0,RELEASE=1,INIT=2,INIT_WAIT=3,DELAY=4,
            FRAME=5,WINDOW=6,WINDOW_WAIT=7,PIXEL=8,PIXEL_WAIT=9;
 localparam integer MS_CYCLES=CLK_HZ/1000;

 reg [31:0] delay_count;
 reg [7:0] init_index;
 reg [3:0] window_index;
 reg [8:0] x,y;
 reg [21:0] frame_occupied;
 reg frame_valid;
 wire [15:0] pixel;
 wire menu;
 wire manager_page;wire [3:0] manager_result;
 wire [15:0] manager_pixel,legacy_pixel;wire manager_page_frame;
 demo_touch_control manager(.clk(clk),.rst_n(rst_n),.tap(tap),.x(touch_x),.y(touch_y),.snapshot(state==FRAME),
 .active(demo_active),.observed(demo_observed),.registered(demo_registered),.ids(demo_ids),
 .result_valid(demo_result_valid),.result(demo_result),.page(manager_page),.enter_request(enter_request),.exit_request(exit_request),
 .selected_id(demo_selected_id),.ui_result(manager_result));
 demo_touch_ui manager_ui(.clk(clk),.rst_n(rst_n),.snapshot(state==FRAME),.page(manager_page),
 .active(demo_active),.ids(demo_ids),.ids_decimal(demo_ids_decimal),.codes(demo_codes),.selected_id(demo_selected_id),.result(manager_result),
 .x(x),.y(y),.page_frame(manager_page_frame),.rgb(manager_pixel));
 assign pixel=manager_page_frame ? manager_pixel:legacy_pixel;
 parking_touch_actions buttons(clk,rst_n,tap && !manager_page,touch_x,touch_y,menu,mode_set,mode_value,actions,track_tap);
 assign manager_status=manager_result;
 assign key12=actions[1:0];
 reg frame_menu,frame_tracking;reg [2:0] frame_mode;reg [3:0] frame_status;
 always @(posedge clk or negedge rst_n)if(!rst_n)frame_status<=0;else if(state==FRAME)frame_status<=app_status;
 always @(posedge clk or negedge rst_n) if(!rst_n)begin frame_menu<=0;frame_mode<=0;frame_tracking<=0;end
 else if(state==FRAME)begin frame_menu<=menu;frame_mode<=active_mode;frame_tracking<=tracking;end
 reg [17:0] last_touch,frame_touch;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin last_touch<=0;frame_touch<=0;end
  else begin
   if(tap)last_touch<={touch_y,touch_x};
   if(state==FRAME)frame_touch<=last_touch;
  end
 end
 wire [255:0] debug_frame={238'd0,frame_touch};
 parking_touch_ui ui(clk,rst_n,debug_frame,raw_x,raw_y,pen_sync[1],last_contact,x,y,frame_occupied,frame_valid,frame_calibration,frame_menu,frame_mode,frame_tracking,frame_status,legacy_pixel);
 reg start,tx_dc;
 reg [15:0] tx_data;
 wire busy,done;
 parking_lcd_word #(.HALF_PERIOD(SPI_HALF)) serial(
  clk,rst_n,start,tx_dc,tx_data,busy,done,lcd_sck,lcd_mosi,lcd_cs_n,dc);
 function [17:0] init_op;
  input [7:0] index;
  begin
   case(index)
           8'd0: init_op=18'h000f1;
        8'd1: init_op=18'h10036;
        8'd2: init_op=18'h10004;
        8'd3: init_op=18'h10000;
        8'd4: init_op=18'h1003c;
        8'd5: init_op=18'h1000f;
        8'd6: init_op=18'h1008f;
        8'd7: init_op=18'h000f2;
        8'd8: init_op=18'h10018;
        8'd9: init_op=18'h100a3;
        8'd10: init_op=18'h10012;
        8'd11: init_op=18'h10002;
        8'd12: init_op=18'h100b2;
        8'd13: init_op=18'h10012;
        8'd14: init_op=18'h100ff;
        8'd15: init_op=18'h10010;
        8'd16: init_op=18'h10000;
        8'd17: init_op=18'h000f8;
        8'd18: init_op=18'h10021;
        8'd19: init_op=18'h10004;
        8'd20: init_op=18'h000f9;
        8'd21: init_op=18'h10000;
        8'd22: init_op=18'h10008;
        8'd23: init_op=18'h00036;
        8'd24: init_op=18'h10008;
        8'd25: init_op=18'h000b4;
        8'd26: init_op=18'h10000;
        8'd27: init_op=18'h000c1;
        8'd28: init_op=18'h10041;
        8'd29: init_op=18'h000c5;
        8'd30: init_op=18'h10000;
        8'd31: init_op=18'h10091;
        8'd32: init_op=18'h10080;
        8'd33: init_op=18'h10000;
        8'd34: init_op=18'h000e0;
        8'd35: init_op=18'h1000f;
        8'd36: init_op=18'h1001f;
        8'd37: init_op=18'h1001c;
        8'd38: init_op=18'h1000c;
        8'd39: init_op=18'h1000f;
        8'd40: init_op=18'h10008;
        8'd41: init_op=18'h10048;
        8'd42: init_op=18'h10098;
        8'd43: init_op=18'h10037;
        8'd44: init_op=18'h1000a;
        8'd45: init_op=18'h10013;
        8'd46: init_op=18'h10004;
        8'd47: init_op=18'h10011;
        8'd48: init_op=18'h1000d;
        8'd49: init_op=18'h10000;
        8'd50: init_op=18'h000e1;
        8'd51: init_op=18'h1000f;
        8'd52: init_op=18'h10032;
        8'd53: init_op=18'h1002e;
        8'd54: init_op=18'h1000b;
        8'd55: init_op=18'h1000d;
        8'd56: init_op=18'h10005;
        8'd57: init_op=18'h10047;
        8'd58: init_op=18'h10075;
        8'd59: init_op=18'h10037;
        8'd60: init_op=18'h10006;
        8'd61: init_op=18'h10010;
        8'd62: init_op=18'h10003;
        8'd63: init_op=18'h10024;
        8'd64: init_op=18'h10020;
        8'd65: init_op=18'h10000;
        8'd66: init_op=18'h0003a;
        8'd67: init_op=18'h10055;
        8'd68: init_op=18'h00011;
        8'd69: init_op=18'h00036;
        8'd70: init_op={2'b01,8'h00,MADCTL};
        8'd71: init_op=18'h200ff;
        8'd72: init_op=18'h00029;
    default:init_op=18'h20001;
   endcase
  end
 endfunction
 wire [17:0] op=init_op(init_index);
 // Register byte parameters are each padded to 16 SPI bits.
 function [16:0] window_op;
  input [3:0] index;
  begin
   case(index)
    0:window_op=17'h0002a;
    1:window_op=17'h10000;2:window_op=17'h10000;
    3:window_op=17'h10001;4:window_op=17'h100df; // 479
    5:window_op=17'h0002b;
    6:window_op=17'h10000;7:window_op=17'h10000;
    8:window_op=17'h10001;9:window_op=17'h1003f; // 319
    default:window_op=17'h0002c;
   endcase
  end
 endfunction
 wire [16:0] win_op=window_op(window_index);
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=RESET;lcd_rst_n<=0;delay_count<=0;init_index<=0;
   window_index<=0;x<=0;y<=0;frame_occupied<=0;frame_valid<=0;
   start<=0;tx_dc<=1;tx_data<=0;tp_start<=0;
   frame_calibration<=3;frame_sensitivity<=1;frame_detail<=0;frame_selected<=0;
   frame_changes<=0;frame_strong<=0;frame_samples<=0;frame_enable<=0;
  end else begin
   start<=0;tp_start<=0;
   case(state)
    RESET:if(delay_count==120*MS_CYCLES-1)begin
     lcd_rst_n<=1;delay_count<=0;state<=RELEASE;
    end else delay_count<=delay_count+1'b1;
    RELEASE:if(delay_count==120*MS_CYCLES-1)begin
     delay_count<=0;state<=INIT;
    end else delay_count<=delay_count+1'b1;
    INIT:begin
     if(init_index==73)state<=FRAME;
     else if(op[17])begin
      // Exact init table specialization: index 71 delays 255 ms; all other
      // delay opcodes are 1 ms. Both products are compile-time constants.
      delay_count<=init_index==8'd71 ? 255*MS_CYCLES:MS_CYCLES;state<=DELAY;
     end
     else if(!busy)begin
      tx_dc<=op[16];tx_data<=op[15:0];
      // The final MADCTL parameter is patched by the generated table.
      start<=1;state<=INIT_WAIT;
     end
    end
    INIT_WAIT:if(done)begin init_index<=init_index+1'b1;state<=INIT;end
    DELAY:if(delay_count==0)begin init_index<=init_index+1'b1;state<=INIT;end
     else delay_count<=delay_count-1'b1;
    FRAME:begin
     frame_occupied<=occupied;frame_valid<=status_valid;
     frame_calibration<=calibration;frame_detail<=detail;frame_enable<=touch_enable;
     frame_selected<=selected_bay;frame_sensitivity<=sensitivity[selected_bay*2+:2];
     frame_changes<=metric_changes;frame_strong<=metric_strong;frame_samples<=metric_samples;
     window_index<=0;x<=0;y<=0;state<=WINDOW;
    end
    WINDOW:if(!busy)begin
     tx_dc<=win_op[16];tx_data<=win_op[15:0];start<=1;state<=WINDOW_WAIT;
    end
    WINDOW_WAIT:if(done)begin
     if(window_index==10)state<=12;
     else begin window_index<=window_index+1'b1;state<=WINDOW;end
    end
    12:state<=PIXEL;
    PIXEL:if(!busy)begin tx_dc<=1;tx_data<=pixel;start<=1;state<=PIXEL_WAIT;end
    PIXEL_WAIT:if(done)begin
     if(x==479)begin x<=0;if(y[3:0]==15)state<=10;else begin y<=y+1'b1;state<=12;end end
     else begin x<=x+1'b1;state<=12;end
    end
    10:begin tp_start<=1;state<=11;end
    11:if(tp_done)begin
     if(y==319)state<=FRAME;else begin y<=y+1'b1;state<=12;end
    end
    default:state<=RESET;
   endcase
  end
 end
endmodule

// SPI mode 0, MSB first. CS rises after every 16-bit module word to latch it.
module parking_lcd_word #(parameter integer HALF_PERIOD=4)(
 input clk,rst_n,start,input data_command,input [15:0] data,
 output reg busy,done,sck,mosi,cs_n,dc);
 reg [15:0] shift;
 reg [4:0] bits_left;
 reg [15:0] ticks;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin busy<=0;done<=0;sck<=0;mosi<=0;cs_n<=1;dc<=1;shift<=0;bits_left<=0;ticks<=0;end
  else begin
   done<=0;
   if(!busy)begin
    if(start)begin
     busy<=1;cs_n<=0;dc<=data_command;sck<=0;
     shift<=data;mosi<=data[15];bits_left<=16;ticks<=0;
    end
   end else if(ticks==HALF_PERIOD-1)begin
    ticks<=0;
    if(!sck)sck<=1;
    else begin
     sck<=0;
     if(bits_left==1)begin cs_n<=1;busy<=0;done<=1;end
     else begin bits_left<=bits_left-1'b1;shift<={shift[14:0],1'b0};mosi<=shift[14];end
    end
   end else ticks<=ticks+1'b1;
  end
 end
endmodule





