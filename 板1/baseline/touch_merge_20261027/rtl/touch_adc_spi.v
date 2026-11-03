`timescale 1ns/1ps
// XPT2046-compatible ADC: mode 0, 24 clocks, differential 12-bit, PD=00.
// Always poll X/Y/Z1/Z2; discard the first conversion after each channel change.
// No IRQ or pressure threshold can prevent observing raw data.
module touch_adc_spi #(parameter integer HALF_PERIOD=100)(
 input clk,rst_n,start,miso,output reg busy,done,
 output reg [11:0] raw_x,raw_y,raw_z1,raw_z2,
 output reg sck,mosi,cs_n);
 (* async_reg="true" *) reg [1:0] miso_sync;
 reg [15:0] ticks;reg [4:0] bit_index;reg [2:0] conversion;
 reg [23:0] shift,rx;reg [1:0] phase;
 localparam IDLE=0,SHIFT=1,PAUSE=2,FINISH=3;
 function [23:0] command;
 input [2:0] index;
 begin case(index[2:1])
 0:command=24'hd00000;1:command=24'h900000;
 2:command=24'hb00000;default:command=24'hc00000;
 endcase end
 endfunction
 always @(posedge clk or negedge rst_n)
 if(!rst_n)miso_sync<=0;else miso_sync<={miso_sync[0],miso};
 always @(posedge clk or negedge rst_n)begin
 if(!rst_n)begin
 busy<=0;done<=0;raw_x<=0;raw_y<=0;raw_z1<=0;raw_z2<=0;
 sck<=0;mosi<=0;cs_n<=1;ticks<=0;bit_index<=0;conversion<=0;shift<=0;rx<=0;phase<=IDLE;
 end else begin
 done<=0;
 case(phase)
 IDLE:if(start)begin
 busy<=1;ticks<=0;conversion<=0;shift<=command(0);rx<=0;
 bit_index<=0;mosi<=1;cs_n<=0;sck<=0;phase<=SHIFT;
 end
 SHIFT:if(ticks==HALF_PERIOD-1)begin
 ticks<=0;
 if(!sck)begin sck<=1;rx<={rx[22:0],miso_sync[1]};end
 else begin
 sck<=0;
 if(bit_index==23)begin
 cs_n<=1;mosi<=0;
 case(conversion)
 1:raw_x<=rx[14:3];3:raw_y<=rx[14:3];
 5:raw_z1<=rx[14:3];7:raw_z2<=rx[14:3];
 endcase
 phase<=conversion==7 ? FINISH:PAUSE;
 end else begin bit_index<=bit_index+1'b1;shift<={shift[22:0],1'b0};mosi<=shift[22];end
 end
 end else ticks<=ticks+1'b1;
 PAUSE:if(ticks==HALF_PERIOD*16-1)begin
 ticks<=0;conversion<=conversion+1'b1;bit_index<=0;rx<=0;
 sck<=0;cs_n<=0;mosi<=1;shift<=command(conversion+3'd1);phase<=SHIFT;
 end else ticks<=ticks+1'b1;
 FINISH:if(ticks==HALF_PERIOD*16-1)begin
 ticks<=0;busy<=0;done<=1;phase<=IDLE;
 end else ticks<=ticks+1'b1;
 endcase
 end
 end
endmodule
