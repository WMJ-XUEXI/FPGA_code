`timescale 1ns/1ps
// Compatibility wrapper around the ADC reader verified by the user's 1024 test.
// Integrated pixel clock is approximately 75MHz; HALF_PERIOD=150 -> 250kHz.
module parking_touch_spi #(parameter integer HALF_PERIOD=150)(
 input clk,rst_n,start,miso,irq_n,
 output wire busy,done,valid,output wire [11:0] raw_x,raw_y,
 output wire sck,mosi,cs_n,output wire [11:0] diag_z1,diag_z2);
 wire [11:0] raw_z1,raw_z2;
 touch_adc_spi #(.HALF_PERIOD(HALF_PERIOD)) adc(
 .clk(clk),.rst_n(rst_n),.start(start),.miso(miso),.busy(busy),.done(done),
 .raw_x(raw_x),.raw_y(raw_y),.raw_z1(raw_z1),.raw_z2(raw_z2),
 .sck(sck),.mosi(mosi),.cs_n(cs_n));
 // Optional pressure contact hint. Default contact selection is in the controller.
 assign diag_z1=raw_z1;assign diag_z2=raw_z2;
 assign valid=done && raw_z1>12'd50 && raw_z1<12'd4000;
endmodule
