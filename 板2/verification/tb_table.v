`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,de=0;reg [10:0] x=0;reg [9:0] y=0;
 wire ov,oh,od;wire [23:0] rgb;reg [86:0] rows[0:7];wire [9:0] hy=y-400;
 recognition_osd dut(.clk(clk),.rst_n(rst),.vs(1'b0),.hs(1'b0),.de(de),.rgb(24'h303030),.x(x),.y(y),.plate({49{1'b1}}),.locked(1'b0),.busy(1'b0),.stage(4'd0),.characters(4'd0),
 .preview_wr(1'b0),.preview_addr(10'd0),.preview_pixel(1'b0),.invalidate(1'b0),.reason(3'd0),.present(1'b0),.history_record(rows[hy[7:5]]),.history_ready(1'b1),.history_full(1'b0),.history_used(6'd8),.history_page(2'd0),.total_in(16'h0008),.total_out(16'h0004),.total_flow(16'h0012),.occupied(8'h04),.panel_mode(2'd0),.now_bcd(56'd0),.clock_valid(1'b0),.history_stamp(114'd0),.summary_record(66'd0),.panel_hidden(1'b0),.out_vs(ov),.out_hs(oh),.out_de(od),.out_rgb(rgb));
 integer f,xx,yy,n=0;always @(negedge clk)if(od)begin $fwrite(f,"%c%c%c",rgb[23:16],rgb[15:8],rgb[7:0]);n=n+1;end
 initial begin
 rows[0]=87'h6e53a6409d90b001000000;
rows[1]=87'h481ea84a9d496002010101;
rows[2]=87'h6951213eb9480003020202;
rows[3]=87'h4892a44899a00004030303;
rows[4]=87'h6911a24281481005040404;
rows[5]=87'h4911a24281513006050505;
rows[6]=87'h695223448558a007060606;
rows[7]=87'h4992a4468960f008070707;
 f=$fopen("parking_table.ppm","wb");$fwrite(f,"P6\n1280 720\n255\n");repeat(4)@(negedge clk);rst=1;
 for(yy=0;yy<720;yy=yy+1)for(xx=0;xx<1280;xx=xx+1)begin @(negedge clk);x=xx;y=yy;de=1;end
 @(negedge clk);de=0;repeat(6)@(negedge clk);$fclose(f);
 if(n!=921600)$fatal(1,"table video pixel count mismatch");$display("PASS eight-row parking table rendered, 921600 aligned pixels");$finish;
 end
endmodule
