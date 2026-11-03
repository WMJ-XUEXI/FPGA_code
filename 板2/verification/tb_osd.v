`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,vs=0,hs=0,de=0,wr=0,pixel=0,invalidate=0;reg [23:0] rgb=0;reg [10:0] x=0;reg [9:0] y=0,addr=0;
 wire ov,oh,od;wire [23:0] out;
 reg [48:0] PLATE={7'd57,7'd39,7'd38,7'd32,7'd39,7'd50,7'd11};reg locked=1,busy=0,present=1;reg [2:0] reason=0;
 recognition_osd dut(clk,rst,vs,hs,de,rgb,x,y,PLATE,locked,busy,4'b1010,4'd7,wr,addr,pixel,invalidate,reason,present,87'd0,1'b0,1'b0,6'd0,2'd0,16'd0,16'd0,16'd0,8'd0,ov,oh,od,out,2'd0,56'd0,1'b0,114'd0,66'd0,1'b0);
 reg [15:0] font[0:1019];reg [23:0] expected=0,next_expected;
 reg [23:0] expected_delay=0;reg ev_delay=0,eh_delay=0,ed_delay=0;reg ev=0,eh=0,ed=0;integer i,xx,yy,glyph,index,row,col,checks=0;
 reg [39:0] word;reg [7:0] ascii;integer mode;
 task send;input [10:0] sx;input [9:0] sy;input active;
 begin
  @(negedge clk);x=sx;y=sy;de=active;vs=sx[0];hs=sy[0];rgb={sx[7:0],sy[7:0],8'h5a};
  next_expected=rgb;
  if(!active)next_expected=0;
  else if(sx>=1008 && sx<1232 && sy>=48 && sy<78)begin
   index=(sx-1008)/32;glyph=(PLATE>>(index*7))&127;if(glyph>=67)glyph=67;row=(sy-48)/2;col=((sx-1008)%32)/2;
   if(font[glyph*15+row]&(16'h8000>>col))next_expected=locked ? 24'h00ff40:24'hffff00;
  end
  else if(sx>=1008 && sx<1088 && sy>=128 && sy<143)begin
   if(!present)word="SCAN ";else if(locked)word="READY";else if(reason==1)word="ANGLE";else if(reason==2)word="SPLIT";else if(reason==3)word="CHECK";else if(busy)word="READ ";else word="HOLD ";
   index=(sx-1008)/16;ascii=(word>>((4-index)*8))&255;glyph=ascii-24;row=sy-128;col=(sx-1008)%16;
   if(ascii!=32 && font[glyph*15+row]&(16'h8000>>col))next_expected=locked ? 24'h00ff40:24'hffff00;
  end
  else if(sx>=1008 && sx<1136 && sy>=96 && sy<112 && (sx-1008)%32<16)
   next_expected=(4'b1010>>( (sx-1008)/32 ))&1 ? 24'h00ff40:24'hff8000;
  else if(sx>=1008 && sx<1136 && sy>=160 && sy<288 && !invalidate)
   next_expected=((((sx-1008)/4)^((sy-160)/4))&1) ? 24'hffffff:24'h101020;
  @(posedge clk);#1;
  if(out!==expected_delay || ov!==ev_delay || oh!==eh_delay || od!==ed_delay)$fatal(1,"OSD RGB/sync alignment x=%0d y=%0d actual=%h expected=%h",sx,sy,out,expected_delay);
  expected_delay=expected;ev_delay=ev;eh_delay=eh;ed_delay=ed;expected=next_expected;ev=vs;eh=hs;ed=de;checks=checks+1;
 end endtask
 initial begin
  $readmemh("../model/font.hex",font);repeat(4)@(negedge clk);rst=1;
  for(i=0;i<1024;i=i+1)begin @(negedge clk);wr=1;addr=i;pixel=(i%32)^(i/32);end
  @(negedge clk);wr=0;
  for(yy=0;yy<310;yy=yy+1)for(xx=996;xx<1240;xx=xx+1)send(xx,yy,1);
  send(0,0,0);send(0,0,0);
  locked=0;
  for(mode=0;mode<3;mode=mode+1)begin
   PLATE=mode==0 ? {7{7'd67}}:mode==1 ? {7{7'd68}}:{49{1'b1}};
   for(yy=48;yy<78;yy=yy+1)for(xx=1008;xx<1232;xx=xx+1)send(xx,yy,1);
  end
  for(mode=0;mode<7;mode=mode+1)begin
   present=mode!=0;reason=mode>=1 && mode<=3 ? mode:0;busy=mode==5;locked=mode==6;
   for(yy=128;yy<143;yy=yy+1)for(xx=1008;xx<1088;xx=xx+1)send(xx,yy,1);
  end
  send(0,0,0);send(0,0,0);
  $display("PASS OSD known/unknown glyphs, all status labels and %0d aligned video pixels",checks);$finish;
 end
endmodule
