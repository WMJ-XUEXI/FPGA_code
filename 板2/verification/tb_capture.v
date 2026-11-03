`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fs=0,pv=0;reg [10:0] x=0,l=320,r=959;reg [9:0] y=0,t=180,b=339;reg [23:0] rgb=0;
 wire [9:0] gray={2'd0,rgb[23:16]}+{1'd0,rgb[15:8],1'b0}+{2'd0,rgb[7:0]};
 wire done,good;wire [48:0] plate;wire [3:0] status,nchars;
 plate_capture dut(.clk(clk),.rst_n(rst),.frame_start(fs),.pixel_valid(pv),.x(x),.y(y),.gray(gray[9:2]),.rgb(rgb),
 .buttons_n(2'b11),.box_valid(1'b1),.enable(1'b1),.retry(1'b0),.box_l(l),.box_r(r),.box_t(t),.box_b(b),
 .attempt_done(done),.attempt_plate(plate),.attempt_confident(good),.capture_status(status),.detected_characters(nchars));
 reg [23:0] photo[0:16383];integer xx,yy,f,samples=0,completed=0,predictions=0;
 always @(posedge clk)begin
  if(dut.sample)samples=samples+1;
  if(dut.core_done)begin predictions=predictions+1;$display("CNN slot=%0d class=%0d margin=%0d",dut.slot,dut.prediction,dut.margin);end
  if(done)begin
   completed=completed+1;
   $display("ATTEMPT samples=%0d chars=%0d status=%b good=%b plate=%h",samples,nchars,status,good,plate);
   if(samples!=16384)$fatal(1,"dynamic ROI sample count incorrect");
   if(nchars!=7)$fatal(1,"photo segmentation failed");
   samples=0;
  end
 end
 initial begin
  $readmemh("board1.hex",photo);
  repeat(4)@(negedge clk);rst=1;
  for(f=0;f<2;f=f+1)begin
   @(negedge clk);fs=1;@(negedge clk);fs=0;repeat(5)@(negedge clk);
   for(yy=0;yy<720;yy=yy+1)begin
    for(xx=0;xx<1280;xx=xx+1)begin
     @(negedge clk);x=xx;y=yy;pv=1;rgb=24'h202020;
     if(xx>=l && xx<=r && yy>=t && yy<=b)rgb=photo[((yy-t)*64/(b-t+1))*256+((xx-l)*256/(r-l+1))];
    end
    @(negedge clk);pv=0;repeat(370)@(negedge clk);
   end
   pv=0;repeat(1700000)@(negedge clk);
   l=192;r=959;t=240;b=431;
  end
  if(completed!=2 || predictions!=14)$fatal(1,"CNN incomplete");
  $display("PASS dynamic ROI at two positions/sizes, seven real CNN inferences per attempt");$finish;
 end
endmodule
