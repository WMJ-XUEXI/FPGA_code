`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,fs=0,pv=0;reg [10:0] x=0;reg [9:0] y=0;reg [23:0] rgb=0;
 wire bv,cv,measured,new_target,verified;wire [10:0] l,r,rl,rr;wire [9:0] t,b,rt,rb;wire [3:0] status;wire [15:0] seq;
 blue_plate_locator locator(clk,rst,fs,pv,1'b0,1'b0,x,y,rgb,bv,cv,l,r,rl,rr,t,b,rt,rb,status,seq,measured,new_target);
 wire [9:0] gray={2'd0,rgb[23:16]}+{1'd0,rgb[15:8],1'b0}+{2'd0,rgb[7:0]};
 wire done,good,locked,enable,abort_capture,event_out;wire [48:0] raw,plate;wire [3:0] cs,nchars;wire [15:0] visits;
 plate_capture capture(.clk(clk),.rst_n(rst),.frame_start(fs),.pixel_valid(pv),.x(x),.y(y),.gray(gray[9:2]),.rgb(rgb),
 .buttons_n(2'b11),.box_valid(bv && measured),.enable(enable),.retry(abort_capture),.box_l(l),.box_r(r),.box_t(t),.box_b(b),
 .attempt_done(done),.attempt_plate(raw),.attempt_confident(good),.capture_status(cs),.detected_characters(nchars));
 plate_confirmation confirm(clk,rst,fs,measured,1'b0,done,good,raw,plate,locked,enable,abort_capture,event_out,visits,new_target,verified);
 reg [23:0] photo[0:921599];reg [1023:0] file_name;integer f,xx,yy,completed=0;reg expect_reject;
 always @(posedge clk)begin
  if(capture.core_done)$display("CNN slot=%0d class=%0d margin=%0d",capture.slot,capture.prediction,capture.margin);
  if(done)begin completed=completed+1;$display("ATTEMPT %0d chars=%0d good=%b raw=%h",completed,nchars,good,raw);end
 end
 initial begin
  expect_reject=$test$plusargs("REJECT");
  if(!$value$plusargs("PHOTO=%s",file_name))file_name="board1_full.hex";
  $readmemh(file_name,photo);repeat(4)@(negedge clk);rst=1;
  for(f=0;f<12;f=f+1)begin
   @(negedge clk);fs=1;@(negedge clk);fs=0;repeat(8)@(negedge clk);
   for(yy=0;yy<720;yy=yy+1)begin
    for(xx=0;xx<1280;xx=xx+1)begin @(negedge clk);x=xx;y=yy;pv=1;rgb=photo[yy*1280+xx];end
    @(negedge clk);pv=0;repeat(370)@(negedge clk);
   end
   repeat(30000)@(negedge clk);
   $display("FRAME %0d bbox=%b %0d,%0d..%0d,%0d locked=%b plate=%h",f,bv,l,t,r,b,locked,plate);
  end
  if(expect_reject)begin
   if(completed==0 || visits>1 || (locked && plate!={7'd57,7'd39,7'd38,7'd32,7'd39,7'd50,7'd11}))$fatal(1,"difficult diagnostic was accepted with wrong identity or not exercised");
   if(locked)$display("PASS difficult diagnostic: correctly recovered previous failure");else $display("PASS difficult diagnostic: safely rejected without fabricated identity");$finish;
  end
  if(!locked || visits!=1)$fatal(1,"full pipeline did not lock one visit");
  if(plate!={7'd57,7'd39,7'd38,7'd32,7'd39,7'd50,7'd11})$fatal(1,"full pipeline misrecognized diagnostic plate");
  $display("PASS actual locator -> dynamic crop -> segmentation -> CNN -> whole-plate confirmation");$finish;
 end
endmodule
