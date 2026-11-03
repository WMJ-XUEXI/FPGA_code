`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;
 reg rst=0,de=0,user=0,last=0;reg [23:0] data=0;wire rd,vs,hs,ode;wire [23:0] rgb;
 blueplate_video_mixer dut(clk,rst,1'b0,1'b0,de,user,last,4'd0,4'b1111,rd,data,vs,hs,ode,rgb);
 reg [23:0] pixels[0:921599];integer handle,xx,yy,frame,n=0;
 wire locked=dut.locked;wire [48:0] plate=dut.confirmed;
 integer cycles=0,first_present_cycle=-1,first_lock_cycle=-1;
 wire present=dut.bv;wire [5:0] used=dut.history_used;
 always @(posedge clk)if(rst)begin
  cycles=cycles+1;
  if(present && first_present_cycle<0)first_present_cycle=cycles;
  if(locked && first_lock_cycle<0)begin first_lock_cycle=cycles;$display("LATENCY present=%0d locked=%0d delta=%0d clocks",first_present_cycle,first_lock_cycle,first_lock_cycle-first_present_cycle);end
 end
 always @(negedge clk)if(frame==11 && ode)begin $fwrite(handle,"%c%c%c",rgb[23:16],rgb[15:8],rgb[7:0]);n=n+1;end
 always @(posedge clk)if(dut.attempt_done)$display("ATTEMPT good=%b raw=%h",dut.attempt_good,dut.attempt_plate);
 initial begin
  $readmemh("board1_full.hex",pixels);handle=$fopen("final_video.ppm","wb");$fwrite(handle,"P6\n1280 720\n255\n");
  frame=-1;repeat(4)@(negedge clk);rst=1;
  for(frame=0;frame<12;frame=frame+1)begin
   for(yy=0;yy<720;yy=yy+1)begin
    for(xx=0;xx<1280;xx=xx+1)begin
     @(posedge clk);#1;de=1;user=xx==0 && yy==0;last=xx==1279;data=pixels[yy*1280+xx];
    end
    @(posedge clk);#1;de=0;user=0;last=0;repeat(370)@(posedge clk);
   end
   repeat(20000)@(posedge clk);$display("VIDEO frame=%0d locked=%b plate=%h",frame,locked,plate);
  end
  $fclose(handle);
  if(used!=1)$fatal(1,"video session created duplicate records");
  if(n!=921600 || !locked || plate!={7'd57,7'd39,7'd38,7'd32,7'd39,7'd50,7'd11})$fatal(1,"full mixer/video/CNN mismatch pixels=%0d plate=%h",n,plate);
  $display("PASS complete HDMI mixer: 921600 pixels, correct locked plate and actual CNN preview");$finish;
 end
endmodule
