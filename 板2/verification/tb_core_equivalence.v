`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,wr=0,start=0;reg [9:0] addr=0;reg [6:0] pixel=0;
 wire busy,done,old_busy,old_done;wire [6:0] pred,old_pred;wire [15:0] margin,old_margin;
 cnn_serial_core dut(clk,rst,wr,addr,pixel,start,2'd0,busy,done,pred,margin);
 cnn_serial_core_baseline old(clk,rst,wr,addr,pixel,start,2'd0,old_busy,old_done,old_pred,old_margin);
 reg [6:0] vectors[0:20479];reg [31:0] scores[0:1339];
 integer n,i,count=0,cycles=0,reference_cycles=0;
 wire [2:0] state=dut.state,phase=dut.phase;wire [11:0] ra=dut.read_address,wa=dut.write_address;wire [6:0] oc=dut.oc;
 wire signed [19:0] result=dut.score,old_result=old.score;
 always @(negedge clk)if(rst)begin
  if(busy!==old_busy || done!==old_done || pred!==old_pred || margin!==old_margin)$fatal(1,"core interface/cycle mismatch");
  if((state==3 || (state==4 && !dut.last_term_q)) && !dut.is_pool && phase!=5 && ra>=1024)$fatal(1,"active A read out of compact bounds");
  if(state==5 && dut.is_pool && wa>=1024)$fatal(1,"A pool write out of compact bounds");
 end
 initial begin
  $readmemh("core_vectors.hex",vectors);$readmemh("core_scores.hex",scores);
  repeat(4)@(negedge clk);rst=1;
  for(n=0;n<20;n=n+1)begin
   for(i=0;i<1024;i=i+1)begin @(negedge clk);wr=1;addr=i;pixel=vectors[n*1024+i];end
   @(negedge clk);wr=0;start=1;@(negedge clk);start=0;cycles=0;
   while(!done)begin
    @(negedge clk);cycles=cycles+1;
    if(state==5 && phase==5)begin
     if(result!==old_result || $signed(result)!==$signed(scores[n*67+oc]))$fatal(1,"logit mismatch vector=%0d class=%0d",n,oc);
     count=count+1;
    end
   end
   if(n==0)reference_cycles=cycles;else if(cycles!=reference_cycles)$fatal(1,"latency changed with input");
  end
  if(count!=1340)$fatal(1,"missing class logits");
  $display("PASS compact core equivalence: 20 glyph/zero/full/random inputs, 1340 integer logits, identical baseline predictions/margins/cycles, bounded 1024-entry A accesses; cycles=%0d",reference_cycles);$finish;
 end
 initial begin #40000000;$fatal(1,"timeout");end
endmodule
