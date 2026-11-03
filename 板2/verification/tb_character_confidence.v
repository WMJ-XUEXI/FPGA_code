`timescale 1ns/1ps
module tb;
 reg [6:0] prediction;reg [15:0] margin;reg province=0,plate_mode=1;wire confident;
 character_confidence dut(prediction,margin,province,plate_mode,confident);
 task check;input [6:0] p;input [15:0] m;input pr,mode,expected;begin
  prediction=p;margin=m;province=pr;plate_mode=mode;#1;
  if(confident!==expected)$fatal(1,"class confidence mismatch p=%0d m=%0d",p,m);
 end endtask
 initial begin
  check(0,255,1,1,0);check(0,256,1,1,1);
  check(21,255,1,1,0);check(21,256,1,1,1);
  check(57,191,0,1,0);check(57,192,0,1,1);
  check(37,191,0,1,0);check(37,192,0,1,1);
  check(39,191,0,1,0);check(39,192,0,1,1);check(42,191,0,1,0);check(42,192,0,1,1);
  check(11,127,1,1,0);check(11,128,1,1,1);
  check(32,63,0,1,0);check(32,64,0,1,1);
  check(37,64,0,0,1);check(127,65535,0,1,0);
  $display("PASS target character confidence: 京/琼 256, Q/6 192, existing province/Roman thresholds, digit compatibility and unknown rejection");$finish;
 end
endmodule
