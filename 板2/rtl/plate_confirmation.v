// Physical presence controls rearming; OCR quality controls READY only.
module plate_confirmation #(parameter ABSENT_FRAMES=30,CONFIRMATIONS=4)(
 input clk,rst_n,frame_start,present,retry,attempt_done,attempt_good,
 input [48:0] attempt_plate,
 output reg [48:0] plate,output reg locked,output enable,output abort_capture,
 output reg entry_event,output reg [15:0] visits,input new_target,output verified,
 input measurement_valid,input clear_all,output reg exit_event,output session_active
);
 reg active,blocked,changing;reg [7:0] absent;reg [1:0] quality_lost;
 reg [48:0] h0,h1,h2,h3;
 wire [2:0] votes=3'd1+{2'd0,attempt_plate==h0}+{2'd0,attempt_plate==h1}+{2'd0,attempt_plate==h2}+{2'd0,attempt_plate==h3};
 function [2:0] differences;input [48:0] a,b;integer i;begin differences=0;for(i=0;i<7;i=i+1)if(a[i*7 +:7]!=b[i*7 +:7])differences=differences+1'b1;end endfunction
 wire leave=frame_start && (active || blocked) && !present && !measurement_valid && absent>=ABSENT_FRAMES-1;
 wire acquire=new_target && !locked && !blocked;
 assign abort_capture=clear_all || leave || acquire || (retry && !locked);
 assign enable=active && measurement_valid && !blocked && !clear_all;
 assign verified=locked && quality_lost<2 && !changing;assign session_active=active;
 task clear_votes;begin h0<={49{1'b1}};h1<={49{1'b1}};h2<={49{1'b1}};h3<={49{1'b1}};end endtask
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin active<=0;blocked<=0;changing<=0;absent<=0;quality_lost<=0;plate<={49{1'b1}};locked<=0;entry_event<=0;exit_event<=0;visits<=0;clear_votes;end
  else begin
   entry_event<=0;exit_event<=0;
   if(clear_all)begin active<=0;blocked<=1;changing<=0;absent<=0;quality_lost<=0;locked<=0;visits<=0;plate<={49{1'b1}};clear_votes;end
   else if(leave)begin active<=0;blocked<=0;changing<=0;absent<=0;quality_lost<=0;locked<=0;exit_event<=locked;plate<={49{1'b1}};clear_votes;end
   else begin
    if(frame_start)begin
     if(measurement_valid)begin if(!blocked)active<=1;quality_lost<=0;end
     else begin if(quality_lost<3)quality_lost<=quality_lost+1'b1;clear_votes;end
     if(present || measurement_valid)absent<=0;
     else if((active || blocked) && absent<ABSENT_FRAMES)absent<=absent+1'b1;
    end
    if(acquire)begin active<=1;absent<=0;clear_votes;end
    else if(retry && !locked)clear_votes;
    else if(attempt_done && active && measurement_valid && !blocked)begin
     if(locked)changing<=!attempt_good || attempt_plate!=plate;
     h3<=h2;h2<=h1;h1<=h0;h0<=attempt_good ? attempt_plate:{49{1'b1}};
     if(attempt_good && attempt_plate!={49{1'b1}} && votes>=(locked && differences(attempt_plate,plate)<=1 ? 5:CONFIRMATIONS) && (!locked || attempt_plate!=plate))begin
      changing<=0;plate<=attempt_plate;locked<=1;entry_event<=1;if(visits!=65535)visits<=visits+1'b1;
     end
    end
   end
  end
 end
endmodule
