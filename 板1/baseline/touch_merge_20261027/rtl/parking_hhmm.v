// Display-only sequential seconds -> HH:MM. No pixel-rate divider or DSP.
// Above 99:59:59 the display saturates and marks '+', original seconds unchanged.
module parking_hhmm(input clk,rst_n,snapshot,input [31:0] seconds,
 output reg [3:0] h10,h1,m10,m1,output reg over_range,output reg busy);
 reg [18:0] remaining;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin remaining<=0;h10<=0;h1<=0;m10<=0;m1<=0;over_range<=0;busy<=0;end
  else if(snapshot)begin
   remaining<=seconds>359999 ? 19'd359999:seconds[18:0];
   h10<=0;h1<=0;m10<=0;m1<=0;over_range<=seconds>359999;busy<=1;
  end else if(busy)begin
   if(remaining>=3600)begin
    remaining<=remaining-19'd3600;
    if(h1==9)begin h1<=0;h10<=h10+1'b1;end else h1<=h1+1'b1;
   end else if(remaining>=60)begin
    remaining<=remaining-19'd60;
    if(m1==9)begin m1<=0;m10<=m10+1'b1;end else m1<=m1+1'b1;
   end else busy<=0;
  end
 end
endmodule
