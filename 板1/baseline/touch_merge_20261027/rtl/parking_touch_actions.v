module parking_touch_actions(input clk,rst_n,tap,input [8:0] x,y,
 output wire menu,output reg mode_set,output wire [2:0] mode_value,
 output reg [3:0] actions,output reg track_tap);
 assign menu=0;assign mode_value=0;
 always @(posedge clk or negedge rst_n)
 if(!rst_n)begin actions<=0;track_tap<=0;mode_set<=0;end
 else begin
 actions<=0;track_tap<=0;mode_set<=0;
 if(tap && x>=376 && x<472)begin
 if(y>=32 && y<80)actions[0]<=1;
 else if(y>=88 && y<136)actions[1]<=1;
 else if(y>=144 && y<192)track_tap<=1;
 else if(y>=200 && y<248)mode_set<=1;
 end end
endmodule
