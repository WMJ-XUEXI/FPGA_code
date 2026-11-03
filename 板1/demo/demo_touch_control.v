module demo_touch_control(
 input clk,rst_n,tap,input [8:0] x,y,input snapshot,
 input [2:0] active,observed,registered,input [47:0] ids,
 input result_valid,input [3:0] result,
 output reg page,enter_request,exit_request,output reg [15:0] selected_id,output reg [3:0] ui_result
);
 reg [2:0] active_q,observed_q,registered_q;reg [47:0] ids_q;
 integer i;reg [1:0] eligible;reg [15:0] only_id;
 always @*begin
  eligible=0;only_id=0;
  for(i=0;i<3;i=i+1)if(active_q[i] && observed_q[i] && !registered_q[i])begin eligible=eligible+1'b1;only_id=ids_q[i*16+:16];end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin page<=0;enter_request<=0;exit_request<=0;selected_id<=0;ui_result<=0;active_q<=0;observed_q<=0;registered_q<=0;ids_q<=0;end
  else begin
   enter_request<=0;exit_request<=0;
   if(snapshot)begin active_q<=active;observed_q<=observed;registered_q<=registered;ids_q<=ids;end
   if(result_valid)ui_result<=result;
   if(tap)begin
    if(x>=360 && x<472 && y>=264 && y<312)page<=!page;
    else if(page)begin
     if(x>=16 && x<464 && y>=96 && y<128)selected_id<=active_q[0] ? ids_q[15:0]:16'd0;
     else if(x>=16 && x<464 && y>=136 && y<168)selected_id<=active_q[1] ? ids_q[31:16]:16'd0;
     else if(x>=16 && x<464 && y>=176 && y<208)selected_id<=active_q[2] ? ids_q[47:32]:16'd0;
     else if(y>=40 && y<80 && x>=16 && x<224)begin
      if(selected_id!=0)enter_request<=1;
      else if(eligible==1)begin selected_id<=only_id;enter_request<=1;end
      else ui_result<=6;
     end else if(y>=40 && y<80 && x>=240 && x<464)begin
      if(selected_id!=0)exit_request<=1;else ui_result<=1;
     end
    end
   end
  end
 end
endmodule
