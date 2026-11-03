// Demo registration only. No parking timer, CNN or UDP identity.
// All ports are in the HDMI/LCD pixel clock domain. Commit at frame_start.
module demo_registration(
 input clk,rst_n,enable,frame_start,
 input [2:0] active,observed,input [47:0] ids,
 input enter_request,exit_request,input [15:0] selected_id,
 output busy,output reg result_valid,output reg [3:0] result,
 output reg [2:0] registered,output reg [5:0] plate_codes
);
 reg [2:0] used;reg [15:0] bound_id[0:2];reg [1:0] code[0:2];
 reg [1:0] ordinal;reg pending,operation_exit;reg [15:0] request_id;
 reg [2:0] present;integer i,k,j;reg [1:0] target,match_count;reg signed [2:0] record,free_record;
 assign busy=pending;
 always @*begin
  present=0;registered=0;plate_codes=0;
  for(i=0;i<3;i=i+1)for(k=0;k<3;k=k+1)
   if(used[k] && active[i] && bound_id[k]==ids[i*16+:16])begin
    present[k]=1;registered[i]=1;plate_codes[i*2+:2]=code[k];
   end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   used<=0;ordinal<=0;pending<=0;operation_exit<=0;request_id<=0;result_valid<=0;result<=0;
   for(j=0;j<3;j=j+1)begin bound_id[j]<=0;code[j]<=0;end
  end else if(!enable)begin used<=0;ordinal<=0;pending<=0;result_valid<=0;result<=0;end
  else begin
   result_valid<=0;
   if(enter_request || exit_request)begin
    if(pending)begin result<=5;result_valid<=1;end
    else if(enter_request && exit_request)begin result<=6;result_valid<=1;end
    else begin pending<=1;operation_exit<=exit_request;request_id<=selected_id;end
   end
   if(frame_start)begin
    used<=used & present;
    if(pending)begin
     pending<=0;result_valid<=1;result<=0;target=0;match_count=0;record=-1;free_record=-1;
     for(j=0;j<3;j=j+1)begin
      if(active[j] && ids[j*16+:16]==request_id)begin target=j;match_count=match_count+1;end
      if(used[j] && present[j] && bound_id[j]==request_id)record=j;
      if((!used[j] || !present[j]) && free_record<0)free_record=j;
     end
     if(request_id==0)result<=1;
     else if(match_count!=1)result<=6;
     else if(operation_exit)begin
      if(record<0)result<=4;else used[record]<=0;
     end else if(!observed[target])result<=2;
     else if(record>=0)result<=3;
     else if(free_record<0)result<=7;
     else begin
      used[free_record]<=1;bound_id[free_record]<=request_id;
      code[free_record]<=ordinal==0 ? 2'd1:ordinal==1 ? 2'd2:2'd3;
      if(ordinal<2)ordinal<=ordinal+1'b1;
     end
    end
   end
  end
 end
endmodule
