`timescale 1ns/1ps
// Three tracks, one serial 3x3 greedy distance matcher. No global optimizer.
// Measurements and predictions are distinguished; merged/tied observations
// coast existing IDs rather than creating cloned identities.
module vehicle_tracks(
 input clk,rst_n,enable,start,frame_error,
 input [7:0] bay_w,
 input [2:0] candidate_valid,candidate_whole,
 input [23:0] candidate_l,candidate_r,
 input [20:0] candidate_t,candidate_b,
 input [8:0] candidate_pose,
 output reg done,ambiguous,
 output reg [2:0] active,observed,new_target,
 output [47:0] ids,
 output [23:0] lefts,rights,centers_x,
 output [20:0] tops,bottoms,centers_y,
 output [8:0] poses,
 output reg [2:0] whole,
 output [59:0] decimal_ids
);
 localparam IDLE=0,PREDICT=1,MERGE=2,SEARCH=3,COMPARE=4,APPLY=5,RETIRE=6,BIRTH=7,FINISH=8;
 reg [3:0] state;
 reg [1:0] ti,ci,best_t,best_c,round;
 reg [2:0] cv,cw,matched_t,matched_c,blocked;
 reg [23:0] cl,cr;reg [20:0] ct,cb;reg [8:0] cp;
 reg [7:0] l[0:2],r[0:2],cx[0:2],pred_x[0:2];
 reg [6:0] t[0:2],b[0:2],cy[0:2],pred_y[0:2];
 reg [15:0] identity[0:2],next_id;
 reg [3:0] missing[0:2];
 reg signed [4:0] vx[0:2],vy[0:2];
 reg [2:0] angle[0:2];
 reg [2:0] angle_pending[0:2];reg [2:0] angle_vote;
 reg [1:0] merge_count;
 reg [8:0] best_cost,cost_q;
 reg pair_ok,best_valid,tied;
 genvar g;
 generate for(g=0;g<3;g=g+1)begin: exports
  assign ids[g*16+:16]=identity[g];
  assign decimal_ids[g*20+:20]=20'd0; // Compatibility port, no BCD hardware.
  assign lefts[g*8+:8]=l[g];assign rights[g*8+:8]=r[g];
  assign tops[g*7+:7]=t[g];assign bottoms[g*7+:7]=b[g];
  assign centers_x[g*8+:8]=cx[g];assign centers_y[g*7+:7]=cy[g];
  assign poses[g*3+:3]=angle[g];
 end endgenerate
 function [7:0] predict_x;
  input [7:0] old;input signed [4:0] v;reg signed [9:0] sum;
  begin sum=$signed({2'd0,old})+v;predict_x=sum<0 ? 8'd0:sum>159 ? 8'd159:sum[7:0];end
 endfunction
 function [6:0] predict_y;
  input [6:0] old;input signed [4:0] v;reg signed [9:0] sum;
  begin sum=$signed({3'd0,old})+v;predict_y=sum<0 ? 7'd0:sum>89 ? 7'd89:sum[6:0];end
 endfunction
 function signed [4:0] velocity;
  input [7:0] fresh,old;reg signed [8:0] d;
  begin d=$signed({1'b0,fresh})-$signed({1'b0,old});velocity=d>8 ? 5'sd8:d< -8 ? -5'sd8:d[4:0];end
 endfunction
 wire [8:0] ccx2={1'b0,cl[ci*8+:8]}+{1'b0,cr[ci*8+:8]};
 wire [7:0] ccy2={1'b0,ct[ci*7+:7]}+{1'b0,cb[ci*7+:7]};
 wire [7:0] ccx=ccx2[8:1];wire [6:0] ccy=ccy2[7:1];
 wire [8:0] dx=pred_x[ti]>ccx ? {1'b0,pred_x[ti]}-{1'b0,ccx}:{1'b0,ccx}-{1'b0,pred_x[ti]};
 wire [7:0] dy=pred_y[ti]>ccy ? {1'b0,pred_y[ti]}-{1'b0,ccy}:{1'b0,ccy}-{1'b0,pred_y[ti]};
 wire [8:0] radius=({1'b0,bay_w}>>1)+9'd4;
 wire inside_box=active[ti] && pred_x[ti]>=cl[ci*8+:8] && pred_x[ti]<=cr[ci*8+:8] &&
  pred_y[ti]>=ct[ci*7+:7] && pred_y[ti]<=cb[ci*7+:7];
 wire [1:0] next_merge_count=merge_count+inside_box;
 wire free_available=active!=3'b111;
 wire [1:0] free_slot=!active[0] ? 2'd0:!active[1] ? 2'd1:2'd2;
 wire id_collision=(active[0] && identity[0]==next_id) ||
  (active[1] && identity[1]==next_id) || (active[2] && identity[2]==next_id);
 wire [15:0] increment_id=next_id==65535 ? 16'd1:next_id+1'b1;
 integer i;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;ti<=0;ci<=0;best_t<=0;best_c<=0;round<=0;cv<=0;cw<=0;cl<=0;cr<=0;ct<=0;cb<=0;cp<=0;
   matched_t<=0;matched_c<=0;blocked<=0;merge_count<=0;best_cost<=511;cost_q<=0;
   pair_ok<=0;best_valid<=0;tied<=0;done<=0;ambiguous<=0;active<=0;observed<=0;new_target<=0;whole<=0;next_id<=1;angle_vote<=0;
   for(i=0;i<3;i=i+1)begin
    l[i]<=0;r[i]<=0;t[i]<=0;b[i]<=0;cx[i]<=0;cy[i]<=0;pred_x[i]<=0;pred_y[i]<=0;
    identity[i]<=0;missing[i]<=0;vx[i]<=0;vy[i]<=0;angle[i]<=0;angle_pending[i]<=0;
   end
  end else begin
   done<=0;
   if(!enable)begin state<=IDLE;active<=0;observed<=0;new_target<=0;whole<=0;ambiguous<=0;end
   else case(state)
    IDLE:if(start)begin
     cv<=frame_error ? 3'd0:candidate_valid;cw<=candidate_whole;
     cl<=candidate_l;cr<=candidate_r;ct<=candidate_t;cb<=candidate_b;cp<=candidate_pose;
     matched_t<=0;matched_c<=0;blocked<=0;observed<=0;new_target<=0;whole<=0;
     ambiguous<=frame_error;round<=0;ti<=0;ci<=0;state<=PREDICT;
    end
    PREDICT:begin
     pred_x[ti]<=predict_x(cx[ti],vx[ti]);pred_y[ti]<=predict_y(cy[ti],vy[ti]);
     if(ti==2)begin ti<=0;ci<=0;merge_count<=0;state<=MERGE;end else ti<=ti+1'b1;
    end
    MERGE:begin
     if(ti==2)begin
      if(cv[ci] && next_merge_count>=2)begin blocked[ci]<=1;ambiguous<=1;end
      merge_count<=0;ti<=0;
      if(ci==2)begin ci<=0;best_cost<=511;best_valid<=0;tied<=0;state<=SEARCH;end else ci<=ci+1'b1;
     end else begin merge_count<=next_merge_count;ti<=ti+1'b1;end
    end
    SEARCH:begin
     cost_q<=dx+{1'b0,dy};
     pair_ok<=active[ti] && !matched_t[ti] && cv[ci] && !matched_c[ci] && !blocked[ci] && dx<=radius && {1'b0,dy}<=radius;
     state<=COMPARE;
    end
    COMPARE:begin
     if(pair_ok)begin
      if(!best_valid || cost_q<best_cost)begin best_valid<=1;best_cost<=cost_q;best_t<=ti;best_c<=ci;tied<=0;end
      else if(cost_q==best_cost && (ti==best_t || ci==best_c))tied<=1;
     end
     if(ci==2 && ti==2)state<=APPLY;
     else begin
      if(ci==2)begin ci<=0;ti<=ti+1'b1;end else ci<=ci+1'b1;
      state<=SEARCH;
     end
    end
    APPLY:if(best_valid && !tied)begin
     matched_t[best_t]<=1;matched_c[best_c]<=1;observed[best_t]<=1;whole[best_t]<=cw[best_c];
     l[best_t]<=cl[best_c*8+:8];r[best_t]<=cr[best_c*8+:8];
     t[best_t]<=ct[best_c*7+:7];b[best_t]<=cb[best_c*7+:7];
     if(cp[best_c*3+:3]==angle[best_t])angle_vote[best_t]<=0;
     else if(angle_vote[best_t] && angle_pending[best_t]==cp[best_c*3+:3])begin angle[best_t]<=cp[best_c*3+:3];angle_vote[best_t]<=0;end
     else begin angle_pending[best_t]<=cp[best_c*3+:3];angle_vote[best_t]<=1;end
     // Use best_c, not the final search index, to recover the measurement.
     cx[best_t]<=({1'b0,cl[best_c*8+:8]}+{1'b0,cr[best_c*8+:8]})>>1;
     cy[best_t]<=({1'b0,ct[best_c*7+:7]}+{1'b0,cb[best_c*7+:7]})>>1;
     if(missing[best_t]==0)begin
      vx[best_t]<=velocity(({1'b0,cl[best_c*8+:8]}+{1'b0,cr[best_c*8+:8]})>>1,cx[best_t]);
      vy[best_t]<=velocity(({1'b0,ct[best_c*7+:7]}+{1'b0,cb[best_c*7+:7]})>>1,{1'b0,cy[best_t]});
     end
     missing[best_t]<=0;ti<=0;ci<=0;best_cost<=511;best_valid<=0;tied<=0;
     if(round==2)state<=RETIRE;else begin round<=round+1'b1;state<=SEARCH;end
    end else begin
     if(tied)begin ambiguous<=1;blocked<=blocked | (cv & ~matched_c);end
     ti<=0;state<=RETIRE;
    end
    RETIRE:begin
     if(active[ti] && !matched_t[ti])begin
      if(missing[ti]==7)begin active[ti]<=0;missing[ti]<=0;end
      else begin
       missing[ti]<=missing[ti]+1'b1;cx[ti]<=pred_x[ti];cy[ti]<=pred_y[ti];
       l[ti]<=predict_x(l[ti],vx[ti]);r[ti]<=predict_x(r[ti],vx[ti]);
       t[ti]<=predict_y(t[ti],vy[ti]);b[ti]<=predict_y(b[ti],vy[ti]);
      end
     end
     if(ti==2)begin ci<=0;state<=BIRTH;end else ti<=ti+1'b1;
    end
    BIRTH:if(cv[ci] && !matched_c[ci] && !blocked[ci] && free_available && id_collision)begin next_id<=increment_id;end
    else begin
     if(cv[ci] && !matched_c[ci] && !blocked[ci] && free_available)begin
      active[free_slot]<=1;observed[free_slot]<=1;new_target[free_slot]<=1;whole[free_slot]<=cw[ci];
      identity[free_slot]<=next_id;next_id<=increment_id;
      
      l[free_slot]<=cl[ci*8+:8];r[free_slot]<=cr[ci*8+:8];t[free_slot]<=ct[ci*7+:7];b[free_slot]<=cb[ci*7+:7];
      cx[free_slot]<=ccx;cy[free_slot]<=ccy;angle[free_slot]<=cp[ci*3+:3];angle_vote[free_slot]<=0;
      vx[free_slot]<=0;vy[free_slot]<=0;missing[free_slot]<=0;
     end
     if(ci==2)state<=FINISH;else ci<=ci+1'b1;
    end
    FINISH:begin done<=1;state<=IDLE;end
    default:state<=IDLE;
   endcase
  end
 end
endmodule

