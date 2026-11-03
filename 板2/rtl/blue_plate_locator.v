// Full-field 4x4 sampling, 3x3 Sobel, bounded row components. No frame RAM.
module blueplate_features(
 input clk,rst_n,valid,input [10:0] x,input [9:0] y,input [23:0] rgb,
 output reg feature_valid,output reg [8:0] fx,output reg [7:0] fy,
 output reg blue,edge_pixel
);
 wire sample=valid && x[1:0]==0 && y[1:0]==0 && x<1280 && y<720;
 wire [9:0] gray_sum={2'd0,rgb[23:16]}+{1'd0,rgb[15:8],1'b0}+{2'd0,rgb[7:0]};
 wire is_blue=rgb[7:0]>=48 && {1'b0,rgb[7:0]}>{1'b0,rgb[23:16]}+9'd16 &&
                                          {1'b0,rgb[7:0]}>{1'b0,rgb[15:8]}+9'd8 && {1'b0,rgb[7:0]}>={1'b0,rgb[23:16]}+{3'd0,rgb[7:2]} && {1'b0,rgb[7:0]}>={1'b0,rgb[15:8]}+{4'd0,rgb[7:3]};
 (* ram_style="bram" *)reg [7:0] line1[0:319],line2[0:319];
 (* ram_style="dram" *)reg blue_line[0:319];
 reg [7:0] read1,read2,gq;reg read_blue,bq,sample_q;
 reg [8:0] qx;reg [7:0] qy;
 reg [7:0] t0,t1,m0,m1,b0,b1;reg middle_blue;
 wire [10:0] right_sum={3'd0,read2}+{2'd0,read1,1'b0}+{3'd0,gq};
 wire [10:0] left_sum={3'd0,t0}+{2'd0,m0,1'b0}+{3'd0,b0};
 wire [10:0] top_sum={3'd0,t0}+{2'd0,t1,1'b0}+{3'd0,read2};
 wire [10:0] bottom_sum={3'd0,b0}+{2'd0,b1,1'b0}+{3'd0,gq};
 wire [10:0] dx=right_sum>=left_sum ? right_sum-left_sum:left_sum-right_sum;
 wire [10:0] dy=bottom_sum>=top_sum ? bottom_sum-top_sum:top_sum-bottom_sum;
 wire [11:0] magnitude={1'b0,dx}+{1'b0,dy};
 always @(posedge clk)begin
  if(sample)begin read1<=line1[x[10:2]];read2<=line2[x[10:2]];read_blue<=blue_line[x[10:2]];end
  if(sample_q)begin line1[qx]<=gq;line2[qx]<=read1;blue_line[qx]<=bq;end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   sample_q<=0;qx<=0;qy<=0;gq<=0;bq<=0;t0<=0;t1<=0;m0<=0;m1<=0;b0<=0;b1<=0;middle_blue<=0;
   feature_valid<=0;fx<=0;fy<=0;blue<=0;edge_pixel<=0;
  end else begin
   sample_q<=sample;
   if(sample)begin qx<=x[10:2];qy<=y[9:2];gq<=gray_sum[9:2];bq<=is_blue;end
   feature_valid<=0;
   if(sample_q)begin
    if(qx==0)begin t0<=0;m0<=0;b0<=0;end
    else begin t0<=t1;m0<=m1;b0<=b1;end
    t1<=read2;m1<=read1;b1<=gq;middle_blue<=read_blue;
    if(qx>=2 && qy>=2)begin
     feature_valid<=1;fx<=qx-1'b1;fy<=qy-1'b1;blue<=middle_blue;edge_pixel<=magnitude>=160;
    end
   end
  end
 end
endmodule

module blue_plate_locator(
 input clk,rst_n,frame_start,pixel_valid,clear,freeze,
 input [10:0] x,input [9:0] y,input [23:0] rgb,
 output reg box_valid,output reg candidate_valid,
 output reg [10:0] xmin,xmax,raw_xmin,raw_xmax,
 output reg [9:0] ymin,ymax,raw_ymin,raw_ymax,
 output [3:0] status,output reg [15:0] frame_sequence,
 output reg measurement_valid,output reg new_target
);
 wire fv,fb,fe;wire [8:0] fx;wire [7:0] fy;
 blueplate_features features(clk,rst_n,pixel_valid,x,y,rgb,fv,fx,fy,fb,fe);
 reg run_active;reg [4:0] gap;
 reg [8:0] run_left,run_right,run_blue,run_edges;reg [3:0] run_islands;
 wire finish_run=fv && run_active && ((!fb && gap==12) || fx==318);
 wire [9:0] run_width={1'b0,run_right}-{1'b0,run_left}+10'd1;
 wire run_good=run_width>=16 && run_width<=280 && {1'b0,run_blue}>=(run_width>>1);
 reg [3:0] active;
 reg [8:0] lefts[0:3],rights[0:3];reg [7:0] tops[0:3],bottoms[0:3];
 reg [15:0] blues[0:3],edges[0:3];reg [7:0] text_rows[0:3];
 reg best_valid;reg [8:0] best_l,best_r;reg [7:0] best_t,best_b;reg [15:0] best_score;
 wire choose_valid;wire [8:0] choose_l,choose_r;wire [7:0] choose_t,choose_b;
 reg [50:0] snapshots[0:3];reg [50:0] pair01,pair23,best_snapshot,current_winner,best_aligned,ranked;
 reg matched,free_found;reg [1:0] matched_index,free_index;
 reg [15:0] blue_count,edge_count;reg blue_seen,edge_seen;
 reg pending;reg [8:0] pending_l,pending_r;reg [7:0] pending_t,pending_b;reg [3:0] misses;
 integer k,j,s;
 function qualified;
  input [8:0] l,r;input [7:0] t,b;input [15:0] bn,en;input [7:0] tr;
  reg [9:0] w;reg [8:0] h;reg [15:0] hh,ww;
  begin
   w={1'b0,r}-{1'b0,l}+10'd1;h={1'b0,b}-{1'b0,t}+9'd1;hh={7'd0,h};ww={6'd0,w};
   qualified=w>=24 && w<=280 && h>=6 && h<=96 && ww>=(hh<<1) && ww<=((hh<<2)+(hh<<1)) &&
             {1'b0,bn}>=(ww<<1)+(hh<<2) && {1'b0,en}>=(hh<<3)+(hh<<1)+(ww>>1) && tr>=(h>=16 ? h>>3:9'd2);
  end
 endfunction
 // Require overlap in both axes and bounded size change. A larger blue
 // background cannot steal an established track solely through colour area.
 function associated;
  input [8:0] l,r;input [7:0] t,b;
  reg [9:0] w,tw,ix,ll,rr;reg [8:0] h,th,iy,tt,bb;
  begin
   w={1'b0,r}-{1'b0,l}+1;h={1'b0,b}-{1'b0,t}+1;
   tw={1'b0,xmax[10:2]}-{1'b0,xmin[10:2]}+1;th={1'b0,ymax[9:2]}-{1'b0,ymin[9:2]}+1;
   ll=l>xmin[10:2] ? {1'b0,l}:{1'b0,xmin[10:2]};rr=r<xmax[10:2] ? {1'b0,r}:{1'b0,xmax[10:2]};
   tt=t>ymin[9:2] ? {1'b0,t}:{1'b0,ymin[9:2]};bb=b<ymax[9:2] ? {1'b0,b}:{1'b0,ymax[9:2]};
   ix=rr>=ll ? rr-ll+1:0;iy=bb>=tt ? bb-tt+1:0;
   associated=box_valid && ix>=(w>>1) && ix>=(tw>>1) && iy>=(h>>1) && iy>=(th>>1) &&
    w<=tw+(tw>>2) && tw<=w+(w>>2) && h<=th+(th>>2) && th<=h+(h>>2);
  end
 endfunction
 function [15:0] candidate_score;
  input [8:0] l,r;input [7:0] t,b;input [15:0] count;
  begin candidate_score={associated(l,r,t,b),count[14:0]};end
 endfunction
 function weak_qualified;
  input [8:0] l,r;input [7:0] t,b;input [15:0] bn,en;input [7:0] tr;
  reg [9:0] w;reg [8:0] h;reg [15:0] ww,hh;
  begin w={1'b0,r}-{1'b0,l}+1;h={1'b0,b}-{1'b0,t}+1;ww=w;hh=h;
   weak_qualified=associated(l,r,t,b) && w>=24 && h>=6 && h<=96 && ww>=(hh<<1) && ww<=((hh<<2)+(hh<<1)) &&
     bn>=ww+(hh<<1) && en>=(hh<<3)+(ww>>1) && tr>=2;
  end
 endfunction
 function [10:0] filter_x;input [10:0] old_v,new_v;reg [11:0] delta,sum;begin
  delta=old_v>=new_v ? old_v-new_v:new_v-old_v;sum={1'b0,old_v}+{1'b0,new_v};
  filter_x=delta<=4 ? old_v:delta<=12 ? sum[11:1]:new_v;
 end endfunction
 function [9:0] filter_y;input [9:0] old_v,new_v;reg [10:0] delta,sum;begin
  delta=old_v>=new_v ? old_v-new_v:new_v-old_v;sum={1'b0,old_v}+{1'b0,new_v};
  filter_y=delta<=4 ? old_v:delta<=12 ? sum[10:1]:new_v;
 end endfunction
 function close_x;input [8:0] a,b;begin close_x=a>=b ? a-b<=20:b-a<=20;end endfunction
 function close_y;input [7:0] a,b;begin close_y=a>=b ? a-b<=12:b-a<=12;end endfunction
 function [10:0] screen_left;input [8:0] v;begin screen_left=v>=2 ? {v-9'd2,2'b00}:11'd0;end endfunction
 function [10:0] screen_right;input [8:0] v;begin screen_right=v<317 ? {v+9'd2,2'b11}:11'd1279;end endfunction
 function [9:0] screen_top;input [7:0] v;begin screen_top=v>=2 ? {v-8'd2,2'b00}:10'd0;end endfunction
 function [9:0] screen_bottom;input [7:0] v;begin screen_bottom=v<177 ? {v+8'd2,2'b11}:10'd719;end endfunction
 // Candidate qualification, pair ranking and final ranking use separate clocks.
 // Frame blanking provides ample settling time before frame_start consumes ranked.
 function [50:0] pick;
  input [50:0] a,b;
  begin pick=(!a[50] || (b[50] && b[49:34]>a[49:34])) ? b:a;end
 endfunction
 assign choose_valid=ranked[50];assign choose_l=ranked[33:25];assign choose_r=ranked[24:16];
 assign choose_t=ranked[15:8];assign choose_b=ranked[7:0];
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   for(s=0;s<4;s=s+1)snapshots[s]<=0;
   pair01<=0;pair23<=0;best_snapshot<=0;current_winner<=0;best_aligned<=0;ranked<=0;
  end else if(clear || frame_start)begin
   for(s=0;s<4;s=s+1)snapshots[s]<=0;
   pair01<=0;pair23<=0;best_snapshot<=0;current_winner<=0;best_aligned<=0;ranked<=0;
  end else begin
   for(s=0;s<4;s=s+1)snapshots[s]<={active[s] && (qualified(lefts[s],rights[s],tops[s],bottoms[s],blues[s],edges[s],text_rows[s]) || weak_qualified(lefts[s],rights[s],tops[s],bottoms[s],blues[s],edges[s],text_rows[s])),candidate_score(lefts[s],rights[s],tops[s],bottoms[s],blues[s]),lefts[s],rights[s],tops[s],bottoms[s]};
   pair01<=pick(snapshots[0],snapshots[1]);pair23<=pick(snapshots[2],snapshots[3]);
   best_snapshot<={best_valid,best_score,best_l,best_r,best_t,best_b};
   current_winner<=pick(pair01,pair23);best_aligned<=best_snapshot;
   ranked<=pick(best_aligned,current_winner);
  end
 end
 assign status={box_valid,measurement_valid,edge_seen,blue_seen};
 always @*begin
  matched=0;matched_index=0;free_found=0;free_index=0;
  for(j=3;j>=0;j=j-1)begin
   if(!active[j])begin free_found=1;free_index=j[1:0];end
   if(active[j] && {1'b0,fy}<={1'b0,bottoms[j]}+9'd2 &&
      {1'b0,run_left}<={1'b0,rights[j]}+10'd8 && {1'b0,run_right}+10'd8>={1'b0,lefts[j]})begin
    matched=1;matched_index=j[1:0];
   end
  end
  if(!free_found)begin
   for(j=3;j>=0;j=j-1)if({1'b0,fy}>{1'b0,bottoms[j]}+9'd2)begin free_found=1;free_index=j[1:0];end
  end

 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   run_active<=0;gap<=0;run_islands<=0;run_left<=0;run_right<=0;run_blue<=0;run_edges<=0;active<=0;
   best_valid<=0;best_l<=0;best_r<=0;best_t<=0;best_b<=0;best_score<=0;
   blue_count<=0;edge_count<=0;blue_seen<=0;edge_seen<=0;pending<=0;pending_l<=0;pending_r<=0;pending_t<=0;pending_b<=0;misses<=0;
   measurement_valid<=0;new_target<=0;box_valid<=0;candidate_valid<=0;xmin<=0;xmax<=0;ymin<=0;ymax<=0;raw_xmin<=0;raw_xmax<=0;raw_ymin<=0;raw_ymax<=0;frame_sequence<=0;
   for(k=0;k<4;k=k+1)begin lefts[k]<=0;rights[k]<=0;tops[k]<=0;bottoms[k]<=0;blues[k]<=0;edges[k]<=0;text_rows[k]<=0;end
  end else if(clear)begin
   run_active<=0;gap<=0;run_islands<=0;active<=0;best_valid<=0;best_score<=0;blue_count<=0;edge_count<=0;
   blue_seen<=0;edge_seen<=0;pending<=0;misses<=0;box_valid<=0;candidate_valid<=0;measurement_valid<=0;new_target<=0;
  end else if(frame_start)begin
   frame_sequence<=frame_sequence+1'b1;blue_seen<=blue_count>=64;edge_seen<=edge_count>=64;
   blue_count<=0;edge_count<=0;run_active<=0;gap<=0;run_islands<=0;active<=0;best_valid<=0;best_score<=0;
   new_target<=0;
   if(!freeze)begin
    candidate_valid<=choose_valid;measurement_valid<=0;
    if(choose_valid)begin raw_xmin<=screen_left(choose_l);raw_xmax<=screen_right(choose_r);raw_ymin<=screen_top(choose_t);raw_ymax<=screen_bottom(choose_b);end
    if(box_valid)begin
     if(choose_valid && associated(choose_l,choose_r,choose_t,choose_b))begin
      measurement_valid<=1;misses<=0;pending<=0;
      xmin<=filter_x(xmin,screen_left(choose_l));xmax<=filter_x(xmax,screen_right(choose_r));
      ymin<=filter_y(ymin,screen_top(choose_t));ymax<=filter_y(ymax,screen_bottom(choose_b));
     end else begin
      pending<=0;
      if(misses<1)misses<=misses+1'b1;else begin box_valid<=0;misses<=0;end
     end
    end else if(choose_valid)begin
     pending<=1;pending_l<=choose_l;pending_r<=choose_r;pending_t<=choose_t;pending_b<=choose_b;misses<=0;
     if(pending && close_x(choose_l,pending_l) && close_x(choose_r,pending_r) && close_y(choose_t,pending_t) && close_y(choose_b,pending_b))begin
      box_valid<=1;measurement_valid<=1;new_target<=1;
      xmin<=screen_left(choose_l);xmax<=screen_right(choose_r);ymin<=screen_top(choose_t);ymax<=screen_bottom(choose_b);
     end
    end else pending<=0;
   end
  end else begin
   new_target<=0;
   if(fv)begin
   if(fb)blue_count<=blue_count+1'b1;if(fe)edge_count<=edge_count+1'b1;
   if(finish_run)begin
    if(run_good)begin
     if(matched)begin
      if(run_left<lefts[matched_index])lefts[matched_index]<=run_left;
      if(run_right>rights[matched_index])rights[matched_index]<=run_right;
      if(run_islands>=4 && text_rows[matched_index]!=255)text_rows[matched_index]<=text_rows[matched_index]+1'b1;
      bottoms[matched_index]<=fy;blues[matched_index]<=blues[matched_index]+{7'd0,run_blue};edges[matched_index]<=edges[matched_index]+{7'd0,run_edges};
     end else if(free_found)begin
      if(active[free_index] && snapshots[free_index][50] &&
         (!best_valid || candidate_score(lefts[free_index],rights[free_index],tops[free_index],bottoms[free_index],blues[free_index])>best_score))begin
       best_valid<=1;best_l<=lefts[free_index];best_r<=rights[free_index];best_t<=tops[free_index];best_b<=bottoms[free_index];best_score<=candidate_score(lefts[free_index],rights[free_index],tops[free_index],bottoms[free_index],blues[free_index]);
      end
      active[free_index]<=1;lefts[free_index]<=run_left;rights[free_index]<=run_right;tops[free_index]<=fy;bottoms[free_index]<=fy;
      blues[free_index]<={7'd0,run_blue};edges[free_index]<={7'd0,run_edges};text_rows[free_index]<=run_islands>=4 ? 1:0;
     end
    end
    run_active<=0;gap<=0;run_islands<=0;run_blue<=0;run_edges<=0;
   end else if(fb)begin
    if(!run_active)begin run_islands<=1;run_left<=fx;run_blue<=1;run_edges<=fe ? 1:0;end
    else begin if(gap!=0 && run_islands<15)run_islands<=run_islands+1'b1;run_blue<=run_blue+1'b1;if(fe)run_edges<=run_edges+1'b1;end
    run_right<=fx;run_active<=1;gap<=0;
   end else if(run_active)begin
    if(fe)run_edges<=run_edges+1'b1;
    gap<=gap+1'b1;
   end
   end
  end
 end
endmodule
