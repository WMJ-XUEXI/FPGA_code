`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,event_in=0;reg [48:0] plate=0;reg de=0;reg [10:0] x=0,xq=0;reg [9:0] y=0,yq=0;reg deq=0;
 wire [5:0] used;wire [3:0] visible_rows=used>=8 ? 4'd8:used==0 ? 4'd1:used[3:0];wire [9:0] hy=y-(10'd656-{1'b0,visible_rows,5'd0});wire [86:0] record;wire ready,full;wire [15:0] ti,to,tf;wire [7:0] occ;
 parking_records records(clk,rst,1'b0,event_in,plate,{2'd0,hy[7:5]},record,ready,full,used,1'b0,ti,to,tf,occ);
 always @(posedge clk)begin xq<=x;yq<=y;deq<=de;end
 wire vs,hs,od;wire [23:0] rgb;
 recognition_osd osd(.clk(clk),.rst_n(rst),.vs(1'b0),.hs(1'b0),.de(deq),.rgb(24'h303030),.x(xq),.y(yq),.plate({49{1'b1}}),.locked(1'b0),.busy(1'b0),.stage(4'd0),.characters(4'd0),.preview_wr(1'b0),.preview_addr(10'd0),.preview_pixel(1'b0),.invalidate(1'b0),.reason(3'd0),.present(1'b0),.history_record(record),.history_ready(ready),.history_full(full),.history_used(used),.history_page(2'd0),.total_in(ti),.total_out(to),.total_flow(tf),.occupied(occ),.panel_mode(2'd0),.now_bcd(56'd0),.clock_valid(1'b0),.history_stamp(114'd0),.summary_record(66'd0),.panel_hidden(1'b0),.out_vs(vs),.out_hs(hs),.out_de(od),.out_rgb(rgb));
 wire [3:0] state=records.state;wire pending=records.event_pending;
 task appear;input [48:0] value;begin @(negedge clk);plate=value;event_in=1;@(negedge clk);event_in=0;repeat(4)@(negedge clk);while(state!=1 || pending)@(negedge clk);end endtask
 integer f,n=0,xx,yy;always @(negedge clk)if(od)begin $fwrite(f,"%c%c%c",rgb[23:16],rgb[15:8],rgb[7:0]);n=n+1;end
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(40)@(negedge clk);
  appear(49'he53a6409d90b);appear(49'he53a6409d90b);appear(49'h81ea84a9d496);appear(49'he53a6409d90b);
  if(used!=3 || ti!=3 || to!=1 || tf!=4 || occ!=2)$fatal(1,"timeline statistics mismatch");
  f=$fopen("timeline_preview.ppm","wb");$fwrite(f,"P6\n1280 720\n255\n");
  for(yy=0;yy<720;yy=yy+1)for(xx=0;xx<1280;xx=xx+1)begin @(negedge clk);x=xx;y=yy;de=1;end
  @(negedge clk);de=0;repeat(8)@(negedge clk);$fclose(f);if(n!=921600)$fatal(1,"timeline pixel count mismatch");
  $display("PASS actual chronological record RAM -> OSD: A OUT row1, B IN row2, A IN row3, IN3 OUT1 FLOW4 PARK2");$finish;
 end
endmodule
