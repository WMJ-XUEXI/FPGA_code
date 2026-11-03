`timescale 1ns/1ps
module tb;
 reg clk=0;always #5 clk=~clk;reg rst=0,event_in=0;reg [48:0] plate=0;reg de=0;reg [10:0] x=0,xq=0;reg [9:0] y=0,yq=0;reg deq=0;reg [1:0] mode=0;reg hide=0;integer view_index;reg [56:0] stamp;wire [113:0] stamps;wire accepted,is_in;wire [48:0] accepted_plate;wire [65:0] summary;wire [5:0] unique_used;
 wire [5:0] used;wire [3:0] visible_rows=used>=8 ? 4'd8:used==0 ? 4'd1:used[3:0];wire [9:0] hy=y-(10'd656-{1'b0,visible_rows,5'd0});wire [86:0] record;wire ready,full;wire [15:0] ti,to,tf;wire [7:0] occ;
 parking_records records(clk,rst,1'b0,event_in,plate,{2'd0,hy[7:5]},record,ready,full,used,1'b0,ti,to,tf,occ,stamp,stamps,accepted,is_in,accepted_plate);
 plate_totals totals(clk,rst,1'b0,accepted,is_in,accepted_plate,{2'd0,hy[7:5]},summary,unique_used,);
 always @(posedge clk)begin xq<=x;yq<=y;deq<=de;end
 wire vs,hs,od;wire [23:0] rgb;
 recognition_osd osd(.clk(clk),.rst_n(rst),.vs(1'b0),.hs(1'b0),.de(deq),.rgb(24'h303030),.x(xq),.y(yq),.plate({49{1'b1}}),.locked(1'b0),.busy(1'b0),.stage(4'd0),.characters(4'd0),.preview_wr(1'b0),.preview_addr(10'd0),.preview_pixel(1'b0),.invalidate(1'b0),.reason(3'd0),.present(1'b0),.history_record(record),.history_ready(ready),.history_full(full),.history_used(used),.history_page(2'd0),.total_in(ti),.total_out(to),.total_flow(tf),.occupied(occ),.panel_mode(mode),.now_bcd(56'h20261031120000),.clock_valid(1'b1),.history_stamp(stamps),.summary_record(summary),.panel_hidden(hide),.out_vs(vs),.out_hs(hs),.out_de(od),.out_rgb(rgb));
 wire [3:0] state=records.state;wire pending=records.event_pending;
 task appear;input [48:0] value;input [55:0] timestamp;begin @(negedge clk);plate=value;stamp={1'b1,timestamp};event_in=1;@(negedge clk);event_in=0;repeat(4)@(negedge clk);while(state!=1 || pending)@(negedge clk);repeat(150)@(negedge clk);end endtask
 integer f,n=0,xx,yy;always @(negedge clk)if(od)begin $fwrite(f,"%c%c%c",rgb[23:16],rgb[15:8],rgb[7:0]);n=n+1;end
 initial begin
  repeat(4)@(negedge clk);rst=1;repeat(40)@(negedge clk);
  appear(49'he53a6409d90b,56'h20261030235950);appear(49'he53a6409d90b,56'h20261031000005);appear(49'h81ea84a9d496,56'h20261031000100);appear(49'he53a6409d90b,56'h20261031000200);
  if(used!=3 || ti!=3 || to!=1 || tf!=4 || occ!=2)$fatal(1,"timeline statistics mismatch");
  for(view_index=0;view_index<3;view_index=view_index+1)begin
  mode=view_index==1 ? 1:0;hide=view_index==2;
  n=0;f=hide ? $fopen("hidden_panel.ppm","wb"):mode==1 ? $fopen("timestamps_panel.ppm","wb"):$fopen("duration_panel.ppm","wb");$fwrite(f,"P6\n1280 720\n255\n");
  for(yy=0;yy<720;yy=yy+1)for(xx=0;xx<1280;xx=xx+1)begin @(negedge clk);x=xx;y=yy;de=1;end
  @(negedge clk);de=0;repeat(8)@(negedge clk);$fclose(f);if(n!=921600)$fatal(1,"timeline pixel count mismatch");
  end
  appear(49'd987654,56'h20261031130000);
  if(used!=4 || ti!=4 || to!=1 || occ!=3)$fatal(1,"hidden panel paused recording");
  $display("PASS compact and hidden panels: two rendered views from chronological record RAM and accepted-event aggregate RAM");$finish;
 end
endmodule
