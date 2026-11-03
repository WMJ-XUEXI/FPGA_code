// Synchronous font RAM, one shared glyph lookup, matched video delay.
module recognition_osd(
 input clk,rst_n,vs,hs,de,input [23:0] rgb,input [10:0] x,input [9:0] y,
 input [48:0] plate,input locked,busy,input [3:0] stage,characters,
 input preview_wr,input [9:0] preview_addr,input preview_pixel,input invalidate,
 input [2:0] reason,input present,
 input [86:0] history_record,input history_ready,history_full,input [5:0] history_used,input [1:0] history_page,
 input [15:0] total_in,total_out,total_flow,input [7:0] occupied,
 output reg out_vs,out_hs,out_de,output reg [23:0] out_rgb,
 input [1:0] panel_mode,input [55:0] now_bcd,input clock_valid,
 input [113:0] history_stamp,input [65:0] summary_record,input panel_hidden
);
 (* rom_style="bram" *)reg [15:0] font[0:1274];
 initial $readmemh("E:/Qian_FPGA/example/cnn/sc520_confusion/model/osd_font.hex",font);
 (* ram_style="bram" *)reg preview[0:1023];
 reg preview_ready;reg preview_q;
 wire [7:0] px=x-11'd1008;wire [7:0] py=y-10'd160;
 always @(posedge clk)begin
  if(preview_wr)preview[preview_addr]<=preview_pixel;
  preview_q<=preview[{py[6:2],px[6:2]}];
 end
 always @(posedge clk or negedge rst_n)
  if(!rst_n)preview_ready<=0;else if(invalidate)preview_ready<=0;else if(preview_wr && preview_addr==1023)preview_ready<=1;
 wire text_area=x>=1008 && x<1232 && y>=48 && y<78;
 wire label_area=x>=1008 && x<1088 && y>=128 && y<143;
 wire preview_area=preview_ready && x>=1008 && x<1136 && y>=160 && y<288;
 wire [7:0] tx=x-11'd1008;
 reg [6:0] glyph;
 wire [6:0] safe_glyph=glyph<67 ? glyph:7'd67;
 reg [39:0] label_word;reg [7:0] label_ascii;
 always @*begin
  if(!present)label_word="SCAN ";else if(locked)label_word="READY";
  else if(reason==1)label_word="ANGLE";else if(reason==2)label_word="SPLIT";else if(reason==3)label_word="CHECK";
  else if(busy)label_word="READ ";else label_word="HOLD ";
  case(tx[6:4])
   0:label_ascii=label_word[39:32];1:label_ascii=label_word[31:24];2:label_ascii=label_word[23:16];
   3:label_ascii=label_word[15:8];default:label_ascii=label_word[7:0];
  endcase
 end
 wire time_mode=panel_mode==1;
 wire [5:0] page_base={1'b0,history_page,3'd0};
 wire [5:0] page_left=history_used>page_base ? history_used-page_base:6'd0;
 wire [3:0] row_count=page_left>=8 ? 4'd8:page_left==0 ? 4'd1:page_left[3:0];
 wire [9:0] rows_top=10'd656-{1'b0,row_count,5'd0};
 wire [9:0] panel_top=rows_top-10'd48;wire [9:0] head_top=panel_top+10'd16;
 wire [9:0] clock_y=y-panel_top,head_y=y-head_top;
 wire [10:0] panel_left=time_mode ? 11'd704:11'd864;
 wire history_panel=history_ready && !panel_hidden && x>=panel_left-16 && y>=panel_top;
 wire [9:0] hx=x-panel_left;wire [9:0] hy=y-rows_top;
 wire history_row=x>=panel_left && y>=rows_top && y<656 && (hy[4:0]<15 || (time_mode && hy[4:0]>=16 && hy[4:0]<31));
 wire clock_head=x>=panel_left && y>=panel_top && y<panel_top+15;
 wire history_head=x>=panel_left && y>=head_top && y<head_top+15;
 wire history_foot=x>=panel_left && ((y>=656 && y<671) || (y>=680 && y<695) || (y>=704 && y<719));
 wire history_text=history_panel && (clock_head || history_head || history_foot || (history_row && history_record[86]));
 wire [5:0] hc=hx[9:4];wire [5:0] row_number={1'b0,history_page,hy[7:5]}+6'd1;
 wire [3:0] row_tens=row_number>=30 ? 4'd3:row_number>=20 ? 4'd2:row_number>=10 ? 4'd1:4'd0;
 wire [3:0] row_ones=row_number>=30 ? row_number-30:row_number>=20 ? row_number-20:row_number>=10 ? row_number-10:row_number;
 reg [6:0] history_glyph;reg [223:0] line_word;reg [7:0] ascii;
 function [6:0] ascii_glyph;input [7:0] a;begin
  ascii_glyph=a>=65 && a<=90 ? a-24:a>=48 && a<=57 ? a-17:a==58 ? 7'd68:7'd69;
 end endfunction
 wire [56:0] row_stamp=hy[4] ? history_stamp[56:0]:history_stamp[113:57];
 function [6:0] date_glyph;input [55:0] st;input [5:0] pos;begin
  date_glyph=69;
  case(pos)
   0:date_glyph=31+st[55:52];1:date_glyph=31+st[51:48];2:date_glyph=31+st[47:44];3:date_glyph=31+st[43:40];4:date_glyph=79;
   5:date_glyph=31+st[39:36];6:date_glyph=31+st[35:32];7:date_glyph=80;
   8:date_glyph=31+st[31:28];9:date_glyph=31+st[27:24];10:date_glyph=81;
   12:date_glyph=31+st[23:20];13:date_glyph=31+st[19:16];14:date_glyph=68;
   15:date_glyph=31+st[15:12];16:date_glyph=31+st[11:8];17:date_glyph=68;
   18:date_glyph=31+st[7:4];19:date_glyph=31+st[3:0];
  endcase
 end endfunction
 always @*begin
  history_glyph=69;line_word=0;ascii=32;
  if(clock_head)begin
   if(clock_valid)history_glyph=date_glyph(now_bcd,hc);
   else case(hc)0:history_glyph=82;1:history_glyph=83;2:history_glyph=84;default:history_glyph=69;endcase
  end else if(history_row && time_mode)begin
   if(!hy[4])begin
    case(hc)
     0:history_glyph=31+row_tens;1:history_glyph=31+row_ones;2:history_glyph=68;
     3:history_glyph=history_record[42:36];4:history_glyph=history_record[49:43];5:history_glyph=history_record[56:50];6:history_glyph=history_record[63:57];7:history_glyph=history_record[70:64];8:history_glyph=history_record[77:71];9:history_glyph=history_record[84:78];
     11:history_glyph=49;12:history_glyph=54;
    endcase
   end else if(!history_record[85])begin
    case(hc)11:history_glyph=55;12:history_glyph=61;13:history_glyph=60;endcase
   end
   if(!hy[4] || !history_record[85])begin
    if(row_stamp[56])begin if(hc>=14 && hc<34)history_glyph=date_glyph(row_stamp[55:0],hc-14);end
    else case(hc)14:history_glyph=82;15:history_glyph=83;16:history_glyph=84;endcase
   end
  end else if(history_row)begin
   case(hc)
    0:history_glyph=31+row_tens;1:history_glyph=31+row_ones;2:history_glyph=68;
    3:history_glyph=history_record[42:36];4:history_glyph=history_record[49:43];5:history_glyph=history_record[56:50];
    6:history_glyph=history_record[63:57];7:history_glyph=history_record[70:64];8:history_glyph=history_record[77:71];9:history_glyph=history_record[84:78];
    11:history_glyph=31+history_record[23:20];12:history_glyph=31+history_record[19:16];13:history_glyph=68;
    14:history_glyph=31+history_record[15:12];15:history_glyph=31+history_record[11:8];16:history_glyph=68;
    17:history_glyph=31+history_record[7:4];18:history_glyph=31+history_record[3:0];
    20:history_glyph=history_record[85] ? 7'd49:7'd55;
    21:history_glyph=history_record[85] ? 7'd54:7'd61;
    22:history_glyph=history_record[85] ? 7'd69:7'd60;
   endcase
  end else begin
   if(history_head)line_word=time_mode ? "NO PLATE   IO DATE TIME     ":"NO PLATE   HH:MM:SS IO      ";
   else if(y<680)line_word="                            ";
   else if(y<704)line_word=history_full ? "       FULL K2 NEXT         ":"       P1 OF4 K2 NEXT       ";
   else line_word="K4 VIEW HOLD HIDE K2 PAGE   ";
   if(hc<28)ascii=line_word[223-hc*8 -:8];
   history_glyph=ascii_glyph(ascii);
   if(history_foot)begin
    if(y<680)begin
     case(hc)
      0:history_glyph=70;1:history_glyph=71;8:history_glyph=72;9:history_glyph=71;
      16:history_glyph=73;17:history_glyph=74;18:history_glyph=75;
      3:history_glyph=31+total_in[15:12];4:history_glyph=31+total_in[11:8];5:history_glyph=31+total_in[7:4];6:history_glyph=31+total_in[3:0];
      11:history_glyph=31+total_out[15:12];12:history_glyph=31+total_out[11:8];13:history_glyph=31+total_out[7:4];14:history_glyph=31+total_out[3:0];
      20:history_glyph=31+total_flow[15:12];21:history_glyph=31+total_flow[11:8];22:history_glyph=31+total_flow[7:4];23:history_glyph=31+total_flow[3:0];
     endcase
    end else if(y<704)begin
     if(hc==0)history_glyph=76;if(hc==1)history_glyph=77;if(hc==5)history_glyph=78;
     if(hc==3)history_glyph=31+occupied[7:4];if(hc==4)history_glyph=31+occupied[3:0];
     if(!history_full && hc==8)history_glyph=32+history_page;
    end
   end
  end
 end
 wire [6:0] selected_glyph=history_text ? (history_glyph<85 ? history_glyph:7'd67):label_area ? label_ascii-8'd24:safe_glyph;
 always @*begin
  glyph=67;
  case(tx[7:5])
   0:glyph=plate[6:0];1:glyph=plate[13:7];2:glyph=plate[20:14];3:glyph=plate[27:21];
   4:glyph=plate[34:28];5:glyph=plate[41:35];6:glyph=plate[48:42];
  endcase
 end
 wire [4:0] row=y-10'd48;
 wire [3:0] selected_row=history_text ? (clock_head ? clock_y[3:0]:history_row ? hy[3:0]:history_head ? head_y[3:0]:y<680 ? y-10'd656:y<704 ? y-10'd680:y-10'd704):label_area ? y[3:0]:text_area ? row[4:1]:4'd0;
 // Break record-RAM -> glyph decode -> font-RAM into separate cycles.
 reg [6:0] glyph_p;reg [3:0] row_p,col_p;
 reg text_p,history_p,colon_p,blank_p,preview_area_p,preview_data_p,vs_p,hs_p,de_p,locked_p;
 reg [23:0] rgb_p;
 reg [15:0] font_q;reg [3:0] col_q;reg text_q,preview_area_q,vs_q,hs_q,de_q,locked_q;
 reg [23:0] rgb_q;reg history_q,colon_q,blank_q;reg [3:0] row_q;
 wire markers=x>=1008 && x<1136 && y>=96 && y<112 && tx[4:0]<16;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin glyph_p<=69;row_p<=0;col_p<=0;text_p<=0;history_p<=0;colon_p<=0;blank_p<=0;preview_area_p<=0;preview_data_p<=0;vs_p<=0;hs_p<=0;de_p<=0;locked_p<=0;rgb_p<=0;history_q<=0;colon_q<=0;blank_q<=0;row_q<=0;font_q<=0;col_q<=0;text_q<=0;preview_area_q<=0;vs_q<=0;hs_q<=0;de_q<=0;rgb_q<=0;locked_q<=0;out_vs<=0;out_hs<=0;out_de<=0;out_rgb<=0;end
  else begin
   glyph_p<=selected_glyph<85 ? selected_glyph:7'd67;row_p<=selected_row;
   col_p<=history_text ? hx[3:0]:label_area ? tx[3:0]:tx[4:1];
   text_p<=history_text || text_area || (label_area && label_ascii!=32);history_p<=history_text;
   colon_p<=history_text && history_glyph==68;blank_p<=history_text && history_glyph==69;
   preview_area_p<=preview_area;preview_data_p<=preview_q;
   vs_p<=vs;hs_p<=hs;de_p<=de;locked_p<=locked;
   if(markers)rgb_p<=stage[tx[6:5]] ? 24'h00ff40:24'hff8000;
   else if(history_panel)rgb_p<=24'h101824;else rgb_p<=rgb;
   font_q<=font[({glyph_p,4'd0}-{4'd0,glyph_p})+{7'd0,row_p}];col_q<=col_p;text_q<=text_p;history_q<=history_p;
   colon_q<=colon_p;blank_q<=blank_p;row_q<=row_p;preview_area_q<=preview_area_p;
   vs_q<=vs_p;hs_q<=hs_p;de_q<=de_p;locked_q<=locked_p;rgb_q<=rgb_p;
   out_vs<=vs_q;out_hs<=hs_q;out_de<=de_q;
   if(!de_q)out_rgb<=0;
   else if(text_q && !blank_q && (colon_q ? (col_q>=6 && col_q<=8 && (row_q==5 || row_q==10)):font_q[15-col_q]))out_rgb<=history_q ? 24'hb0ffb0:locked_q ? 24'h00ff40:24'hffff00;
   else if(preview_area_q)out_rgb<=preview_data_p ? 24'hffffff:24'h101020;
   else out_rgb<=rgb_q;
  end
 end
endmodule
