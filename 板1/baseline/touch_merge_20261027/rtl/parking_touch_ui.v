module parking_touch_ui(input clk,rst_n,input [255:0] debug_bus,
 input [11:0] diag_x,diag_y,input diag_irq,diag_contact,input [8:0] x,y,
 input [21:0] occupied,input valid,input [1:0] calibration,input menu,
 input [2:0] mode,input tracking,input [3:0] app_status,output [15:0] rgb);
 reg [15:0] bg,bg_q;reg [255:0] text;
 reg [8:0] tx,ty;reg [7:0] ch,ch_q;reg [3:0] row_q;reg [2:0] col_q;
 wire ink;integer i;integer bay_x,bay_y;reg [4:0] used;reg [4:0] free_count;
 function [7:0] digit;input integer n;begin digit=48+n;end endfunction
 reg [7:0] tens,ones,ft,fo;
 panel_font font(ch_q,row_q,col_q,ink);
 always @(posedge clk)begin bg_q<=bg;ch_q<=ch;row_q<=ty[3:0];col_q<=tx[2:0];end
 assign rgb=ink ? 16'hffff:bg_q;
 always @* begin
 used=0;for(i=0;i<22;i=i+1)used=used+occupied[i];
 free_count=22-used;
 tens=used>=20 ? 50:used>=10 ? 49:48;
 ones=48+(used>=20 ? used-20:used>=10 ? used-10:used);
 ft=free_count>=20 ? 50:free_count>=10 ? 49:48;
 fo=48+(free_count>=20 ? free_count-20:free_count>=10 ? free_count-10:free_count);
 bg=16'h0841;text=0;tx=0;ty=0;ch=32;
 if(y<16 && x>=8 && x<264)begin
 tx=x-8;ty=y;text={"PARKING 22",176'd0};
 end
 // Scene order: left 6, middle-left 5, middle-right 5, right 6.
 // Bit 0..5 = 01..06, bit 6..10 = 07..11, bit 11..15 = 12..16,
 // bit 16..21 = 17..22. Every column is numbered top to bottom.
 bay_x=0;bay_y=0;
 for(i=0;i<22;i=i+1)begin
 bay_x=i<6 ? 8:i<11 ? 126:i<16 ? 184:302;
 bay_y=i<6 ? 32+i*32:i<11 ? 64+(i-6)*32:i<16 ? 64+(i-11)*32:64+(i-16)*32;
 if(x>=bay_x && x<bay_x+54 && y>=bay_y && y<bay_y+28)begin
 bg=!valid ? 16'h4208:occupied[i] ? 16'hb800:16'h0340;
 if(x>=bay_x+18 && x<bay_x+34 && y>=bay_y+6 && y<bay_y+22)begin
 tx=x-bay_x-18;ty=y-bay_y-6;
 text={digit((i+1)/10),digit((i+1)%10),240'd0};
 end end end
 if(x>=376 && x<472)begin
 if(y>=32 && y<80)begin bg=16'h1949;if(y>=48 && y<64)begin text={"KEY1",224'd0};tx=x-408;ty=y-48;end end
 if(y>=88 && y<136)begin bg=16'h1949;if(y>=104 && y<120)begin text={"KEY2",224'd0};tx=x-408;ty=y-104;end end
 if(y>=144 && y<192)begin bg=tracking ? 16'h046c:16'h1949;if(y>=160 && y<176)begin text={"KEY3",224'd0};tx=x-408;ty=y-160;end end
 if(y>=200 && y<248)begin bg=16'h1949;if(y>=216 && y<232)begin text={"KEY4",224'd0};tx=x-408;ty=y-216;end end
 end
 if(y>=264 && y<280 && x>=8 && x<264)begin
 tx=x-8;ty=y-264;
 if(valid)text={"USED ",tens,ones,"  FREE ",ft,fo,"  TOTAL 22",48'd0};
 else case(app_status)
 1:text={"WARM UP",200'd0};
 2:text={"SCANNING",192'd0};
 3:text={"ANALYZING",184'd0};
 4:text={"KEY1 CONFIRM",160'd0};
 5:text={"LEARNING",192'd0};
 6:text={"UPDATING",192'd0};
 7:text={"TRACK LEARN KEY1",128'd0};
 8:text={"TRACK ACTIVE",160'd0};
 9:text={"RETRY KEY2",176'd0};
 10:text={"NO CAMERA",184'd0};
 default:text={"NO DATA SOURCE",144'd0};endcase
 end
 if(y>=264 && y<312 && x>=360 && x<472)begin
 bg=16'h1949;text={"MANAGE",208'd0};tx=x-392;ty=y-280;
 end
 if(tx<256 && ty<16)ch=text[255-tx[7:3]*8-:8];
 end
endmodule


