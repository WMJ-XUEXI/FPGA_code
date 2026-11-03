// UART 115200 8N1: TYYYYMMDDhhmmss\n. No date is invented before calibration.
module calendar_clock #(parameter CLOCK_HZ=75000000,BAUD=115200)(
 input clk,rst_n,uart_rx,output uart_tx,
 output reg [55:0] now_bcd,output reg valid,output reg day_changed
);
 localparam BIT_CYCLES=(CLOCK_HZ+BAUD/2)/BAUD;
 (* ASYNC_REG="TRUE" *)reg rx_meta,rx_sync;
 reg [15:0] rx_count;reg [3:0] rx_state;reg [7:0] rx_byte;reg byte_ready;
 reg [55:0] command;reg [4:0] digits;reg collecting;
 reg [31:0] divider;
 reg [9:0] tx_shift;reg [3:0] tx_left;reg [15:0] tx_count;
 assign uart_tx=tx_left!=0 ? tx_shift[0]:1'b1;
 function [7:0] inc2;input [7:0] a;begin inc2=a[3:0]==9 ? {a[7:4]+4'd1,4'd0}:a+1'b1;end endfunction
 function [15:0] inc4;input [15:0] a;begin
  if(a[3:0]!=9)inc4=a+1'b1;
  else if(a[7:4]!=9)inc4={a[15:4]+12'd1,4'd0};
  else if(a[11:8]!=9)inc4={a[15:8]+8'd1,8'd0};
  else inc4={a[15:12]+4'd1,12'd0};
 end endfunction
 function leap;input [15:0] year;reg [2:0] lo,hi;begin
  lo={1'b0,year[4],1'b0}+{1'b0,year[1:0]};
  hi={1'b0,year[12],1'b0}+{1'b0,year[9:8]};
  leap=lo[1:0]==0 && (year[7:0]!=0 || hi[1:0]==0);
 end endfunction
 function [7:0] month_days;input [15:0] year;input [7:0] month;begin
  case(month)
   8'h02:month_days=leap(year) ? 8'h29:8'h28;
   8'h04,8'h06,8'h09,8'h11:month_days=8'h30;
   default:month_days=8'h31;
  endcase
 end endfunction
 wire command_valid=digits==14 && command[55:40]>=16'h2000 &&
  command[39:32]>=1 && command[39:32]<=8'h12 && command[31:24]>=1 &&
  command[31:24]<=month_days(command[55:40],command[39:32]) &&
  command[23:16]<=8'h23 && command[15:8]<=8'h59 && command[7:0]<=8'h59;
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin rx_meta<=1;rx_sync<=1;rx_count<=0;rx_state<=0;rx_byte<=0;byte_ready<=0;end
  else begin
   rx_meta<=uart_rx;rx_sync<=rx_meta;byte_ready<=0;
   if(rx_state==0)begin if(!rx_sync)begin rx_count<=BIT_CYCLES/2-1;rx_state<=1;end end
   else if(rx_count!=0)rx_count<=rx_count-1'b1;
   else begin
    rx_count<=BIT_CYCLES-1;
    if(rx_state==1)begin if(!rx_sync)rx_state<=2;else rx_state<=0;end
    else if(rx_state<=9)begin rx_byte[rx_state-2]<=rx_sync;rx_state<=rx_state+1'b1;end
    else begin byte_ready<=rx_sync;rx_state<=0;end
   end
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin now_bcd<=0;valid<=0;day_changed<=0;divider<=0;command<=0;digits<=0;collecting<=0;tx_shift<=10'h3ff;tx_left<=0;tx_count<=0;end
  else begin
   day_changed<=0;
   if(tx_left!=0)begin
    if(tx_count!=0)tx_count<=tx_count-1'b1;
    else begin tx_shift<={1'b1,tx_shift[9:1]};tx_left<=tx_left-1'b1;tx_count<=BIT_CYCLES-1;end
   end
   if(divider==CLOCK_HZ-1)begin
    divider<=0;
    if(valid)begin
     if(now_bcd[7:0]!=8'h59)now_bcd[7:0]<=inc2(now_bcd[7:0]);
     else begin
      now_bcd[7:0]<=0;
      if(now_bcd[15:8]!=8'h59)now_bcd[15:8]<=inc2(now_bcd[15:8]);
      else begin
       now_bcd[15:8]<=0;
       if(now_bcd[23:16]!=8'h23)now_bcd[23:16]<=inc2(now_bcd[23:16]);
       else begin
        now_bcd[23:0]<=0;day_changed<=1;
        if(now_bcd[31:24]!=month_days(now_bcd[55:40],now_bcd[39:32]))now_bcd[31:24]<=inc2(now_bcd[31:24]);
        else begin
         now_bcd[31:24]<=1;
         if(now_bcd[39:32]!=8'h12)now_bcd[39:32]<=inc2(now_bcd[39:32]);
         else begin now_bcd[39:32]<=1;now_bcd[55:40]<=inc4(now_bcd[55:40]);end
        end
       end
      end
     end
    end
   end else divider<=divider+1'b1;
   if(byte_ready)begin
    if(rx_byte=="T")begin collecting<=1;digits<=0;command<=0;end
    else if(collecting && rx_byte==10)begin
     collecting<=0;
     if(command_valid)begin
      now_bcd<=command;valid<=1;divider<=0;
      day_changed<=!valid || command[55:24]!=now_bcd[55:24];
     end
     if(tx_left==0)begin tx_shift<={1'b1,command_valid ? 8'h4b:8'h45,1'b0};tx_left<=10;tx_count<=BIT_CYCLES-1;end
    end
    else if(collecting && rx_byte!=13)begin
     if(rx_byte>="0" && rx_byte<="9" && digits<14)begin command<={command[51:0],rx_byte[3:0]};digits<=digits+1'b1;end
     else begin collecting<=0;digits<=0;if(tx_left==0)begin tx_shift<={1'b1,8'h45,1'b0};tx_left<=10;tx_count<=BIT_CYCLES-1;end end
    end
   end
  end
 end
endmodule
