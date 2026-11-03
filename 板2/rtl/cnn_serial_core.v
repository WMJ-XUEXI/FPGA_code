// One shared 8x8 MAC. Stream synchronous reads while consuming the prior term.
// Same integer arithmetic and accumulation order as the serial baseline.
`include "E:/Qian_FPGA/example/cnn/sc520_confusion/rtl/model_config.vh"
`ifndef CNN_HIDDEN_SIZE
`define CNN_HIDDEN_SIZE 32
`endif
module cnn_serial_core(
 input clk,rst_n,input_wr,input [9:0] input_addr,input [6:0] input_pixel,
 input start,input [1:0] alphabet,
 output reg busy,done,output reg [6:0] prediction,output reg [15:0] margin
);
 localparam ACC_BITS=`CNN_ACC_BITS;
 localparam HIDDEN=`CNN_HIDDEN_SIZE;
 (* ram_style="bram" *) reg [6:0] feature_a[0:1023];
 (* ram_style="bram" *) reg [6:0] feature_b[0:4095];
 (* rom_style="bram" *) reg signed [7:0] weights[0:323+355*HIDDEN];
 (* rom_style="dram" *) reg signed [ACC_BITS-1:0] biases[0:78+HIDDEN];
 initial begin $readmemh(`CNN_WEIGHTS_FILE,weights);$readmemh(`CNN_BIASES_FILE,biases);end
 localparam IDLE=0,BEGIN_OUT=1,LOAD_BIAS=2,READ=3,MAC=4,STORE=5,FINISH=6;
 (* fsm_encoding="sequential" *) reg [2:0] state;
 reg [2:0] phase;
 reg [6:0] oc;
 reg [4:0] ox,oy;
 reg [2:0] ic;
 reg [1:0] kx,ky,pool_index;
 reg [8:0] fi;
 reg [1:0] alphabet_q;
 reg signed [ACC_BITS-1:0] accumulator,best,second;
 reg [6:0] best_index;
 reg [6:0] pool_max;
 // Each feature RAM has its OWN synchronous read register. A shared mux
 // before one register prevented TD from mapping these arrays into BRAM.
 reg [6:0] read_a_q,read_b_q;
 wire is_pool=phase==1 || phase==3;
 wire [6:0] act_q=(is_pool || phase==5) ? read_b_q:read_a_q;
 reg signed [7:0] weight_q;
 reg signed [ACC_BITS-1:0] bias_q;
 reg [11:0] read_address,write_address;
 reg [14:0] weight_address;
 reg [6:0] bias_address;
 reg last_term_q;
 reg [4:0] shift;
 wire is_fc=phase>=4;
 wire last_term=is_pool ? pool_index==3 : is_fc ? fi==(phase==4 ? 9'd287:HIDDEN-1) :
                       kx==2 && ky==2 && ic==(phase==0 ? 0:3);
 wire last_position=is_fc || (ox==(phase==0 ? 29:phase==1 ? 14:phase==2 ? 12:5) &&
                             oy==(phase==0 ? 29:phase==1 ? 14:phase==2 ? 12:5));
 wire last_channel=oc==(phase<=1 ? 3:phase<=3 ? 7:phase==4 ? HIDDEN-1:66);
 wire permitted=alphabet_q==0 ? oc<31 : alphabet_q==1 ? (oc>=41 && oc!=49 && oc!=55) :
                alphabet_q==2 ? (oc>=31 && oc!=49 && oc!=55) : (oc>=31 && oc<=40);
 // Class49=I and55=O are excluded in ordinary-plate positions only.
 wire signed [14:0] product=$signed({1'b0,act_q})*$signed(weight_q);
 wire signed [ACC_BITS-1:0] score=accumulator>>>shift;
 wire [6:0] activation=score<=0 ? 7'd0 : score>=127 ? 7'd127 : score[6:0];
 wire [ACC_BITS-1:0] separation=best-second;
 // Resource/timing: fixed address scaling uses shifts/adds, not DSPs.
 function [11:0] times900; input [11:0] v; begin times900=(v<<9)+(v<<8)+(v<<7)+(v<<2);end endfunction
 function [11:0] times225; input [11:0] v; begin times225=(v<<8)-(v<<5)+v;end endfunction
 function [11:0] times169; input [11:0] v; begin times169=(v<<7)+(v<<5)+(v<<3)+v;end endfunction
 function [11:0] times36; input [11:0] v; begin times36=(v<<5)+(v<<2);end endfunction
 function [11:0] times30; input [11:0] v; begin times30=(v<<5)-(v<<1);end endfunction
 function [11:0] times15; input [11:0] v; begin times15=(v<<4)-v;end endfunction
 function [11:0] times13; input [11:0] v; begin times13=(v<<3)+(v<<2)+v;end endfunction
 function [11:0] times6; input [11:0] v; begin times6=(v<<2)+(v<<1);end endfunction
 function [13:0] weight9; input [13:0] v; begin weight9=(v<<3)+v;end endfunction
 function [13:0] weight3; input [13:0] v; begin weight3=(v<<1)+v;end endfunction
 function [13:0] weight36; input [13:0] v; begin weight36=(v<<5)+(v<<2);end endfunction
 function [13:0] weight288; input [13:0] v; begin weight288=(v<<8)+(v<<5);end endfunction
 function [14:0] weight_hidden; input [6:0] v; begin weight_hidden=(v<<5)+(HIDDEN==48 ? (v<<4):15'd0);end endfunction
 always @* begin
  read_address=0;write_address=0;weight_address=0;bias_address=0;shift=0;
  case(phase)
   0:begin
    read_address=(({7'd0,oy}+ky)<<5)+ox+kx;
    weight_address=weight9({7'd0,oc})+weight3({12'd0,ky})+kx;
    write_address=times900({5'd0,oc})+times30({7'd0,oy})+ox;
    bias_address=oc;shift=`CNN_SHIFT_0;
   end
   1:begin
    read_address=times900({5'd0,oc})+times30({6'd0,oy,1'b0}+pool_index[1])+{6'd0,ox,1'b0}+pool_index[0];
    write_address=times225({5'd0,oc})+times15({7'd0,oy})+ox;
   end
   2:begin
    read_address=times225({9'd0,ic})+times15({7'd0,oy}+ky)+ox+kx;
    weight_address=14'd36+weight36({7'd0,oc})+weight9({11'd0,ic})+weight3({12'd0,ky})+kx;
    write_address=times169({5'd0,oc})+times13({7'd0,oy})+ox;
    bias_address=7'd4+oc;shift=`CNN_SHIFT_1;
   end
   3:begin
    read_address=times169({5'd0,oc})+times13({6'd0,oy,1'b0}+pool_index[1])+{6'd0,ox,1'b0}+pool_index[0];
    write_address=times36({5'd0,oc})+times6({7'd0,oy})+ox;
   end
   4:begin read_address={3'd0,fi};weight_address=14'd324+weight288({7'd0,oc})+fi;
    write_address={5'd0,oc};bias_address=7'd12+oc;shift=`CNN_SHIFT_2;end
   5:begin read_address={3'd0,fi};weight_address=324+288*HIDDEN+weight_hidden(oc)+fi;
    bias_address=12+HIDDEN+oc;shift=`CNN_SHIFT_3;end
  endcase
 end
 // A stores input (1024), Pool1 (900) or Pool2 (288), never convolution outputs.
 // Inactive-bank reads are disabled rather than addressing outside its live range.
 // Read outputs have no reset, enabling inferred RAM rather than FF banks.
 always @(posedge clk) begin
  if(input_wr && !busy)feature_a[input_addr]<=input_pixel;
  else if(state==STORE && is_pool)feature_a[write_address[9:0]]<=pool_max;
  if(state==BEGIN_OUT && !is_pool)bias_q<=biases[bias_address];
  if(state==READ || (state==MAC && !last_term_q))begin
   if(!is_pool && phase!=5)read_a_q<=feature_a[read_address[9:0]];
   if(is_pool || phase==5)read_b_q<=feature_b[read_address];
   if(!is_pool)weight_q<=weights[weight_address];
  end
  if(state==STORE)begin
   if(!is_pool && phase!=5)feature_b[write_address]<=activation;
  end
 end
 always @(posedge clk or negedge rst_n)begin
  if(!rst_n)begin
   state<=IDLE;busy<=0;done<=0;prediction<=127;margin<=0;phase<=0;oc<=0;ox<=0;oy<=0;
   ic<=0;kx<=0;ky<=0;fi<=0;pool_index<=0;pool_max<=0;accumulator<=0;alphabet_q<=0;
   best<={1'b1,{(ACC_BITS-1){1'b0}}};second<={1'b1,{(ACC_BITS-1){1'b0}}};best_index<=127;last_term_q<=0;
  end else begin
   done<=0;
   // Address counters point to the next request; read data is consumed one
   // clock later. Delay end-of-dot-product with the RAM outputs, including
   // the final pool element. No read is issued past the final term.
   if(state==READ || (state==MAC && !last_term_q))begin
    last_term_q<=last_term;
    if(!last_term)begin
     if(is_pool)pool_index<=pool_index+1'b1;
     else if(is_fc)fi<=fi+1'b1;
     else if(kx==2)begin kx<=0;if(ky==2)begin ky<=0;ic<=ic+1'b1;end else ky<=ky+1'b1;end
     else kx<=kx+1'b1;
    end
   end
   case(state)
    IDLE:if(start)begin
     busy<=1;phase<=0;oc<=0;ox<=0;oy<=0;alphabet_q<=alphabet;
     best<={1'b1,{(ACC_BITS-1){1'b0}}};second<={1'b1,{(ACC_BITS-1){1'b0}}};best_index<=127;state<=BEGIN_OUT;
    end
    BEGIN_OUT:begin ic<=0;kx<=0;ky<=0;fi<=0;pool_index<=0;pool_max<=0;state<=LOAD_BIAS;end
    LOAD_BIAS:begin accumulator<=is_pool ? {ACC_BITS{1'b0}}:bias_q;state<=READ;end
    READ:state<=MAC;
    MAC:begin
     if(is_pool)begin if(act_q>pool_max)pool_max<=act_q;end
     else accumulator<=accumulator+{{(ACC_BITS-15){product[14]}},product};
     if(last_term_q)state<=STORE;
    end
    STORE:begin
     if(phase==5 && permitted)begin
      if(score>best)begin second<=best;best<=score;best_index<=oc;end
      else if(score>second)second<=score;
     end
     if(last_position && last_channel)begin
      if(phase==5)state<=FINISH;
      else begin phase<=phase+1'b1;oc<=0;ox<=0;oy<=0;state<=BEGIN_OUT;end
     end else begin
      if(last_position)begin oc<=oc+1'b1;ox<=0;oy<=0;end
      else if(ox==(phase==0 ? 29:phase==1 ? 14:phase==2 ? 12:5))begin ox<=0;oy<=oy+1'b1;end
      else ox<=ox+1'b1;
      state<=BEGIN_OUT;
     end
    end
    FINISH:begin
     prediction<=separation>=4 ? best_index:7'd127;
     margin<=|separation[ACC_BITS-1:16] ? 16'hffff:separation[15:0];
     busy<=0;done<=1;state<=IDLE;
    end
    default:begin state<=IDLE;busy<=0;end
   endcase
  end
 end
endmodule
