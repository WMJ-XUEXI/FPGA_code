// SW1: 1=normal/strong light, 0=low light. 24 MHz, 20 ms debounce.
module light_mode_switch #(
    parameter integer DEBOUNCE_CYCLES=480000
)(
    input wire clk,rst_n,sw1,
    output reg strong_light
);
    (* async_reg="true" *) reg sw_meta,sw_sync;
    reg [18:0] stable_count;
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sw_meta<=1;sw_sync<=1;strong_light<=1;stable_count<=0;
        end else begin
            sw_meta<=sw1;sw_sync<=sw_meta;
            if(sw_sync==strong_light) stable_count<=0;
            else if(stable_count==DEBOUNCE_CYCLES-1) begin
                strong_light<=sw_sync;stable_count<=0;
            end else stable_count<=stable_count+1'b1;
        end
    end
endmodule
