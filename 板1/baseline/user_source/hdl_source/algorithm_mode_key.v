// K4: active-low, one event per debounced press, seven modes in order.
// Clock this block in the display pixel-clock domain (74.25 MHz at 720p60).
// Mode changes commit only on a one-clock frame_start pulse.
module algorithm_mode_key #(
    parameter integer DEBOUNCE_CYCLES = 1500000
)(
    input wire clk,
    input wire rst_n,
    input wire key4_n,
    input wire frame_start,
    output reg [2:0] requested_mode,
    output reg [2:0] active_mode
);
    localparam MODE_ISP = 3'd0;
    // 0=ISP, 1=GRAY, 2=SOBEL, 3=BINARY, 4=MOTION, 5=ENHANCE, 6=PARKING.
    (* async_reg = "true" *) reg key_meta, key_sync;
    reg stable_key;
    reg [20:0] debounce_count;
    wire changed_long_enough = (key_sync != stable_key)
                            && (debounce_count == DEBOUNCE_CYCLES-1);
    wire pressed = changed_long_enough && !key_sync;
    wire [2:0] next_requested = pressed
                              ? (requested_mode==3'd6 ? MODE_ISP : requested_mode+3'd1)
                              : requested_mode;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_meta <= 1'b1;
            key_sync <= 1'b1;
            stable_key <= 1'b1;
            debounce_count <= 0;
            requested_mode <= MODE_ISP;
            active_mode <= MODE_ISP;
        end else begin
            key_meta <= key4_n;
            key_sync <= key_meta;
            if (key_sync == stable_key) begin
                debounce_count <= 0;
            end else if (changed_long_enough) begin
                stable_key <= key_sync;
                debounce_count <= 0;
            end else begin
                debounce_count <= debounce_count + 21'd1;
            end
            requested_mode <= next_requested;
            if (frame_start) active_mode <= next_requested;
        end
    end
endmodule
