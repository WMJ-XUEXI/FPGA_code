`timescale 1ns/1ps

// Eleven bytes, every 12 video frames (5 Hz at 60 fps): A5 5A 01 seq flags occ[7:0]
// occ[15:8] occ[21:16] detected used xor(bytes 2..9).
module parking_uart_status #(
    parameter integer CLOCK_HZ = 74250000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire frame_start,
    input wire [21:0] occupied,
    input wire [4:0] detected,
    input wire [4:0] used,
    input wire valid,
    input wire strong_light,
    input wire recal_required,
    output wire tx
);
    // Reuse the existing frame pulse instead of a 24-bit pixel-clock timer.
    reg [3:0] report_frames;
    wire report_tick=frame_start && report_frames==4'd11;
    reg [1:0] state;
    reg [3:0] byte_index;
    reg [7:0] sequence;
    reg [7:0] flags_latched;
    reg [21:0] occupied_latched;
    reg [7:0] detected_latched, used_latched;
    reg tx_valid;
    reg [7:0] tx_data;
    wire tx_busy;

    wire [7:0] checksum = 8'h01 ^ sequence ^ flags_latched ^
        occupied_latched[7:0] ^ occupied_latched[15:8] ^
        {2'b00, occupied_latched[21:16]} ^ detected_latched ^ used_latched;

    function [7:0] packet_byte;
        input [3:0] index;
        input [7:0] seq, flags, found, busy_count, check;
        input [21:0] mask;
        begin
            case (index)
                0: packet_byte = 8'hA5;
                1: packet_byte = 8'h5A;
                2: packet_byte = 8'h01;
                3: packet_byte = seq;
                4: packet_byte = flags;
                5: packet_byte = mask[7:0];
                6: packet_byte = mask[15:8];
                7: packet_byte = {2'b00, mask[21:16]};
                8: packet_byte = found;
                9: packet_byte = busy_count;
                10: packet_byte = check;
                default: packet_byte = 8'h00;
            endcase
        end
    endfunction

    parking_uart_tx #(.CLOCK_HZ(CLOCK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n), .data(tx_data), .send(tx_valid),
        .busy(tx_busy), .tx(tx)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            report_frames <= 0;
            state <= 0;
            byte_index <= 0;
            sequence <= 0;
            flags_latched <= 0;
            occupied_latched <= 0;
            detected_latched <= 0;
            used_latched <= 0;
            tx_valid <= 0;
            tx_data <= 0;
        end else begin
            tx_valid <= 0;
            if (frame_start) begin
                if (report_tick) report_frames <= 0;
                else report_frames <= report_frames + 1'b1;
            end
            case (state)
                0: if (report_tick) begin
                    sequence <= sequence + 1'b1;
                    flags_latched <= {5'b0, recal_required, strong_light, valid};
                    occupied_latched <= valid ? occupied : 22'b0;
                    detected_latched <= {3'b0, detected};
                    used_latched <= valid ? {3'b0, used} : 8'b0;
                    byte_index <= 0;
                    state <= 1;
                end
                1: if (!tx_busy) begin
                    tx_data <= packet_byte(byte_index, sequence, flags_latched,
                        detected_latched, used_latched, checksum, occupied_latched);
                    tx_valid <= 1;
                    state <= 2;
                end
                2: if (tx_busy) state <= 3;
                3: if (!tx_busy) begin
                    if (byte_index == 10) state <= 0;
                    else begin
                        byte_index <= byte_index + 1'b1;
                        state <= 1;
                    end
                end
            endcase
        end
    end
endmodule

module parking_uart_tx #(
    parameter integer CLOCK_HZ = 74250000,
    parameter integer BAUD = 115200
) (
    input wire clk,
    input wire rst_n,
    input wire [7:0] data,
    input wire send,
    output reg busy,
    output reg tx
);
    localparam integer BIT_CYCLES = CLOCK_HZ / BAUD;
    reg [8:0] shift;
    reg [9:0] baud_count;
    reg [3:0] bit_index;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            busy <= 0;
            tx <= 1;
            shift <= 9'h1ff;
            baud_count <= 0;
            bit_index <= 0;
        end else if (!busy) begin
            tx <= 1;
            if (send) begin
                shift <= {1'b1, data};
                tx <= 0;
                busy <= 1;
                baud_count <= 0;
                bit_index <= 0;
            end
        end else if (baud_count == BIT_CYCLES - 1) begin
            baud_count <= 0;
            if (bit_index == 9) begin
                busy <= 0;
                tx <= 1;
            end else begin
                tx <= shift[0];
                shift <= {1'b1, shift[8:1]};
                bit_index <= bit_index + 1'b1;
            end
        end else begin
            baud_count <= baud_count + 1'b1;
        end
    end
endmodule
