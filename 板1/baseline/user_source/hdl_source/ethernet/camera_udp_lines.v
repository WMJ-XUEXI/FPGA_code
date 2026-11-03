module camera_udp_lines (
    input wire pixel_clk,
    input wire pixel_reset,
    input wire pixel_de,
    input wire pixel_vsync,
    input wire [23:0] pixel_rgb,
    input wire tx_clk,
    input wire tx_reset,
    input wire udp_busy,
    output reg udp_req,
    output reg udp_valid,
    output reg [7:0] udp_data,
    output wire [15:0] udp_len
);
    // Two 640-pixel datagrams per line: 12-byte header followed by RGB565.
    assign udp_len = 16'd1292;
    reg de_d;
    reg vs_d;
    reg [10:0] x;
    reg [9:0] y;
    reg [15:0] frame_id;
    // The source is the tuned ISP output; do not brighten it a second time.
    wire [15:0] rgb565 = {pixel_rgb[23:19],pixel_rgb[15:10],pixel_rgb[7:3]};
    wire [52:0] fifo_in = {frame_id, y, x, rgb565};
    wire [11:0] written;
    reg capture_segment;
    wire segment_start=x==0 || x==640;
    wire segment_space=written<=12'd3455;
    wire fifo_write = pixel_de && y < 10'd720 && x < 11'd1280 &&
        (segment_start ? segment_space : capture_segment);

    always @(posedge pixel_clk or posedge pixel_reset) begin
        if (pixel_reset) begin
            de_d <= 1'b0;
            vs_d <= 1'b0;
            x <= 11'd0;
            y <= 10'd0;
            frame_id <= 16'd0;
            capture_segment <= 1'b0;
        end else begin
            if(pixel_de && segment_start) capture_segment<=segment_space;
            de_d <= pixel_de;
            vs_d <= pixel_vsync;
            if (pixel_vsync && !vs_d) begin
                y <= 10'd0;
                frame_id <= frame_id + 1'b1;
            end
            if (pixel_de) x <= x + 1'b1;
            else x <= 11'd0;
            if (!pixel_de && de_d) y <= y + 1'b1;
        end
    end

    wire [52:0] fifo_out;
    wire [11:0] available;
    wire fifo_empty;
    wire fifo_read;
    udp_pkg_buf #(
        .DATA_WIDTH_W(53), .DATA_WIDTH_R(53),
        .ADDR_WIDTH_W(12), .ADDR_WIDTH_R(12),
        .SHOW_AHEAD_EN(1'b1), .OUTREG_EN("NOREG")
    ) line_fifo (
        .rst(pixel_reset | tx_reset),
        .clkw(pixel_clk), .we(fifo_write), .di(fifo_in),
        .clkr(tx_clk), .re(fifo_read), .dout(fifo_out),
        .valid(), .full_flag(), .empty_flag(fifo_empty),
        .afull(), .aempty(), .wrusedw(written), .rdusedw(available)
    );

    localparam IDLE=3'd0, DROP=3'd1, ACK=3'd2,
               SEND=3'd3, DONE=3'd4;
    reg [2:0] state;
    reg [10:0] byte_index;
    reg [15:0] send_frame;
    reg [9:0] send_line;
    reg send_half;
    wire aligned = (fifo_out[26:16] == 11'd0 || fifo_out[26:16] == 11'd640);
    assign fifo_read = (state == DROP && !fifo_empty && !aligned) ||
                       (state == SEND && byte_index >= 11'd12 && byte_index <= 11'd1291 && byte_index[0]);
    always @(posedge tx_clk or posedge tx_reset) begin
        if (tx_reset) begin
            state <= IDLE;
            udp_req <= 1'b0;
            udp_valid <= 1'b0;
            udp_data <= 8'd0;
            byte_index <= 11'd0;
            send_frame <= 16'd0;
            send_line <= 10'd0;
            send_half <= 1'b0;
        end else begin
            udp_req <= 1'b0;
            udp_valid <= 1'b0;
            case (state)
                IDLE: begin
                    if (!fifo_empty && !aligned)
                        state <= DROP;
                    else if (available >= 12'd640 && !udp_busy) begin
                        send_frame <= fifo_out[52:37];
                        send_line <= fifo_out[36:27];
                        send_half <= (fifo_out[26:16] == 11'd640);
                        udp_req <= 1'b1;
                        state <= ACK;
                    end
                end
                DROP: begin
                    if (!fifo_empty) begin
                        if (aligned) state <= IDLE;
                    end
                end
                ACK: if (udp_busy) begin
                    byte_index <= 11'd0;
                    state <= SEND;
                end
                SEND: begin
                    udp_valid <= 1'b1;
                    case (byte_index)
                        11'd0: udp_data <= "P";
                        11'd1: udp_data <= "K";
                        11'd2: udp_data <= "V";
                        11'd3: udp_data <= "1";
                        11'd4: udp_data <= send_frame[15:8];
                        11'd5: udp_data <= send_frame[7:0];
                        11'd6: udp_data <= {6'd0,send_line[9:8]};
                        11'd7: udp_data <= send_line[7:0];
                        11'd8: udp_data <= 8'd5;
                        11'd9: udp_data <= 8'd0;
                        11'd10: udp_data <= 8'd2;
                        11'd11: udp_data <= {7'd0, send_half};
                        default: begin
                            udp_data <= byte_index[0] ? fifo_out[7:0] : fifo_out[15:8];
                        end
                    endcase
                    if (byte_index == 11'd1291) state <= DONE;
                    else byte_index <= byte_index + 1'b1;
                end
                DONE: if (!udp_busy) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
endmodule
