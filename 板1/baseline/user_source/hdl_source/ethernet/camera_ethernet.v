module camera_ethernet (
    input wire sys_clk,
    input wire pixel_clk,
    input wire reset_n,
    input wire pixel_de,
    input wire pixel_vsync,
    input wire [23:0] pixel_rgb,
    output wire phy_reset_n,
    input wire [3:0] rgmii_rxd,
    input wire rgmii_rx_ctl,
    input wire rgmii_rxc,
    output wire [3:0] rgmii_txd,
    output wire rgmii_tx_ctl,
    output wire rgmii_txc
);
    wire clk_125, clk_125_90, clk_25, pll_lock;
    pll ethernet_pll (
        .refclk(sys_clk), .reset(1'b0), .lock(pll_lock),
        .clk0_out(clk_125), .clk1_out(clk_125_90), .clk2_out(clk_25)
    );
    uiphyrst #(.CLK_FREQ(32'd25_000_000)) phy_reset (
        .I_CLK(clk_25), .I_rstn(pll_lock & reset_n),
        .I_phyrst(pll_lock), .O_phyrst(phy_reset_n)
    );
    wire reset = !phy_reset_n;
    wire busy, req, valid;
    wire [7:0] data;
    wire [15:0] length;
    camera_udp_lines packetizer (
        .pixel_clk(pixel_clk), .pixel_reset(!reset_n),
        .pixel_de(pixel_de), .pixel_vsync(pixel_vsync), .pixel_rgb(pixel_rgb),
        .tx_clk(clk_125), .tx_reset(reset), .udp_busy(busy),
        .udp_req(req), .udp_valid(valid), .udp_data(data), .udp_len(length)
    );
    wire gmii_rxclk, gmii_rxvalid, gmii_txvalid;
    wire [7:0] gmii_rxdata, gmii_txdata;
    uiudp_stack stack (
        .I_uclk(clk_125), .I_reset(reset),
        .I_mac_local_addr(48'h02123456789a),
        .I_udp_local_port(16'd6002),
        .I_ip_local_addr(32'hc0a88902),
        .I_udp_dest_port(16'd16001),
        .I_ip_dest_addr(32'hc0a88901),
        .O_W_udp_busy(busy), .I_W_udp_req(req),
        .I_W_udp_valid(valid), .I_W_udp_data(data), .I_W_udp_len(length),
        .O_R_udp_valid(), .O_R_udp_data(), .O_R_udp_len(), .O_R_udp_src_port(),
        .I_gmii_rclk(gmii_rxclk), .I_gmii_rvalid(gmii_rxvalid),
        .I_gmii_rdata(gmii_rxdata),
        .I_gmii_tclk(clk_125), .O_gmii_tvalid(gmii_txvalid),
        .O_gmii_tdata(gmii_txdata), .O_ip_rerror(), .O_mac_rerror()
    );
    rgmii_interface phy_interface (
        .speed_10_100(1'b0),
        .gmii_tx_clk_d(clk_125), .gmii_tx_reset_d(reset),
        .gmii_tx_reset(reset), .gmii_tx_clk(clk_125),
        .gmii_txd(gmii_txdata), .gmii_tx_en(gmii_txvalid), .gmii_tx_er(1'b0),
        .gmii_rx_reset(reset), .gmii_rx_clk(gmii_rxclk),
        .gmii_rxd(gmii_rxdata), .gmii_rx_dv(gmii_rxvalid), .gmii_rx_er(),
        .rgmii_txd(rgmii_txd), .rgmii_tx_ctl(rgmii_tx_ctl), .rgmii_txc(rgmii_txc),
        .rgmii_rxd(rgmii_rxd), .rgmii_rx_ctl(rgmii_rx_ctl), .rgmii_rxc(rgmii_rxc),
        .gmii_crs(), .gmii_col(), .link_status(), .clock_speed(), .duplex_status()
    );
endmodule
