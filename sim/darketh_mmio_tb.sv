`timescale 1ns / 1ps

module darketh_mmio_tb;
    localparam int unsigned MAX_FRAME_BYTES = 8;

    localparam int unsigned REG_STATUS = 32'h00;
    localparam int unsigned REG_RX_LEN = 32'h04;
    localparam int unsigned REG_RX_DATA = 32'h08;
    localparam int unsigned REG_RX_CTRL = 32'h0c;
    localparam int unsigned REG_TX_STATUS = 32'h10;
    localparam int unsigned REG_TX_LEN = 32'h14;
    localparam int unsigned REG_TX_DATA = 32'h18;
    localparam int unsigned REG_TX_CTRL = 32'h1c;
    localparam int unsigned REG_CFG_MAC_LO = 32'h20;
    localparam int unsigned REG_CFG_MAC_HI = 32'h24;
    localparam int unsigned REG_CFG_FLAGS = 32'h28;

    localparam logic [31:0] STATUS_RX_AVAILABLE = 32'h0000_0001;
    localparam logic [31:0] STATUS_RX_OVERFLOW = 32'h0000_0002;
    localparam logic [31:0] STATUS_RX_DROPPED = 32'h0000_0004;
    localparam logic [31:0] STATUS_RX_READY = 32'h0000_0100;

    localparam logic [31:0] STATUS_TX_READY = 32'h0000_0001;
    localparam logic [31:0] STATUS_TX_BUSY = 32'h0000_0002;
    localparam logic [31:0] STATUS_TX_OVERFLOW = 32'h0000_0004;
    localparam logic [31:0] STATUS_TX_DONE = 32'h0000_0008;
    localparam logic [31:0] STATUS_TX_WRITTEN = 32'h0000_0010;

    logic CLK = 1'b0;
    logic RES = 1'b1;

    logic XDREQ = 1'b0;
    logic XRD = 1'b0;
    logic XWR = 1'b0;
    logic [3:0] XBE = 4'hf;
    logic [31:0] XADDR = 32'd0;
    logic [31:0] XATAI = 32'd0;
    logic [31:0] XATAO;
    logic XDACK;

    logic rx_byte_valid = 1'b0;
    logic [7:0] rx_byte = 8'd0;
    logic rx_frame_valid = 1'b0;
    logic rx_frame_drop = 1'b0;

    logic rx_ready_for_frame;
    logic rx_frame_available;
    logic rx_overflow;
    logic rx_dropped;
    logic rx_busy;
    logic rx_irq;

    logic tx_byte_ready = 1'b1;
    logic tx_byte_valid;
    logic [7:0] tx_byte;
    logic tx_frame_start;
    logic tx_frame_end;
    logic tx_ready_for_frame;
    logic tx_busy;
    logic tx_done;
    logic tx_overflow;
    logic cfg_mac_filter_enable;
    logic [47:0] cfg_local_mac;
    logic cfg_accept_broadcast;
    logic cfg_accept_multicast;

    logic [7:0] sample_frame [0:5];
    logic [7:0] tx_seen [0:5];
    int tx_seen_count = 0;

    always #5 CLK = !CLK;

    always_ff @(posedge CLK) begin
        if (tx_byte_valid) begin
            if (tx_seen_count < 6) begin
                tx_seen[tx_seen_count] <= tx_byte;
            end
            tx_seen_count <= tx_seen_count + 1;
        end
    end

    darketh_mmio #(
        .MAX_FRAME_BYTES(MAX_FRAME_BYTES)
    ) dut (
        .CLK(CLK),
        .RES(RES),
        .XDREQ(XDREQ),
        .XRD(XRD),
        .XWR(XWR),
        .XBE(XBE),
        .XADDR(XADDR),
        .XATAI(XATAI),
        .XATAO(XATAO),
        .XDACK(XDACK),
        .rx_byte_valid(rx_byte_valid),
        .rx_byte(rx_byte),
        .rx_frame_valid(rx_frame_valid),
        .rx_frame_drop(rx_frame_drop),
        .rx_ready_for_frame(rx_ready_for_frame),
        .rx_frame_available(rx_frame_available),
        .rx_overflow(rx_overflow),
        .rx_dropped(rx_dropped),
        .rx_busy(rx_busy),
        .rx_irq(rx_irq),
        .tx_byte_ready(tx_byte_ready),
        .tx_byte_valid(tx_byte_valid),
        .tx_byte(tx_byte),
        .tx_frame_start(tx_frame_start),
        .tx_frame_end(tx_frame_end),
        .tx_ready_for_frame(tx_ready_for_frame),
        .tx_busy(tx_busy),
        .tx_done(tx_done),
        .tx_overflow(tx_overflow),
        .cfg_mac_filter_enable(cfg_mac_filter_enable),
        .cfg_local_mac(cfg_local_mac),
        .cfg_accept_broadcast(cfg_accept_broadcast),
        .cfg_accept_multicast(cfg_accept_multicast)
    );

    task automatic mmio_read(input logic [31:0] addr, output logic [31:0] data);
        begin
            @(posedge CLK);
            XADDR <= addr;
            XRD <= 1'b1;
            XWR <= 1'b0;
            XDREQ <= 1'b1;
            do @(posedge CLK); while (!XDACK);
            data = XATAO;
            XRD <= 1'b0;
            XDREQ <= 1'b0;
            @(posedge CLK);
        end
    endtask

    task automatic mmio_write(input logic [31:0] addr, input logic [31:0] data);
        begin
            @(posedge CLK);
            XADDR <= addr;
            XATAI <= data;
            XRD <= 1'b0;
            XWR <= 1'b1;
            XDREQ <= 1'b1;
            @(posedge CLK);
            XWR <= 1'b0;
            XDREQ <= 1'b0;
            @(posedge CLK);
        end
    endtask

    task automatic feed_byte(input logic [7:0] data);
        begin
            @(posedge CLK);
            rx_byte <= data;
            rx_byte_valid <= 1'b1;
            @(posedge CLK);
            rx_byte_valid <= 1'b0;
        end
    endtask

    task automatic commit_frame;
        begin
            @(posedge CLK);
            rx_frame_valid <= 1'b1;
            @(posedge CLK);
            rx_frame_valid <= 1'b0;
        end
    endtask

    task automatic drop_frame;
        begin
            @(posedge CLK);
            rx_frame_drop <= 1'b1;
            @(posedge CLK);
            rx_frame_drop <= 1'b0;
        end
    endtask

    task automatic check_equal(input logic [31:0] got, input logic [31:0] exp, input string what);
        begin
            if (got !== exp) begin
                $display("FAIL %s: got=%08x exp=%08x", what, got, exp);
                $fatal;
            end
        end
    endtask

    initial begin
        logic [31:0] data;

        sample_frame[0] = 8'hde;
        sample_frame[1] = 8'had;
        sample_frame[2] = 8'hbe;
        sample_frame[3] = 8'hef;
        sample_frame[4] = 8'h08;
        sample_frame[5] = 8'h00;

        repeat (4) @(posedge CLK);
        RES <= 1'b0;
        repeat (2) @(posedge CLK);

        mmio_read(REG_STATUS, data);
        check_equal(data, STATUS_RX_READY, "reset rx status");

        mmio_read(REG_TX_STATUS, data);
        check_equal(data, STATUS_TX_READY, "reset tx status");

        check_equal({31'd0, cfg_mac_filter_enable}, 32'd1, "reset cfg filter enable");
        check_equal(cfg_local_mac[31:0], 32'h2020_2001, "reset cfg mac lo");
        check_equal({16'd0, cfg_local_mac[47:32]}, 32'h0000_0220, "reset cfg mac hi");
        check_equal({29'd0, cfg_accept_multicast, cfg_accept_broadcast, cfg_mac_filter_enable},
                    32'h0000_0007,
                    "reset cfg flags");

        mmio_write(REG_CFG_MAC_LO, 32'haabb_ccdd);
        mmio_write(REG_CFG_MAC_HI, 32'h0000_0220);
        mmio_write(REG_CFG_FLAGS, 32'h0000_0005);

        mmio_read(REG_CFG_MAC_LO, data);
        check_equal(data, 32'haabb_ccdd, "cfg mac lo readback");
        mmio_read(REG_CFG_MAC_HI, data);
        check_equal(data, 32'h0000_0220, "cfg mac hi readback");
        mmio_read(REG_CFG_FLAGS, data);
        check_equal(data, 32'h0000_0005, "cfg flags readback");
        check_equal(cfg_local_mac[31:0], 32'haabb_ccdd, "cfg mac lo output");
        check_equal({16'd0, cfg_local_mac[47:32]}, 32'h0000_0220, "cfg mac hi output");
        check_equal({29'd0, cfg_accept_multicast, cfg_accept_broadcast, cfg_mac_filter_enable},
                    32'h0000_0005,
                    "cfg flags output");

        foreach (sample_frame[i]) begin
            feed_byte(sample_frame[i]);
        end
        commit_frame();

        mmio_read(REG_STATUS, data);
        check_equal(data & (STATUS_RX_AVAILABLE | STATUS_RX_READY), STATUS_RX_AVAILABLE, "frame available");

        mmio_read(REG_RX_LEN, data);
        check_equal(data, 32'd6, "frame length");

        foreach (sample_frame[i]) begin
            mmio_read(REG_RX_DATA, data);
            check_equal(data, {24'd0, sample_frame[i]}, "frame byte");
        end

        mmio_read(REG_STATUS, data);
        check_equal(data, STATUS_RX_READY, "empty after pop");

        feed_byte(8'h55);
        drop_frame();

        mmio_read(REG_STATUS, data);
        check_equal(data & (STATUS_RX_AVAILABLE | STATUS_RX_DROPPED), STATUS_RX_DROPPED, "drop flag");
        mmio_write(REG_RX_CTRL, 32'h0000_0002);

        for (int i = 0; i < MAX_FRAME_BYTES + 1; i++) begin
            feed_byte(i[7:0]);
        end
        commit_frame();

        mmio_read(REG_STATUS, data);
        check_equal(data & (STATUS_RX_AVAILABLE | STATUS_RX_OVERFLOW | STATUS_RX_DROPPED),
                    STATUS_RX_OVERFLOW | STATUS_RX_DROPPED,
                    "overflow drops frame");
        mmio_write(REG_RX_CTRL, 32'h0000_0002);

        mmio_read(REG_STATUS, data);
        check_equal(data, STATUS_RX_READY, "flags cleared");

        mmio_write(REG_TX_LEN, 32'd6);
        foreach (sample_frame[i]) begin
            mmio_write(REG_TX_DATA, {24'd0, sample_frame[i]});
        end

        mmio_read(REG_TX_STATUS, data);
        check_equal(data & (STATUS_TX_READY | STATUS_TX_WRITTEN),
                    STATUS_TX_READY | STATUS_TX_WRITTEN,
                    "tx frame staged");

        mmio_write(REG_TX_CTRL, 32'h0000_0001);

        do begin
            mmio_read(REG_TX_STATUS, data);
        end while (!(data & STATUS_TX_DONE));

        check_equal(tx_seen_count, 6, "tx byte count");
        foreach (sample_frame[i]) begin
            check_equal({24'd0, tx_seen[i]}, {24'd0, sample_frame[i]}, "tx byte");
        end
        check_equal(tx_frame_end, 1'b0, "tx frame end pulse cleared");

        mmio_write(REG_TX_CTRL, 32'h0000_0004);

        mmio_read(REG_TX_STATUS, data);
        check_equal(data, STATUS_TX_READY, "tx flags cleared");

        $display("TEST PASS: darketh_mmio rx tx");
        $finish;
    end
endmodule
