`timescale 1ns / 1ps

module darketh_mmio_tb;
    localparam int unsigned MAX_FRAME_BYTES = 8;

    localparam int unsigned REG_STATUS = 32'h00;
    localparam int unsigned REG_RX_LEN = 32'h04;
    localparam int unsigned REG_RX_DATA = 32'h08;
    localparam int unsigned REG_RX_CTRL = 32'h0c;

    localparam logic [31:0] STATUS_RX_AVAILABLE = 32'h0000_0001;
    localparam logic [31:0] STATUS_RX_OVERFLOW = 32'h0000_0002;
    localparam logic [31:0] STATUS_RX_DROPPED = 32'h0000_0004;
    localparam logic [31:0] STATUS_RX_BUSY = 32'h0000_0008;
    localparam logic [31:0] STATUS_RX_READY = 32'h0000_0100;

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

    logic [7:0] sample_frame [0:5];

    always #5 CLK = !CLK;

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
        .rx_irq(rx_irq)
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
        check_equal(data, STATUS_RX_READY, "reset status");

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

        $display("TEST PASS: darketh_mmio");
        $finish;
    end
endmodule
