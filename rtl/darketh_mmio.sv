/*
 * DarkRISCV memory-mapped Ethernet frame endpoint.
 *
 * RX stores one complete Ethernet frame without preamble/SFD/FCS and exposes
 * it to firmware as a byte pop register. TX accepts a firmware-written frame
 * and emits bytes for the FPGA Ethernet TX path.
 */

`timescale 1ns / 1ps

module darketh_mmio #(
    parameter int unsigned MAX_FRAME_BYTES = 2048
)(
    input  logic        CLK,
    input  logic        RES,

    input  logic        XDREQ,
    input  logic        XRD,
    input  logic        XWR,
    input  logic [3:0]  XBE,
    input  logic [31:0] XADDR,
    input  logic [31:0] XATAI,
    output logic [31:0] XATAO,
    output logic        XDACK,

    input  logic        rx_byte_valid,
    input  logic [7:0]  rx_byte,
    input  logic        rx_frame_valid,
    input  logic        rx_frame_drop,

    output logic        rx_ready_for_frame,
    output logic        rx_frame_available,
    output logic        rx_overflow,
    output logic        rx_dropped,
    output logic        rx_busy,
    output logic        rx_irq,

    input  logic        tx_byte_ready,
    output logic        tx_byte_valid,
    output logic [7:0]  tx_byte,
    output logic        tx_frame_start,
    output logic        tx_frame_end,
    output logic        tx_ready_for_frame,
    output logic        tx_busy,
    output logic        tx_done,
    output logic        tx_overflow
);
    localparam int unsigned PTR_WIDTH = $clog2(MAX_FRAME_BYTES + 1);

    localparam logic [3:0] REG_STATUS = 4'h0;
    localparam logic [3:0] REG_RX_LEN = 4'h1;
    localparam logic [3:0] REG_RX_DATA = 4'h2;
    localparam logic [3:0] REG_RX_CTRL = 4'h3;
    localparam logic [3:0] REG_TX_STATUS = 4'h4;
    localparam logic [3:0] REG_TX_LEN = 4'h5;
    localparam logic [3:0] REG_TX_DATA = 4'h6;
    localparam logic [3:0] REG_TX_CTRL = 4'h7;

    localparam logic [0:0] CTRL_RX_RELEASE = 1'b1;
    localparam logic [1:1] CTRL_RX_CLEAR_FLAGS = 1'b1;
    localparam logic [0:0] CTRL_TX_START = 1'b1;
    localparam logic [1:1] CTRL_TX_ABORT = 1'b1;
    localparam logic [2:2] CTRL_TX_CLEAR_FLAGS = 1'b1;

    logic [7:0] rx_mem [0:MAX_FRAME_BYTES-1];
    logic [7:0] tx_mem [0:MAX_FRAME_BYTES-1];

    logic [PTR_WIDTH-1:0] rx_write_len = '0;
    logic [PTR_WIDTH-1:0] rx_frame_len = '0;
    logic [PTR_WIDTH-1:0] rx_read_idx = '0;

    logic [PTR_WIDTH-1:0] tx_config_len = '0;
    logic [PTR_WIDTH-1:0] tx_write_len = '0;
    logic [PTR_WIDTH-1:0] tx_send_idx = '0;

    logic [1:0] dtack = '0;

    wire read_start = XDREQ && XRD && (dtack == 0);
    wire write_start = XDREQ && XWR;
    wire [3:0] reg_addr = XADDR[5:2];

    assign XDACK = (dtack == 1) || write_start;

    assign rx_ready_for_frame = !rx_frame_available;
    assign rx_irq = rx_frame_available;
    assign tx_ready_for_frame = !tx_busy && !tx_overflow;

    always_ff @(posedge CLK) begin
        if (RES) begin
            dtack <= '0;
        end else begin
            dtack <= dtack ? dtack - 1'b1 : read_start ? 2'd1 : 2'd0;
        end
    end

    always_ff @(posedge CLK) begin
        if (RES) begin
            XATAO <= 32'd0;
            rx_frame_available <= 1'b0;
            rx_overflow <= 1'b0;
            rx_dropped <= 1'b0;
            rx_busy <= 1'b0;
            rx_write_len <= '0;
            rx_frame_len <= '0;
            rx_read_idx <= '0;
            tx_byte_valid <= 1'b0;
            tx_byte <= 8'd0;
            tx_frame_start <= 1'b0;
            tx_frame_end <= 1'b0;
            tx_busy <= 1'b0;
            tx_done <= 1'b0;
            tx_overflow <= 1'b0;
            tx_config_len <= '0;
            tx_write_len <= '0;
            tx_send_idx <= '0;
        end else begin
            tx_byte_valid <= 1'b0;
            tx_frame_start <= 1'b0;
            tx_frame_end <= 1'b0;

            if (rx_byte_valid) begin
                rx_busy <= 1'b1;

                if (rx_frame_available) begin
                    rx_overflow <= 1'b1;
                end else if (rx_write_len < MAX_FRAME_BYTES[PTR_WIDTH-1:0]) begin
                    rx_mem[rx_write_len] <= rx_byte;
                    rx_write_len <= rx_write_len + 1'b1;
                end else begin
                    rx_overflow <= 1'b1;
                end
            end

            if (rx_frame_valid) begin
                rx_busy <= 1'b0;
                if (!rx_frame_available && !rx_overflow && (rx_write_len != 0)) begin
                    rx_frame_available <= 1'b1;
                    rx_frame_len <= rx_write_len;
                    rx_read_idx <= '0;
                end else begin
                    rx_dropped <= 1'b1;
                end
                rx_write_len <= '0;
            end

            if (rx_frame_drop) begin
                rx_busy <= 1'b0;
                rx_dropped <= 1'b1;
                rx_write_len <= '0;
            end

            if (tx_busy && tx_byte_ready) begin
                tx_byte <= tx_mem[tx_send_idx];
                tx_byte_valid <= 1'b1;
                tx_frame_start <= tx_send_idx == 0;
                tx_frame_end <= (tx_send_idx + 1'b1) >= tx_config_len;

                if ((tx_send_idx + 1'b1) >= tx_config_len) begin
                    tx_busy <= 1'b0;
                    tx_done <= 1'b1;
                    tx_config_len <= '0;
                    tx_write_len <= '0;
                    tx_send_idx <= '0;
                end else begin
                    tx_send_idx <= tx_send_idx + 1'b1;
                end
            end

            if (write_start && (reg_addr == REG_RX_CTRL)) begin
                if (XATAI[0] == CTRL_RX_RELEASE) begin
                    rx_frame_available <= 1'b0;
                    rx_frame_len <= '0;
                    rx_read_idx <= '0;
                end

                if (XATAI[1] == CTRL_RX_CLEAR_FLAGS) begin
                    rx_overflow <= 1'b0;
                    rx_dropped <= 1'b0;
                end
            end

            if (write_start && (reg_addr == REG_TX_LEN) && !tx_busy) begin
                tx_done <= 1'b0;
                tx_write_len <= '0;

                if ((XATAI[PTR_WIDTH-1:0] != 0) &&
                    (XATAI[PTR_WIDTH-1:0] <= MAX_FRAME_BYTES[PTR_WIDTH-1:0])) begin
                    tx_config_len <= XATAI[PTR_WIDTH-1:0];
                end else begin
                    tx_config_len <= '0;
                    tx_overflow <= 1'b1;
                end
            end

            if (write_start && (reg_addr == REG_TX_DATA) && !tx_busy) begin
                tx_done <= 1'b0;

                if (!tx_overflow && (tx_write_len < tx_config_len) &&
                    (tx_write_len < MAX_FRAME_BYTES[PTR_WIDTH-1:0])) begin
                    tx_mem[tx_write_len] <= XATAI[7:0];
                    tx_write_len <= tx_write_len + 1'b1;
                end else begin
                    tx_overflow <= 1'b1;
                end
            end

            if (write_start && (reg_addr == REG_TX_CTRL)) begin
                if (XATAI[2] == CTRL_TX_CLEAR_FLAGS) begin
                    tx_done <= 1'b0;
                    tx_overflow <= 1'b0;
                end

                if (XATAI[1] == CTRL_TX_ABORT) begin
                    tx_busy <= 1'b0;
                    tx_config_len <= '0;
                    tx_write_len <= '0;
                    tx_send_idx <= '0;
                end

                if (XATAI[0] == CTRL_TX_START) begin
                    tx_done <= 1'b0;

                    if (!tx_busy && !tx_overflow && (tx_config_len != 0) &&
                        (tx_write_len == tx_config_len)) begin
                        tx_busy <= 1'b1;
                        tx_send_idx <= '0;
                    end else begin
                        tx_overflow <= 1'b1;
                    end
                end
            end

            if (read_start) begin
                unique case (reg_addr)
                    REG_STATUS: begin
                        XATAO <= {
                            23'd0,
                            rx_ready_for_frame,
                            4'd0,
                            rx_busy,
                            rx_dropped,
                            rx_overflow,
                            rx_frame_available
                        };
                    end

                    REG_RX_LEN: begin
                        XATAO <= {{(32-PTR_WIDTH){1'b0}}, rx_frame_len};
                    end

                    REG_RX_DATA: begin
                        if (rx_frame_available && (rx_read_idx < rx_frame_len)) begin
                            XATAO <= {24'd0, rx_mem[rx_read_idx]};

                            if ((rx_read_idx + 1'b1) >= rx_frame_len) begin
                                rx_frame_available <= 1'b0;
                                rx_frame_len <= '0;
                                rx_read_idx <= '0;
                            end else begin
                                rx_read_idx <= rx_read_idx + 1'b1;
                            end
                        end else begin
                            XATAO <= 32'd0;
                        end
                    end

                    REG_TX_STATUS: begin
                        XATAO <= {
                            27'd0,
                            (tx_config_len != 0) && (tx_write_len == tx_config_len),
                            tx_done,
                            tx_overflow,
                            tx_busy,
                            tx_ready_for_frame
                        };
                    end

                    REG_TX_LEN: begin
                        XATAO <= {{(32-PTR_WIDTH){1'b0}}, tx_config_len};
                    end

                    default: begin
                        XATAO <= 32'd0;
                    end
                endcase
            end
        end
    end
endmodule
