/*
 * DarkRISCV memory-mapped Ethernet frame endpoint.
 *
 * RX side stores one complete Ethernet frame without preamble/SFD/FCS and
 * exposes it to firmware as a byte pop register. This is the intended shape
 * for an LwIP netif ethernet_input() pbuf.
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
    output logic        rx_irq
);
    localparam int unsigned PTR_WIDTH = $clog2(MAX_FRAME_BYTES + 1);

    localparam logic [3:0] REG_STATUS = 4'h0;
    localparam logic [3:0] REG_RX_LEN = 4'h1;
    localparam logic [3:0] REG_RX_DATA = 4'h2;
    localparam logic [3:0] REG_RX_CTRL = 4'h3;

    localparam logic [0:0] CTRL_RX_RELEASE = 1'b1;
    localparam logic [1:1] CTRL_CLEAR_FLAGS = 1'b1;

    logic [7:0] rx_mem [0:MAX_FRAME_BYTES-1];

    logic [PTR_WIDTH-1:0] rx_write_len = '0;
    logic [PTR_WIDTH-1:0] rx_frame_len = '0;
    logic [PTR_WIDTH-1:0] rx_read_idx = '0;

    logic [1:0] dtack = '0;

    wire read_start = XDREQ && XRD && (dtack == 0);
    wire write_start = XDREQ && XWR;
    wire [3:0] reg_addr = XADDR[5:2];

    assign XDACK = (dtack == 1) || write_start;

    assign rx_ready_for_frame = !rx_frame_available;
    assign rx_irq = rx_frame_available;

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
        end else begin
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

            if (write_start && (reg_addr == REG_RX_CTRL)) begin
                if (XATAI[0] == CTRL_RX_RELEASE) begin
                    rx_frame_available <= 1'b0;
                    rx_frame_len <= '0;
                    rx_read_idx <= '0;
                end

                if (XATAI[1] == CTRL_CLEAR_FLAGS) begin
                    rx_overflow <= 1'b0;
                    rx_dropped <= 1'b0;
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

                    default: begin
                        XATAO <= 32'd0;
                    end
                endcase
            end
        end
    end
endmodule
