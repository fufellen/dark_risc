/*
 * DarkRISCV diagnostic MMIO adapter for the nand2mario Tang Primer 20K DDR3
 * controller user port.
 *
 * This is intentionally a small command/register peripheral, not executable
 * external RAM yet. Firmware writes ADDR/WDATA, starts a 32-bit read or write,
 * polls STATUS.DONE, then reads RDATA.
 */

`timescale 1ns / 1ps

module darkddr3_mmio #(
    parameter int unsigned DDR_ADDR_WIDTH = 26,
    parameter int unsigned REFRESH_INTERVAL_CYCLES = 781
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

    output logic                       ddr_rd,
    output logic                       ddr_wr,
    output logic                       ddr_refresh,
    output logic [DDR_ADDR_WIDTH-1:0]  ddr_addr,
    output logic [15:0]                ddr_din,
    input  logic [15:0]                ddr_dout,
    input  logic                       ddr_data_ready,
    input  logic                       ddr_busy,
    input  logic                       ddr_write_level_done,
    input  logic                       ddr_read_calib_done
);
    localparam logic [3:0] REG_STATUS = 4'h0;
    localparam logic [3:0] REG_ADDR = 4'h1;
    localparam logic [3:0] REG_WDATA = 4'h2;
    localparam logic [3:0] REG_RDATA = 4'h3;
    localparam logic [3:0] REG_CTRL = 4'h4;
    localparam logic [3:0] REG_REFRESH_COUNT = 4'h5;

    localparam logic [31:0] CTRL_START_READ = 32'h0000_0001;
    localparam logic [31:0] CTRL_START_WRITE = 32'h0000_0002;
    localparam logic [31:0] CTRL_START_REFRESH = 32'h0000_0004;
    localparam logic [31:0] CTRL_CLEAR_DONE = 32'h0000_0100;
    localparam logic [31:0] CTRL_CLEAR_ERROR = 32'h0000_0200;

    typedef enum logic [3:0] {
        ST_IDLE,
        ST_WRITE_LO_WAIT,
        ST_WRITE_HI_REQ,
        ST_WRITE_HI_WAIT,
        ST_READ_LO_WAIT,
        ST_READ_HI_REQ,
        ST_READ_HI_WAIT,
        ST_REFRESH_WAIT
    } state_t;

    state_t state = ST_IDLE;

    logic [1:0] dtack = 2'd0;
    logic [DDR_ADDR_WIDTH-1:0] cmd_addr = '0;
    logic [31:0] cmd_wdata = 32'd0;
    logic [31:0] cmd_rdata = 32'd0;
    logic pending_read = 1'b0;
    logic pending_write = 1'b0;
    logic pending_refresh = 1'b0;
    logic refresh_pending = 1'b0;
    logic refresh_cmd_active = 1'b0;
    logic op_done = 1'b0;
    logic op_error = 1'b0;
    logic [31:0] refresh_counter = 32'd0;
    logic [31:0] refresh_count = 32'd0;

    wire init_done = ddr_write_level_done && ddr_read_calib_done;
    wire op_busy = (state != ST_IDLE) || pending_read || pending_write || pending_refresh;
    wire ready_for_cmd = init_done && !ddr_busy && !op_busy && !refresh_pending;
    wire read_start = XDREQ && XRD && (dtack == 0);
    wire write_start = XDREQ && XWR;
    wire [3:0] reg_addr = XADDR[5:2];
    wire pending_any = pending_read || pending_write || pending_refresh;
    wire state_accepts_start = (state == ST_IDLE) || (state == ST_REFRESH_WAIT);

    assign XDACK = (dtack == 1) || write_start;

    always_ff @(posedge CLK) begin
        if (RES) begin
            dtack <= 2'd0;
        end else begin
            dtack <= dtack ? dtack - 1'b1 : read_start ? 2'd1 : 2'd0;
        end
    end

    always_ff @(posedge CLK) begin
        if (RES) begin
            XATAO <= 32'd0;
            ddr_rd <= 1'b0;
            ddr_wr <= 1'b0;
            ddr_refresh <= 1'b0;
            ddr_addr <= '0;
            ddr_din <= 16'd0;
            cmd_addr <= '0;
            cmd_wdata <= 32'd0;
            cmd_rdata <= 32'd0;
            pending_read <= 1'b0;
            pending_write <= 1'b0;
            pending_refresh <= 1'b0;
            refresh_pending <= 1'b0;
            refresh_cmd_active <= 1'b0;
            op_done <= 1'b0;
            op_error <= 1'b0;
            refresh_counter <= 32'd0;
            refresh_count <= 32'd0;
            state <= ST_IDLE;
        end else begin
            ddr_rd <= 1'b0;
            ddr_wr <= 1'b0;
            ddr_refresh <= 1'b0;

            if ((REFRESH_INTERVAL_CYCLES != 0) && init_done) begin
                if (refresh_counter >= REFRESH_INTERVAL_CYCLES - 1) begin
                    refresh_counter <= 32'd0;
                    refresh_pending <= 1'b1;
                end else begin
                    refresh_counter <= refresh_counter + 1'b1;
                end
            end

            if (write_start) begin
                unique case (reg_addr)
                    REG_ADDR: begin
                        cmd_addr <= XATAI[DDR_ADDR_WIDTH-1:0];
                    end

                    REG_WDATA: begin
                        cmd_wdata <= XATAI;
                    end

                    REG_CTRL: begin
                        if ((XATAI & CTRL_CLEAR_DONE) != 0) begin
                            op_done <= 1'b0;
                        end

                        if ((XATAI & CTRL_CLEAR_ERROR) != 0) begin
                            op_error <= 1'b0;
                        end

                        if ((XATAI & (CTRL_START_READ | CTRL_START_WRITE | CTRL_START_REFRESH)) != 0) begin
                            if (!init_done || !state_accepts_start || pending_any) begin
                                op_error <= 1'b1;
                            end else if ((XATAI & CTRL_START_READ) != 0) begin
                                pending_read <= 1'b1;
                                op_done <= 1'b0;
                            end else if ((XATAI & CTRL_START_WRITE) != 0) begin
                                pending_write <= 1'b1;
                                op_done <= 1'b0;
                            end else begin
                                pending_refresh <= 1'b1;
                            end
                        end
                    end

                    default: begin
                    end
                endcase
            end

            unique case (state)
                ST_IDLE: begin
                    if (init_done && !ddr_busy) begin
                        if (refresh_pending || pending_refresh) begin
                            ddr_refresh <= 1'b1;
                            refresh_cmd_active <= pending_refresh;
                            refresh_pending <= 1'b0;
                            pending_refresh <= 1'b0;
                            state <= ST_REFRESH_WAIT;
                        end else if (pending_write) begin
                            ddr_addr <= cmd_addr;
                            ddr_din <= cmd_wdata[15:0];
                            ddr_wr <= 1'b1;
                            pending_write <= 1'b0;
                            state <= ST_WRITE_LO_WAIT;
                        end else if (pending_read) begin
                            ddr_addr <= cmd_addr;
                            ddr_rd <= 1'b1;
                            pending_read <= 1'b0;
                            state <= ST_READ_LO_WAIT;
                        end
                    end
                end

                ST_WRITE_LO_WAIT: begin
                    if (!ddr_busy) begin
                        state <= ST_WRITE_HI_REQ;
                    end
                end

                ST_WRITE_HI_REQ: begin
                    if (!ddr_busy) begin
                        ddr_addr <= cmd_addr + {{(DDR_ADDR_WIDTH-1){1'b0}}, 1'b1};
                        ddr_din <= cmd_wdata[31:16];
                        ddr_wr <= 1'b1;
                        state <= ST_WRITE_HI_WAIT;
                    end
                end

                ST_WRITE_HI_WAIT: begin
                    if (!ddr_busy) begin
                        op_done <= 1'b1;
                        state <= ST_IDLE;
                    end
                end

                ST_READ_LO_WAIT: begin
                    if (ddr_data_ready) begin
                        cmd_rdata[15:0] <= ddr_dout;
                        state <= ST_READ_HI_REQ;
                    end
                end

                ST_READ_HI_REQ: begin
                    if (!ddr_busy) begin
                        ddr_addr <= cmd_addr + {{(DDR_ADDR_WIDTH-1){1'b0}}, 1'b1};
                        ddr_rd <= 1'b1;
                        state <= ST_READ_HI_WAIT;
                    end
                end

                ST_READ_HI_WAIT: begin
                    if (ddr_data_ready) begin
                        cmd_rdata[31:16] <= ddr_dout;
                        op_done <= 1'b1;
                        state <= ST_IDLE;
                    end
                end

                ST_REFRESH_WAIT: begin
                    if (!ddr_busy) begin
                        refresh_count <= refresh_count + 1'b1;
                        if (refresh_cmd_active) begin
                            op_done <= 1'b1;
                            refresh_cmd_active <= 1'b0;
                        end
                        state <= ST_IDLE;
                    end
                end

                default: begin
                    state <= ST_IDLE;
                    op_error <= 1'b1;
                end
            endcase

            if (read_start) begin
                unique case (reg_addr)
                    REG_STATUS: begin
                        XATAO <= {
                            23'd0,
                            ready_for_cmd,
                            refresh_pending,
                            op_error,
                            op_done,
                            op_busy,
                            ddr_busy,
                            ddr_read_calib_done,
                            ddr_write_level_done,
                            init_done
                        };
                    end

                    REG_ADDR: begin
                        XATAO <= {{(32-DDR_ADDR_WIDTH){1'b0}}, cmd_addr};
                    end

                    REG_WDATA: begin
                        XATAO <= cmd_wdata;
                    end

                    REG_RDATA: begin
                        XATAO <= cmd_rdata;
                    end

                    REG_REFRESH_COUNT: begin
                        XATAO <= refresh_count;
                    end

                    default: begin
                        XATAO <= 32'd0;
                    end
                endcase
            end
        end
    end
endmodule
