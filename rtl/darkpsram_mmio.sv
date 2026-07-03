/*
 * DarkRISCV diagnostic/buffer MMIO adapter for IS66WVS1M8 SerialRAM.
 *
 * This is a register peripheral, not a linear memory mapping. Firmware writes
 * ADDR/WDATA, starts a 32-bit SPI read or write, polls STATUS.DONE, then reads
 * RDATA. The external RAM can be used as a packet/ring buffer by firmware.
 */

`timescale 1ns / 1ps
`default_nettype none

module darkpsram_mmio #(
    parameter int unsigned POWERUP_WAIT_CYCLES = 100_000,
    parameter int unsigned RESET_WAIT_CYCLES = 50_000,
    parameter int unsigned GAP_CYCLES = 8,
    parameter bit USE_QSPI = 1'b0,
    parameter bit USE_CDC = 1'b0   // контроллер в своём домене PSRAM_CLK
)(
    input  wire logic   CLK,
    input  wire logic   RES,

    // домен контроллера (используется только при USE_CDC=1; иначе завести CLK/!RES)
    input  wire logic   PSRAM_CLK,
    input  wire logic   PSRAM_RST_N,

    input  wire logic   XDREQ,
    input  wire logic   XRD,
    input  wire logic   XWR,
    input  wire logic [3:0]  XBE,
    input  wire logic [31:0] XADDR,
    input  wire logic [31:0] XATAI,
    output logic [31:0] XATAO,
    output logic        XDACK,

    input  wire  logic [3:0] psram_din,
    output logic [3:0] psram_dout,
    output logic [3:0] psram_douten,
    output logic       psram_sck,
    output logic       psram_ce_n
);
    localparam logic [3:0] REG_STATUS = 4'h0;
    localparam logic [3:0] REG_ADDR = 4'h1;
    localparam logic [3:0] REG_WDATA = 4'h2;
    localparam logic [3:0] REG_RDATA = 4'h3;
    localparam logic [3:0] REG_CTRL = 4'h4;
    localparam logic [3:0] REG_ID = 4'h5;
    localparam logic [3:0] REG_OP_COUNT = 4'h6;

    localparam logic [31:0] CTRL_START_READ = 32'h0000_0001;
    localparam logic [31:0] CTRL_START_WRITE = 32'h0000_0002;
    localparam logic [31:0] CTRL_CLEAR_DONE = 32'h0000_0100;
    localparam logic [31:0] CTRL_CLEAR_ERROR = 32'h0000_0200;
    localparam logic [7:0] READ_CMD = USE_QSPI ? 8'hEB : 8'h03;
    localparam logic [7:0] WRITE_CMD = USE_QSPI ? 8'h38 : 8'h02;
    localparam logic [3:0] READ_WAIT_STATES = USE_QSPI ? 4'd6 : 4'd0;

    typedef enum logic [3:0] {
        ST_POWERUP_WAIT,
        ST_INIT_CLK_PULSE,
        ST_RESET_ENABLE,
        ST_RESET_ENABLE_WAIT,
        ST_RESET_COMMAND,
        ST_RESET_COMMAND_WAIT,
        ST_RESET_RECOVERY_WAIT,
        ST_IDLE,
        ST_WRITE_WAIT,
        ST_READ_WAIT,
        ST_GAP
    } state_t;

    state_t state = ST_POWERUP_WAIT;
    state_t next_after_gap = ST_RESET_ENABLE;

    logic [1:0]  dtack = 2'd0;
    logic [31:0] wait_counter = 32'd0;
    logic [7:0]  init_pulse_counter = 8'd0;
    logic        init_sck = 1'b0;
    logic        init_drive = 1'b1;

    logic [23:0] cmd_addr = 24'd0;
    logic [31:0] cmd_wdata = 32'd0;
    logic [31:0] cmd_rdata = 32'd0;
    logic        pending_read = 1'b0;
    logic        pending_write = 1'b0;
    logic        init_done = 1'b0;
    logic        op_done = 1'b0;
    logic        op_error = 1'b0;
    logic [31:0] op_count = 32'd0;

    logic [23:0] ctrl_addr = 24'd0;
    logic [31:0] ctrl_data_i = 32'd0;
    wire  [31:0] ctrl_data_o;
    logic [2:0]  ctrl_size = 3'd4;
    logic        ctrl_start = 1'b0;
    wire         ctrl_done;
    logic [3:0]  ctrl_wait_states = 4'd0;
    logic [7:0]  ctrl_cmd = 8'h03;
    logic        ctrl_rd_wr = 1'b1;
    logic        ctrl_qspi = 1'b0;
    logic        ctrl_qpi = 1'b0;
    logic        ctrl_short_cmd = 1'b0;
    wire         ctrl_sck;
    wire         ctrl_ce_n;
    wire  [3:0]  ctrl_dout;
    wire  [3:0]  ctrl_douten;

    wire op_busy = (state != ST_IDLE) || pending_read || pending_write;
    wire ready_for_cmd = init_done && !op_busy;
    wire read_start = XDREQ && XRD && (dtack == 0);
    wire write_start = XDREQ && XWR;
    wire [3:0] reg_addr = XADDR[5:2];

    assign XDACK = (dtack == 1) || write_start;

    generate if (USE_CDC) begin : gen_ctrl_cdc
        // Контроллер в домене PSRAM_CLK через toggle-мост. Медленные сигналы
        // init-битбэнга синхронизируются в домен пинов двойным триггером.
        logic init_drive_meta = 1'b1;
        logic init_drive_ps = 1'b1;
        logic init_sck_meta = 1'b0;
        logic init_sck_ps = 1'b0;
        always_ff @(posedge PSRAM_CLK) begin
            init_drive_meta <= init_drive;
            init_drive_ps <= init_drive_meta;
            init_sck_meta <= init_sck;
            init_sck_ps <= init_sck_meta;
        end

        darkpsram_ctrl_cdc psram_ctrl (
            .cpu_clk(CLK),
            .cpu_res(RES),
            .addr(ctrl_addr),
            .data_i(ctrl_data_i),
            .data_o(ctrl_data_o),
            .size(ctrl_size),
            .start(ctrl_start),
            .done(ctrl_done),
            .wait_states(ctrl_wait_states),
            .cmd(ctrl_cmd),
            .rd_wr(ctrl_rd_wr),
            .qspi(ctrl_qspi),
            .qpi(ctrl_qpi),
            .short_cmd(ctrl_short_cmd),
            .psram_clk(PSRAM_CLK),
            .psram_rst_n(PSRAM_RST_N),
            .psram_sck(ctrl_sck),
            .psram_ce_n(ctrl_ce_n),
            .psram_din(psram_din),
            .psram_dout(ctrl_dout),
            .psram_douten(ctrl_douten)
        );

        assign psram_sck = init_drive_ps ? init_sck_ps : ctrl_sck;
        assign psram_ce_n = init_drive_ps ? 1'b1 : ctrl_ce_n;
        assign psram_dout = init_drive_ps ? 4'b0001 : ctrl_dout;
        assign psram_douten = init_drive_ps ? 4'b0001 : ctrl_douten;
    end else begin : gen_ctrl_direct
        EF_PSRAM_CTRL psram_ctrl (
            .clk(CLK),
            .rst_n(!RES),
            .addr(ctrl_addr),
            .data_i(ctrl_data_i),
            .data_o(ctrl_data_o),
            .size(ctrl_size),
            .start(ctrl_start),
            .done(ctrl_done),
            .wait_states(ctrl_wait_states),
            .cmd(ctrl_cmd),
            .rd_wr(ctrl_rd_wr),
            .qspi(ctrl_qspi),
            .qpi(ctrl_qpi),
            .short_cmd(ctrl_short_cmd),
            .sck(ctrl_sck),
            .ce_n(ctrl_ce_n),
            .din(psram_din),
            .dout(ctrl_dout),
            .douten(ctrl_douten)
        );

        assign psram_sck = init_drive ? init_sck : ctrl_sck;
        assign psram_ce_n = init_drive ? 1'b1 : ctrl_ce_n;
        assign psram_dout = init_drive ? 4'b0001 : ctrl_dout;
        assign psram_douten = init_drive ? 4'b0001 : ctrl_douten;
    end endgenerate

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
            state <= ST_POWERUP_WAIT;
            next_after_gap <= ST_RESET_ENABLE;
            wait_counter <= 32'd0;
            init_pulse_counter <= 8'd0;
            init_sck <= 1'b0;
            init_drive <= 1'b1;
            cmd_addr <= 24'd0;
            cmd_wdata <= 32'd0;
            cmd_rdata <= 32'd0;
            pending_read <= 1'b0;
            pending_write <= 1'b0;
            init_done <= 1'b0;
            op_done <= 1'b0;
            op_error <= 1'b0;
            op_count <= 32'd0;
            ctrl_addr <= 24'd0;
            ctrl_data_i <= 32'd0;
            ctrl_size <= 3'd4;
            ctrl_start <= 1'b0;
            ctrl_wait_states <= 4'd0;
            ctrl_cmd <= 8'h03;
            ctrl_rd_wr <= 1'b1;
            ctrl_qspi <= 1'b0;
            ctrl_qpi <= 1'b0;
            ctrl_short_cmd <= 1'b0;
        end else begin
            ctrl_start <= 1'b0;

            if (write_start) begin
                unique case (reg_addr)
                    REG_ADDR: begin
                        cmd_addr <= XATAI[23:0];
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

                        if ((XATAI & (CTRL_START_READ | CTRL_START_WRITE)) != 0) begin
                            if (!ready_for_cmd) begin
                                op_error <= 1'b1;
                            end else if ((XATAI & CTRL_START_READ) != 0) begin
                                pending_read <= 1'b1;
                                op_done <= 1'b0;
                            end else begin
                                pending_write <= 1'b1;
                                op_done <= 1'b0;
                            end
                        end
                    end

                    default: begin
                    end
                endcase
            end

            unique case (state)
                ST_POWERUP_WAIT: begin
                    init_drive <= 1'b1;
                    init_sck <= 1'b0;
                    if (wait_counter >= POWERUP_WAIT_CYCLES - 1) begin
                        wait_counter <= 32'd0;
                        state <= ST_INIT_CLK_PULSE;
                    end else begin
                        wait_counter <= wait_counter + 1'b1;
                    end
                end

                ST_INIT_CLK_PULSE: begin
                    init_drive <= 1'b1;
                    init_sck <= !init_sck;
                    if (init_pulse_counter >= 8'd7) begin
                        init_pulse_counter <= 8'd0;
                        init_sck <= 1'b0;
                        init_drive <= 1'b0;
                        state <= ST_RESET_ENABLE;
                    end else begin
                        init_pulse_counter <= init_pulse_counter + 1'b1;
                    end
                end

                ST_RESET_ENABLE: begin
                    ctrl_cmd <= 8'h66;
                    ctrl_addr <= 24'h000000;
                    ctrl_data_i <= 32'h00000000;
                    ctrl_size <= 3'd1;
                    ctrl_wait_states <= 4'd0;
                    ctrl_rd_wr <= 1'b0;
                    ctrl_qspi <= 1'b0;
                    ctrl_qpi <= 1'b0;
                    ctrl_short_cmd <= 1'b1;
                    ctrl_start <= 1'b1;
                    state <= ST_RESET_ENABLE_WAIT;
                end

                ST_RESET_ENABLE_WAIT: begin
                    if (ctrl_done) begin
                        next_after_gap <= ST_RESET_COMMAND;
                        wait_counter <= 32'd0;
                        state <= ST_GAP;
                    end
                end

                ST_RESET_COMMAND: begin
                    ctrl_cmd <= 8'h99;
                    ctrl_addr <= 24'h000000;
                    ctrl_data_i <= 32'h00000000;
                    ctrl_size <= 3'd1;
                    ctrl_wait_states <= 4'd0;
                    ctrl_rd_wr <= 1'b0;
                    ctrl_qspi <= 1'b0;
                    ctrl_qpi <= 1'b0;
                    ctrl_short_cmd <= 1'b1;
                    ctrl_start <= 1'b1;
                    state <= ST_RESET_COMMAND_WAIT;
                end

                ST_RESET_COMMAND_WAIT: begin
                    if (ctrl_done) begin
                        wait_counter <= 32'd0;
                        state <= ST_RESET_RECOVERY_WAIT;
                    end
                end

                ST_RESET_RECOVERY_WAIT: begin
                    if (wait_counter >= RESET_WAIT_CYCLES - 1) begin
                        wait_counter <= 32'd0;
                        init_done <= 1'b1;
                        // synthesis translate_off
                        if (USE_QSPI) begin
                            $display("darkpsram_mmio mode=QSPI");
                        end else begin
                            $display("darkpsram_mmio mode=SPI");
                        end
                        // synthesis translate_on
                        state <= ST_IDLE;
                    end else begin
                        wait_counter <= wait_counter + 1'b1;
                    end
                end

                ST_IDLE: begin
                    if (pending_write) begin
                        ctrl_cmd <= WRITE_CMD;
                        ctrl_addr <= cmd_addr;
                        ctrl_data_i <= cmd_wdata;
                        ctrl_size <= 3'd4;
                        ctrl_wait_states <= 4'd0;
                        ctrl_rd_wr <= 1'b0;
                        ctrl_qspi <= USE_QSPI;
                        ctrl_qpi <= 1'b0;
                        ctrl_short_cmd <= 1'b0;
                        ctrl_start <= 1'b1;
                        pending_write <= 1'b0;
                        state <= ST_WRITE_WAIT;
                    end else if (pending_read) begin
                        ctrl_cmd <= READ_CMD;
                        ctrl_addr <= cmd_addr;
                        ctrl_data_i <= 32'h00000000;
                        ctrl_size <= 3'd4;
                        ctrl_wait_states <= READ_WAIT_STATES;
                        ctrl_rd_wr <= 1'b1;
                        ctrl_qspi <= USE_QSPI;
                        ctrl_qpi <= 1'b0;
                        ctrl_short_cmd <= 1'b0;
                        ctrl_start <= 1'b1;
                        pending_read <= 1'b0;
                        state <= ST_READ_WAIT;
                    end
                end

                ST_WRITE_WAIT: begin
                    if (ctrl_done) begin
                        op_done <= 1'b1;
                        op_count <= op_count + 1'b1;
                        state <= ST_IDLE;
                    end
                end

                ST_READ_WAIT: begin
                    if (ctrl_done) begin
                        cmd_rdata <= ctrl_data_o;
                        op_done <= 1'b1;
                        op_count <= op_count + 1'b1;
                        state <= ST_IDLE;
                    end
                end

                ST_GAP: begin
                    if (wait_counter >= GAP_CYCLES - 1) begin
                        wait_counter <= 32'd0;
                        state <= next_after_gap;
                    end else begin
                        wait_counter <= wait_counter + 1'b1;
                    end
                end

                default: begin
                    state <= ST_POWERUP_WAIT;
                    op_error <= 1'b1;
                end
            endcase

            if (read_start) begin
                unique case (reg_addr)
                    REG_STATUS: begin
                        XATAO <= {
                            23'd0,
                            ready_for_cmd,
                            1'b0,
                            op_error,
                            op_done,
                            op_busy,
                            3'd0,
                            init_done
                        };
                    end

                    REG_ADDR: begin
                        XATAO <= {8'd0, cmd_addr};
                    end

                    REG_WDATA: begin
                        XATAO <= cmd_wdata;
                    end

                    REG_RDATA: begin
                        XATAO <= cmd_rdata;
                    end

                    REG_ID: begin
                        XATAO <= 32'h4953_3636;
                    end

                    REG_OP_COUNT: begin
                        XATAO <= op_count;
                    end

                    default: begin
                        XATAO <= 32'd0;
                    end
                endcase
            end
        end
    end
endmodule

`default_nettype wire
