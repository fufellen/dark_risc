`timescale 1ns / 1ps

module darkddr3_mmio_tb;
    localparam int unsigned DDR_ADDR_WIDTH = 8;

    localparam logic [31:0] REG_STATUS = 32'h00;
    localparam logic [31:0] REG_ADDR = 32'h04;
    localparam logic [31:0] REG_WDATA = 32'h08;
    localparam logic [31:0] REG_RDATA = 32'h0c;
    localparam logic [31:0] REG_CTRL = 32'h10;
    localparam logic [31:0] REG_REFRESH_COUNT = 32'h14;

    localparam logic [31:0] CTRL_START_READ = 32'h0000_0001;
    localparam logic [31:0] CTRL_START_WRITE = 32'h0000_0002;
    localparam logic [31:0] CTRL_START_REFRESH = 32'h0000_0004;
    localparam logic [31:0] CTRL_CLEAR_DONE = 32'h0000_0100;
    localparam logic [31:0] CTRL_CLEAR_ERROR = 32'h0000_0200;

    localparam logic [31:0] STATUS_INIT_DONE = 32'h0000_0001;
    localparam logic [31:0] STATUS_OP_BUSY = 32'h0000_0010;
    localparam logic [31:0] STATUS_OP_DONE = 32'h0000_0020;
    localparam logic [31:0] STATUS_OP_ERROR = 32'h0000_0040;
    localparam logic [31:0] STATUS_READY_FOR_CMD = 32'h0000_0100;

    logic CLK = 1'b0;
    logic RES = 1'b1;

    logic XDREQ = 1'b0;
    logic XWINDOW = 1'b0;   // 0 = регистры, 1 = прозрачное окно
    logic XRD = 1'b0;
    logic XWR = 1'b0;
    logic [3:0] XBE = 4'hf;
    logic [31:0] XADDR = 32'd0;
    logic [31:0] XATAI = 32'd0;
    logic [31:0] XATAO;
    logic XDACK;

    logic ddr_rd;
    logic ddr_wr;
    logic ddr_refresh;
    logic [DDR_ADDR_WIDTH-1:0] ddr_addr;
    logic [15:0] ddr_din;
    logic [15:0] ddr_dout = 16'd0;
    logic ddr_data_ready = 1'b0;
    logic ddr_busy = 1'b1;
    logic ddr_write_level_done = 1'b0;
    logic ddr_read_calib_done = 1'b0;

    logic [15:0] mem [0:(1 << DDR_ADDR_WIDTH)-1];
    int busy_count = 0;
    int init_count = 0;
    int pending_read_count = 0;
    logic [DDR_ADDR_WIDTH-1:0] pending_read_addr = '0;

    always #5 CLK = !CLK;

    darkddr3_mmio #(
        .DDR_ADDR_WIDTH(DDR_ADDR_WIDTH),
        .REFRESH_INTERVAL_CYCLES(0)
    ) dut (
        .CLK(CLK),
        .RES(RES),
        .XDREQ(XDREQ),
        .XWINDOW(XWINDOW),
        .XRD(XRD),
        .XWR(XWR),
        .XBE(XBE),
        .XADDR(XADDR),
        .XATAI(XATAI),
        .XATAO(XATAO),
        .XDACK(XDACK),
        .ddr_rd(ddr_rd),
        .ddr_wr(ddr_wr),
        .ddr_refresh(ddr_refresh),
        .ddr_addr(ddr_addr),
        .ddr_din(ddr_din),
        .ddr_dout(ddr_dout),
        .ddr_data_ready(ddr_data_ready),
        .ddr_busy(ddr_busy),
        .ddr_write_level_done(ddr_write_level_done),
        .ddr_read_calib_done(ddr_read_calib_done)
    );

    always @(posedge CLK) begin
        ddr_data_ready <= 1'b0;

        if (RES) begin
            ddr_busy <= 1'b1;
            ddr_write_level_done <= 1'b0;
            ddr_read_calib_done <= 1'b0;
            init_count <= 0;
            busy_count <= 0;
            pending_read_count <= 0;
        end else begin
            if (!ddr_write_level_done || !ddr_read_calib_done) begin
                init_count <= init_count + 1;
                ddr_busy <= 1'b1;
                if (init_count == 4) begin
                    ddr_write_level_done <= 1'b1;
                    ddr_read_calib_done <= 1'b1;
                    ddr_busy <= 1'b0;
                end
            end else if (busy_count != 0) begin
                busy_count <= busy_count - 1;
                ddr_busy <= busy_count != 1;
            end else begin
                ddr_busy <= 1'b0;
            end

            if (pending_read_count != 0) begin
                pending_read_count <= pending_read_count - 1;
                if (pending_read_count == 1) begin
                    ddr_dout <= mem[pending_read_addr];
                    ddr_data_ready <= 1'b1;
                end
            end

            if (ddr_wr) begin
                mem[ddr_addr] <= ddr_din;
                ddr_busy <= 1'b1;
                busy_count <= 2;
            end

            if (ddr_rd) begin
                pending_read_addr <= ddr_addr;
                pending_read_count <= 3;
                ddr_busy <= 1'b1;
                busy_count <= 3;
            end

            if (ddr_refresh) begin
                ddr_busy <= 1'b1;
                busy_count <= 8;
            end
        end
    end

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

    /* Прозрачное окно: адаптер держит XDACK до готовности данных, поэтому
     * ждать его надо и на записи тоже — в отличие от регистрового доступа. */
    task automatic win_write(input logic [31:0] addr, input logic [31:0] data);
        begin
            @(posedge CLK);
            XADDR <= addr; XATAI <= data; XWINDOW <= 1'b1;
            XRD <= 1'b0; XWR <= 1'b1; XDREQ <= 1'b1;
            do @(posedge CLK); while (!XDACK);
            XWR <= 1'b0; XDREQ <= 1'b0; XWINDOW <= 1'b0;
            @(posedge CLK);
        end
    endtask

    task automatic win_read(input logic [31:0] addr, output logic [31:0] data);
        begin
            @(posedge CLK);
            XADDR <= addr; XWINDOW <= 1'b1;
            XRD <= 1'b1; XWR <= 1'b0; XDREQ <= 1'b1;
            do @(posedge CLK); while (!XDACK);
            data = XATAO;
            XRD <= 1'b0; XDREQ <= 1'b0; XWINDOW <= 1'b0;
            @(posedge CLK);
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

    task automatic wait_status(input logic [31:0] mask, input logic [31:0] exp, input string what);
        logic [31:0] data;
        begin
            for (int i = 0; i < 100; i++) begin
                mmio_read(REG_STATUS, data);
                if ((data & mask) == exp) begin
                    return;
                end
            end

            $display("FAIL timeout %s: last=%08x mask=%08x exp=%08x", what, data, mask, exp);
            $fatal;
        end
    endtask

    initial begin
        logic [31:0] data;

        mem[8'h20] = 16'h1234;
        mem[8'h21] = 16'habcd;

        repeat (4) @(posedge CLK);
        RES <= 1'b0;

        wait_status(STATUS_INIT_DONE | STATUS_READY_FOR_CMD,
                    STATUS_INIT_DONE | STATUS_READY_FOR_CMD,
                    "ddr init");

        mmio_write(REG_ADDR, 32'h0000_0010);
        mmio_write(REG_WDATA, 32'h5566_7788);
        mmio_write(REG_CTRL, CTRL_START_WRITE);
        wait_status(STATUS_OP_DONE | STATUS_OP_BUSY, STATUS_OP_DONE, "write32 done");

        check_equal({16'd0, mem[8'h10]}, 32'h0000_7788, "write lower half");
        check_equal({16'd0, mem[8'h11]}, 32'h0000_5566, "write upper half");

        mmio_write(REG_CTRL, CTRL_CLEAR_DONE);
        mmio_write(REG_ADDR, 32'h0000_0010);
        mmio_write(REG_CTRL, CTRL_START_READ);
        wait_status(STATUS_OP_DONE | STATUS_OP_BUSY, STATUS_OP_DONE, "read32 done");
        mmio_read(REG_RDATA, data);
        check_equal(data, 32'h5566_7788, "read32 data");

        mmio_write(REG_CTRL, CTRL_CLEAR_DONE);
        mmio_write(REG_ADDR, 32'h0000_0020);
        mmio_write(REG_CTRL, CTRL_START_READ);
        mmio_write(REG_CTRL, CTRL_START_WRITE);
        wait_status(STATUS_OP_ERROR, STATUS_OP_ERROR, "busy command error");
        wait_status(STATUS_OP_DONE | STATUS_OP_BUSY, STATUS_OP_DONE, "read after error done");
        mmio_read(REG_RDATA, data);
        check_equal(data, 32'habcd_1234, "read32 preloaded data");

        mmio_write(REG_CTRL, CTRL_CLEAR_DONE | CTRL_CLEAR_ERROR);
        wait_status(STATUS_READY_FOR_CMD | STATUS_OP_ERROR | STATUS_OP_DONE,
                    STATUS_READY_FOR_CMD,
                    "clear flags");

        mmio_write(REG_CTRL, CTRL_START_REFRESH);
        wait_status(STATUS_READY_FOR_CMD | STATUS_OP_BUSY,
                    STATUS_READY_FOR_CMD,
                    "manual refresh done");
        mmio_read(REG_REFRESH_COUNT, data);
        check_equal(data, 32'd1, "refresh count");

        mmio_write(REG_CTRL, CTRL_CLEAR_DONE | CTRL_CLEAR_ERROR);
        mmio_write(REG_CTRL, CTRL_START_REFRESH);
        mmio_write(REG_ADDR, 32'h0000_0030);
        mmio_write(REG_WDATA, 32'hcafe_babe);
        mmio_write(REG_CTRL, CTRL_START_WRITE);
        wait_status(STATUS_OP_DONE | STATUS_OP_BUSY | STATUS_OP_ERROR,
                    STATUS_OP_DONE,
                    "write queued during refresh");
        check_equal({16'd0, mem[8'h30]}, 32'h0000_babe, "queued write lower half");
        check_equal({16'd0, mem[8'h31]}, 32'h0000_cafe, "queued write upper half");

        begin
            logic [31:0] win_got;
            win_write(32'h0000_0010, 32'ha5a5_1234);
            win_read(32'h0000_0010, win_got);
            check_equal(win_got, 32'ha5a5_1234, "window rw");
            win_write(32'h0000_0020, 32'h0f0f_5678);
            win_read(32'h0000_0020, win_got);
            check_equal(win_got, 32'h0f0f_5678, "window rw 2");
            win_read(32'h0000_0010, win_got);
            check_equal(win_got, 32'ha5a5_1234, "window keep");
            $display("window transparent access ok");
        end

        $display("TEST PASS: darkddr3_mmio");
        $finish;
    end
endmodule
