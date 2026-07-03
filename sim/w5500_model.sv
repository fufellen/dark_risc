`ifndef _w5500_model_
`define _w5500_model_
`timescale 1ns/1ps

/*
    Поведенческая модель WIZnet W5500 (только симуляция).
    Справочник: docs/skills/fpga-dev/w5500.md (в суперпроекте verilog).

    Поддержано ровно то, что нужно для w5500demo / MSOP TCP:
    - SPI slave mode 0, VDM-кадр: 2 байта адреса + control byte
      {BSB[4:0], RWB, OM[1:0]} + данные с автоинкрементом, пока CS низкий;
    - общие регистры (VERSIONR=0x04, MR/GAR/SUBR/SHAR/SIPR как хранилище);
    - socket 0: Sn_MR/Sn_CR/Sn_IR/Sn_SR/Sn_PORT, TX-буфер 2 КиБ,
      Sn_TX_FSR/Sn_TX_WR, команды OPEN/LISTEN/SEND/CLOSE/DISCON;
    - после LISTEN через CONNECT_DELAY_NS сокет сам переходит в
      ESTABLISHED («клиент подключился»);
    - SEND: забирает TX-данные [tx_rd..Sn_TX_WR), проверяет маркеры MSOP
      (FF FE в начале, FF 9B в конце), печатает
      "w5500 model: msop frame ok len=..." и ставит Sn_IR.SEND_OK.
*/
module w5500_model #(
    parameter real CONNECT_DELAY_NS = 50_000.0 // задержка «подключения клиента» после LISTEN
)(
    input  wire sck,
    input  wire cs_n,
    input  wire mosi,
    output logic miso,
    input  wire rst_n
);
    initial miso = 1'bz;

    // W5500 константы
    localparam SN_CR_OPEN   = 8'h01,
               SN_CR_LISTEN = 8'h02,
               SN_CR_DISCON = 8'h08,
               SN_CR_CLOSE  = 8'h10,
               SN_CR_SEND   = 8'h20,
               SN_CR_RECV   = 8'h40;
    localparam SOCK_CLOSED      = 8'h00,
               SOCK_INIT        = 8'h13,
               SOCK_LISTEN      = 8'h14,
               SOCK_ESTABLISHED = 8'h17;
    localparam SN_IR_SENDOK = 8'h10;
    localparam int TXBUF_SIZE = 2048;

    // регистры
    logic [7:0] common_regs [0:255];
    logic [7:0] sock0_regs  [0:255];
    logic [7:0] tx_buf      [0:TXBUF_SIZE-1];
    int         tx_rd = 0; // внутренний указатель чтения TX (Sn_TX_RD)
    int         msop_frames_ok = 0;
    int         model_errors = 0;

    task automatic model_reset();
        for (int i = 0; i < 256; i++) begin
            common_regs[i] = 8'h00;
            sock0_regs[i] = 8'h00;
        end
        common_regs[8'h39] = 8'h04; // VERSIONR
        sock0_regs[8'h03] = SOCK_CLOSED;
        tx_rd = 0;
        set_tx_fsr();
    endtask

    initial model_reset();
    always @(negedge rst_n) model_reset();

    function automatic int tx_wr_ptr();
        return {sock0_regs[8'h24], sock0_regs[8'h25]};
    endfunction

    task automatic set_tx_fsr();
        int free_sz;
        free_sz = TXBUF_SIZE - ((tx_wr_ptr() - tx_rd) & 16'hFFFF);
        if (free_sz < 0) free_sz = 0;
        sock0_regs[8'h20] = (free_sz >> 8) & 8'hFF;
        sock0_regs[8'h21] = free_sz & 8'hFF;
    endtask

    // «клиент» подключается через CONNECT_DELAY_NS после LISTEN
    task automatic schedule_connect();
        fork begin
            #(CONNECT_DELAY_NS);
            if (sock0_regs[8'h03] == SOCK_LISTEN) begin
                sock0_regs[8'h03] = SOCK_ESTABLISHED;
                $display("w5500 model: client connected (ESTABLISHED)");
            end
        end join_none
    endtask

    task automatic do_send();
        int wr, len, base;
        logic [7:0] b0, b1, bl0, bl1;
        wr = tx_wr_ptr();
        len = (wr - tx_rd) & 16'hFFFF;
        base = tx_rd;
        if (len <= 4) begin
            $display("w5500 model: SEND with bad len=%0d", len);
            model_errors = model_errors + 1;
        end else begin
            b0 = tx_buf[base % TXBUF_SIZE];
            b1 = tx_buf[(base + 1) % TXBUF_SIZE];
            bl0 = tx_buf[(base + len - 2) % TXBUF_SIZE];
            bl1 = tx_buf[(base + len - 1) % TXBUF_SIZE];
            if (b0 === 8'hFF && b1 === 8'hFE && bl0 === 8'hFF && bl1 === 8'h9B) begin
                msop_frames_ok = msop_frames_ok + 1;
                $display("w5500 model: msop frame ok len=%0d count=%0d",
                         len, msop_frames_ok);
            end else begin
                $display("w5500 model: SEND payload is not MSOP: len=%0d head=%02x%02x tail=%02x%02x",
                         len, b0, b1, bl0, bl1);
                model_errors = model_errors + 1;
            end
        end
        tx_rd = wr;
        set_tx_fsr();
        sock0_regs[8'h02] = sock0_regs[8'h02] | SN_IR_SENDOK;
    endtask

    task automatic sock0_command(input logic [7:0] cmd);
        case (cmd)
            SN_CR_OPEN: begin
                if (sock0_regs[8'h00][3:0] == 4'h1) begin // Sn_MR = TCP
                    sock0_regs[8'h03] = SOCK_INIT;
                    tx_rd = tx_wr_ptr(); // указатели выравниваются при OPEN
                    set_tx_fsr();
                end else
                    sock0_regs[8'h03] = SOCK_CLOSED;
            end
            SN_CR_LISTEN: begin
                sock0_regs[8'h03] = SOCK_LISTEN;
                schedule_connect();
            end
            SN_CR_SEND:   do_send();
            SN_CR_DISCON, SN_CR_CLOSE: sock0_regs[8'h03] = SOCK_CLOSED;
            SN_CR_RECV:   ;
            default:
                $display("w5500 model: unsupported Sn_CR=0x%02x", cmd);
        endcase
    endtask

    // ─────────────── запись/чтение блоков ───────────────
    task automatic block_write(input logic [4:0] bsb, input int addr,
                               input logic [7:0] data);
        case (bsb)
            5'd0: common_regs[addr & 8'hFF] = data;
            5'd1: begin
                if ((addr & 8'hFF) == 8'h01)
                    sock0_command(data);
                else if ((addr & 8'hFF) == 8'h02) // Sn_IR: RW1C
                    sock0_regs[8'h02] = sock0_regs[8'h02] & ~data;
                else
                    sock0_regs[addr & 8'hFF] = data;
            end
            5'd2: tx_buf[addr % TXBUF_SIZE] = data;
            default: begin
                $display("w5500 model: write to unsupported BSB=%0d", bsb);
                model_errors = model_errors + 1;
            end
        endcase
    endtask

    function automatic logic [7:0] block_read(input logic [4:0] bsb, input int addr);
        case (bsb)
            5'd0: return common_regs[addr & 8'hFF];
            5'd1: begin
                if ((addr & 8'hFF) == 8'h01)
                    return 8'h00; // Sn_CR автоочищается
                return sock0_regs[addr & 8'hFF];
            end
            5'd2: return tx_buf[addr % TXBUF_SIZE];
            default: return 8'h00;
        endcase
    endfunction

    // ─────────────── SPI slave (mode 0), VDM ───────────────
    logic [7:0] rx_shift;
    int rx_bit = 0;
    int byte_idx = 0;
    logic [15:0] frame_addr;
    logic [4:0]  frame_bsb;
    logic        frame_rwb; // 1 = запись

    always @(posedge sck) begin
        if (!cs_n && rst_n) begin
            rx_shift = {rx_shift[6:0], mosi};
            rx_bit = rx_bit + 1;
            if (rx_bit == 8) begin
                rx_bit = 0;
                if (byte_idx == 0)
                    frame_addr[15:8] = rx_shift;
                else if (byte_idx == 1)
                    frame_addr[7:0] = rx_shift;
                else if (byte_idx == 2) begin
                    frame_bsb = rx_shift[7:3];
                    frame_rwb = rx_shift[2];
                end else if (frame_rwb) begin
                    block_write(frame_bsb, frame_addr, rx_shift);
                    frame_addr = frame_addr + 1'b1;
                end
                byte_idx = byte_idx + 1;
            end
        end
    end

    // передача: бит по спаду SCK, MSB first; байт k предзагружается
    // на 8-м спаде байта k-1 (схема как в ltdc_x3_model)
    logic [7:0] tx_shift = 0;
    int neg_idx = 0;
    int tx_byte_idx = 0;

    function automatic logic [7:0] next_tx_byte();
        logic [7:0] r;
        if (tx_byte_idx >= 3 && !frame_rwb) begin
            r = block_read(frame_bsb, frame_addr);
            frame_addr = frame_addr + 1'b1;
            return r;
        end
        return 8'h00;
    endfunction

    always @(negedge cs_n) begin
        rx_bit = 0;
        byte_idx = 0;
        neg_idx = 0;
        tx_byte_idx = 1;
        tx_shift = 0;
        miso = 1'b0;
    end

    always @(posedge cs_n) miso = 1'bz;

    always @(negedge sck) begin
        if (!cs_n && rst_n) begin
            if (neg_idx == 7) begin
                tx_shift = next_tx_byte();
                tx_byte_idx = tx_byte_idx + 1;
                miso <= tx_shift[7];
                neg_idx = 0;
            end else begin
                miso <= tx_shift[6 - neg_idx];
                neg_idx = neg_idx + 1;
            end
        end
    end

endmodule
`endif // _w5500_model_
