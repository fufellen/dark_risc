/*
 * CDC-мост между darkpsram_mmio (домен CPU) и EF_PSRAM_CTRL (домен psram_clk).
 *
 * Паттерн — как в проверенном darkddr3_ui_cdc: toggle-хендшейк запрос/ответ,
 * многобитные поля команды латчатся на стороне CPU до переключения toggle и
 * остаются стабильными, пока транзакция в полёте (quasi-static CDC).
 *
 * Протокол повторяет EF_PSRAM_CTRL: start — импульс 1 такт cpu_clk при
 * done=0; done — уровень с момента завершения до следующего start; data_o
 * валиден вместе с done. Контроллер на стороне psram_clk получает импульс
 * start длиной 1 такт psram_clk, его одноцикловый done латчится в rsp-toggle.
 */

`timescale 1ns / 1ps
`default_nettype none

module darkpsram_ctrl_cdc (
    // домен CPU (darkpsram_mmio)
    input  wire logic        cpu_clk,
    input  wire logic        cpu_res,          // активный 1
    input  wire logic [23:0] addr,
    input  wire logic [31:0] data_i,
    output logic      [31:0] data_o,
    input  wire logic [2:0]  size,
    input  wire logic        start,
    output wire              done,
    input  wire logic [3:0]  wait_states,
    input  wire logic [7:0]  cmd,
    input  wire logic        rd_wr,
    input  wire logic        qspi,
    input  wire logic        qpi,
    input  wire logic        short_cmd,

    // домен PSRAM-контроллера
    input  wire logic        psram_clk,
    input  wire logic        psram_rst_n,      // активный 0
    output wire              psram_sck,
    output wire              psram_ce_n,
    input  wire logic [3:0]  psram_din,
    output wire [3:0]        psram_dout,
    output wire [3:0]        psram_douten
);

    // -------- CPU-домен: латч команды + req-toggle --------
    // инициализированы, чтобы EF_PSRAM_CTRL не видел X в вычислении done
    logic [23:0] lat_addr = '0;
    logic [31:0] lat_data_i = '0;
    logic [2:0]  lat_size = 3'd4;
    logic [3:0]  lat_wait_states = '0;
    logic [7:0]  lat_cmd = 8'h03;
    logic        lat_rd_wr = 1'b1;
    logic        lat_qspi = 1'b0;
    logic        lat_qpi = 1'b0;
    logic        lat_short_cmd = 1'b0;

    logic req_toggle_cpu = 1'b0;
    logic pending_cpu = 1'b0;
    logic done_r = 1'b0;

    // start в тот же такт немедленно снимает done: darkpsram_mmio входит в
    // WAIT-состояние на следующем такте и не должен увидеть старый уровень
    assign done = done_r & ~start;

    logic rsp_toggle_meta, rsp_toggle_sync, rsp_toggle_seen;

    // сигналы psram-домена (объявлены до использования в CPU-блоке)
    logic rsp_toggle_ps = 1'b0;
    logic [31:0] rsp_data_ps = '0;

    always_ff @(posedge cpu_clk) begin
        if (cpu_res) begin
            req_toggle_cpu <= 1'b0;
            pending_cpu <= 1'b0;
            done_r <= 1'b0;
            rsp_toggle_meta <= 1'b0;
            rsp_toggle_sync <= 1'b0;
            rsp_toggle_seen <= 1'b0;
            data_o <= 32'd0;
        end else begin
            rsp_toggle_meta <= rsp_toggle_ps;
            rsp_toggle_sync <= rsp_toggle_meta;

            if (start && !pending_cpu) begin
                // synthesis translate_off
                $display("cdc: cpu start cmd=%02h req=%b t=%0t", cmd, !req_toggle_cpu, $time);
                // synthesis translate_on
                lat_addr <= addr;
                lat_data_i <= data_i;
                lat_size <= size;
                lat_wait_states <= wait_states;
                lat_cmd <= cmd;
                lat_rd_wr <= rd_wr;
                lat_qspi <= qspi;
                lat_qpi <= qpi;
                lat_short_cmd <= short_cmd;
                req_toggle_cpu <= !req_toggle_cpu;
                pending_cpu <= 1'b1;
                done_r <= 1'b0;
            end else if (pending_cpu && (rsp_toggle_sync != rsp_toggle_seen)) begin
                rsp_toggle_seen <= rsp_toggle_sync;
                data_o <= rsp_data_ps;   // стабилен: записан до rsp-toggle
                done_r <= 1'b1;
                pending_cpu <= 1'b0;
            end
        end
    end

    // -------- PSRAM-домен: приём req, запуск контроллера, ответ --------
    logic req_toggle_meta_ps, req_toggle_sync_ps, req_toggle_seen_ps;
    logic ctrl_start_ps = 1'b0;
    logic busy_ps = 1'b0;

    wire        ctrl_done_ps;
    wire [31:0] ctrl_data_o_ps;

    always_ff @(posedge psram_clk or negedge psram_rst_n) begin
        if (!psram_rst_n) begin
            req_toggle_meta_ps <= 1'b0;
            req_toggle_sync_ps <= 1'b0;
            req_toggle_seen_ps <= 1'b0;
            rsp_toggle_ps <= 1'b0;
            ctrl_start_ps <= 1'b0;
            busy_ps <= 1'b0;
        end else begin
            req_toggle_meta_ps <= req_toggle_cpu;
            req_toggle_sync_ps <= req_toggle_meta_ps;
            ctrl_start_ps <= 1'b0;

            if (!busy_ps && (req_toggle_sync_ps != req_toggle_seen_ps)) begin
                req_toggle_seen_ps <= req_toggle_sync_ps;
                ctrl_start_ps <= 1'b1;
                busy_ps <= 1'b1;
                // synthesis translate_off
                $display("cdc: ps start cmd=%02h t=%0t", lat_cmd, $time);
                // synthesis translate_on
            end else if (busy_ps && ctrl_done_ps) begin
                rsp_data_ps <= ctrl_data_o_ps;
                rsp_toggle_ps <= !rsp_toggle_ps;
                busy_ps <= 1'b0;
                // synthesis translate_off
                $display("cdc: ps done data=%08h t=%0t", ctrl_data_o_ps, $time);
                // synthesis translate_on
            end
        end
    end

    EF_PSRAM_CTRL psram_ctrl (
        .clk(psram_clk),
        .rst_n(psram_rst_n),
        .addr(lat_addr),
        .data_i(lat_data_i),
        .data_o(ctrl_data_o_ps),
        .size(lat_size),
        .start(ctrl_start_ps),
        .done(ctrl_done_ps),
        .wait_states(lat_wait_states),
        .cmd(lat_cmd),
        .rd_wr(lat_rd_wr),
        .qspi(lat_qspi),
        .qpi(lat_qpi),
        .short_cmd(lat_short_cmd),
        .sck(psram_sck),
        .ce_n(psram_ce_n),
        .din(psram_din),
        .dout(psram_dout),
        .douten(psram_douten)
    );

endmodule

`default_nettype wire
