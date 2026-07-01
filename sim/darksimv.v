/*
 * Copyright (c) 2018, Marcelo Samsoniuk
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

`timescale 1ns / 1ps
`include "../rtl/config.vh"

// clock and reset logic

module darksimv;

    reg CLK = 0;
    
    reg RES = 1;

    initial while(1) #(500e6/`BOARD_CK) CLK = !CLK; // clock generator w/ freq defined by config.vh

    integer i;

    initial
    begin
`ifdef __ICARUS__
        $dumpfile("darksocv.vcd");
        $dumpvars();

    `ifdef __REGDUMP__
        for(i=0;i!=`RLEN;i=i+1)
        begin
            $dumpvars(0,soc0.bridge0.core0.REGS[i]);
        end
    `endif
`endif
        $display("reset (startup)");
        #1e3    RES = 0;            // wait 1us in reset state
        //#1000e3 RES = 1;            // run  1ms
        //$display("reset (restart)");
        //#1e3    RES = 0;            // wait 1us in reset state
        //#1000e3 $finish();          // run  1ms
    end

    wire TX;
    reg RX = 1;
    wire [31:0] LED;
    wire [31:0] OPORT;
    wire [3:0] DEBUG;

    task uart_rx_byte(input [7:0] data);
        integer bit_idx;
    begin
        RX <= 1;
        repeat(`__BAUD__) @(posedge CLK);
        RX <= 0;
        repeat(`__BAUD__) @(posedge CLK);

        for(bit_idx=0; bit_idx<8; bit_idx=bit_idx+1)
        begin
            RX <= data[bit_idx];
            repeat(`__BAUD__) @(posedge CLK);
        end

        RX <= 1;
        repeat(`__BAUD__) @(posedge CLK);
    end
    endtask

`ifdef DARKETH_MMIO
    reg        ETH_RX_BYTE_VALID = 0;
    reg  [7:0] ETH_RX_BYTE = 0;
    reg        ETH_RX_FRAME_VALID = 0;
    reg        ETH_RX_FRAME_DROP = 0;
    wire       ETH_RX_READY_FOR_FRAME;
    wire       ETH_RX_FRAME_AVAILABLE;
    reg        ETH_TX_BYTE_READY = 1;
    wire [7:0] ETH_TX_BYTE;
    wire       ETH_TX_BYTE_VALID;
    wire       ETH_TX_FRAME_START;
    wire       ETH_TX_FRAME_END;
    wire       ETH_TX_READY_FOR_FRAME;
    wire       ETH_TX_BUSY;
    wire       ETH_TX_DONE;
    wire       ETH_TX_OVERFLOW;
    wire       ETH_CFG_MAC_FILTER_ENABLE;
    wire [47:0] ETH_CFG_LOCAL_MAC;
    wire       ETH_CFG_ACCEPT_BROADCAST;
    wire       ETH_CFG_ACCEPT_MULTICAST;

    task eth_byte(input [7:0] data);
    begin
        @(posedge CLK);
        ETH_RX_BYTE <= data;
        ETH_RX_BYTE_VALID <= 1;
        @(posedge CLK);
        ETH_RX_BYTE_VALID <= 0;
    end
    endtask

    task eth_commit_frame;
    begin
        @(posedge CLK);
        ETH_RX_FRAME_VALID <= 1;
        @(posedge CLK);
        ETH_RX_FRAME_VALID <= 0;
    end
    endtask

    task eth_u16(input [15:0] data);
    begin
        eth_byte(data[15:8]);
        eth_byte(data[7:0]);
    end
    endtask

    task eth_u32(input [31:0] data);
    begin
        eth_byte(data[31:24]);
        eth_byte(data[23:16]);
        eth_byte(data[15:8]);
        eth_byte(data[7:0]);
    end
    endtask

`ifdef DARKETH_LWIP_TCP_DATA_FRAME
`ifdef DARKETH_LWIP_TCP_DATA_TARGET_FRAMES
    localparam integer TCP_DATA_MSOP_TARGET_FRAMES = `DARKETH_LWIP_TCP_DATA_TARGET_FRAMES;
`else
    localparam integer TCP_DATA_MSOP_TARGET_FRAMES = 3;
`endif
    localparam integer TCP_DATA_MSOP_FRAME_BYTES = 758;
    reg [7:0] eth_tx_capture [0:1599];
    integer eth_tx_len = 0;
    reg tcp_data_synack_seen = 0;
    reg [31:0] tcp_data_server_iss = 0;
    integer tcp_data_ip_id = 16'h0100;
    reg [7:0] tcp_data_msop_capture [0:1023];
    integer tcp_data_msop_len = 0;
    integer tcp_data_msop_segments = 0;
    integer tcp_data_msop_frame_segments = 0;
    integer tcp_data_msop_frame_count = 0;
    reg tcp_data_msop_seen = 0;
    integer tcp_data_ip_header_len = 0;
    integer tcp_data_tcp_header_len = 0;
    integer tcp_data_ip_total_len = 0;
    integer tcp_data_payload_offset = 0;
    integer tcp_data_payload_len = 0;
    integer tcp_data_payload_idx = 0;
    reg [31:0] tcp_data_tx_seq = 0;
    reg [31:0] tcp_data_ack_value = 0;
    reg tcp_data_ack_request = 0;
    reg tcp_data_ack_sender_busy = 0;

    task tcp_data_fail(input [1023:0] message);
    begin
        $display("FAIL darketh sim tcp data %0s", message);
        $fatal;
    end
    endtask

    task tcp_data_check_msop_payload;
    begin
        if (tcp_data_msop_frame_count >= TCP_DATA_MSOP_TARGET_FRAMES) begin
            tcp_data_fail("unexpected extra msop frame");
        end

        if (tcp_data_msop_capture[0] !== 8'hff ||
            tcp_data_msop_capture[1] !== 8'hfe ||
            tcp_data_msop_capture[2] !== 8'h01 ||
            tcp_data_msop_capture[3] !== 8'h01 ||
            tcp_data_msop_capture[4] !== 8'h00 ||
            tcp_data_msop_capture[5] !== (tcp_data_msop_frame_count & 8'hff) ||
            tcp_data_msop_capture[6] !== ((tcp_data_msop_frame_count >> 8) & 8'hff) ||
            tcp_data_msop_capture[7] !== 8'hb4 ||
            tcp_data_msop_capture[8] !== 8'h00 ||
            tcp_data_msop_capture[17] !== 8'h00 ||
            tcp_data_msop_capture[18] !== 8'h00 ||
            tcp_data_msop_capture[19] !== 8'h00 ||
            tcp_data_msop_capture[20] !== 8'h00 ||
            tcp_data_msop_capture[21] !== 8'h01 ||
            tcp_data_msop_capture[22] !== 8'h90 ||
            tcp_data_msop_capture[23] !== 8'hd0 ||
            tcp_data_msop_capture[24] !== 8'h03 ||
            tcp_data_msop_capture[25] !== 8'h00 ||
            tcp_data_msop_capture[26] !== 8'h40 ||
            tcp_data_msop_capture[27] !== 8'h7e ||
            tcp_data_msop_capture[28] !== 8'h05 ||
            tcp_data_msop_capture[29] !== 8'h00 ||
            tcp_data_msop_capture[30] !== 8'hd0 ||
            tcp_data_msop_capture[31] !== 8'h07 ||
            tcp_data_msop_capture[32] !== 8'h02 ||
            tcp_data_msop_capture[33] !== 8'h00 ||
            tcp_data_msop_capture[34] !== 8'h03 ||
            tcp_data_msop_capture[35] !== 8'h02 ||
            tcp_data_msop_capture[TCP_DATA_MSOP_FRAME_BYTES - 2] !== 8'hff ||
            tcp_data_msop_capture[TCP_DATA_MSOP_FRAME_BYTES - 1] !== 8'h9b) begin
            tcp_data_fail("msop payload format mismatch");
        end

        $display("darketh sim tcp data msop header ok points=180 angle_res=2000 dist_bytes=2 echo=3/2");

        tcp_data_msop_frame_count = tcp_data_msop_frame_count + 1;
        $display("darketh sim tcp data msop frame ok count=%0d len=%04x frame=%0d segments=%0d",
                 tcp_data_msop_frame_count, tcp_data_msop_len[15:0],
                 tcp_data_msop_frame_count - 1, tcp_data_msop_frame_segments);

        if (tcp_data_msop_frame_count >= TCP_DATA_MSOP_TARGET_FRAMES) begin
            tcp_data_msop_seen = 1'b1;
            $display("darketh sim tcp data msop payload ok len=%04x frames=%0d segments=%0d",
                     tcp_data_msop_len[15:0], tcp_data_msop_frame_count,
                     tcp_data_msop_segments);
        end

        tcp_data_msop_len = 0;
        tcp_data_msop_frame_segments = 0;
    end
    endtask

    task eth_send_tcp_data_segment(
        input [31:0] seq,
        input [31:0] ack,
        input [7:0] flags
    );
    begin
        eth_byte(8'h02);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h01);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_u16(16'h0800);

        eth_byte(8'h45);
        eth_byte(8'h00);
        eth_u16(16'd40);
        eth_u16(tcp_data_ip_id[15:0]);
        tcp_data_ip_id = tcp_data_ip_id + 1;
        eth_u16(16'h0000);
        eth_byte(8'h40);
        eth_byte(8'h06);
        eth_u16(16'h0000);
        eth_u32(32'hc0a80f0d);
        eth_u32(32'hc0a80f14);

        eth_u16(16'd40000);
        eth_u16(16'd50100);
        eth_u32(seq);
        eth_u32(ack);
        eth_byte(8'h50);
        eth_byte(flags);
        eth_u16(16'h2000);
        eth_u16(16'h0000);
        eth_u16(16'h0000);

        eth_commit_frame();
        wait(ETH_RX_FRAME_AVAILABLE);
        wait(ETH_RX_READY_FOR_FRAME);
    end
    endtask

    task eth_send_tcp_data_syn;
    begin
        $display("darketh sim rx tcp data syn");
        eth_send_tcp_data_segment(32'h01020304, 32'h00000000, 8'h02);
        $display("darketh sim tcp data syn consumed");
    end
    endtask

    task eth_send_tcp_data_ack;
    begin
        wait(tcp_data_synack_seen);
        $display("darketh sim rx tcp data ack iss=%x", tcp_data_server_iss);
        eth_send_tcp_data_segment(32'h01020305, tcp_data_server_iss + 32'd1,
                                  8'h10);
        $display("darketh sim tcp data ack consumed");
    end
    endtask

    initial
    begin
        forever begin
            wait(tcp_data_ack_request);
            tcp_data_ack_sender_busy = 1'b1;
            tcp_data_ack_request = 1'b0;
            #100_000;
            eth_send_tcp_data_segment(32'h01020305, tcp_data_ack_value, 8'h10);
            $display("darketh sim tcp data payload ack=%x consumed",
                     tcp_data_ack_value);
            tcp_data_ack_sender_busy = 1'b0;
        end
    end
`endif

`ifdef DARKETH_LWIP_FRAME
`ifdef DARKPSRAM_MMIO
`ifdef DARKETH_LWIP_TCP_DATA_FRAME
    localparam integer DARKETH_LWIP_SIM_TIMEOUT_NS =
        200_000_000 + ((TCP_DATA_MSOP_TARGET_FRAMES > 5) ?
                       ((TCP_DATA_MSOP_TARGET_FRAMES - 5) * 25_000_000) :
                       0);
`else
    localparam integer DARKETH_LWIP_SIM_TIMEOUT_NS = 200_000_000;
`endif
`else
    localparam integer DARKETH_LWIP_SIM_TIMEOUT_NS = 30_000_000;
`endif

    initial
    begin
        wait(RES == 0);
        #DARKETH_LWIP_SIM_TIMEOUT_NS;
        $display("FAIL darketh lwip sim timeout timeout_ns=%0d ready=%b available=%b",
                 DARKETH_LWIP_SIM_TIMEOUT_NS, ETH_RX_READY_FOR_FRAME,
                 ETH_RX_FRAME_AVAILABLE);
        $fatal;
    end
`endif

    initial
    begin
        wait(RES == 0);
        #20_000;
`ifdef DARKETH_LWIP_FRAME
        #100_000;
        $display("darketh sim rx arp request");
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_byte(8'h08);
        eth_byte(8'h06);
        eth_byte(8'h00);
        eth_byte(8'h01);
        eth_byte(8'h08);
        eth_byte(8'h00);
        eth_byte(8'h06);
        eth_byte(8'h04);
        eth_byte(8'h00);
        eth_byte(8'h01);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h14);
        eth_byte(8'h0a);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h14);
        eth_byte(8'h14);
        eth_commit_frame();
        wait(ETH_RX_FRAME_AVAILABLE);
        wait(ETH_RX_READY_FOR_FRAME);
        $display("darketh sim arp consumed");
        #20_000;

        $display("darketh sim rx lidar discovery broadcast");
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_byte(8'h08);
        eth_byte(8'h00);
        eth_byte(8'h45);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h2d);
        eth_byte(8'h00);
        eth_byte(8'h02);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h40);
        eth_byte(8'h11);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h14);
        eth_byte(8'h0a);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'h0f);
        eth_byte(8'ha0);
        eth_byte(8'hc3);
        eth_byte(8'hb7);
        eth_byte(8'h00);
        eth_byte(8'h19);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hff);
        eth_byte(8'hfe);
        eth_byte(8'h4c);
        eth_byte(8'h49);
        eth_byte(8'h44);
        eth_byte(8'h41);
        eth_byte(8'h52);
        eth_byte(8'h5f);
        eth_byte(8'h52);
        eth_byte(8'h45);
        eth_byte(8'h51);
        eth_byte(8'h53);
        eth_byte(8'h00);
        eth_byte(8'h01);
        eth_byte(8'h11);
        eth_byte(8'hff);
        eth_byte(8'h9b);
        eth_byte(8'h00);
        eth_commit_frame();
        wait(ETH_RX_FRAME_AVAILABLE);
        wait(ETH_RX_READY_FOR_FRAME);
        $display("darketh sim discovery consumed");
        #20_000;

        $display("darketh sim rx lidar net_config command");
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_byte(8'h08);
        eth_byte(8'h00);
        eth_byte(8'h45);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h3f);
        eth_byte(8'h00);
        eth_byte(8'h04);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h40);
        eth_byte(8'h11);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h14);
        eth_byte(8'h0a);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'h0f);
        eth_byte(8'ha0);
        eth_byte(8'hc3);
        eth_byte(8'hb5);
        eth_byte(8'h00);
        eth_byte(8'h2b);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hff);
        eth_byte(8'hfe);
        eth_byte(8'h0c);
        eth_byte(8'ha1);
        eth_byte(8'h22);
        eth_byte(8'h60);
        eth_byte(8'h01);
        eth_byte(8'h18);
        eth_byte(8'h00);
        eth_byte(8'h02);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h01);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h0f);
        eth_byte(8'h14);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h0f);
        eth_byte(8'h0d);
        eth_byte(8'h00);
        eth_byte(8'hb4);
        eth_byte(8'hc3);
        eth_byte(8'hb4);
        eth_byte(8'hc3);
        eth_byte(8'h00);
        eth_byte(8'hb5);
        eth_byte(8'hc3);
        eth_byte(8'hb5);
        eth_byte(8'hc3);
        eth_byte(8'hff);
        eth_byte(8'h9b);
        eth_commit_frame();
        wait(ETH_RX_FRAME_AVAILABLE);
        wait(ETH_RX_READY_FOR_FRAME);
        $display("darketh sim net_config consumed");
        #1_000_000;

        $display("darketh sim rx arp request after net_config");
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'hff);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_byte(8'h08);
        eth_byte(8'h06);
        eth_byte(8'h00);
        eth_byte(8'h01);
        eth_byte(8'h08);
        eth_byte(8'h00);
        eth_byte(8'h06);
        eth_byte(8'h04);
        eth_byte(8'h00);
        eth_byte(8'h01);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h0f);
        eth_byte(8'h0d);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h0f);
        eth_byte(8'h14);
        eth_commit_frame();
        wait(ETH_RX_FRAME_AVAILABLE);
        wait(ETH_RX_READY_FOR_FRAME);
        $display("darketh sim net_config arp consumed");
        #1_000_000;

        $display("darketh sim rx lidar full-status command after net_config");
        eth_byte(8'h02);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h20);
        eth_byte(8'h01);
        eth_byte(8'h02);
        eth_byte(8'haa);
        eth_byte(8'hbb);
        eth_byte(8'hcc);
        eth_byte(8'hdd);
        eth_byte(8'hee);
        eth_byte(8'h08);
        eth_byte(8'h00);
        eth_byte(8'h45);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h35);
        eth_byte(8'h00);
        eth_byte(8'h05);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h40);
        eth_byte(8'h11);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h0f);
        eth_byte(8'h0d);
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h0f);
        eth_byte(8'h14);
        eth_byte(8'h0f);
        eth_byte(8'ha1);
        eth_byte(8'hc3);
        eth_byte(8'hb5);
        eth_byte(8'h00);
        eth_byte(8'h21);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hff);
        eth_byte(8'hfe);
        eth_byte(8'h0c);
        eth_byte(8'ha0);
        eth_byte(8'h23);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'hff);
        eth_byte(8'h9b);
        eth_commit_frame();
`ifdef DARKETH_LWIP_TCP_DATA_FRAME
        wait(ETH_RX_FRAME_AVAILABLE);
        wait(ETH_RX_READY_FOR_FRAME);
        $display("darketh sim full-status consumed");
        #1_000_000;

        eth_send_tcp_data_syn();
        #200_000;
        eth_send_tcp_data_ack();
`endif
`else
        eth_byte(8'hde);
        eth_byte(8'had);
        eth_byte(8'hbe);
        eth_byte(8'hef);
        eth_byte(8'h08);
        eth_byte(8'h00);
        eth_commit_frame();
`endif
    end

    always@(posedge CLK)
    begin
        if(ETH_TX_BYTE_VALID)
        begin
            if(ETH_TX_FRAME_START) $write("darketh tx data=");
            $write("%x",ETH_TX_BYTE);
            if(ETH_TX_FRAME_END) $display("");
`ifdef DARKETH_LWIP_TCP_DATA_FRAME
            if(ETH_TX_FRAME_START)
            begin
                eth_tx_len = 0;
            end
            if(eth_tx_len < 1600)
            begin
                eth_tx_capture[eth_tx_len] = ETH_TX_BYTE;
                eth_tx_len = eth_tx_len + 1;
            end
            if(ETH_TX_FRAME_END)
            begin
                if(eth_tx_len >= 54 &&
                   eth_tx_capture[12] == 8'h08 &&
                   eth_tx_capture[13] == 8'h00 &&
                   eth_tx_capture[23] == 8'h06 &&
                   eth_tx_capture[34] == 8'hc3 &&
                   eth_tx_capture[35] == 8'hb4 &&
                   eth_tx_capture[36] == 8'h9c &&
                   eth_tx_capture[37] == 8'h40 &&
                   eth_tx_capture[47] == 8'h12)
                begin
                    tcp_data_server_iss = {
                        eth_tx_capture[38],
                        eth_tx_capture[39],
                        eth_tx_capture[40],
                        eth_tx_capture[41]
                    };
                    tcp_data_synack_seen = 1'b1;
                    $display("darketh sim tcp data synack iss=%x",
                             tcp_data_server_iss);
                end

                if(!tcp_data_msop_seen &&
                   eth_tx_len >= 54 &&
                   eth_tx_capture[12] == 8'h08 &&
                   eth_tx_capture[13] == 8'h00 &&
                   eth_tx_capture[23] == 8'h06) begin
                    tcp_data_ip_header_len = (eth_tx_capture[14] & 8'h0f) * 4;
                    tcp_data_ip_total_len =
                        (eth_tx_capture[16] * 256) + eth_tx_capture[17];
                    tcp_data_tcp_header_len =
                        (eth_tx_capture[14 + tcp_data_ip_header_len + 12] >> 4) * 4;
                    tcp_data_payload_offset = 14 + tcp_data_ip_header_len +
                                              tcp_data_tcp_header_len;
                    tcp_data_payload_len = tcp_data_ip_total_len -
                                           tcp_data_ip_header_len -
                                           tcp_data_tcp_header_len;

                    if(tcp_data_payload_len > 0 &&
                       eth_tx_len >= (tcp_data_payload_offset + tcp_data_payload_len) &&
                       eth_tx_capture[14 + tcp_data_ip_header_len] == 8'hc3 &&
                       eth_tx_capture[14 + tcp_data_ip_header_len + 1] == 8'hb4 &&
                       eth_tx_capture[14 + tcp_data_ip_header_len + 2] == 8'h9c &&
                       eth_tx_capture[14 + tcp_data_ip_header_len + 3] == 8'h40) begin
                        tcp_data_tx_seq = {
                            eth_tx_capture[14 + tcp_data_ip_header_len + 4],
                            eth_tx_capture[14 + tcp_data_ip_header_len + 5],
                            eth_tx_capture[14 + tcp_data_ip_header_len + 6],
                            eth_tx_capture[14 + tcp_data_ip_header_len + 7]
                        };
                        tcp_data_ack_value = tcp_data_tx_seq + tcp_data_payload_len;
                        tcp_data_ack_request = 1'b1;
                        tcp_data_msop_segments = tcp_data_msop_segments + 1;
                        tcp_data_msop_frame_segments = tcp_data_msop_frame_segments + 1;

                        for(tcp_data_payload_idx = 0;
                            tcp_data_payload_idx < tcp_data_payload_len;
                            tcp_data_payload_idx = tcp_data_payload_idx + 1) begin
                            if(tcp_data_msop_len >= TCP_DATA_MSOP_FRAME_BYTES) begin
                                tcp_data_fail("msop payload too long");
                            end
                            tcp_data_msop_capture[tcp_data_msop_len] =
                                eth_tx_capture[tcp_data_payload_offset + tcp_data_payload_idx];
                            tcp_data_msop_len = tcp_data_msop_len + 1;
                        end

                        if(tcp_data_msop_len == TCP_DATA_MSOP_FRAME_BYTES) begin
                            tcp_data_check_msop_payload();
                        end
                    end
                end
            end
`endif
        end
    end
`endif

`ifdef DARKDDR3_MMIO
    wire        DDR3_UI_RD;
    wire        DDR3_UI_WR;
    wire        DDR3_UI_REFRESH;
    wire [25:0] DDR3_UI_ADDR;
    wire [15:0] DDR3_UI_DIN;
    reg  [15:0] DDR3_UI_DOUT = 0;
    reg         DDR3_UI_DATA_READY = 0;
    reg         DDR3_UI_BUSY = 1;
    reg         DDR3_UI_WRITE_LEVEL_DONE = 0;
    reg         DDR3_UI_READ_CALIB_DONE = 0;

    reg [15:0] ddr3_mock_mem [0:255];
    reg [25:0] ddr3_pending_read_addr = 0;
    integer ddr3_init_count = 0;
    integer ddr3_busy_count = 0;
    integer ddr3_read_count = 0;

    initial
    begin
        ddr3_mock_mem[8'h40] = 16'h1234;
        ddr3_mock_mem[8'h41] = 16'habcd;

        wait(RES == 0);
        #500_000_000;
        $display("FAIL darkddr3 sim timeout busy=%b wlevel=%b rcalib=%b",
                 DDR3_UI_BUSY, DDR3_UI_WRITE_LEVEL_DONE, DDR3_UI_READ_CALIB_DONE);
        $fatal;
    end

    always@(posedge CLK)
    begin
        DDR3_UI_DATA_READY <= 0;

        if(RES)
        begin
            DDR3_UI_BUSY <= 1;
            DDR3_UI_WRITE_LEVEL_DONE <= 0;
            DDR3_UI_READ_CALIB_DONE <= 0;
            DDR3_UI_DOUT <= 0;
            ddr3_pending_read_addr <= 0;
            ddr3_init_count <= 0;
            ddr3_busy_count <= 0;
            ddr3_read_count <= 0;
        end
        else
        begin
            if(!DDR3_UI_WRITE_LEVEL_DONE || !DDR3_UI_READ_CALIB_DONE)
            begin
                ddr3_init_count <= ddr3_init_count + 1;
                DDR3_UI_BUSY <= 1;

                if(ddr3_init_count == 4)
                begin
                    DDR3_UI_WRITE_LEVEL_DONE <= 1;
                    DDR3_UI_READ_CALIB_DONE <= 1;
                    DDR3_UI_BUSY <= 0;
                end
            end
            else if(ddr3_busy_count != 0)
            begin
                ddr3_busy_count <= ddr3_busy_count - 1;
                DDR3_UI_BUSY <= ddr3_busy_count != 1;
            end
            else
            begin
                DDR3_UI_BUSY <= 0;
            end

            if(ddr3_read_count != 0)
            begin
                ddr3_read_count <= ddr3_read_count - 1;
                if(ddr3_read_count == 1)
                begin
                    DDR3_UI_DOUT <= ddr3_mock_mem[ddr3_pending_read_addr[7:0]];
                    DDR3_UI_DATA_READY <= 1;
                end
            end

            if(DDR3_UI_WR)
            begin
                ddr3_mock_mem[DDR3_UI_ADDR[7:0]] <= DDR3_UI_DIN;
                DDR3_UI_BUSY <= 1;
                ddr3_busy_count <= 2;
            end

            if(DDR3_UI_RD)
            begin
                ddr3_pending_read_addr <= DDR3_UI_ADDR;
                DDR3_UI_BUSY <= 1;
                ddr3_busy_count <= 3;
                ddr3_read_count <= 3;
            end

            if(DDR3_UI_REFRESH)
            begin
                DDR3_UI_BUSY <= 1;
                ddr3_busy_count <= 2;
            end
        end
    end
`endif

`ifdef DARKPSRAM_MMIO
    wire [3:0] PSRAM_DIN;
    wire [3:0] PSRAM_DOUT;
    wire [3:0] PSRAM_DOUTEN;
    wire       PSRAM_SCK;
    wire       PSRAM_CE_N;
    wire [3:0] PSRAM_SIO;

    assign PSRAM_SIO[0] = PSRAM_DOUTEN[0] ? PSRAM_DOUT[0] : 1'bz;
    assign PSRAM_SIO[1] = PSRAM_DOUTEN[1] ? PSRAM_DOUT[1] : 1'bz;
    assign PSRAM_SIO[2] = PSRAM_DOUTEN[2] ? PSRAM_DOUT[2] : 1'bz;
    assign PSRAM_SIO[3] = PSRAM_DOUTEN[3] ? PSRAM_DOUT[3] : 1'bz;
    assign PSRAM_DIN = PSRAM_SIO;

    is66wvs1m8_model #(
        .ADDR_BITS(14)
    ) psram_mock (
        .ce_n(PSRAM_CE_N),
        .sck(PSRAM_SCK),
        .sio(PSRAM_SIO)
    );
`endif

`ifdef __SDRAM__

    // sdram sim model!

    wire        S_NWE,S_CLK;
    wire  [1:0] S_DQM;
    reg  [15:0] S_DBFF = 0;  
    wire [15:0] S_DB = S_NWE ? S_DBFF : 16'hzzzz;

    always@(negedge S_CLK)
    begin
        if(S_NWE==0 && S_DQM[1]==0) S_DBFF[15:8] <= S_DB[15:8];
        if(S_NWE==0 && S_DQM[0]==0) S_DBFF[ 7:0] <= S_DB[ 7:0];
    end

`endif

    darksocv #(
        .SPI_DIV_COEF(1),
`ifdef DARKPSRAM_QSPI
        .PSRAM_USE_QSPI(1'b1)
`else
        .PSRAM_USE_QSPI(1'b0)
`endif
    ) soc0
    (
        .XCLK(CLK),
        .XRES(|RES),
`ifdef DARKETH_MMIO
        .ETH_RX_BYTE_VALID(ETH_RX_BYTE_VALID),
        .ETH_RX_BYTE(ETH_RX_BYTE),
        .ETH_RX_FRAME_VALID(ETH_RX_FRAME_VALID),
        .ETH_RX_FRAME_DROP(ETH_RX_FRAME_DROP),
        .ETH_RX_READY_FOR_FRAME(ETH_RX_READY_FOR_FRAME),
        .ETH_RX_FRAME_AVAILABLE(ETH_RX_FRAME_AVAILABLE),
        .ETH_TX_BYTE_READY(ETH_TX_BYTE_READY),
        .ETH_TX_BYTE(ETH_TX_BYTE),
        .ETH_TX_BYTE_VALID(ETH_TX_BYTE_VALID),
        .ETH_TX_FRAME_START(ETH_TX_FRAME_START),
        .ETH_TX_FRAME_END(ETH_TX_FRAME_END),
        .ETH_TX_READY_FOR_FRAME(ETH_TX_READY_FOR_FRAME),
        .ETH_TX_BUSY(ETH_TX_BUSY),
        .ETH_TX_DONE(ETH_TX_DONE),
        .ETH_TX_OVERFLOW(ETH_TX_OVERFLOW),
        .ETH_CFG_MAC_FILTER_ENABLE(ETH_CFG_MAC_FILTER_ENABLE),
        .ETH_CFG_LOCAL_MAC(ETH_CFG_LOCAL_MAC),
        .ETH_CFG_ACCEPT_BROADCAST(ETH_CFG_ACCEPT_BROADCAST),
        .ETH_CFG_ACCEPT_MULTICAST(ETH_CFG_ACCEPT_MULTICAST),
`endif
`ifdef DARKDDR3_MMIO
        .DDR3_UI_RD(DDR3_UI_RD),
        .DDR3_UI_WR(DDR3_UI_WR),
        .DDR3_UI_REFRESH(DDR3_UI_REFRESH),
        .DDR3_UI_ADDR(DDR3_UI_ADDR),
        .DDR3_UI_DIN(DDR3_UI_DIN),
        .DDR3_UI_DOUT(DDR3_UI_DOUT),
        .DDR3_UI_DATA_READY(DDR3_UI_DATA_READY),
        .DDR3_UI_BUSY(DDR3_UI_BUSY),
        .DDR3_UI_WRITE_LEVEL_DONE(DDR3_UI_WRITE_LEVEL_DONE),
        .DDR3_UI_READ_CALIB_DONE(DDR3_UI_READ_CALIB_DONE),
`endif
`ifdef DARKPSRAM_MMIO
        .PSRAM_DIN(PSRAM_DIN),
        .PSRAM_DOUT(PSRAM_DOUT),
        .PSRAM_DOUTEN(PSRAM_DOUTEN),
        .PSRAM_SCK(PSRAM_SCK),
        .PSRAM_CE_N(PSRAM_CE_N),
`endif
`ifdef __SDRAM__
        .S_CLK(S_CLK),
        .S_NWE(S_NWE),
        .S_DQM(S_DQM),
        .S_DB (S_DB),
`endif
        .IPORT(0),
        .UART_RXD(RX),
        .UART_TXD(TX),
        .LED(LED),
        .OPORT(OPORT),
        .DEBUG(DEBUG)
    );

endmodule
