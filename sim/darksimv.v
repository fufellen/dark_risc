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
    wire RX = 1;
    wire [31:0] LED;
    wire [31:0] OPORT;
    wire [3:0] DEBUG;

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

`ifdef DARKETH_LWIP_FRAME
    initial
    begin
        wait(RES == 0);
        #1_000_000;
        $display("FAIL darketh lwip sim timeout ready=%b available=%b",
                 ETH_RX_READY_FOR_FRAME, ETH_RX_FRAME_AVAILABLE);
        $fatal;
    end
`endif

    initial
    begin
        wait(RES == 0);
        #20_000;
`ifdef DARKETH_LWIP_FRAME
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

        $display("darketh sim rx udp packet");
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
        eth_byte(8'h20);
        eth_byte(8'h00);
        eth_byte(8'h01);
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
        eth_byte(8'hc0);
        eth_byte(8'ha8);
        eth_byte(8'h14);
        eth_byte(8'h14);
        eth_byte(8'h0f);
        eth_byte(8'ha0);
        eth_byte(8'h13);
        eth_byte(8'h8d);
        eth_byte(8'h00);
        eth_byte(8'h0c);
        eth_byte(8'h00);
        eth_byte(8'h00);
        eth_byte(8'h70);
        eth_byte(8'h69);
        eth_byte(8'h6e);
        eth_byte(8'h67);
        eth_commit_frame();
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
        end
    end
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

    darksocv
`ifdef SPI
    #(.SPI_DIV_COEF(1))
`endif
    soc0
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
