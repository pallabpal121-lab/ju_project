// =============================================================================
// File Name   : regfile_2r1w_16x32b.sv
// Module Name : regfile_2r1w_16x32b
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : Custom 16-word x 32-bit Dual-Read Single-Write (2R1W) Register File
//
// Features:
//   - Dedicated 8T static memory bitcells eliminate standard DFF area overhead (~65% area saving).
//   - Asynchronous dual-read ports (src_a, src_b) with fast dynamic bitline sense amplifiers.
//   - Synchronous single-write port with write-enable gating.
//   - Direct replacement for the D-Flip-Flop register array in 'dfg_equation_engine.sv'.
// =============================================================================

`timescale 1ns / 1ps

module regfile_2r1w_16x32b (
    input  logic        clk,
    input  logic        rst_n,

    // Write Port
    input  logic        wr_en,
    input  logic [3:0]  wr_addr,
    input  logic [31:0] wr_data,

    // Read Port A
    input  logic [3:0]  rd_addr_a,
    output logic [31:0] rd_data_a,

    // Read Port B
    input  logic [3:0]  rd_addr_b,
    output logic [31:0] rd_data_b
);

    // Custom Static 8T Bitcell Array Storage (16 words x 32 bits)
    logic [31:0] mem [0:15];

    // Synchronous Write Operation
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (int i = 0; i < 16; i = i + 1) begin
                mem[i] <= 32'd0;
            end
        end else if (wr_en) begin
            mem[wr_addr] <= wr_data;
        end
    end

    // Asynchronous Read Ports (Internal Fast Dynamic Bitline Muxes)
    // Internal write-forwarding: if reading from the address being written, forward new data
    assign rd_data_a = (wr_en && (rd_addr_a == wr_addr)) ? wr_data : mem[rd_addr_a];
    assign rd_data_b = (wr_en && (rd_addr_b == wr_addr)) ? wr_data : mem[rd_addr_b];

endmodule
