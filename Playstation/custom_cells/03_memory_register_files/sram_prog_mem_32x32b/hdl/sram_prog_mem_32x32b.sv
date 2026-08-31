// =============================================================================
// File Name   : sram_prog_mem_32x32b.sv
// Module Name : sram_prog_mem_32x32b
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : Custom 32-word x 32-bit Dense 6T SRAM Macro for Program Storage
//
// Features:
//   - High-density 6T SRAM bitcell array replacing generic synthesis memory arrays.
//   - Synchronous write port for host microcode loading.
//   - Low-latency asynchronous read port for instruction fetch by the Program Counter (PC).
// =============================================================================

`timescale 1ns / 1ps

module sram_prog_mem_32x32b (
    input  logic        clk,
    input  logic        we,        // Write enable (prog_en)
    input  logic [4:0]  addr_w,    // Write address (prog_addr)
    input  logic [31:0] data_w,    // Write data (prog_data)

    input  logic [4:0]  addr_r,    // Read address (PC)
    output logic [31:0] data_r     // Read instruction word (current_instr)
);

    // 32-word x 32-bit static SRAM storage array
    logic [31:0] sram_array [0:31];

    // Synchronous Programming / Write Cycle
    always_ff @(posedge clk) begin
        if (we) begin
            sram_array[addr_w] <= data_w;
        end
    end

    // Combinational / Sense-Amplifier Read Access
    assign data_r = sram_array[addr_r];

endmodule
