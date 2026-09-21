// =============================================================================
// Company / Institution : Jadavpur University (Dept. of ETCE)
// Project               : Dedicated AI Hardware Accelerator
// File Name             : state_bram.sv
// Module Name           : state_bram
// Description           : Industry-Grade Dual-Port Synchronous Block RAM (BRAM)
//                         for state vector storage.
//                         Supports up to 128 variables (128 words x 32 bits).
//                         Port A: Memory-mapped access from AXI4-Lite slave.
//                         Port B: Dedicated high-speed access for the Newton BCD optimizer.
//                         Synthesizable for both ASIC (SRAM macro / standard cells)
//                         and FPGA (true dual-port block RAM).
// =============================================================================

`timescale 1ns / 1ps

module state_bram #(
    parameter int DEPTH      = 128,
    parameter int DATA_WIDTH = 32,
    parameter int ADDR_WIDTH = $clog2(DEPTH)
)(
    input  logic                    clk,

    // -------------------------------------------------------------------------
    // Port A: Host / AXI4-Lite MMIO Interface
    // -------------------------------------------------------------------------
    input  logic                    we_a,
    input  logic [ADDR_WIDTH-1:0]   addr_a,
    input  logic [DATA_WIDTH-1:0]   din_a,
    output logic [DATA_WIDTH-1:0]   dout_a,

    // -------------------------------------------------------------------------
    // Port B: Optimization Engine Interface
    // -------------------------------------------------------------------------
    input  logic                    we_b,
    input  logic [ADDR_WIDTH-1:0]   addr_b,
    input  logic [DATA_WIDTH-1:0]   din_b,
    output logic [DATA_WIDTH-1:0]   dout_b
);

    // 128 x 32-bit Memory Array
    logic [DATA_WIDTH-1:0] mem [0:DEPTH-1];

    // -------------------------------------------------------------------------
    // Synchronous Dual-Port Read / Write Process
    // Combined into a single clocked process to satisfy SystemVerilog single-driver
    // rules across all major EDA tools (Synopsys VCS / Design Compiler).
    // -------------------------------------------------------------------------
    always_ff @(posedge clk) begin
        // Port A (Host / AXI Access)
        if (we_a) begin
            mem[addr_a] <= din_a;
        end
        dout_a <= mem[addr_a];

        // Port B (Newton Optimization Core Access)
        if (we_b) begin
            mem[addr_b] <= din_b;
        end
        dout_b <= mem[addr_b];
    end

endmodule
