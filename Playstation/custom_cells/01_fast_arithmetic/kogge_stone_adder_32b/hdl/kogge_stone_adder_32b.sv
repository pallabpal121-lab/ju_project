// =============================================================================
// File Name   : kogge_stone_adder_32b.sv
// Module Name : kogge_stone_adder_32b
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : 32-bit High-Speed Kogge-Stone Parallel-Prefix Adder
//
// Characteristics:
//   - Radix-2 Prefix Tree with minimum logical depth: ceil(log2(32)) = 5 levels
//   - Extremely low carry propagation latency (5-6 gate delays across 32 bits)
//   - Critical component for the 32-bit Q16.16 ALU and Booth Multiplier CPA
// =============================================================================

`timescale 1ns / 1ps

module kogge_stone_adder_32b (
    input  logic [31:0] a,
    input  logic [31:0] b,
    input  logic        cin,
    output logic [31:0] sum,
    output logic        cout
);

    // Level 0: Bit-level Generate and Propagate
    logic [31:0] g_0, p_0;
    
    // Level 1 to 5 Prefix Signals
    logic [31:0] g_1, p_1;
    logic [31:0] g_2, p_2;
    logic [31:0] g_3, p_3;
    logic [31:0] g_4, p_4;
    logic [31:0] g_5, p_5;

    // Carries
    logic [31:0] c;

    // -------------------------------------------------------------------------
    // Level 0: PG Generation
    // -------------------------------------------------------------------------
    genvar i;
    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_pg0
            assign p_0[i] = a[i] ^ b[i];
            assign g_0[i] = a[i] & b[i];
        end
    endgenerate

    // Include input carry into bit 0 generate
    logic g_0_in;
    assign g_0_in = g_0[0] | (p_0[0] & cin);

    // -------------------------------------------------------------------------
    // Level 1: Distance = 1
    // -------------------------------------------------------------------------
    assign g_1[0] = g_0_in;
    assign p_1[0] = p_0[0];
    generate
        for (i = 1; i < 32; i = i + 1) begin : gen_level1
            assign g_1[i] = g_0[i] | (p_0[i] & (i == 1 ? g_0_in : g_0[i-1]));
            assign p_1[i] = p_0[i] & p_0[i-1];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Level 2: Distance = 2
    // -------------------------------------------------------------------------
    generate
        for (i = 0; i < 2; i = i + 1) begin : gen_level2_pass
            assign g_2[i] = g_1[i];
            assign p_2[i] = p_1[i];
        end
        for (i = 2; i < 32; i = i + 1) begin : gen_level2
            assign g_2[i] = g_1[i] | (p_1[i] & g_1[i-2]);
            assign p_2[i] = p_1[i] & p_1[i-2];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Level 3: Distance = 4
    // -------------------------------------------------------------------------
    generate
        for (i = 0; i < 4; i = i + 1) begin : gen_level3_pass
            assign g_3[i] = g_2[i];
            assign p_3[i] = p_2[i];
        end
        for (i = 4; i < 32; i = i + 1) begin : gen_level3
            assign g_3[i] = g_2[i] | (p_2[i] & g_2[i-4]);
            assign p_3[i] = p_2[i] & p_2[i-4];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Level 4: Distance = 8
    // -------------------------------------------------------------------------
    generate
        for (i = 0; i < 8; i = i + 1) begin : gen_level4_pass
            assign g_4[i] = g_3[i];
            assign p_4[i] = p_3[i];
        end
        for (i = 8; i < 32; i = i + 1) begin : gen_level4
            assign g_4[i] = g_3[i] | (p_3[i] & g_3[i-8]);
            assign p_4[i] = p_3[i] & p_3[i-8];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Level 5: Distance = 16
    // -------------------------------------------------------------------------
    generate
        for (i = 0; i < 16; i = i + 1) begin : gen_level5_pass
            assign g_5[i] = g_4[i];
            assign p_5[i] = p_4[i];
        end
        for (i = 16; i < 32; i = i + 1) begin : gen_level5
            assign g_5[i] = g_4[i] | (p_4[i] & g_4[i-16]);
            assign p_5[i] = p_4[i] & p_4[i-16];
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Carry Vector & Sum Output
    // -------------------------------------------------------------------------
    assign c[0] = cin;
    generate
        for (i = 1; i < 32; i = i + 1) begin : gen_carries
            assign c[i] = g_5[i-1];
        end
    endgenerate

    generate
        for (i = 0; i < 32; i = i + 1) begin : gen_sum
            assign sum[i] = p_0[i] ^ c[i];
        end
    endgenerate

    assign cout = g_5[31];

endmodule
