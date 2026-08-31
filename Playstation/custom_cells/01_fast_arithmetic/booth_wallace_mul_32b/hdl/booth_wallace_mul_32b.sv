// =============================================================================
// File Name   : booth_wallace_mul_32b.sv
// Module Name : booth_wallace_mul_32b
// Project     : Custom Cell Optimization Library
// -----------------------------------------------------------------------------
// Description : High-Performance 32x32 Signed Q16.16 Fixed-Point Multiplier
//
// Architecture:
//   1. Radix-4 Modified Booth Encoding: Halves partial products from 32 to 16.
//   2. 4:2 Carry-Save Compressor Tree: Logarithmic reduction of 16 partial products
//      down to 2 vectors (Sum and Carry) with minimal XOR logic depth.
//   3. High-Speed 64-bit Prefix Carry-Propagate Adder.
//   4. Integrated Q16.16 fixed-point bit-slicing and single-cycle overflow detection.
// =============================================================================

`timescale 1ns / 1ps

module booth_wallace_mul_32b (
    input  logic signed [31:0] src_a,       // Multiplicand (Q16.16)
    input  logic signed [31:0] src_b,       // Multiplier (Q16.16)
    output logic signed [31:0] result,      // Normalized Q16.16 product (bits [47:16])
    output logic signed [63:0] product_64,  // Full 64-bit unscaled product
    output logic               overflow     // Arithmetic overflow flag
);

    // -------------------------------------------------------------------------
    // Step 1: Radix-4 Booth Encoding of Multiplier 'src_b'
    // -------------------------------------------------------------------------
    logic [32:0] b_ext;
    assign b_ext = {src_b, 1'b0}; // b[-1] = 0

    logic [15:0] single;
    logic [15:0] double;
    logic [15:0] neg;

    genvar i;
    generate
        for (i = 0; i < 16; i = i + 1) begin : gen_booth_enc
            booth_encoder u_enc (
                .y_plus (b_ext[2*i + 2]),
                .y_curr (b_ext[2*i + 1]),
                .y_prev (b_ext[2*i]),
                .single (single[i]),
                .double (double[i]),
                .neg    (neg[i])
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Step 2: Partial Product Generation (16 Partial Products)
    // -------------------------------------------------------------------------
    logic signed [63:0] pp [0:15];
    logic signed [63:0] a_ext;
    assign a_ext = {{32{src_a[31]}}, src_a};

    generate
        for (i = 0; i < 16; i = i + 1) begin : gen_pp
            logic signed [63:0] base_pp;
            always_comb begin
                if (single[i]) begin
                    base_pp = (a_ext << (2*i));
                end else if (double[i]) begin
                    base_pp = (a_ext << (2*i + 1));
                end else begin
                    base_pp = 64'sd0;
                end

                if (neg[i]) begin
                    pp[i] = ~base_pp + (64'd1 << (2*i));
                end else begin
                    pp[i] = base_pp;
                end
            end
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Step 3: Wallace Tree Reduction using 4:2 Compressors
    // -------------------------------------------------------------------------
    // Level 1: Compress 16 PPs -> 8 PPs (4 groups of 4:2)
    wire [63:0] l1_sum   [0:3];
    wire [63:0] l1_carry [0:3];

    genvar g_grp, g_bit;
    generate
        for (g_grp = 0; g_grp < 4; g_grp = g_grp + 1) begin : gen_l1_grp
            for (g_bit = 0; g_bit < 64; g_bit = g_bit + 1) begin : gen_l1_bit
                compressor_4to2 u_c42_l1 (
                    .x1   (pp[4*g_grp + 0][g_bit]),
                    .x2   (pp[4*g_grp + 1][g_bit]),
                    .x3   (pp[4*g_grp + 2][g_bit]),
                    .x4   (pp[4*g_grp + 3][g_bit]),
                    .cin  (1'b0),
                    .sum  (l1_sum[g_grp][g_bit]),
                    .carry(l1_carry[g_grp][g_bit]),
                    .cout ()
                );
            end
        end
    endgenerate

    // Level 2: Compress 8 vectors -> 4 vectors (2 groups of 4:2)
    wire [63:0] l2_sum   [0:1];
    wire [63:0] l2_carry [0:1];

    generate
        for (g_grp = 0; g_grp < 2; g_grp = g_grp + 1) begin : gen_l2_grp
            for (g_bit = 0; g_bit < 64; g_bit = g_bit + 1) begin : gen_l2_bit
                wire c_in_bit2 = (g_bit == 0) ? 1'b0 : l1_carry[2*g_grp + 0][g_bit - 1];
                wire c_in_bit4 = (g_bit == 0) ? 1'b0 : l1_carry[2*g_grp + 1][g_bit - 1];
                compressor_4to2 u_c42_l2 (
                    .x1   (l1_sum[2*g_grp + 0][g_bit]),
                    .x2   (c_in_bit2),
                    .x3   (l1_sum[2*g_grp + 1][g_bit]),
                    .x4   (c_in_bit4),
                    .cin  (1'b0),
                    .sum  (l2_sum[g_grp][g_bit]),
                    .carry(l2_carry[g_grp][g_bit]),
                    .cout ()
                );
            end
        end
    endgenerate

    // Level 3: Compress 4 vectors -> 2 vectors (1 group of 4:2)
    wire [63:0] final_sum_vec;
    wire [63:0] final_carry_vec;

    generate
        for (g_bit = 0; g_bit < 64; g_bit = g_bit + 1) begin : gen_l3_bit
            wire c_in_final2 = (g_bit == 0) ? 1'b0 : l2_carry[0][g_bit - 1];
            wire c_in_final4 = (g_bit == 0) ? 1'b0 : l2_carry[1][g_bit - 1];
            compressor_4to2 u_c42_l3 (
                .x1   (l2_sum[0][g_bit]),
                .x2   (c_in_final2),
                .x3   (l2_sum[1][g_bit]),
                .x4   (c_in_final4),
                .cin  (1'b0),
                .sum  (final_sum_vec[g_bit]),
                .carry(final_carry_vec[g_bit]),
                .cout ()
            );
        end
    endgenerate

    // -------------------------------------------------------------------------
    // Step 4: Final 64-bit Addition & Q16.16 Slicing
    // -------------------------------------------------------------------------
    assign product_64 = final_sum_vec + {final_carry_vec[62:0], 1'b0};
    assign result     = product_64[47:16];

    // Overflow check for 32-bit Q16.16 range
    assign overflow   = (product_64[63:47] != 17'sd0) && (product_64[63:47] != 17'sh1FFFF);

endmodule
